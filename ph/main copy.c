/* ============================================================
 *  main.c — PlutoSDR one-way link
 *
 *  Режимы:
 *    ./sdr_test --tx "HELLO"       — циклически шлёт BPSK-кадр
 *    ./sdr_test --rx               — демодулирует и печатает текст
 *    ./sdr_test --rx-raw 5         — пишет 5 секунд сырого IQ в
 *                                    rx_capture.cf32 (для plot_iq.py)
 *
 *  Формат CF32: 1 сэмпл = 2 float (I, Q).
 * ============================================================ */
#define _GNU_SOURCE
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===================== ПАРАМЕТРЫ ===================== */
static const char  *SDR_URI         = "usb:";
static const double SAMPLE_RATE     = 1e6;
static const double CENTER_FREQ     = 800e6;
static const double GAIN_RX         = 40.0;
static const double GAIN_TX         = -20.0;

static const double SUB_FREQ        = 1.0e5;    /* поднесущая 100 кГц */
static const int    SPS             = 8;        /* samples per symbol */

static const int    PREAMBLE_SYMS   = 64;
static const int    SYNC_LEN        = 13;
static const int    MAX_PAYLOAD     = 256;
static const int    FRAME_REPEAT_MS = 200;

static const size_t RX_MTU_LIMIT    = 65536;
static const long   TIMEOUT_US      = 500000;

/* ===================== Barker-13 ===================== */
static const int barker13[13] = {+1,+1,+1,+1,+1,-1,-1,+1,+1,-1,+1,-1,+1};

/* ===================== CRC32 ===================== */
static uint32_t crc32_(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & (-(int32_t)(c & 1)));
    }
    return ~c;
}

/* ===================== ГЕНЕРАЦИЯ КАДРА =====================
 * BPSK на поднесущей SUB_FREQ.
 * out — массив 2*max_samples float (I,Q,I,Q,...).
 * Возвращает число комплексных сэмплов (0 — если не влезло).
 */
static size_t build_frame(const char *text,
                          float *out,
                          size_t max_samples)
{
    size_t plen = strlen(text);
    if (plen > (size_t)MAX_PAYLOAD) plen = (size_t)MAX_PAYLOAD;

    /* payload = text || len(4 LE) || crc32(text,plen)(4 LE) */
    uint8_t payload[MAX_PAYLOAD + 8];
    memcpy(payload, text, plen);

    uint32_t len32 = (uint32_t)plen;
    payload[plen + 0] = (len32 >>  0) & 0xFF;
    payload[plen + 1] = (len32 >>  8) & 0xFF;
    payload[plen + 2] = (len32 >> 16) & 0xFF;
    payload[plen + 3] = (len32 >> 24) & 0xFF;

    uint32_t crc = crc32_((const uint8_t *)text, plen);
    payload[plen + 4] = (crc >>  0) & 0xFF;
    payload[plen + 5] = (crc >>  8) & 0xFF;
    payload[plen + 6] = (crc >> 16) & 0xFF;
    payload[plen + 7] = (crc >> 24) & 0xFF;

    size_t nbytes = plen + 8;
    size_t nbits  = nbytes * 8;
    size_t nsym   = (size_t)PREAMBLE_SYMS + (size_t)SYNC_LEN + nbits;
    size_t nsamp  = nsym * (size_t)SPS;
    if (nsamp > max_samples) return 0;

    size_t k = 0;

    #define EMIT_SYM(SYM)                                                  \
        do {                                                               \
            for (int _j = 0; _j < SPS; _j++, k++) {                        \
                double _t  = (double)k / SAMPLE_RATE;                      \
                double _ph = 2.0 * M_PI * SUB_FREQ * _t;                   \
                out[2*k + 0] = (float)((SYM) * cos(_ph));                  \
                out[2*k + 1] = (float)((SYM) * sin(_ph));                  \
            }                                                              \
        } while (0)

    for (int s = 0; s < PREAMBLE_SYMS; s++) EMIT_SYM(+1);
    for (int s = 0; s < SYNC_LEN;      s++) EMIT_SYM(barker13[s]);
    for (size_t b = 0; b < nbits; b++) {
        int bit = (payload[b / 8] >> (b % 8)) & 1;
        EMIT_SYM(bit ? +1 : -1);
    }
    #undef EMIT_SYM

    return nsamp;
}

/* ===================== ПОИСК SYNC =====================
 * Возвращает позицию первого сэмпла sync (0..n-sync_samps) или -1.
 */
static long find_frame(const float *buf, size_t n, double thresh)
{
    size_t sync_samps = (size_t)SYNC_LEN * (size_t)SPS;
    if (n < sync_samps) return -1;

    for (size_t pos = 0; pos + sync_samps <= n; pos++) {
        double re = 0.0, im = 0.0;
        for (int s = 0; s < SYNC_LEN; s++) {
            int sym = barker13[s];
            for (int j = 0; j < SPS; j++) {
                size_t idx = pos + (size_t)s * SPS + (size_t)j;
                double t   = (double)(s * SPS + j) / SAMPLE_RATE;
                double c   = cos(2.0 * M_PI * SUB_FREQ * t);
                double sn  = sin(2.0 * M_PI * SUB_FREQ * t);
                double xi  = buf[2*idx + 0];
                double xq  = buf[2*idx + 1];
                /* x * conj(carrier) */
                re += (xi*c + xq*sn) * sym;
                im += (xq*c - xi*sn) * sym;
            }
        }
        double mag = sqrt(re*re + im*im) / (double)sync_samps;
        if (mag > thresh) return (long)pos;
    }
    return -1;
}

/* ===================== ДЕМОДУЛЯЦИЯ =====================
 * sync_pos — позиция первого сэмпла sync.
 * Возвращает 0 при успехе, -1 при ошибке.
 */
static int decode_payload(const float *buf, size_t n, size_t sync_pos,
                          char *out, size_t outlen)
{
    size_t data_start = sync_pos + (size_t)SYNC_LEN * (size_t)SPS;

    #define READ_BIT(IDX) ({                                               \
        size_t _s  = data_start + (IDX) * (size_t)SPS;                     \
        double _re = 0.0;                                                  \
        for (int _j = 0; _j < SPS; _j++) {                                 \
            double _t  = (double)_j / SAMPLE_RATE;                         \
            double _c  = cos(2.0 * M_PI * SUB_FREQ * _t);                  \
            double _sn = sin(2.0 * M_PI * SUB_FREQ * _t);                  \
            double _xi = buf[2*(_s + _j) + 0];                             \
            double _xq = buf[2*(_s + _j) + 1];                             \
            _re += _xi * _c + _xq * _sn;                                   \
        }                                                                  \
        (_re > 0.0) ? 1 : 0;                                               \
    })

    /* 32 бита длины */
    if (data_start + (size_t)32 * SPS > n) return -1;
    uint32_t plen = 0;
    for (int b = 0; b < 32; b++)
        plen |= (uint32_t)READ_BIT(b) << b;
    if (plen > (uint32_t)MAX_PAYLOAD) return -1;

    size_t total_bytes = (size_t)plen + 4;         /* payload + crc */
    size_t total_bits  = total_bytes * 8;
    if (data_start + (size_t)32 * SPS + total_bits * (size_t)SPS > n)
        return -1;

    uint8_t bytes[MAX_PAYLOAD + 4];
    memset(bytes, 0, sizeof(bytes));
    for (size_t b = 0; b < total_bits; b++) {
        int bit = READ_BIT((size_t)32 + b);
        if (bit) bytes[b / 8] |= (uint8_t)(1u << (b % 8));
    }
    #undef READ_BIT

    uint32_t rxcrc =  (uint32_t)bytes[plen + 0]
                   | ((uint32_t)bytes[plen + 1] << 8)
                   | ((uint32_t)bytes[plen + 2] << 16)
                   | ((uint32_t)bytes[plen + 3] << 24);
    uint32_t calc = crc32_(bytes, plen);
    if (rxcrc != calc) return -1;

    size_t cl = (size_t)plen < outlen - 1 ? (size_t)plen : outlen - 1;
    memcpy(out, bytes, cl);
    out[cl] = '\0';
    return 0;
}

/* ===================== MAIN ===================== */
static volatile sig_atomic_t stop_flag = 0;
static void on_sigint(int s) { (void)s; stop_flag = 1; }

static void usage(const char *p) {
    fprintf(stderr,
        "usage:\n"
        "  %s --tx \"TEXT\"       send BPSK frame repeatedly\n"
        "  %s --rx                receive + demodulate\n"
        "  %s --rx-raw SECONDS    capture raw IQ to rx_capture.cf32\n",
        p, p, p);
}

int main(int argc, char **argv)
{
    signal(SIGINT, on_sigint);

    if (argc < 2) { usage(argv[0]); return 1; }

    int is_tx = 0, is_rx = 0, is_raw = 0;
    const char *text = NULL;
    double capture_sec = 5.0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--tx")) {
            is_tx = 1;
            if (i + 1 < argc) text = argv[++i];
        } else if (!strcmp(argv[i], "--rx-raw")) {
            is_rx = 1;
            is_raw = 1;
            if (i + 1 < argc) capture_sec = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--rx")) {
            is_rx = 1;
        } else {
            usage(argv[0]); return 1;
        }
    }
    if ((is_tx && is_rx) || (!is_tx && !is_rx) || (is_tx && !text)) {
        usage(argv[0]); return 1;
    }

    /* --- Открываем устройство --- */
    SoapySDRKwargs args = {0};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", SDR_URI);

    SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);
    if (!sdr) {
        fprintf(stderr, "make: %s\n", SoapySDRDevice_lastError());
        return 1;
    }

    size_t channels[] = {0};
    const int dir = is_tx ? SOAPY_SDR_TX : SOAPY_SDR_RX;

    SoapySDRDevice_setSampleRate(sdr, dir, 0, SAMPLE_RATE);
    SoapySDRDevice_setFrequency (sdr, dir, 0, CENTER_FREQ, NULL);
    SoapySDRDevice_setGain      (sdr, dir, 0, is_tx ? GAIN_TX : GAIN_RX);

    SoapySDRStream *stream = SoapySDRDevice_setupStream(
        sdr, dir, SOAPY_SDR_CF32, channels, 1, NULL);
    if (!stream) {
        fprintf(stderr, "setupStream: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return 1;
    }

    size_t mtu = SoapySDRDevice_getStreamMTU(sdr, stream);
    if (mtu > RX_MTU_LIMIT) mtu = RX_MTU_LIMIT;
    fprintf(stderr, "[init] %s MTU=%zu\n", is_tx ? "TX" : "RX", mtu);

    /* ===================== TX ===================== */
    if (is_tx) {
        float *frame = malloc(2 * mtu * sizeof(float));
        if (!frame) { fprintf(stderr, "malloc fail\n"); return 1; }

        size_t fs = build_frame(text, frame, mtu);
        if (!fs) {
            fprintf(stderr, "frame too big (%zu > %zu)\n", fs, mtu);
            free(frame);
            return 1;
        }
        fprintf(stderr, "[TX] frame %zu samples, text=\"%s\"\n", fs, text);

        SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0);

        long long hw = SoapySDRDevice_getHardwareTime(sdr, NULL) + 100000000LL;
        const long long period = (long long)FRAME_REPEAT_MS * 1000000LL;

        while (!stop_flag) {
            const void *buffs[] = { frame };
            int flags = SOAPY_SDR_HAS_TIME;
            int st = SoapySDRDevice_writeStream(sdr, stream, buffs, fs,
                                                &flags, hw, TIMEOUT_US);
            if (st < 0) {
                fprintf(stderr, "[TX-ERR] %d %s\n", st,
                        SoapySDRDevice_lastError());
            } else {
                fprintf(stderr, "[TX] sent %d @ %lld\n", st, hw);
            }
            hw += period;
        }
        free(frame);
    }
    /* ===================== RX-RAW ===================== */
    else if (is_raw) {
        SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0);

        float *buf = malloc(2 * mtu * sizeof(float));
        if (!buf) { fprintf(stderr, "malloc fail\n"); return 1; }

        const char *outname = "rx_capture.cf32";
        FILE *fp = fopen(outname, "wb");
        if (!fp) {
            fprintf(stderr, "cannot open %s\n", outname);
            free(buf);
            return 1;
        }

        size_t total_samples = (size_t)(capture_sec * SAMPLE_RATE);
        size_t written = 0;

        fprintf(stderr, "[RX-RAW] writing %.1f s (%zu samples) to %s\n",
                capture_sec, total_samples, outname);

        while (!stop_flag && written < total_samples) {
            void *buffs[] = { buf };
            int flags = 0;
            long long t = 0;
            int sr = SoapySDRDevice_readStream(sdr, stream, buffs, mtu,
                                               &flags, &t, TIMEOUT_US);
            if (sr < 0) {
                if (sr != SOAPY_SDR_TIMEOUT)
                    fprintf(stderr, "[RX-ERR] %d %s\n", sr,
                            SoapySDRDevice_lastError());
                continue;
            }
            /* CF32: 1 сэмпл = 2 float (I,Q) */
            fwrite(buf, 2 * sizeof(float), (size_t)sr, fp);
            written += (size_t)sr;
        }
        fclose(fp);
        free(buf);

        fprintf(stderr, "[RX-RAW] done. %zu samples written.\n", written);
        fprintf(stderr, "         Now run:\n");
        fprintf(stderr, "           python3 plot_iq.py %s\n", outname);
    }
    /* ===================== RX (demod) ===================== */
    else {
        SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0);

        float *buf = malloc(2 * mtu * sizeof(float));
        if (!buf) { fprintf(stderr, "malloc fail\n"); return 1; }

        FILE *fp = fopen("rx_capture.pcm", "wb");
        if (!fp) fprintf(stderr, "warn: cannot open rx_capture.pcm\n");

        size_t iter = 0;
        while (!stop_flag) {
            void *buffs[] = { buf };
            int flags = 0;
            long long t = 0;
            int sr = SoapySDRDevice_readStream(sdr, stream, buffs, mtu,
                                               &flags, &t, TIMEOUT_US);
            if (sr < 0) {
                if (sr != SOAPY_SDR_TIMEOUT)
                    fprintf(stderr, "[RX-ERR] %d %s\n", sr,
                            SoapySDRDevice_lastError());
                continue;
            }
            if (fp) {
                fwrite(buf, 2 * sizeof(float), (size_t)sr, fp);
                fflush(fp);
            }

            long pos = find_frame(buf, (size_t)sr, 0.3);
            if (pos >= 0) {
                char out[512];
                if (decode_payload(buf, (size_t)sr, (size_t)pos,
                                   out, sizeof(out)) == 0) {
                    printf("\n*** RX: \"%s\" (iter %zu, pos %ld) ***\n",
                           out, iter, pos);
                    fflush(stdout);
                } else {
                    printf("[RX] sync @ %ld but CRC fail (iter %zu)\n",
                           pos, iter);
                }
            }
            iter++;
        }
        if (fp) fclose(fp);
        free(buf);
    }

    SoapySDRDevice_deactivateStream(sdr, stream, 0, 0);
    SoapySDRDevice_closeStream(sdr, stream);
    SoapySDRDevice_unmake(sdr);
    return 0;
}