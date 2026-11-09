/* ============================================================
 *  main.c — PlutoSDR one-way link
 *
 *  QPSK + фиксированный payload + созвездие + муты.
 *
 *  Режимы:
 *    ./sdr_test --tx                — циклически шлёт QPSK-кадр,
 *                                     каждый 2-й кадр «мутится»
 *    ./sdr_test --rx                — демодулирует, пишет constellation.cf32
 *    ./sdr_test --rx-raw 5          — пишет 5 секунд сырого IQ в
 *                                     rx_capture.cf32
 *
 *  Флаги мутности:
 *    --mute-frame   (по умолчанию) чётный кадр — сигнал,
 *                                  нечётный — тишина
 *    --mute-symbol                внутри payload каждый 2-й
 *                                  QPSK-символ = 0
 *    --mute-off                   непрерывный сигнал
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

/* п.4: амплитуда сигнала (отличать от белого шума) */
static const double AMPLITUDE       = 1500.0;

/* п.6: фиксированный MTU */
static const size_t FIXED_MTU       = 1920;

/* п.5: сдвиг на 4 бита (PlutoSDR) — 4 пустых символа перед преамбулой */
static const int    BIT_SHIFT       = 4;

/* п.1: фиксированный payload (4 байта = 32 бита = 16 QPSK-символов) */
static const uint8_t FIXED_PAYLOAD[4] = { 0xAA, 0x55, 0xFF, 0x00 };

static const int    PREAMBLE_SYMS   = 64;
static const int    SYNC_LEN        = 13;
static const long   TIMEOUT_US      = 500000;

/* п.7: режим «мутности» */
typedef enum {
    MUTE_OFF = 0,
    MUTE_FRAME,
    MUTE_SYMBOL
} mute_mode_t;

static mute_mode_t MUTE_MODE = MUTE_FRAME;

/* ===================== Barker-13 ===================== */
static const int barker13[13] = {+1,+1,+1,+1,+1,-1,-1,+1,+1,-1,+1,-1,+1};

/* ===================== QPSK-маппинг (Грей) =====================
 * 00 → (+s,+s)
 * 01 → (-s,+s)
 * 11 → (-s,-s)
 * 10 → (+s,-s)
 * s = 1/√2 → средняя мощность 1
 */
static void qpsk_map(int b0, int b1, double *i, double *q)
{
    double s = 1.0 / sqrt(2.0);
    if (b0 == 0 && b1 == 0) { *i = +s; *q = +s; }
    else if (b0 == 0 && b1 == 1) { *i = -s; *q = +s; }
    else if (b0 == 1 && b1 == 1) { *i = -s; *q = -s; }
    else                         { *i = +s; *q = -s; }
}

/* ===================== ГЕНЕРАЦИЯ КАДРА ===================== */
static size_t build_frame(float *out, size_t max_samples, int mute_frame)
{
    const int nbits = (int)(sizeof(FIXED_PAYLOAD) * 8);
    const int nsym_payload = nbits / 2;

    size_t nsym = (size_t)BIT_SHIFT
                + (size_t)PREAMBLE_SYMS
                + (size_t)SYNC_LEN
                + (size_t)nsym_payload;
    size_t nsamp = nsym * (size_t)SPS;
    if (nsamp > max_samples) return 0;

    /* Вариант A: нечётный кадр = чистая тишина */
    if (mute_frame) {
        memset(out, 0, 2 * nsamp * sizeof(float));
        return nsamp;
    }

    size_t k = 0;

    #define EMIT_SYM(I_, Q_)                                                \
        do {                                                                \
            for (int _j = 0; _j < SPS; _j++, k++) {                         \
                double _t  = (double)k / SAMPLE_RATE;                       \
                double _ph = 2.0 * M_PI * SUB_FREQ * _t;                    \
                out[2*k + 0] = (float)(AMPLITUDE * ((I_) * cos(_ph)         \
                                                  - (Q_) * sin(_ph)));      \
                out[2*k + 1] = (float)(AMPLITUDE * ((I_) * sin(_ph)         \
                                                  + (Q_) * cos(_ph)));      \
            }                                                               \
        } while (0)

    /* п.5: 4 пустых символа в начале */
    for (int s = 0; s < BIT_SHIFT; s++) EMIT_SYM(0.0, 0.0);

    /* Преамбула: +1 по I — BPSK-пилот для АРУ */
    for (int s = 0; s < PREAMBLE_SYMS; s++) EMIT_SYM(+1.0, 0.0);

    /* Barker-13 по I */
    for (int s = 0; s < SYNC_LEN; s++)
        EMIT_SYM((double)barker13[s], 0.0);

    /* Payload: QPSK, при MUTE_SYMBOL — каждый 2-й символ в ноль */
    for (int s = 0; s < nsym_payload; s++) {
        int b = s * 2;
        int byte_idx = b / 8;
        int bit_idx  = b % 8;
        int b0 = (FIXED_PAYLOAD[byte_idx] >> (7 - bit_idx))     & 1;
        int b1 = (FIXED_PAYLOAD[byte_idx] >> (7 - bit_idx - 1)) & 1;

        if (MUTE_MODE == MUTE_SYMBOL && (s % 2) == 1) {
            EMIT_SYM(0.0, 0.0);
        } else {
            double i, q;
            qpsk_map(b0, b1, &i, &q);
            EMIT_SYM(i, q);
        }
    }
    #undef EMIT_SYM

    return nsamp;
}

/* ===================== ПОИСК SYNC ===================== */
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
                re += (xi*c + xq*sn) * sym;
                im += (xq*c - xi*sn) * sym;
            }
        }
        double mag = sqrt(re*re + im*im) / (double)sync_samps;
        if (mag > thresh) return (long)pos;
    }
    return -1;
}

/* ===================== ДЕМОДУЛЯЦИЯ QPSK + СОЗВЕЗДИЕ ===================== */
static int decode_payload_qpsk(const float *buf, size_t n, size_t sync_pos,
                               const uint8_t *expected, size_t exp_len,
                               FILE *cons_out)
{
    size_t data_start = sync_pos + (size_t)SYNC_LEN * (size_t)SPS;

    size_t nbits = exp_len * 8;
    size_t nsym  = nbits / 2;
    if (data_start + nsym * (size_t)SPS > n) return -1;

    #define READ_SYM(IDX, OUT_I, OUT_Q) ({                                 \
        size_t _s  = data_start + (IDX) * (size_t)SPS;                     \
        double _re = 0.0, _im = 0.0;                                       \
        for (int _j = 0; _j < SPS; _j++) {                                 \
            double _t  = (double)_j / SAMPLE_RATE;                         \
            double _c  = cos(2.0 * M_PI * SUB_FREQ * _t);                  \
            double _sn = sin(2.0 * M_PI * SUB_FREQ * _t);                  \
            double _xi = buf[2*(_s + _j) + 0];                             \
            double _xq = buf[2*(_s + _j) + 1];                             \
            _re += _xi * _c + _xq * _sn;                                   \
            _im += _xq * _c - _xi * _sn;                                   \
        }                                                                  \
        (OUT_I) = _re;                                                     \
        (OUT_Q) = _im;                                                     \
    })

    uint8_t bytes[32];
    memset(bytes, 0, sizeof(bytes));

    for (size_t s = 0; s < nsym; s++) {
        double i, q;
        READ_SYM(s, i, q);

        /* п.2: пишем символ в файл созвездия */
        if (cons_out) {
            float pair[2] = { (float)i, (float)q };
            fwrite(pair, sizeof(float), 2, cons_out);
        }

        /* QPSK-демод: знак I и Q */
        int b0 = (i < 0.0) ? 1 : 0;
        int b1 = (q < 0.0) ? 1 : 0;

        /* обратный Грей */
        int bit0, bit1;
        if      (b0 == 0 && b1 == 0) { bit0 = 0; bit1 = 0; } /* 00 */
        else if (b0 == 1 && b1 == 0) { bit0 = 0; bit1 = 1; } /* 01 */
        else if (b0 == 1 && b1 == 1) { bit0 = 1; bit1 = 1; } /* 11 */
        else                         { bit0 = 1; bit1 = 0; } /* 10 */

        size_t b = s * 2;
        bytes[b / 8] |= (uint8_t)(bit0 << (7 - b % 8));
        bytes[(b+1) / 8] |= (uint8_t)(bit1 << (7 - (b+1) % 8));
    }
    #undef READ_SYM

    for (size_t i = 0; i < exp_len; i++) {
        if (bytes[i] != expected[i]) return -1;
    }
    return 0;
}

/* ===================== MAIN ===================== */
static volatile sig_atomic_t stop_flag = 0;
static void on_sigint(int s) { (void)s; stop_flag = 1; }

static void usage(const char *p) {
    fprintf(stderr,
        "usage:\n"
        "  %s --tx [--mute-frame|--mute-symbol|--mute-off]\n"
        "  %s --rx\n"
        "  %s --rx-raw SECONDS\n",
        p, p, p);
}

int main(int argc, char **argv)
{
    signal(SIGINT, on_sigint);
    if (argc < 2) { usage(argv[0]); return 1; }

    int is_tx = 0, is_rx = 0, is_raw = 0;
    double capture_sec = 5.0;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--tx"))        is_tx = 1;
        else if (!strcmp(argv[i], "--rx-raw")) {
            is_rx = 1; is_raw = 1;
            if (i + 1 < argc) capture_sec = atof(argv[++i]);
        }
        else if (!strcmp(argv[i], "--rx"))        is_rx = 1;
        else if (!strcmp(argv[i], "--mute-frame"))  MUTE_MODE = MUTE_FRAME;
        else if (!strcmp(argv[i], "--mute-symbol")) MUTE_MODE = MUTE_SYMBOL;
        else if (!strcmp(argv[i], "--mute-off"))    MUTE_MODE = MUTE_OFF;
        else { usage(argv[0]); return 1; }
    }
    if (is_tx == is_rx) { usage(argv[0]); return 1; }

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
    SoapySDRDevice_setBandwidth (sdr, dir, 0, SAMPLE_RATE);
    SoapySDRDevice_setGain      (sdr, dir, 0, is_tx ? GAIN_TX : GAIN_RX);

    SoapySDRStream *stream = SoapySDRDevice_setupStream(
        sdr, dir, SOAPY_SDR_CF32, channels, 1, NULL);
    if (!stream) {
        fprintf(stderr, "setupStream: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return 1;
    }

    /* п.6: MTU = 1920, но не больше, чем даёт железо */
    size_t mtu = FIXED_MTU;
    size_t hw_mtu = SoapySDRDevice_getStreamMTU(sdr, stream);
    if (hw_mtu > 0 && hw_mtu < mtu) {
        fprintf(stderr, "[warn] hw MTU=%zu < requested %zu\n", hw_mtu, mtu);
        mtu = hw_mtu;
    }
    fprintf(stderr, "[init] %s MTU=%zu\n", is_tx ? "TX" : "RX", mtu);

    /* ===================== TX ===================== */
    if (is_tx) {
        float *frame = malloc(2 * mtu * sizeof(float));
        if (!frame) return 1;

        size_t fs_full = build_frame(frame, mtu, 0);
        if (!fs_full) {
            fprintf(stderr, "frame too big (%zu > %zu)\n", fs_full, mtu);
            free(frame); return 1;
        }

        float *frame_mute = malloc(2 * mtu * sizeof(float));
        if (!frame_mute) { free(frame); return 1; }
        size_t fs_mute = build_frame(frame_mute, mtu, 1);
        if (!fs_mute) { free(frame); free(frame_mute); return 1; }

        fprintf(stderr, "[TX] QPSK frame %zu samples (full), %zu (mute), amp=%.0f, mute_mode=%d\n",
                fs_full, fs_mute, AMPLITUDE, (int)MUTE_MODE);

        SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0);
        long long hw = SoapySDRDevice_getHardwareTime(sdr, NULL) + 100000000LL;
        const long long period = 200000000LL; /* 200 мс */

        unsigned long tx_idx = 0;
        while (!stop_flag) {
            int mute = (MUTE_MODE == MUTE_FRAME) && (tx_idx & 1);
            const void *buffs[] = { mute ? frame_mute : frame };
            size_t fs = mute ? fs_mute : fs_full;

            int flags = SOAPY_SDR_HAS_TIME;
            int st = SoapySDRDevice_writeStream(sdr, stream, buffs, fs,
                                                &flags, hw, TIMEOUT_US);
            if (st < 0)
                fprintf(stderr, "[TX-ERR] %d %s\n", st, SoapySDRDevice_lastError());
            else
                fprintf(stderr, "[TX] %s %d @ %lld\n",
                        mute ? "MUTE" : "DATA", st, hw);

            hw += period;
            tx_idx++;
        }
        free(frame);
        free(frame_mute);
    }
    /* ===================== RX-RAW ===================== */
    else if (is_raw) {
        SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0);
        float *buf = malloc(2 * mtu * sizeof(float));
        if (!buf) { fprintf(stderr, "malloc fail\n"); return 1; }

        const char *outname = "rx_capture.cf32";
        FILE *fp = fopen(outname, "wb");
        if (!fp) { fprintf(stderr, "cannot open %s\n", outname); free(buf); return 1; }

        uint64_t total_samples = (uint64_t)(capture_sec * SAMPLE_RATE);
        uint64_t written = 0;

        fprintf(stderr, "[RX-RAW] writing %.1f s (%llu samples) to %s\n",
                capture_sec, (unsigned long long)total_samples, outname);

        while (!stop_flag && written < total_samples) {
            void *buffs[] = { buf };
            int flags = 0; long long t = 0;
            int sr = SoapySDRDevice_readStream(sdr, stream, buffs, mtu,
                                               &flags, &t, TIMEOUT_US);
            if (sr < 0) {
                if (sr != SOAPY_SDR_TIMEOUT)
                    fprintf(stderr, "[RX-ERR] %d %s\n", sr, SoapySDRDevice_lastError());
                continue;
            }
            fwrite(buf, 2 * sizeof(float), (size_t)sr, fp);
            written += (uint64_t)sr;
        }
        fclose(fp); free(buf);
        fprintf(stderr, "[RX-RAW] done. %llu samples written.\n",
                (unsigned long long)written);
    }
    /* ===================== RX (QPSK + constellation) ===================== */
    else {
        SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0);
        float *buf = malloc(2 * mtu * sizeof(float));
        if (!buf) { fprintf(stderr, "malloc fail\n"); return 1; }

        FILE *cons_fp = fopen("constellation.cf32", "wb");
        if (!cons_fp) fprintf(stderr, "warn: cannot open constellation.cf32\n");

        size_t iter = 0;
        while (!stop_flag) {
            void *buffs[] = { buf };
            int flags = 0; long long t = 0;
            int sr = SoapySDRDevice_readStream(sdr, stream, buffs, mtu,
                                               &flags, &t, TIMEOUT_US);
            if (sr < 0) {
                if (sr != SOAPY_SDR_TIMEOUT)
                    fprintf(stderr, "[RX-ERR] %d %s\n", sr, SoapySDRDevice_lastError());
                continue;
            }

            /* порог масштабируем под AMPLITUDE */
            long pos = find_frame(buf, (size_t)sr, 0.3 * AMPLITUDE);
            if (pos >= 0) {
                if (decode_payload_qpsk(buf, (size_t)sr, (size_t)pos,
                                        FIXED_PAYLOAD,
                                        sizeof(FIXED_PAYLOAD),
                                        cons_fp) == 0) {
                    printf("\n*** RX: QPSK OK (iter %zu, pos %ld) ***\n",
                           iter, pos);
                    fflush(stdout);
                } else {
                    printf("[RX] sync @ %ld but payload mismatch (iter %zu)\n",
                           pos, iter);
                }
            }
            iter++;
        }
        if (cons_fp) fclose(cons_fp);
        free(buf);
        fprintf(stderr, "[RX] constellation saved to constellation.cf32\n");
    }

    SoapySDRDevice_deactivateStream(sdr, stream, 0, 0);
    SoapySDRDevice_closeStream(sdr, stream);
    SoapySDRDevice_unmake(sdr);
    return 0;
}