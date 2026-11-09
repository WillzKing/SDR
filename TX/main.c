#define _POSIX_C_SOURCE 200809L
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define BEACON_FREQ      200000.0      /* 200 кГц */
#define BEACON_AMP       12000.0       /* амплитуда тона */

int main(int argc, char **argv)
{
    if (argc != 2) {
        printf("Запуск: ./build/tx USB_URI\nАдрес посмотри командой iio_info -s\n");
        return 1;
    }

    SoapySDRKwargs args = {0};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", argv[1]);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "0");
    SoapySDRKwargs_set(&args, "loopback", "0");
    SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);
    if (!sdr) {
        fprintf(stderr, "TX Pluto: %s\n", SoapySDRDevice_lastError());
        return 1;
    }

    SoapySDRStream *tx = NULL;
    int16_t *data = NULL;
    FILE *file = NULL;
    size_t channel = 0;
    int active = 0, result = 1;

    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, 1000000) ||
        SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, 800000000, NULL) ||
        SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, -40)) goto cleanup;
    tx = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16,
                                    &channel, 1, NULL);
    if (!tx) goto cleanup;
    size_t mtu = SoapySDRDevice_getStreamMTU(sdr, tx);
    if (!mtu || mtu > SIZE_MAX / (2 * sizeof(int16_t))) goto cleanup;
    data = calloc(2 * mtu, sizeof(int16_t));
    if (!data) goto cleanup;

    /* Заполняем буфер тоном 200 кГц (маячок).
       Это заменило прежние data[2*n] = 1500 << 4; (DC-сигнал). */
    for (size_t n = 0; n < mtu; ++n) {
        double t = (double)n / 1000000.0;   /* SAMPLE_RATE = 1e6 */
        double p = 2.0 * M_PI * BEACON_FREQ * t;
        data[2 * n]     = (int16_t)(BEACON_AMP * cos(p));
        data[2 * n + 1] = (int16_t)(BEACON_AMP * sin(p));
    }

    file = fopen("txdata.pcm", "wb");
    if (!file) { perror("txdata.pcm"); goto cleanup; }
    if (fwrite(data, sizeof(int16_t), 2 * mtu, file) != 2 * mtu) {
        perror("Запись txdata.pcm"); goto cleanup;
    }
    int closed = fclose(file);
    file = NULL;
    if (closed) { perror("Закрытие txdata.pcm"); goto cleanup; }

    if (SoapySDRDevice_activateStream(sdr, tx, 0, 0, 0)) goto cleanup;
    active = 1;
    size_t total = 0;
    printf("TX: передаём тон %.0f Гц, около 0.2 секунды\n", BEACON_FREQ);

    /* Фаза накапливается между буферами, чтобы тон был непрерывным. */
    double phase = 0.0;
    const double dphi = 2.0 * M_PI * BEACON_FREQ / 1000000.0;

    while (total < 200000) {
        /* Перегенерируем буфер с учётом текущей фазы. */
        for (size_t n = 0; n < mtu; ++n) {
            data[2 * n]     = (int16_t)(BEACON_AMP * cos(phase));
            data[2 * n + 1] = (int16_t)(BEACON_AMP * sin(phase));
            phase += dphi;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        }

        size_t offset = 0;
        while (offset < mtu) {
            const void *buffers[] = {data + 2 * offset};
            int flags = 0;
            int count = SoapySDRDevice_writeStream(sdr, tx, buffers, mtu - offset,
                                                  &flags, 0, 400000);
            if (count <= 0 || (size_t)count > mtu - offset) {
                fprintf(stderr, "Ошибка TX: %d\n", count); goto cleanup;
            }
            offset += (size_t)count;
        }
        total += mtu;
    }
    // Даём очереди время отправить последние буферы.
    struct timespec delay = {0, 300000000};
    while (nanosleep(&delay, &delay) && errno == EINTR) {}
    printf("TX: драйвер принял %zu отсчётов. Буфер сохранён в txdata.pcm\n", total);
    result = 0;

cleanup:
    if (result) fprintf(stderr, "TX не завершён: %s\n", SoapySDRDevice_lastError());
    if (active) {
        /* Пауза перед остановкой потока — лечит
           "Failed to send stop TX stream cmd (-1)". */
        struct timespec ts = {0, 500000000};
        while (nanosleep(&ts, &ts) && errno == EINTR) {}
        SoapySDRDevice_deactivateStream(sdr, tx, 0, 0);
    }
    if (tx) SoapySDRDevice_closeStream(sdr, tx);
    if (file && fclose(file)) result = 1;
    free(data);
    SoapySDRDevice_unmake(sdr);
    return result;
}