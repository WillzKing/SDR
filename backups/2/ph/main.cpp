#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

int main(void)
{
    // ============================================================
    //  URI двух плат
    // ============================================================
    const char *URI_RX = "usb:1.7.5";   // плата, которая ПРИНИМАЕТ
    const char *URI_TX = "usb:1.8.5";   // плата, которая ПЕРЕДАЁТ

    // ============================================================
    //  1. Открываем RX-устройство
    // ============================================================
    SoapySDRKwargs rx_args = {};
    SoapySDRKwargs_set(&rx_args, "driver", "plutosdr");
    SoapySDRKwargs_set(&rx_args, "uri", URI_RX);
    SoapySDRKwargs_set(&rx_args, "direct", "1");
    SoapySDRKwargs_set(&rx_args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&rx_args, "loopback", "0");
    

    SoapySDRDevice *sdr_rx = SoapySDRDevice_make(&rx_args);
    SoapySDRKwargs_clear(&rx_args);

    if (sdr_rx == NULL) {
        printf("RX SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    // ============================================================
    //  2. Открываем TX-устройство (без timestamp_every — чтобы
    //     не было "Backwards timestamp step")
    // ============================================================
    SoapySDRKwargs tx_args = {};
    SoapySDRKwargs_set(&tx_args, "driver", "plutosdr");
    SoapySDRKwargs_set(&tx_args, "uri", URI_TX);
    SoapySDRKwargs_set(&tx_args, "direct", "1");
    SoapySDRKwargs_set(&tx_args, "loopback", "0");

    SoapySDRDevice *sdr_tx = SoapySDRDevice_make(&tx_args);
    SoapySDRKwargs_clear(&tx_args);

    if (sdr_tx == NULL) {
        printf("TX SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr_rx);
        return EXIT_FAILURE;
    }

    // ============================================================
    //  3. Общие параметры
    // ============================================================
    const double sample_rate  = 1e6;      // 1 MSPS
    const double carrier_freq = 800e6;    // 800 МГц
    const double tx_gain      = -20.0;    // dB — если сигнал слабый, поднять до -10 или 0
    const double rx_gain      =  30.0;    // dB

    // ============================================================
    //  4. Настраиваем RX-устройство
    // ============================================================
    SoapySDRDevice_setSampleRate(sdr_rx, SOAPY_SDR_RX, 0, sample_rate);
    SoapySDRDevice_setFrequency(sdr_rx, SOAPY_SDR_RX, 0, carrier_freq, NULL);
    SoapySDRDevice_setGain(sdr_rx, SOAPY_SDR_RX, 0, rx_gain);

    // ============================================================
    //  5. Настраиваем TX-устройство
    // ============================================================
    SoapySDRDevice_setSampleRate(sdr_tx, SOAPY_SDR_TX, 0, sample_rate);
    SoapySDRDevice_setFrequency(sdr_tx, SOAPY_SDR_TX, 0, carrier_freq, NULL);
    SoapySDRDevice_setGain(sdr_tx, SOAPY_SDR_TX, 0, tx_gain);

    printf("RX freq: %lf, TX freq: %lf\n",
           SoapySDRDevice_getFrequency(sdr_rx, SOAPY_SDR_RX, 0),
           SoapySDRDevice_getFrequency(sdr_tx, SOAPY_SDR_TX, 0));

    // ============================================================
    //  6. Потоки
    // ============================================================
    size_t channels[] = {0};
    size_t channel_count = 1;

    SoapySDRStream *rxStream = SoapySDRDevice_setupStream(
        sdr_rx, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (rxStream == NULL) {
        printf("setupStream rx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr_rx);
        SoapySDRDevice_unmake(sdr_tx);
        return EXIT_FAILURE;
    }

    SoapySDRStream *txStream = SoapySDRDevice_setupStream(
        sdr_tx, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (txStream == NULL) {
        printf("setupStream tx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_closeStream(sdr_rx, rxStream);
        SoapySDRDevice_unmake(sdr_rx);
        SoapySDRDevice_unmake(sdr_tx);
        return EXIT_FAILURE;
    }

    // ============================================================
    //  7. Буферы
    // ============================================================
    size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr_rx, rxStream);
    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr_tx, txStream);
    printf("MTU - TX: %lu, RX: %lu\n", tx_mtu, rx_mtu);

    int16_t tx_buff[2 * tx_mtu];
    int16_t rx_buffer[2 * rx_mtu];

    // ============================================================
    //  8. Формируем TX-сигнал: треугольник ("зубчики")
    // ============================================================
    {
        static double phase = 0.0;
        const double ampl   = 2000.0;
        const int    period = 100;   // сэмплов на один зубец

        for (size_t i = 0; i < tx_mtu; i++) {
            double p   = phase / period;
            double tri = 2.0 * (p < 0.5 ? p : (1.0 - p)) - 1.0;
            int16_t v  = (int16_t)(ampl * tri);

            tx_buff[2*i]     = v;   // I
            tx_buff[2*i + 1] = v;   // Q

            phase += 1.0;
            if (phase >= period) phase -= period;
        }
    }

    // ============================================================
    //  9. Активируем потоки
    // ============================================================
    SoapySDRDevice_activateStream(sdr_rx, rxStream, 0, 0, 0);
    SoapySDRDevice_activateStream(sdr_tx, txStream, 0, 0, 0);

    printf("Start test...\n");
    const long timeoutUs = 400000;

    // Файл для записи принятого сигнала
    FILE *file = fopen("txdata.pcm", "w");
    if (file == NULL) {
        printf("Cannot open txdata.pcm for writing\n");
        SoapySDRDevice_deactivateStream(sdr_rx, rxStream, 0, 0);
        SoapySDRDevice_deactivateStream(sdr_tx, txStream, 0, 0);
        SoapySDRDevice_closeStream(sdr_rx, rxStream);
        SoapySDRDevice_closeStream(sdr_tx, txStream);
        SoapySDRDevice_unmake(sdr_rx);
        SoapySDRDevice_unmake(sdr_tx);
        return EXIT_FAILURE;
    }

    size_t iteration_count = 100;

    for (size_t buffers_read = 0; buffers_read < iteration_count; buffers_read++)
    {
        // ---------- 9.1. Читаем RX ----------
        void *rx_buffs[] = {rx_buffer};
        int rx_flags;
        long long rx_timeNs;

        int sr = SoapySDRDevice_readStream(sdr_rx, rxStream, rx_buffs,
                                           rx_mtu, &rx_flags, &rx_timeNs, timeoutUs);
        if (sr < 0) {
            continue;
        }

        printf("RX buf %lu: samples=%i, flags=%i, time=%lli\n",
               buffers_read, sr, rx_flags, rx_timeNs);

        // Пишем принятые сэмплы в файл
        fwrite(rx_buffer, 2 * rx_mtu * sizeof(int16_t), 1, file);

        // ---------- 9.2. Отправляем TX ----------
        void *tx_buffs[] = {tx_buff};
        int tx_flags = 0;
        int st = SoapySDRDevice_writeStream(sdr_tx, txStream,
                    (const void * const*)tx_buffs, tx_mtu,
                    &tx_flags, 0, timeoutUs);
        if ((size_t)st != tx_mtu) {
            printf("TX Failed: %i\n", st);
        }
    }

    fclose(file);

    // ============================================================
    //  10. Очистка
    // ============================================================
    SoapySDRDevice_deactivateStream(sdr_rx, rxStream, 0, 0);
    SoapySDRDevice_deactivateStream(sdr_tx, txStream, 0, 0);
    SoapySDRDevice_closeStream(sdr_rx, rxStream);
    SoapySDRDevice_closeStream(sdr_tx, txStream);
    SoapySDRDevice_unmake(sdr_rx);
    SoapySDRDevice_unmake(sdr_tx);

    printf("test complete!\n");
    return EXIT_SUCCESS;
}