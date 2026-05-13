// #include <Arduino.h>
// #include "pins.h"
// #include <RadioLib.h>

// #include "USB.h"
// #include "USBCDC.h"

// #include "flrc.h"

// USBCDC USBSerial;
// #undef Serial
// #define Serial USBSerial

// LR2021Driver driver;

// #define BENCHMARK_PACKET_COUNT 200 // total packets to send per run

// #ifdef IS_CAM
// // ─── ESP-IDF GDMA SPI additions ──────────────────────────────────────────────
// // spi_device_transmit() on ESP32-S3 uses GDMA automatically for transfers
// // larger than the CPU FIFO threshold (~32 bytes). The 257 and 258-byte FIFO
// // writes will both use GDMA, freeing the CPU during the ~43 µs transfer window.
// // We use a single spi_device_handle_t on the IDF-initialized HSPI (SPI2) bus.
// // RadioLib's SPIClass wraps the same bus; mySPI.begin() is called AFTER
// // spi_bus_initialize() so Arduino detects the bus is already up and skips
// // double-init. All hot-path transfers go through spiHandle via spiSend/spiSendRecv.
// #include "driver/spi_master.h"
// #include "esp_heap_caps.h"
// #include "driver/spi_common.h" // for SPICOMMON_BUSFLAG_MASTER

// // DMA-capable buffers — heap_caps_malloc guarantees 32-bit aligned DMA memory
// // chunk1: opcode(2) + first 255 payload bytes = 257 bytes
// // chunk2: opcode(2) + next 256 payload bytes  = 258 bytes
// #define DMA_CMD_HDR 2
// #define DMA_BUF1_SIZE (DMA_CMD_HDR + FIFO_CHUNK_1) // 257 bytes
// #define DMA_BUF2_SIZE (DMA_CMD_HDR + FIFO_CHUNK_2) // 258 bytes

// static uint8_t *dmaBuf1 = nullptr; // pre-built WriteRadioTxFifo chunk 1
// static uint8_t *dmaBuf2 = nullptr; // pre-built WriteRadioTxFifo chunk 2
// // Single IDF device handle for ALL SPI transfers — DMA and short control commands.
// // Replaces spiDma; mySPI is kept only for RadioLib's internal init calls.
// static spi_device_handle_t spiHandle = nullptr;
// static bool dmaReady = false;
// // ─────────────────────────────────────────────────────────────────────────────

// // ─── 511-byte FIFO streaming additions ───────────────────────────────────────
// // The LR2021 TX FIFO is 256 bytes deep. For 511-byte packets we must:
// //   1. Write first 255 bytes before SetTx
// //   2. When TxFifoLow IRQ fires (FIFO dropping below threshold), write remaining 256 bytes
// // Both chunks use WriteRadioTxFifo opcode 0x00 0x02 (DS Table 6-2)
// // ConfigFifoIrq opcode: 0x01 0x1A  (DS Table 6-50)
// // TxFifo Low flag bit: 0x02        (DS §6.10.1)

// #define FIFO_CHUNK_1 255    // bytes in first write (fills FIFO without overflow)
// #define FIFO_CHUNK_2 256    // bytes in second write (remainder of 511)
// #define FIFO_LOW_THRESH 128 // fire TxFifoLow IRQ when FIFO drops below this level

// // State shared between loop() and the TxFifoLow ISR
// volatile bool fifoRefillNeeded = false; // set by TxFifoLow ISR
// volatile bool txDone = false;           // set by TxDone ISR (already existed)

// // Pointer to the second chunk in the payload buffer — set before SetTx
// static uint8_t *fifoRefillPtr = nullptr;
// static uint16_t fifoRefillLen = 0;

// // Payload buffer — single buffer is fine since we never overlap packets
// uint8_t payload[PAYLOAD_SIZE];

// int packetsSent = 0;
// int packetsOk = 0;
// int packetsFailed = 0;
// unsigned long benchStartMs = 0;

// volatile bool radioEvent = false;

// IRAM_ATTR void setRadioEventFlag(void)
// {
//     radioEvent = true;
// }

// #endif

// // Called once after IDF bus init and RadioLib init complete.
// // Registers ONE spi_device_handle_t on SPI2_HOST for all hot-path transfers.
// // RadioLib continues using mySPI for its own calls; we never call mySPI
// // in loop() — all hot-path SPI goes through spiHandle via spiSend/spiSendRecv.
// static bool initSpiDma()
// {
//     dmaBuf1 = (uint8_t *)heap_caps_malloc(DMA_BUF1_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
//     dmaBuf2 = (uint8_t *)heap_caps_malloc(DMA_BUF2_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
//     if (!dmaBuf1 || !dmaBuf2)
//     {
//         Serial.println(F("[DMA] alloc failed"));
//         return false;
//     }

//     // Pre-build command headers — these never change
//     dmaBuf1[0] = 0x00; // WriteRadioTxFifo opcode byte 0
//     dmaBuf1[1] = 0x02; // WriteRadioTxFifo opcode byte 1
//     dmaBuf2[0] = 0x00;
//     dmaBuf2[1] = 0x02;

//     spi_device_interface_config_t cfg = {};
//     cfg.mode = 0;                  // SPI_MODE0
//     cfg.clock_speed_hz = 48000000; // 48 MHz — matches spiSettings
//     cfg.spics_io_num = LR2021_CS;  // CS managed by IDF driver
//     cfg.queue_size = 1;
//     // No command/address phases — we embed opcode in tx_buffer
//     cfg.command_bits = 0;
//     cfg.address_bits = 0;

//     esp_err_t r = spi_bus_add_device(SPI2_HOST, &cfg, &spiHandle);
//     if (r != ESP_OK)
//     {
//         Serial.print(F("[DMA] spi_bus_add_device failed: "));
//         Serial.println(esp_err_to_name(r));
//         return false;
//     }
//     Serial.println(F("[DMA] GDMA SPI ready"));
//     return true;
// }

// // Sends `len` bytes from `buf` via IDF spi_device_transmit.
// // IDF uses GDMA automatically for transfers > ~32 bytes on ESP32-S3.
// // BUSY must be polled by the caller before invoking this — IDF manages CS only.
// static void IRAM_ATTR spiSend(const uint8_t *buf, size_t len)
// {
//     spi_transaction_t t = {};
//     t.length = len * 8; // bits
//     t.tx_buffer = buf;
//     t.rx_buffer = nullptr;
//     spi_device_transmit(spiHandle, &t); // blocks until transfer completes
// }

// // Send+receive variant — used for GetAndClearIrqStatus where we need the
// // 6-byte response. tx and rx must both be len bytes long.
// static void IRAM_ATTR spiSendRecv(const uint8_t *tx, uint8_t *rx, size_t len)
// {
//     spi_transaction_t t = {};
//     t.length = len * 8; // bits
//     t.tx_buffer = tx;
//     t.rx_buffer = rx;
//     spi_device_transmit(spiHandle, &t);
// }

// // DMA-backed WriteRadioTxFifo. Copies `len` bytes from `src` into the
// // pre-allocated DMA buffer (after the 2-byte opcode header), then fires
// // spi_device_transmit() which uses GDMA on ESP32-S3 for transfers > ~32 bytes.
// // Falls back to spiSend() (still IDF, just non-DMA path) if dmaReady is false
// // — dmaReady is only false if heap alloc failed, which is fatal anyway.
// static inline void IRAM_ATTR dmaWriteTxFifo(uint8_t *dmaBuf, const uint8_t *src,
//                                             size_t len, size_t totalBufSize)
// {
//     memcpy(dmaBuf + DMA_CMD_HDR, src, len); // update payload in DMA buffer

//     // dmaReady check kept for safety; both paths now use spiHandle
//     if (dmaReady)
//     {
//         spi_transaction_t t = {};
//         t.length = totalBufSize * 8; // bits
//         t.tx_buffer = dmaBuf;
//         t.rx_buffer = nullptr;
//         // spi_device_transmit manages CS; BUSY is polled by the caller before this
//         spi_device_transmit(spiHandle, &t); // blocks until GDMA transfer completes
//     }
//     else
//     {
//         // Fallback: still use IDF handle, just without pre-built DMA buffer
//         // (spiHandle is always valid if we reach this point)
//         spiSend(dmaBuf, totalBufSize);
//     }
// }

// void setup()
// {
//     setCpuFrequencyMhz(240);

//     USB.begin();
//     Serial.begin(115200);

//     pinMode(LED_RED, OUTPUT);
//     pinMode(LED_BLUE, OUTPUT);
//     pinMode(LED_GREEN, OUTPUT);
//     pinMode(LED_ORANGE, OUTPUT);

// #ifdef IS_CAM
//     while (!Serial)
//     {
//     };
//     delay(50);
// #endif

// #ifdef IS_CAM
//     Serial.println(F("Mode: CAM (Transmitter - Benchmark)"));
// #elifdef IS_EAGLE
//     Serial.println(F("Mode: EAGLE (Receiver - Benchmark)"));
// #endif

//     LR2021Result result = driver.init();
//     if (!result.ok())
//     {
//         Serial.print(F("[LR2021] Init failed at stage: "));
//         Serial.println(result.stageStr());
//         Serial.print(F("  RadioLib code: "));
//         Serial.println(result.radioLibCode);
//         while (true)
//         {
//             delay(10);
//         }
//     }
//     Serial.println(F("[LR2021] Init OK"));
// }

// void loop()
// {
// }