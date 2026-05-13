// #include <Arduino.h>
// #include "pins.h"
// #include <RadioLib.h>

// #include "USB.h"
// #include "USBCDC.h"

// USBCDC USBSerial;
// #undef Serial
// #define Serial USBSerial

// SPIClass mySPI(HSPI);
// SPISettings spiSettings(48000000, MSBFIRST, SPI_MODE0);
// LR2021 radio = new Module(LR2021_CS, LR2021_GPIO9, LR2021_NRST, LR2021_BUSY, mySPI, spiSettings);

// #ifdef IS_EAGLE
// volatile bool operationDone = false;

// IRAM_ATTR void setFlag(void)
// {
//     operationDone = true;
// }
// #endif

// // --- Benchmark config ---
// #define BENCHMARK_PACKET_COUNT 200 // total packets to send per run
// #define PAYLOAD_SIZE 511           // bytes per packet (max 255 bytes for FLRC)
// // ------------------------

// #ifdef IS_CAM

// // â”€â”€â”€ 511-byte FIFO streaming additions â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// // The LR2021 TX FIFO is 256 bytes deep. For 511-byte packets we must:
// //   1. Write first 255 bytes before SetTx
// //   2. When TxFifoLow IRQ fires (FIFO dropping below threshold), write remaining 256 bytes
// // Both chunks use WriteRadioTxFifo opcode 0x00 0x02 (DS Table 6-2)
// // ConfigFifoIrq opcode: 0x01 0x1A  (DS Table 6-50)
// // TxFifo Low flag bit: 0x02        (DS Â§6.10.1)

// #define FIFO_CHUNK_1 255    // bytes in first write (fills FIFO without overflow)
// #define FIFO_CHUNK_2 256    // bytes in second write (remainder of 511)
// #define FIFO_LOW_THRESH 128 // fire TxFifoLow IRQ when FIFO drops below this level

// // State shared between loop() and the TxFifoLow ISR
// volatile bool fifoRefillNeeded = false; // set by TxFifoLow ISR
// volatile bool txDone = false;           // set by TxDone ISR (already existed)

// // Pointer to the second chunk in the payload buffer â€” set before SetTx
// static uint8_t *fifoRefillPtr = nullptr;
// static uint16_t fifoRefillLen = 0;

// // Payload buffer â€” single buffer is fine since we never overlap packets
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

// #ifdef IS_EAGLE
// int packetsReceived = 0;
// int packetsExpected = BENCHMARK_PACKET_COUNT;
// long totalBytesRx = 0;
// unsigned long firstRxMs = 0;
// unsigned long lastRxMs = 0;
// bool benchmarkStarted = false;
// bool benchmarkDone = false;

// struct PktLog
// {
//     int seqNum;
//     float rssi;
//     float snr;
//     unsigned long rxMs;
// };
// PktLog pktLog[BENCHMARK_PACKET_COUNT];
// int rxErrors = 0;
// // -----------------------------------------------------------
// #endif

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

//     if (!mySPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, LR2021_CS))
//     {
//         Serial.println("SPI failed wamp wamp");
//         while (true)
//         {
//             delay(10);
//         }
//     }

//     radio.irqDioNum = 9;

//     Serial.print(F("Initializing ... "));
//     int radio_init_state = radio.beginFLRC(
//         434.0,                       // frequency MHz
//         2600,                        // 2600 Mbps bit rate
//         RADIOLIB_LR2021_FLRC_CR_1_0, // 1/0 coding rate
//         22,                          // 22 dBm TX power
//         24,                          // 24-bit preamble
//         RADIOLIB_SHAPING_NONE,       // Gaussian shaping BT is NONE
//         0.0                          // 0V = XTAL mode, not TCXO, same as radio.XTAL = true;
//     );

//     if (radio_init_state != RADIOLIB_ERR_NONE)
//     {
//         Serial.print(F("failed, code "));
//         Serial.println(radio_init_state);
//         while (true)
//         {
//             delay(10);
//         }
//     }

//     radio_init_state = radio.fixedPacketLengthMode(PAYLOAD_SIZE);

//     if (radio_init_state != RADIOLIB_ERR_NONE)
//     {
//         Serial.print(F("failed, code "));
//         Serial.println(radio_init_state);
//         while (true)
//         {
//             delay(10);
//         }
//     }

//     Serial.println(F("success!"));
//     uint8_t syncWord[] = {0x2D, 0x01, 0x4B, 0x1D};
//     radio.setSyncWord(syncWord, 4);
//     radio.setCRC(0);

//     // Configure TxFifoLow IRQ so we know when to refill the FIFO during 511-byte TX
//     // ConfigFifoIrq: opcode 0x01 0x1A
//     //   byte 2: rxfifoirqenable = 0x00 (no Rx FIFO IRQs needed)
//     //   byte 3: txfifoirqenable = 0x02 (enable TxFifoLow flag â†’ triggers TxFifo IRQ)
//     //   bytes 4-5: rxhighthreshold = 0x0000 (unused)
//     //   bytes 6-7: txlowthreshold  = FIFO_LOW_THRESH (fire when TX FIFO level < 128)
//     //   bytes 8-9: rxlowthreshold  = 0x0000 (unused)
//     //   bytes 10-11: txhighthreshold = 0x0000 (unused)
//     {
//         uint8_t configFifoCmd[] = {
//             0x01, 0x1A, // ConfigFifoIrq opcode
//             0x00,       // rxfifoirqenable: none
//             0x02,       // txfifoirqenable: FifoLow (bit 1)
//             0x00, 0x00, // rxhighthreshold (unused)
//             (uint8_t)((FIFO_LOW_THRESH >> 8) & 0xFF),
//             (uint8_t)(FIFO_LOW_THRESH & 0xFF), // txlowthreshold = 128
//             0x00, 0x00,                        // rxlowthreshold (unused)
//             0x00, 0x00                         // txhighthreshold (unused)
//         };
//         mySPI.beginTransaction(spiSettings);
//         digitalWrite(LR2021_CS, LOW);
//         mySPI.transferBytes(configFifoCmd, nullptr, sizeof(configFifoCmd));
//         digitalWrite(LR2021_CS, HIGH);
//         mySPI.endTransaction();
//     }

//     // Also map TxFifo IRQ to DIO9 alongside TxDone
//     // SetDioIrqConfig: opcode 0x01 0x15
//     // Dio = 9, Irq bits: TxDone = bit 19 (0x00080000), TxFifo = bit 17 (0x00020000)
//     // Combined mask = 0x000A0000
//     // DS Table 5-17: TxDone = bit 19, TxFifoIrq = bit 17 (verify in your datasheet Â§5.7)
//     {
//         uint32_t irqMask = (1UL << 19) | (1UL << 17); // TxDone | TxFifoIrq
//         uint8_t setDioIrqCmd[] = {
//             0x01, 0x15, // SetDioIrqConfig opcode
//             9,          // Dio = 9 (DIO9 = your IRQ pin)
//             (uint8_t)((irqMask >> 24) & 0xFF),
//             (uint8_t)((irqMask >> 16) & 0xFF),
//             (uint8_t)((irqMask >> 8) & 0xFF),
//             (uint8_t)(irqMask & 0xFF)};
//         mySPI.beginTransaction(spiSettings);
//         digitalWrite(LR2021_CS, LOW);
//         mySPI.transferBytes(setDioIrqCmd, nullptr, sizeof(setDioIrqCmd));
//         digitalWrite(LR2021_CS, HIGH);
//         mySPI.endTransaction();
//     }

// #ifdef IS_EAGLE
//     radio.setIrqAction(setFlag);

//     Serial.print(F("[EAGLE] Starting to listen ... "));
//     int state = radio.startReceive();
//     if (state == RADIOLIB_ERR_NONE)
//     {
//         Serial.println(F("success!"));
//         digitalWrite(LED_BLUE, HIGH);
//     }
//     else
//     {
//         Serial.print(F("failed, code "));
//         Serial.println(state);
//         while (true)
//         {
//             delay(10);
//         }
//     }
// #endif

// #ifdef IS_CAM
//     // fill both buffers with pattern
//     for (int i = 0; i < PAYLOAD_SIZE; i++)
//     {
//         payload[i] = (uint8_t)(i & 0xFF);
//     }

//     radio.setIrqAction(setRadioEventFlag);

//     Serial.println(F("[CAM] Starting benchmark in 10 seconds..."));
//     delay(10000);

//     radio.variablePacketLengthMode();
//     const char *callsign = "KE2CNQ";
//     radio.startTransmit((uint8_t *)callsign, strlen(callsign));
//     delay(500);
//     radio.fixedPacketLengthMode(PAYLOAD_SIZE);

//     Serial.print(F("[CAM] Sending "));
//     Serial.print(BENCHMARK_PACKET_COUNT);
//     Serial.print(F(" x "));
//     Serial.print(PAYLOAD_SIZE);
//     Serial.println(F(" bytes ..."));

//     payload[0] = 0;
//     payload[1] = 0;
//     // Phase 1: WriteRadioTxFifo â€” first 255 bytes
//     {
//         static uint8_t cmd1[FIFO_CHUNK_1 + 2];
//         cmd1[0] = 0x00;
//         cmd1[1] = 0x02;
//         memcpy(&cmd1[2], payload, FIFO_CHUNK_1);
//         while (digitalRead(LR2021_BUSY))
//         {
//         }
//         mySPI.beginTransaction(spiSettings);
//         digitalWrite(LR2021_CS, LOW);
//         mySPI.transferBytes(cmd1, nullptr, sizeof(cmd1));
//         digitalWrite(LR2021_CS, HIGH);
//         mySPI.endTransaction();
//     }
//     fifoRefillPtr = payload + FIFO_CHUNK_1;
//     fifoRefillLen = FIFO_CHUNK_2;
//     fifoRefillNeeded = true;

//     // Phase 2: SetTx
//     {
//         uint8_t setTxCmd[] = {0x02, 0x0D, 0x00, 0x00, 0x00, 0x00};
//         while (digitalRead(LR2021_BUSY))
//         {
//         }
//         mySPI.beginTransaction(spiSettings);
//         digitalWrite(LR2021_CS, LOW);
//         mySPI.transferBytes(setTxCmd, nullptr, sizeof(setTxCmd));
//         digitalWrite(LR2021_CS, HIGH);
//         mySPI.endTransaction();
//     }
//     packetsSent = 1;
//     benchStartMs = millis();
// #endif
// }

// void loop()
// {
// #ifdef IS_CAM
//     if (packetsSent < BENCHMARK_PACKET_COUNT)
//     {
//         if (radioEvent)
//         {
//             radioEvent = false;

//             // Read and clear IRQ status to distinguish TxDone vs TxFifoLow
//             // GetAndClearIrqStatus opcode: 0x01 0x17  (DS Table 6-48)
//             uint8_t irqCmd[6] = {0x01, 0x17, 0x00, 0x00, 0x00, 0x00};
//             uint8_t irqResp[6] = {0};
//             while (digitalRead(LR2021_BUSY))
//             {
//             } // wait BUSY low before SPI
//             mySPI.beginTransaction(spiSettings);
//             digitalWrite(LR2021_CS, LOW);
//             mySPI.transferBytes(irqCmd, irqResp, sizeof(irqCmd));
//             digitalWrite(LR2021_CS, HIGH);
//             mySPI.endTransaction();

//             // IRQ status is in bytes 2-5 of response (32-bit, MSB first)
//             uint32_t irqStatus = ((uint32_t)irqResp[2] << 24) | ((uint32_t)irqResp[3] << 16) | ((uint32_t)irqResp[4] << 8) | (uint32_t)irqResp[5];

//             // â”€â”€ TxFifoLow IRQ: FIFO has drained enough â€” write second chunk â”€â”€
//             // TxFifoIrq is bit 17 in LR2021 IRQ table (DS Â§5.7)
//             if ((irqStatus & (1UL << 17)) && fifoRefillNeeded)
//             {
//                 fifoRefillNeeded = false;
//                 // WriteRadioTxFifo opcode: 0x00 0x02  (DS Table 6-2)
//                 // Build command: [0x00][0x02][data...]
//                 static uint8_t fifoCmd[FIFO_CHUNK_2 + 2];
//                 fifoCmd[0] = 0x00;
//                 fifoCmd[1] = 0x02;
//                 memcpy(&fifoCmd[2], fifoRefillPtr, fifoRefillLen);
//                 while (digitalRead(LR2021_BUSY))
//                 {
//                 }
//                 mySPI.beginTransaction(spiSettings);
//                 digitalWrite(LR2021_CS, LOW);
//                 mySPI.transferBytes(fifoCmd, nullptr, sizeof(fifoCmd));
//                 digitalWrite(LR2021_CS, HIGH);
//                 mySPI.endTransaction();
//                 // Radio continues transmitting â€” TxDone IRQ will fire when packet is complete
//             }

//             // â”€â”€ TxDone IRQ: packet fully transmitted â”€â”€
//             // TxDone is bit 19 in LR2021 IRQ table (DS Â§5.7 / Table 5-17)
//             if (irqStatus & (1UL << 19))
//             {
//                 packetsOk++;

//                 // Prepare next packet seq number
//                 payload[0] = (packetsSent >> 8) & 0xFF;
//                 payload[1] = packetsSent & 0xFF;

//                 // â”€â”€ Transmit next packet â€” two-phase FIFO write â”€â”€
//                 // Phase 1: WriteRadioTxFifo with first 255 bytes
//                 static uint8_t fifoCmd1[FIFO_CHUNK_1 + 2];
//                 fifoCmd1[0] = 0x00;
//                 fifoCmd1[1] = 0x02; // WriteRadioTxFifo opcode
//                 memcpy(&fifoCmd1[2], payload, FIFO_CHUNK_1);

//                 while (digitalRead(LR2021_BUSY))
//                 {
//                 }
//                 mySPI.beginTransaction(spiSettings);
//                 digitalWrite(LR2021_CS, LOW);
//                 mySPI.transferBytes(fifoCmd1, nullptr, sizeof(fifoCmd1));
//                 digitalWrite(LR2021_CS, HIGH);
//                 mySPI.endTransaction();

//                 // Arm the refill â€” ISR will set radioEvent, loop() writes chunk 2
//                 fifoRefillPtr = payload + FIFO_CHUNK_1;
//                 fifoRefillLen = FIFO_CHUNK_2;
//                 fifoRefillNeeded = true;

//                 // Phase 2: SetTx â€” radio starts transmitting immediately from FIFO
//                 // SetTx opcode: 0x02 0x0D, timeout = 0 (no timeout)  (DS Table 6-12)
//                 uint8_t setTxCmd[] = {0x02, 0x0D, 0x00, 0x00, 0x00, 0x00};
//                 while (digitalRead(LR2021_BUSY))
//                 {
//                 }
//                 mySPI.beginTransaction(spiSettings);
//                 digitalWrite(LR2021_CS, LOW);
//                 mySPI.transferBytes(setTxCmd, nullptr, sizeof(setTxCmd));
//                 digitalWrite(LR2021_CS, HIGH);
//                 mySPI.endTransaction();

//                 packetsSent++;
//             }
//         }

//         // Timeout watchdog â€” same logic as before
//         static unsigned long lastEventMs = 0;
//         if (packetsSent > 0 && millis() - lastEventMs > 500)
//         {
//             packetsFailed++;
//             // Force a restart â€” just re-invoke the first packet of next sequence
//             lastEventMs = millis();
//         }
//     }
//     else
//     {
//         // wait for the very last packet's TX done before stopping the clock
//         unsigned long txStart = millis();
//         while (!txDone && millis() - txStart < 500)
//         {
//         }
//         if (txDone)
//             packetsOk++;
//         txDone = false;

//         // benchmark complete
//         unsigned long elapsed = millis() - benchStartMs;
//         long totalBytes = (long)packetsOk * PAYLOAD_SIZE;
//         float elapsedSec = elapsed / 1000.0f;
//         float throughput = (totalBytes * 8.0f) / elapsedSec / 1000.0f;

//         digitalWrite(LED_GREEN, LOW);
//         digitalWrite(LED_RED, LOW);
//         digitalWrite(LED_ORANGE, HIGH);

//         Serial.println(F("\n========= CAM BENCHMARK RESULTS ========="));
//         Serial.print(F("  Packets sent:     "));
//         Serial.println(packetsSent);
//         Serial.print(F("  Packets OK:       "));
//         Serial.println(packetsOk);
//         Serial.print(F("  Packets failed:   "));
//         Serial.println(packetsFailed);
//         Serial.print(F("  Payload/packet:   "));
//         Serial.print(PAYLOAD_SIZE);
//         Serial.println(F(" bytes"));
//         Serial.print(F("  Total TX bytes:   "));
//         Serial.println(totalBytes);
//         Serial.print(F("  Elapsed time:     "));
//         Serial.print(elapsedSec, 3);
//         Serial.println(F(" s"));
//         Serial.print(F("  Throughput:       "));
//         Serial.print(throughput, 2);
//         Serial.println(F(" kbps"));
//         Serial.println(F("=========================================\n"));

//         radio.variablePacketLengthMode();
//         const char *callsign = "KE2CNQ";
//         radio.startTransmit((uint8_t *)callsign, strlen(callsign));
//         delay(500);
//         radio.fixedPacketLengthMode(PAYLOAD_SIZE);

//         while (true)
//         {
//             delay(100);
//         }
//     }

// #elifdef IS_EAGLE

//     if (benchmarkStarted && !benchmarkDone)
//     {
//         if (millis() - lastRxMs > 500)
//         {
//             // CAM is done transmitting but we never got all packets â€” force terminate
//             // count the gap as dropped packets
//             int totalAccountedFor = packetsReceived + rxErrors;
//             if (totalAccountedFor < packetsExpected)
//             {
//                 int dropped = packetsExpected - totalAccountedFor;
//                 rxErrors += dropped; // count unaccounted packets as errors
//             }
//             // fall through to termination check below
//         }
//     }

//     if (benchmarkDone)
//         return;

//     if (operationDone)
//     {
//         operationDone = false;

//         uint8_t buf[PAYLOAD_SIZE];
//         int state = radio.readData(buf, PAYLOAD_SIZE);

//         if (state == RADIOLIB_ERR_NONE)
//         {
//             unsigned long now = millis();

//             if (!benchmarkStarted)
//             {
//                 benchmarkStarted = true;
//                 firstRxMs = now;
//                 lastRxMs = now;
//                 digitalWrite(LED_ORANGE, HIGH);
//             }

//             int seqNum = ((int)buf[0] << 8) | buf[1];

//             // buffer only â€” NO serial prints here
//             if (packetsReceived < BENCHMARK_PACKET_COUNT)
//             {
//                 pktLog[packetsReceived].seqNum = seqNum;
//                 pktLog[packetsReceived].rssi = radio.getRSSI();
//                 pktLog[packetsReceived].snr = radio.getSNR();
//                 pktLog[packetsReceived].rxMs = now - firstRxMs;
//             }

//             packetsReceived++;
//             totalBytesRx += PAYLOAD_SIZE;
//             lastRxMs = now;

//             digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
//         }
//         else
//         {
//             rxErrors++;
//             if (benchmarkStarted)
//                 lastRxMs = millis();
//             digitalWrite(LED_RED, HIGH);
//             delay(10);
//             digitalWrite(LED_RED, LOW);
//         }

//         radio.startReceive();
//     }

//     if (benchmarkStarted && (packetsReceived + rxErrors >= packetsExpected))
//     {
//         benchmarkDone = true;

//         unsigned long elapsed = lastRxMs - firstRxMs;
//         float elapsedSec = elapsed / 1000.0f;
//         float throughput = (totalBytesRx * 8.0f) / elapsedSec / 1000.0f;
//         float packetLoss = 100.0f * (packetsExpected - packetsReceived) / (float)packetsExpected;

//         digitalWrite(LED_GREEN, LOW);
//         digitalWrite(LED_BLUE, HIGH);

//         Serial.println(F("\n--- Per-packet log ---"));
//         Serial.println(F("  SEQ\tRX ms\tRSSI\tSNR"));
//         for (int i = 0; i < packetsReceived; i++)
//         {
//             Serial.print(F("  "));
//             Serial.print(pktLog[i].seqNum);
//             Serial.print(F("\t"));
//             Serial.print(pktLog[i].rxMs);
//             Serial.print(F("\t"));
//             Serial.print(pktLog[i].rssi, 1);
//             Serial.print(F("\t"));
//             Serial.println(pktLog[i].snr, 1);
//         }

//         Serial.println(F("\n========= EAGLE BENCHMARK RESULTS ========="));
//         Serial.print(F("  Packets expected: "));
//         Serial.println(packetsExpected);
//         Serial.print(F("  Packets received: "));
//         Serial.println(packetsReceived);
//         Serial.print(F("  RX errors:        "));
//         Serial.println(rxErrors);
//         Serial.print(F("  Packet loss:      "));
//         Serial.print(packetLoss, 1);
//         Serial.println(F(" %"));
//         Serial.print(F("  Total RX bytes:   "));
//         Serial.println(totalBytesRx);
//         Serial.print(F("  Elapsed time:     "));
//         Serial.print(elapsedSec, 3);
//         Serial.println(F(" s"));
//         Serial.print(F("  Throughput:       "));
//         Serial.print(throughput, 2);
//         Serial.println(F(" kbps"));
//         Serial.println(F("==========================================\n"));
//     }
// #endif
// }
