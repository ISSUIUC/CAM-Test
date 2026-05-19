#include <Arduino.h>
#include "pins.h"
#include <string.h>

#include "USB.h"
#include "USBCDC.h"

#include "radio.h"

USBCDC USBSerial;
#undef Serial
#define Serial USBSerial

LR2021FSKDriver driver;

#define BENCHMARK_PACKET_COUNT 200

#ifdef IS_CAM
uint8_t packetPool[BENCHMARK_PACKET_COUNT][PAYLOAD_SIZE_FSK];
uint8_t *packetPtrs[BENCHMARK_PACKET_COUNT];
int packetsOk = 0;
int packetsFailed = 0;
unsigned long benchStartus = 0;
#endif

#ifdef IS_EAGLE
int packetsReceived = 0;
int packetsExpected = BENCHMARK_PACKET_COUNT;
long totalBytesRx = 0;
unsigned long firstRxUs = 0;
unsigned long lastRxUs = 0;
bool benchmarkStarted = false;
bool benchmarkDone = false;
int rxErrors = 0;

// Total log slots = good packets + possible CRC errors, cap generously
#define PKT_LOG_MAX (BENCHMARK_PACKET_COUNT * 2)

struct PktLog
{
    int seqNum; // -1 for CRC error entries
    float rssiAvg;
    float rssiSync;
    float lqi;
    int length;
    unsigned long rxUs;
    bool crcError; // true if this entry is a CRC mismatch, not a good packet
};
PktLog pktLog[PKT_LOG_MAX];
int pktLogCount = 0; // tracks total entries (good + CRC errors)
#endif

void setup()
{
    setCpuFrequencyMhz(240);

    USB.begin();
    Serial.begin(115200);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);

    while (!Serial)
    {
        delay(100);
    }

#ifdef IS_CAM
    delay(500);
    Serial.println(F("Mode: CAM (Transmitter - Benchmark FSK)"));
#elifdef IS_EAGLE
    while (!Serial)
    {
        delay(100);
    }
    Serial.println(F("Mode: EAGLE (Receiver - Benchmark FSK)"));
#endif

    LR2021Error result = driver.init();
    if (!result.ok())
    {
        Serial.print(F("[LR2021] Init failed at stage: "));
        Serial.println(result.stageStr());
        Serial.print(F("  RadioLib code: "));
        Serial.println(result.radioLibCode);
        while (true)
        {
            delay(10);
        }
    }
    Serial.println(F("[LR2021] Init OK"));

#ifdef IS_EAGLE
    Serial.println(F("[EAGLE] Listening for packets..."));
    digitalWrite(LED_BLUE, HIGH);
#endif

#ifdef IS_CAM
    // Build the packet pool — stamp sequence numbers and fill payload
    for (int i = 0; i < BENCHMARK_PACKET_COUNT; i++)
    {
        packetPool[i][0] = (i >> 8) & 0xFF; // seq MSB
        packetPool[i][1] = i & 0xFF;        // seq LSB
        for (int j = 2; j < PAYLOAD_SIZE_FSK; j++)
            packetPool[i][j] = (uint8_t)(j & 0xFF);
        packetPtrs[i] = packetPool[i];
    }

    Serial.println(F("[CAM] Starting benchmark in 2 seconds..."));
    delay(2000);

    Serial.print(F("[CAM] Sending "));
    Serial.print(BENCHMARK_PACKET_COUNT);
    Serial.print(F(" x "));
    Serial.print(PAYLOAD_SIZE_FSK);
    Serial.println(F(" bytes ..."));

    benchStartus = micros();
    LR2021Error txResult = driver.transmitBurst(packetPtrs, BENCHMARK_PACKET_COUNT, PAYLOAD_SIZE_FSK);

    if (!txResult.ok())
    {
        packetsFailed = BENCHMARK_PACKET_COUNT;
        Serial.print(F("[CAM] Burst TX failed: "));
        Serial.println(txResult.stageStr());
    }
    else
    {
        packetsOk = BENCHMARK_PACKET_COUNT;
    }

    // Print results immediately in setup — loop() does nothing for CAM now
    unsigned long elapsed = micros() - benchStartus;
    long totalBytes = (long)packetsOk * PAYLOAD_SIZE_FSK;
    float elapsedSec = elapsed / (1000.0f * 1000.0f);
    float throughput = (totalBytes * 8.0f) / elapsedSec / 1000.0f;

    digitalWrite(LED_ORANGE, HIGH);

    Serial.println(F("\n========= CAM BENCHMARK RESULTS ========="));
    Serial.print(F("  Packets sent:    "));
    Serial.println(BENCHMARK_PACKET_COUNT);
    Serial.print(F("  Packets OK:      "));
    Serial.println(packetsOk);
    Serial.print(F("  Packets failed:  "));
    Serial.println(packetsFailed);
    Serial.print(F("  Payload/packet:  "));
    Serial.print(PAYLOAD_SIZE_FSK);
    Serial.println(F(" bytes"));
    Serial.print(F("  Total TX bytes:  "));
    Serial.println(totalBytes);
    Serial.print(F("  Elapsed time:    "));
    Serial.print(elapsedSec, 3);
    Serial.println(F(" s"));
    Serial.print(F("  Throughput:      "));
    Serial.print(throughput, 2);
    Serial.println(F(" kbps"));
    Serial.println(F("=========================================\n"));
#endif
}

void loop()
{
#ifdef IS_CAM
    delay(100);
#endif

#ifdef IS_EAGLE
    if (benchmarkDone)
        return;

    static uint8_t buf[PAYLOAD_SIZE_FSK];
    static LR2021FskPktStatus pktStatus;
    LR2021Error rxResult = driver.receive(buf, PAYLOAD_SIZE_FSK, &pktStatus);

    unsigned long now = micros();

    if (rxResult.driverCode == LR2021_ERR_RX_TIMEOUT)
    {
        if (benchmarkStarted && (now - lastRxUs > 500UL * 1000UL))
        {
            int totalAccountedFor = packetsReceived + rxErrors;
            if (totalAccountedFor < packetsExpected)
                rxErrors += packetsExpected - totalAccountedFor;

            goto finish;
        }
        return;
    }

    if (rxResult.driverCode == LR2021_ERR_CRC_MISMATCH)
    {
        rxErrors++;
        if (benchmarkStarted)
            lastRxUs = now;

        // Log the CRC error entry
        if (pktLogCount < PKT_LOG_MAX)
        {
            pktLog[pktLogCount].seqNum = -1;
            pktLog[pktLogCount].rssiAvg = pktStatus.rssiAvg;
            pktLog[pktLogCount].rssiSync = pktStatus.rssiSync;
            pktLog[pktLogCount].lqi = pktStatus.lqi;
            pktLog[pktLogCount].length = pktStatus.pktlen;
            pktLog[pktLogCount].rxUs = benchmarkStarted ? (now - firstRxUs) : 0;
            pktLog[pktLogCount].crcError = true;
            pktLogCount++;
        }

        // Immediate print
        char crcBuf[96];
        int clen = snprintf(crcBuf, sizeof(crcBuf),
                            "[EAGLE] CRC error @ +%lu us  RSSI_avg=%.1f  RSSI_sync=%.1f  LQI=%.2f  len=%d\n",
                            benchmarkStarted ? (now - firstRxUs) : 0UL,
                            pktStatus.rssiAvg, pktStatus.rssiSync,
                            pktStatus.lqi, pktStatus.pktlen);
        if (clen > 0)
        {
            Serial.write(crcBuf, (size_t)clen);
            Serial.flush();
        }

        digitalWrite(LED_RED, HIGH);
        delay(10);
        digitalWrite(LED_RED, LOW);
        goto check_done;
    }

    if (!rxResult.ok())
    {
        rxErrors++;
        if (benchmarkStarted)
            lastRxUs = now;

        char errBuf[64];
        int elen = snprintf(errBuf, sizeof(errBuf), "[EAGLE] RX error: %s\n", rxResult.stageStr());
        if (elen > 0)
        {
            Serial.write(errBuf, (size_t)elen);
            Serial.flush();
        }

        goto check_done;
    }

    // Good packet
    {
        if (!benchmarkStarted)
        {
            benchmarkStarted = true;
            firstRxUs = now;
            lastRxUs = now;
            digitalWrite(LED_ORANGE, HIGH);
        }

        int seqNum = ((int)buf[0] << 8) | buf[1];

        if (pktLogCount < PKT_LOG_MAX)
        {
            pktLog[pktLogCount].seqNum = seqNum;
            pktLog[pktLogCount].length = pktStatus.pktlen;
            pktLog[pktLogCount].rssiAvg = pktStatus.rssiAvg;
            pktLog[pktLogCount].rssiSync = pktStatus.rssiSync;
            pktLog[pktLogCount].lqi = pktStatus.lqi;
            pktLog[pktLogCount].rxUs = now - firstRxUs;
            pktLog[pktLogCount].crcError = false;
            pktLogCount++;
        }

        packetsReceived++;
        totalBytesRx += PAYLOAD_SIZE_FSK;
        lastRxUs = now;

        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    }

check_done:
    if (!benchmarkStarted || (packetsReceived + rxErrors < packetsExpected))
        return;

finish:
    benchmarkDone = true;

    unsigned long elapsed = lastRxUs - firstRxUs;
    float elapsedSec = elapsed / (1000.0f * 1000.0f);
    float throughput = (totalBytesRx * 8.0f) / elapsedSec / 1000.0f;
    float packetLoss = 100.0f * (packetsExpected - packetsReceived) / (float)packetsExpected;

    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);

    Serial.println(F("\n========= EAGLE BENCHMARK RESULTS ========="));
    Serial.print(F("  Packets expected: "));
    Serial.println(packetsExpected);
    Serial.print(F("  Packets received: "));
    Serial.println(packetsReceived);
    Serial.print(F("  RX errors:        "));
    Serial.println(rxErrors);
    Serial.print(F("  Packet loss:      "));
    Serial.print(packetLoss, 1);
    Serial.println(F(" %"));
    Serial.print(F("  Total RX bytes:   "));
    Serial.println(totalBytesRx);
    Serial.print(F("  Elapsed time:     "));
    Serial.print(elapsedSec, 3);
    Serial.println(F(" s"));
    Serial.print(F("  Throughput:       "));
    Serial.print(throughput, 2);
    Serial.println(F(" kbps"));
    Serial.println(F("==========================================\n"));

    Serial.println(F("\n--- Per-packet log ---"));
    Serial.println(F("  SEQ\t\tRX_us\t\tRSSI_avg\tRSSI_sync\tLQI\tLength"));

    for (int i = 0; i < pktLogCount; i++)
    {
        char lineBuf[128];
        int llen;
        if (pktLog[i].crcError)
        {
            llen = snprintf(lineBuf, sizeof(lineBuf),
                            "  [CRC]\t\t%lu\t\t%.1f\t\t%.1f\t\t%.2f\t%d\n",
                            pktLog[i].rxUs,
                            pktLog[i].rssiAvg,
                            pktLog[i].rssiSync,
                            pktLog[i].lqi,
                            pktLog[i].length);
        }
        else
        {
            llen = snprintf(lineBuf, sizeof(lineBuf),
                            "  %d\t\t%lu\t\t%.1f\t\t%.1f\t\t%.2f\t%d\n",
                            pktLog[i].seqNum,
                            pktLog[i].rxUs,
                            pktLog[i].rssiAvg,
                            pktLog[i].rssiSync,
                            pktLog[i].lqi,
                            pktLog[i].length);
        }
        Serial.write(lineBuf, llen);
        Serial.flush();
    }
#endif
}