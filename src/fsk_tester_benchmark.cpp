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
uint8_t payload[PAYLOAD_SIZE_FSK];
int packetsSent = 0;
int packetsOk = 0;
int packetsFailed = 0;
unsigned long benchStartMs = 0;
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

struct PktLog
{
    int seqNum;
    float rssiAvg;
    float rssiSync;
    float lqi;
    int length;
    unsigned long rxUs;
};
PktLog pktLog[BENCHMARK_PACKET_COUNT];
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
    delay(500);
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

#ifdef IS_CAM
    for (int i = 0; i < PAYLOAD_SIZE_FSK; i++)
        payload[i] = (uint8_t)(i & 0xFF);

    Serial.println(F("[CAM] Starting benchmark in 2 seconds..."));
    delay(2000);

    Serial.print(F("[CAM] Sending "));
    Serial.print(BENCHMARK_PACKET_COUNT);
    Serial.print(F(" x "));
    Serial.print(PAYLOAD_SIZE_FSK);
    Serial.println(F(" bytes ..."));

    payload[0] = 0;
    payload[1] = 0;

    LR2021Error txResult = driver.transmit(payload, PAYLOAD_SIZE_FSK);
    if (!txResult.ok())
    {
        Serial.print(F("[CAM] First TX failed: "));
        Serial.println(txResult.stageStr());
    }
    else
    {
        packetsOk++;
    }
    packetsSent = 1;
    benchStartMs = millis();
#endif

#ifdef IS_EAGLE
    Serial.println(F("[EAGLE] Listening for packets..."));
    digitalWrite(LED_BLUE, HIGH);
#endif
}

void loop()
{
#ifdef IS_CAM
    if (packetsSent < BENCHMARK_PACKET_COUNT)
    {
        payload[0] = (packetsSent >> 8) & 0xFF;
        payload[1] = packetsSent & 0xFF;

        LR2021Error txResult = driver.transmit(payload, PAYLOAD_SIZE_FSK);
        if (!txResult.ok())
        {
            packetsFailed++;
            Serial.print(F("[CAM] TX failed: "));
            Serial.println(txResult.stageStr());
        }
        else
        {
            packetsOk++;
        }
        packetsSent++;
    }
    else
    {
        unsigned long elapsed = millis() - benchStartMs;
        long totalBytes = (long)packetsOk * PAYLOAD_SIZE_FSK;
        float elapsedSec = elapsed / 1000.0f;
        float throughput = (totalBytes * 8.0f) / elapsedSec / 1000.0f;

        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_ORANGE, HIGH);

        Serial.println(F("\n========= CAM BENCHMARK RESULTS ========="));
        Serial.print(F("  Packets sent:    "));
        Serial.println(packetsSent);
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

        while (true)
        {
            delay(100);
        }
    }

#elifdef IS_EAGLE
    if (benchmarkDone)
        return;

    static uint8_t buf[PAYLOAD_SIZE_FSK];
    static LR2021FskPktStatus pktStatus;
    LR2021Error rxResult = driver.receive(buf, PAYLOAD_SIZE_FSK, &pktStatus);

    unsigned long now = micros();

    if (rxResult.driverCode == LR2021_ERR_RX_TIMEOUT)
    {
        // check for stall: if benchmark started and no packet for 500ms, close it out
        if (benchmarkStarted && (now - lastRxUs > 500UL * 1000UL))
        {
            int totalAccountedFor = packetsReceived + rxErrors;
            if (totalAccountedFor < packetsExpected)
                rxErrors += packetsExpected - totalAccountedFor;

            // force completion
            goto finish;
        }
        return;
    }

    if (rxResult.driverCode == LR2021_ERR_CRC_MISMATCH)
    {
        rxErrors++;
        if (benchmarkStarted)
            lastRxUs = now;
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

    // good packet
    {
        if (!benchmarkStarted)
        {
            benchmarkStarted = true;
            firstRxUs = now;
            lastRxUs = now;
            digitalWrite(LED_ORANGE, HIGH);
        }

        int seqNum = ((int)buf[0] << 8) | buf[1];

        if (packetsReceived < BENCHMARK_PACKET_COUNT)
        {
            pktLog[packetsReceived].seqNum = seqNum;
            pktLog[packetsReceived].length = pktStatus.pktlen;
            pktLog[packetsReceived].rssiAvg = pktStatus.rssiAvg;
            pktLog[packetsReceived].rssiSync = pktStatus.rssiSync;
            pktLog[packetsReceived].lqi = pktStatus.lqi;
            pktLog[packetsReceived].rxUs = now - firstRxUs;
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
    Serial.println(F("  SEQ\tRX_us\tRSSI_avg\tRSSI_sync\tLQI\tLength"));

    for (int i = 0; i < packetsReceived && i < BENCHMARK_PACKET_COUNT; i++)
    {
        char lineBuf[128];
        int llen = snprintf(lineBuf, sizeof(lineBuf),
                            "  %d\t%lu\t%.1f\t\t%.1f\t\t%.2f\t%d\n",
                            pktLog[i].seqNum,
                            pktLog[i].rxUs,
                            pktLog[i].rssiAvg,
                            pktLog[i].rssiSync,
                            pktLog[i].lqi,
                            pktLog[i].length);
        Serial.write(lineBuf, llen);
        Serial.flush();
    }
#endif
}