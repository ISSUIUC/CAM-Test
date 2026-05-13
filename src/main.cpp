#include <Arduino.h>
#include "pins.h"
#include <RadioLib.h>

#include "USB.h"
#include "USBCDC.h"

#include "flrc.h"

USBCDC USBSerial;
#undef Serial
#define Serial USBSerial

LR2021Driver driver;

#define BENCHMARK_PACKET_COUNT 200

#ifdef IS_CAM
uint8_t payload[PAYLOAD_SIZE];
int packetsSent = 0;
int packetsOk = 0;
int packetsFailed = 0;
unsigned long benchStartMs = 0;
#endif

#ifdef IS_EAGLE
int packetsReceived = 0;
int packetsExpected = BENCHMARK_PACKET_COUNT;
long totalBytesRx = 0;
unsigned long firstRxMs = 0;
unsigned long lastRxMs = 0;
bool benchmarkStarted = false;
bool benchmarkDone = false;
int rxErrors = 0;

struct PktLog
{
    int seqNum;
    float rssi;
    int length;
    unsigned long rxMs;
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
    };
    delay(50);

#ifdef IS_CAM
    Serial.println(F("Mode: CAM (Transmitter - Benchmark)"));
#elifdef IS_EAGLE
    Serial.println(F("Mode: EAGLE (Receiver - Benchmark)"));
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
    for (int i = 0; i < PAYLOAD_SIZE; i++)
        payload[i] = (uint8_t)(i & 0xFF);

    Serial.println(F("[CAM] Starting benchmark in 2 seconds..."));
    delay(2000);

    driver.transmitCallSign();

    Serial.print(F("[CAM] Sending "));
    Serial.print(BENCHMARK_PACKET_COUNT);
    Serial.print(F(" x "));
    Serial.print(PAYLOAD_SIZE);
    Serial.println(F(" bytes ..."));

    payload[0] = 0;
    payload[1] = 0;

    LR2021Error txResult = driver.transmit(payload);
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

        LR2021Error txResult = driver.transmit(payload);
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
        long totalBytes = (long)packetsOk * PAYLOAD_SIZE;
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
        Serial.print(PAYLOAD_SIZE);
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

        driver.transmitCallSign();

        while (true)
        {
            delay(100);
        }
    }

#elifdef IS_EAGLE
    if (benchmarkDone)
        return;

    if (benchmarkStarted && !benchmarkDone)
    {
        if (millis() - lastRxMs > 500)
        {
            int totalAccountedFor = packetsReceived + rxErrors;
            if (totalAccountedFor < packetsExpected)
                rxErrors += packetsExpected - totalAccountedFor;
        }
    }

    static uint8_t buf[PAYLOAD_SIZE];
    LR2021FlrcPktStatus pktStatus;
    LR2021Error rxResult = driver.receive(buf, PAYLOAD_SIZE, &pktStatus);

    if (rxResult.driverCode == LR2021_ERR_CRC_MISMATCH)
    {
        rxErrors++;
        if (benchmarkStarted)
            lastRxMs = millis();
        digitalWrite(LED_RED, HIGH);
        delay(10);
        digitalWrite(LED_RED, LOW);
    }
    else if (rxResult.driverCode == LR2021_ERR_PKT_LEN_FAILED)
    {
        rxErrors++;
        if (benchmarkStarted)
            lastRxMs = millis();

        Serial.print("Packet length failure :(:\t");
        Serial.println(pktStatus.packet_length_bytes);
    }
    else if (rxResult.ok())
    {
        unsigned long now = millis();

        if (!benchmarkStarted)
        {
            benchmarkStarted = true;
            firstRxMs = now;
            lastRxMs = now;
            digitalWrite(LED_ORANGE, HIGH);
        }

        int seqNum = ((int)buf[0] << 8) | buf[1];

        if (packetsReceived < BENCHMARK_PACKET_COUNT)
        {
            pktLog[packetsReceived].seqNum = seqNum;
            pktLog[packetsReceived].length = pktStatus.packet_length_bytes;
            pktLog[packetsReceived].rssi = pktStatus.rssi_avg_in_dbm - (pktStatus.rssi_avg_half_dbm ? 0.5f : 0.0f);
            pktLog[packetsReceived].rxMs = now - firstRxMs;
        }

        packetsReceived++;
        totalBytesRx += PAYLOAD_SIZE;
        lastRxMs = now;

        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    }

    if (benchmarkStarted && (packetsReceived + rxErrors >= packetsExpected))
    {
        benchmarkDone = true;

        unsigned long elapsed = lastRxMs - firstRxMs;
        float elapsedSec = elapsed / 1000.0f;
        float throughput = (totalBytesRx * 8.0f) / elapsedSec / 1000.0f;
        float packetLoss = 100.0f * (packetsExpected - packetsReceived) / (float)packetsExpected;

        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, HIGH);

        Serial.println(F("\n--- Per-packet log ---"));
        Serial.println(F("  SEQ\tRX ms\tRSSI\tLength"));
        for (int i = 0; i < packetsReceived; i++)
        {
            Serial.print(F("  "));
            Serial.print(pktLog[i].seqNum);
            Serial.print(F("\t"));
            Serial.print(pktLog[i].rxMs);
            Serial.print(F("\t"));
            Serial.print(pktLog[i].rssi, 1);
            Serial.print(F("\t"));
            Serial.println(pktLog[i].length);
        }

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
    }
#endif
}