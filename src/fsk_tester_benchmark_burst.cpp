#include <Arduino.h>
#include "pins.h"
#include <SPI.h>

#include "USB.h"
#include "USBCDC.h"

#include "fsk.h"

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

#define PKT_LOG_MAX (BENCHMARK_PACKET_COUNT * 2)

struct PktLog
{
    int seqNum; // -1 for CRC error entries
    float rssiAvg;
    float rssiSync;
    float lqi;
    int length;
    unsigned long rxUs; // microseconds after firstRxUs
    bool crcError;
};

struct EagleRxCtx
{
    PktLog log[PKT_LOG_MAX];
    int logCount = 0;
    int packetsRx = 0;
    int rxErrors = 0;
    long totalBytesRx = 0;
    unsigned long firstRxUs = 0;
    unsigned long lastRxUs = 0;
    bool firstPacket = true;
};

static void onRxPacket(const uint8_t *buf,
                       uint8_t len,
                       const LR2021FskPktStatus &status,
                       bool crcError,
                       void *userData)
{
    EagleRxCtx *ctx = static_cast<EagleRxCtx *>(userData);
    unsigned long now = micros();

    if (ctx->firstPacket)
    {
        ctx->firstRxUs = now;
        ctx->lastRxUs = now;
        ctx->firstPacket = false;
        digitalWrite(LED_ORANGE, HIGH);
    }
    ctx->lastRxUs = now;

    if (ctx->logCount < PKT_LOG_MAX)
    {
        PktLog &entry = ctx->log[ctx->logCount++];
        entry.rssiAvg = status.rssiAvg;
        entry.rssiSync = status.rssiSync;
        entry.lqi = status.lqi;
        entry.length = status.pktlen;
        entry.rxUs = now - ctx->firstRxUs;
        entry.crcError = crcError;

        if (!crcError && buf != nullptr)
            entry.seqNum = ((int)buf[0] << 8) | buf[1];
        else
            entry.seqNum = -1;
    }

    if (!crcError && buf != nullptr)
    {
        ctx->packetsRx++;
        ctx->totalBytesRx += len;
        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    }
    else
    {
        ctx->rxErrors++;
    }
}

static EagleRxCtx ctx;

#endif

void setup()
{
    setCpuFrequencyMhz(240);

    USB.begin();
    Serial.begin(115200);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);

    while (!Serial)
    {
        delay(100);
    }

#ifdef IS_CAM
    Serial.println(F("Mode: CAM (Transmitter - Benchmark FSK)"));
#elifdef IS_EAGLE
    Serial.println(F("Mode: EAGLE (Receiver - Benchmark FSK)"));
#endif

    LR2021Error result = driver.init(SPI);
    if (!result.ok())
    {
        Serial.print(F("[LR2021] Init failed at stage: "));
        Serial.println(result.stageStr());
        Serial.print(F(" RadioLib code: "));
        Serial.println(result.radioLibCode);
        while (true)
        {
            delay(10);
        }
    }
    Serial.println(F("[LR2021] Init OK"));

#ifdef IS_EAGLE

    // Serial.println(F("[EAGLE] Waiting for first packet..."));
    digitalWrite(LED_BLUE, HIGH);

    // // Wait for the first packet with a long timeout
    // LR2021Error firstPktResult = driver.receiveOpen(
    //     PAYLOAD_SIZE_FSK,
    //     onRxPacket,
    //     &ctx,
    //     10000UL * 1000UL // 10 second timeout to wait for first packet
    // );

    // if (!ctx.packetsRx && !ctx.rxErrors)
    // {
    //     Serial.println(F("[EAGLE] No packet received within timeout. Aborting."));
    //     while (true)
    //     {
    //         delay(10);
    //     }
    // }

    // Serial.println(F("[EAGLE] First packet received! Starting benchmark..."));

    // // Reset context for the actual benchmark
    // ctx.logCount = 0;
    // ctx.packetsRx = 0;
    // ctx.rxErrors = 0;
    // ctx.totalBytesRx = 0;
    // ctx.firstPacket = true;

    unsigned long t0 = micros();

    // Now start the actual benchmark receive
    LR2021Error rxResult = driver.receiveOpen(
        PAYLOAD_SIZE_FSK,
        onRxPacket,
        &ctx,
        500UL * 1000UL // 500 ms of silence = burst is done
    );

    unsigned long elapsed = micros() - t0;

    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);

    float elapsedSec = elapsed / (1000.0f * 1000.0f);
    float throughput = (ctx.totalBytesRx * 8.0f) / elapsedSec / 1000.0f;
    float packetLoss = (ctx.packetsRx + ctx.rxErrors) > 0
                           ? 100.0f * ctx.rxErrors / (float)(ctx.packetsRx + ctx.rxErrors)
                           : 0.0f;

    if (rxResult.driverCode == LR2021_ERR_RX_TIMEOUT)
        Serial.println(F("[EAGLE] WARNING: cold timeout — nothing received"));

    Serial.println(F("\n========= EAGLE BENCHMARK RESULTS ========="));
    Serial.print(F(" Packets received:  "));
    Serial.println(ctx.packetsRx);
    Serial.print(F(" RX errors (CRC):   "));
    Serial.println(ctx.rxErrors);
    Serial.print(F(" Packet loss:       "));
    Serial.print(packetLoss, 1);
    Serial.println(F(" %"));
    Serial.print(F(" Total RX bytes:    "));
    Serial.println(ctx.totalBytesRx);
    Serial.print(F(" Elapsed time:      "));
    Serial.print(elapsedSec, 3);
    Serial.println(F(" s"));
    Serial.print(F(" Throughput:        "));
    Serial.print(throughput, 2);
    Serial.println(F(" kbps"));
    Serial.println(F("==========================================\n"));

    Serial.println(F("\n--- Per-packet log ---"));
    Serial.println(F(" SEQ\t\tRX_us\t\tRSSI_avg\tRSSI_sync\tLQI\tLength"));

    for (int i = 0; i < ctx.logCount; i++)
    {
        char buf[128];
        int n;
        if (ctx.log[i].crcError)
            n = snprintf(buf, sizeof(buf),
                         " [CRC]\t\t%lu\t\t%.1f\t\t%.1f\t\t%.2f\t%d\n",
                         ctx.log[i].rxUs,
                         ctx.log[i].rssiAvg, ctx.log[i].rssiSync,
                         ctx.log[i].lqi, ctx.log[i].length);
        else
            n = snprintf(buf, sizeof(buf),
                         " %d\t\t%lu\t\t%.1f\t\t%.1f\t\t%.2f\t%d\n",
                         ctx.log[i].seqNum, ctx.log[i].rxUs,
                         ctx.log[i].rssiAvg, ctx.log[i].rssiSync,
                         ctx.log[i].lqi, ctx.log[i].length);
        if (n > 0)
        {
            Serial.write(buf, n);
            Serial.flush();
        }
    }
#endif

#ifdef IS_CAM
    for (int i = 0; i < BENCHMARK_PACKET_COUNT; i++)
    {
        packetPool[i][0] = (i >> 8) & 0xFF;
        packetPool[i][1] = i & 0xFF;
        for (int j = 2; j < PAYLOAD_SIZE_FSK; j++)
            packetPool[i][j] = (uint8_t)(j & 0xFF);
        packetPtrs[i] = packetPool[i];
    }

    digitalWrite(LED_GREEN, HIGH);

    // Serial.println(F("[CAM] Starting benchmark in 2 seconds..."));
    // delay(2000);

    Serial.print(F("[CAM] Sending "));
    Serial.print(BENCHMARK_PACKET_COUNT);
    Serial.print(F(" x "));
    Serial.print(PAYLOAD_SIZE_FSK);
    Serial.println(F(" bytes ..."));

    benchStartus = micros();
    LR2021Error txResult = driver.transmitBurst(
        packetPtrs, BENCHMARK_PACKET_COUNT, PAYLOAD_SIZE_FSK);

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

    unsigned long elapsed = micros() - benchStartus;
    long totalBytes = (long)packetsOk * PAYLOAD_SIZE_FSK;
    float elapsedSec = elapsed / (1000.0f * 1000.0f);
    float throughput = (totalBytes * 8.0f) / elapsedSec / 1000.0f;

    digitalWrite(LED_ORANGE, HIGH);

    Serial.println(F("\n========= CAM BENCHMARK RESULTS ========="));
    Serial.print(F(" Packets sent:    "));
    Serial.println(BENCHMARK_PACKET_COUNT);
    Serial.print(F(" Packets OK:      "));
    Serial.println(packetsOk);
    Serial.print(F(" Packets failed:  "));
    Serial.println(packetsFailed);
    Serial.print(F(" Payload/packet:  "));
    Serial.print(PAYLOAD_SIZE_FSK);
    Serial.println(F(" bytes"));
    Serial.print(F(" Total TX bytes:  "));
    Serial.println(totalBytes);
    Serial.print(F(" Elapsed time:    "));
    Serial.print(elapsedSec, 3);
    Serial.println(F(" s"));
    Serial.print(F(" Throughput:      "));
    Serial.print(throughput, 2);
    Serial.println(F(" kbps"));
    Serial.println(F("=========================================\n"));
#endif
}

void loop()
{
    delay(100);
}