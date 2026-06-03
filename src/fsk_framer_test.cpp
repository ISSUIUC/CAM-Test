#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

#include "USB.h"
#include "USBCDC.h"

#include "fsk.h"
#include "fsk_framer.h"
#include "fsk_reassembler.h"

USBCDC USBSerial;
#undef Serial
#define Serial USBSerial

#define YUV_WIDTH 360u
#define YUV_HEIGHT 240u
#define IMAGE_SIZE (YUV_WIDTH * YUV_HEIGHT * 2u) // 172 800 bytes

static LR2021FSKDriver driver;

#ifdef IS_CAM
static constexpr uint16_t FRAG_COUNT =
    (IMAGE_SIZE + FSK_FRAG_DATA_SIZE - 1) / FSK_FRAG_DATA_SIZE;

static uint8_t (*packetPool)[PAYLOAD_SIZE_FSK] = nullptr;
static uint8_t **packetPtrs = nullptr;
static uint8_t *imageFrame = nullptr;

static void build_yuv422_frame(uint8_t *buf)
{
    uint32_t off = 0;
    for (uint16_t row = 0; row < YUV_HEIGHT; row++)
    {
        uint8_t luma = (row < YUV_HEIGHT / 2) ? 0xFF : 0x00;
        for (uint16_t col = 0; col < YUV_WIDTH; col += 2)
        {
            buf[off++] = luma;
            buf[off++] = 0x80;
            buf[off++] = luma;
            buf[off++] = 0x80;
        }
    }
}
#endif // IS_CAM

#ifdef IS_EAGLE

static FskReassemblyState rxState;

static void blink_task(void *pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        digitalWrite(LED_RED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1000));
        digitalWrite(LED_RED, LOW);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void chunk_output(uint8_t *buf, size_t len)
{
    uint32_t off = 0;
    uint8_t tmp[512];

    while (off < len)
    {
        uint32_t chunk = len - off;
        if (chunk > 512)
            chunk = 512;

        memcpy(tmp, buf + off, chunk);

        size_t wrote = Serial.write(tmp, chunk);
        Serial.flush();

        if (wrote > 0)
            off += wrote;
        else
            vTaskDelay(pdMS_TO_TICKS(1));

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// RX state — mirrors benchmark pattern
static bool rxStarted = false;
static bool rxDone = false;
static uint32_t fragsRx = 0;
static uint32_t crcDrops = 0;
static unsigned long firstRxUs = 0;
static unsigned long lastRxUs = 0;

#endif // IS_EAGLE

void setup()
{
    setCpuFrequencyMhz(240);

    USB.begin();
#ifdef IS_CAM
    Serial.begin(115200);
#endif
#ifdef IS_EAGLE
    Serial.begin(230400);
#endif
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);

    while (!Serial)
        delay(100);

#ifdef IS_CAM
    delay(500);
    Serial.println(F("Mode: CAM (Transmitter - FSK Framer Test)"));
#elifdef IS_EAGLE
    Serial.println(F("Mode: EAGLE (Receiver - FSK Framer Test)"));
#endif

    LR2021Error result = driver.init(SPI);
    if (!result.ok())
    {
        Serial.print(F("[LR2021] Init failed at stage: "));
        Serial.println(result.stageStr());
        Serial.print(F("  RadioLib code: "));
        Serial.println(result.radioLibCode);
        while (true)
            delay(10);
    }
    Serial.println(F("[LR2021] Init OK"));

#ifdef IS_EAGLE
    if (!fsk_reassembler_init(&rxState, IMAGE_SIZE))
    {
        Serial.println(F("[EAGLE] FATAL: reassembler SPIRAM alloc failed"));
        while (true)
            delay(10);
    }
    Serial.println(F("[EAGLE] Reassembler init OK"));
    Serial.println(F("[EAGLE] Listening for fragments..."));
    digitalWrite(LED_BLUE, HIGH);

    xTaskCreatePinnedToCore(
        blink_task, // task function
        "blink",    // name
        1024,       // stack (minimal — just toggling a pin)
        nullptr,    // param
        1,          // priority
        nullptr,    // handle (don't need it)
        0           // Core 0
    );
#endif

#ifdef IS_CAM
    imageFrame = (uint8_t *)heap_caps_malloc(IMAGE_SIZE, MALLOC_CAP_SPIRAM);
    packetPool = (uint8_t (*)[PAYLOAD_SIZE_FSK])
        heap_caps_malloc((size_t)FRAG_COUNT * PAYLOAD_SIZE_FSK, MALLOC_CAP_SPIRAM);
    packetPtrs = (uint8_t **)heap_caps_malloc((size_t)FRAG_COUNT * sizeof(uint8_t *), MALLOC_CAP_SPIRAM);

    if (!imageFrame || !packetPool || !packetPtrs)
    {
        Serial.println(F("[CAM] FATAL: SPIRAM alloc failed"));
        while (true)
            delay(10);
    }

    build_yuv422_frame(imageFrame);
    Serial.println(F("[CAM] YUV422 frame built (top=white / bottom=black)"));

    Serial.print(F("[CAM] Fragmenting "));
    Serial.print(IMAGE_SIZE);
    Serial.print(F(" bytes into "));
    Serial.print(FRAG_COUNT);
    Serial.println(F(" FSK packets..."));

    FskFrame frame = fsk_frame_build(
        /*frame_id=*/0x01,
        imageFrame,
        IMAGE_SIZE,
        packetPool,
        packetPtrs,
        FRAG_COUNT);

    if (!frame.valid)
    {
        Serial.println(F("[CAM] FATAL: fsk_frame_build() returned invalid frame"));
        while (true)
            delay(10);
    }

    Serial.println(F("[CAM] Starting transmission in 2 seconds..."));
    delay(2000);

    Serial.print(F("[CAM] Sending "));
    Serial.print(FRAG_COUNT);
    Serial.print(F(" x "));
    Serial.print(PAYLOAD_SIZE_FSK);
    Serial.println(F(" bytes ..."));

    unsigned long benchStartus = micros();
    LR2021Error txResult = fsk_frame_transmit(driver, frame);
    unsigned long elapsed = micros() - benchStartus;

    digitalWrite(LED_ORANGE, HIGH);

    float elapsedSec = elapsed / 1e6f;
    float throughput = (IMAGE_SIZE * 8.0f) / elapsedSec / 1000.0f;

    if (!txResult.ok())
    {
        Serial.print(F("[CAM] Burst TX failed: "));
        Serial.println(txResult.stageStr());
    }

    Serial.println(F("\n========= CAM TX RESULTS ========="));
    Serial.print(F("  Image size      : "));
    Serial.print(IMAGE_SIZE);
    Serial.println(F(" bytes"));
    Serial.print(F("  Fragments sent  : "));
    Serial.println(FRAG_COUNT);
    Serial.print(F("  TX result       : "));
    Serial.println(txResult.ok() ? F("OK") : F("FAILED"));
    Serial.print(F("  Elapsed time    : "));
    Serial.print(elapsedSec, 3);
    Serial.println(F(" s"));
    Serial.print(F("  Throughput      : "));
    Serial.print(throughput, 2);
    Serial.println(F(" kbps"));
    Serial.println(F("===================================\n"));
#endif // IS_CAM
}

void loop()
{
#ifdef IS_CAM
    delay(100);
#endif

#ifdef IS_EAGLE
    if (rxDone)
        return;

    static uint8_t buf[PAYLOAD_SIZE_FSK];
    static LR2021FskPktStatus pktStatus;

    LR2021Error rxResult = driver.receive(buf, PAYLOAD_SIZE_FSK, &pktStatus);
    unsigned long now = micros();

    // -----------------------------------------------------------------------
    // Timeout — check for end-of-burst (500 ms idle)
    // -----------------------------------------------------------------------
    if (rxResult.driverCode == LR2021_ERR_RX_TIMEOUT)
    {
        if (rxStarted && (now - lastRxUs > 500UL * 1000UL))
            goto finish;
        return;
    }

    // -----------------------------------------------------------------------
    // CRC error
    // -----------------------------------------------------------------------
    if (rxResult.driverCode == LR2021_ERR_CRC_MISMATCH)
    {
        crcDrops++;
        rxState.stat_crc_drop++;
        if (rxStarted)
            lastRxUs = now;
        return;
    }

    // -----------------------------------------------------------------------
    // Other hard error
    // -----------------------------------------------------------------------
    if (!rxResult.ok())
    {
        if (rxStarted)
            lastRxUs = now;
        return;
    }

    // -----------------------------------------------------------------------
    // Good packet — feed into reassembler
    // -----------------------------------------------------------------------
    {
        if (!rxStarted)
        {
            rxStarted = true;
            firstRxUs = now;
            lastRxUs = now;
            digitalWrite(LED_ORANGE, HIGH);
        }

        lastRxUs = now;
        fragsRx++;

        fsk_reassembler_ingest(&rxState, buf, (uint32_t)millis());

        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));

        // If reassembler reports the frame is complete, finish immediately
        if (rxState.completed_size > 0)
            goto finish;
    }

    return;

finish:
    rxDone = true;

    // Flush any partial frame still inside the reassembler
    fsk_reassembler_check_timeout(&rxState, (uint32_t)millis());

    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_ORANGE, LOW);
    digitalWrite(LED_BLUE, HIGH);

    {
        unsigned long elapsed = lastRxUs - firstRxUs;
        float elapsedSec = elapsed / 1e6f;
        uint32_t bytesRx = fragsRx * FSK_FRAG_DATA_SIZE;
        float throughput = (bytesRx * 8.0f) / elapsedSec / 1000.0f;

        Serial.println(F("\n========= EAGLE REASSEMBLY RESULTS ========="));
        Serial.print(F("  Frags received  : "));
        Serial.println(fragsRx);
        Serial.print(F("  CRC drops       : "));
        Serial.println(crcDrops);
        Serial.print(F("  Frame resets    : "));
        Serial.println(rxState.stat_frame_resets);
        Serial.print(F("  Frame complete  : "));
        Serial.println(rxState.completed_size > 0 ? F("YES") : F("NO (partial)"));
        Serial.print(F("  Frame size      : "));
        Serial.print(rxState.completed_size);
        Serial.println(F(" bytes"));
        Serial.print(F("  Elapsed time    : "));
        Serial.print(elapsedSec, 3);
        Serial.println(F(" s"));
        Serial.print(F("  Throughput      : "));
        Serial.print(throughput, 2);
        Serial.println(F(" kbps"));
        Serial.println(F("=============================================\n"));
    }

    // Stream the reassembled frame out over USB serial
    {
        uint32_t frameSize = rxState.completed_size > 0 ? rxState.completed_size : (fragsRx * FSK_FRAG_DATA_SIZE);

        Serial.println(F("[EAGLE] Streaming reassembled frame over serial..."));
        Serial.flush();

        chunk_output(rxState.data_buf, frameSize);

        Serial.println(F("[EAGLE] Stream complete."));

        digitalWrite(LED_BLUE, LOW);
    }

#endif // IS_EAGLE
}