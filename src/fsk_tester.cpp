#include <Arduino.h>
#include "pins.h"
#include <string.h>

#include "USB.h"
#include "USBCDC.h"

#include "radio.h"

USBCDC USBSerial;
#undef Serial
#define Serial USBSerial

LR2021FLRCDriver driver;

// ─────────────────────────────────────────────
//  CAM state
// ─────────────────────────────────────────────
#ifdef IS_CAM
static uint8_t txBuf[PAYLOAD_SIZE];
static uint32_t camCounter = 0;
#endif

// ─────────────────────────────────────────────
//  EAGLE state
// ─────────────────────────────────────────────
#ifdef IS_EAGLE
static uint8_t rxBuf[PAYLOAD_SIZE];
static LR2021FlrcPktStatus pktStatus;
static uint32_t eagleCounter = 0;
#endif

// ─────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────
void setup()
{
    setCpuFrequencyMhz(240);

    USB.begin();
    Serial.begin(115200);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);

#ifdef IS_CAM
    delay(500);
    Serial.println(F("Mode: CAM (Transmitter - HelloWorld)"));
#elifdef IS_EAGLE
    while (!Serial)
    {
        delay(100);
    }
    delay(50);
    Serial.println(F("Mode: EAGLE (Receiver - HelloWorld)"));
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
    Serial.println(F("[CAM] Sending helloworld_{n} continuously..."));
    digitalWrite(LED_ORANGE, HIGH);
#elifdef IS_EAGLE
    Serial.println(F("[EAGLE] Listening..."));
    digitalWrite(LED_BLUE, HIGH);
#endif
}

// ─────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────
void loop()
{
// ── CAM ──────────────────────────────────────
#ifdef IS_CAM
    // Build "helloworld_{counter}" into a zero-padded PAYLOAD_SIZE buffer
    memset(txBuf, 0, PAYLOAD_SIZE);
    snprintf((char *)txBuf, PAYLOAD_SIZE, "helloworld_%lu", (unsigned long)camCounter);

    LR2021Error txResult = driver.transmit(txBuf);

    if (!txResult.ok())
    {
        Serial.print(F("[CAM] TX failed ("));
        Serial.print(camCounter);
        Serial.print(F("): "));
        Serial.println(txResult.stageStr());
        digitalWrite(LED_RED, HIGH);
        delay(50);
        digitalWrite(LED_RED, LOW);
    }
    else
    {
        Serial.print(F("[CAM] Sent: "));
        Serial.println((char *)txBuf);
        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    }

    camCounter++;
    delay(250); // 4 pkt/s — easy to read on serial, change freely

// ── EAGLE ─────────────────────────────────────
#elifdef IS_EAGLE
    memset(rxBuf, 0, PAYLOAD_SIZE);
    LR2021Error rxResult = driver.receive(rxBuf, PAYLOAD_SIZE, &pktStatus);

    if (rxResult.driverCode == LR2021_ERR_RX_TIMEOUT)
    {
        // Silent re-arm — no CAM transmitting yet or gap between packets
        return;
    }

    if (rxResult.driverCode == LR2021_ERR_CRC_MISMATCH)
    {
        Serial.print(F("[EAGLE] #"));
        Serial.print(eagleCounter);
        Serial.println(F(" CRC mismatch"));
        digitalWrite(LED_RED, HIGH);
        delay(20);
        digitalWrite(LED_RED, LOW);
        eagleCounter++;
        return;
    }

    if (!rxResult.ok())
    {
        Serial.print(F("[EAGLE] RX error: "));
        Serial.println(rxResult.stageStr());
        eagleCounter++;
        return;
    }

    // Good packet — print message + packet status
    // Force null-termination so we can print as a C-string
    rxBuf[PAYLOAD_SIZE - 1] = '\0';

    float rssiAvg = -(pktStatus.rssi_avg_in_dbm + (pktStatus.rssi_avg_half_dbm ? 0.5f : 0.0f));
    float rssiSync = -(pktStatus.rssi_sync_in_dbm + (pktStatus.rssi_sync_half_dbm ? 0.5f : 0.0f));

    Serial.print(F("[EAGLE] #"));
    Serial.print(eagleCounter);
    Serial.print(F("  msg=\""));
    Serial.print((char *)rxBuf);
    Serial.print(F("\"  RSSI_avg="));
    Serial.print(rssiAvg, 1);
    Serial.print(F(" dBm  RSSI_sync="));
    Serial.print(rssiSync, 1);
    Serial.print(F(" dBm  SW_idx="));
    Serial.println(pktStatus.syncword_index);

    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    eagleCounter++;
#endif
}
