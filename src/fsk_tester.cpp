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

#ifdef IS_CAM
static uint8_t txBuf[PAYLOAD_SIZE_FSK];
static uint32_t camCounter = 0;
#endif

#ifdef IS_EAGLE
static uint8_t rxBuf[PAYLOAD_SIZE_FSK];
static LR2021FskPktStatus pktStatus;
static uint32_t eagleCounter = 0;
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

void loop()
{

#ifdef IS_CAM
    memset(txBuf, 0, PAYLOAD_SIZE_FSK);
    snprintf((char *)txBuf, PAYLOAD_SIZE_FSK, "helloworld_%lu", (unsigned long)camCounter);

    LR2021Error txResult = driver.transmit(txBuf, PAYLOAD_SIZE_FSK);

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
    delay(250);

#elifdef IS_EAGLE
    memset(rxBuf, 0, PAYLOAD_SIZE_FSK);
    LR2021Error rxResult = driver.receive(rxBuf, PAYLOAD_SIZE_FSK, &pktStatus);

    if (rxResult.driverCode == LR2021_ERR_RX_TIMEOUT)
    {
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

    rxBuf[PAYLOAD_SIZE_FSK - 1] = '\0';

    float rssiAvg = -(pktStatus.rssiAvgRaw / 2.0f);
    float rssiSync = -(pktStatus.rssiSyncRaw / 2.0f);

    Serial.print(F("[EAGLE] #"));
    Serial.print(eagleCounter);
    Serial.print(F(",  msg=\""));
    Serial.print((char *)rxBuf);
    Serial.print(F("\",  RSSI_avg="));
    Serial.print(rssiAvg, 3);
    Serial.print(F(" dBm,  RSSI_sync="));
    Serial.print(rssiSync, 3);
    Serial.print(F(" dBm,  LQI ="));
    Serial.print(pktStatus.lqi, 3);
    Serial.print(", Pktlen =");
    Serial.println(pktStatus.pktlen);

    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    eagleCounter++;
#endif
}
