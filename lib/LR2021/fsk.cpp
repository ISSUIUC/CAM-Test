#include "radio.h"

LR2021FSKDriver::LR2021FSKDriver()
    : mySPI(HSPI),
      spiSettings(SPI_SPEED, MSBFIRST, SPI_MODE0),
      radio(new Module(LR2021_CS, LR2021_GPIO9, LR2021_NRST, LR2021_BUSY, mySPI, spiSettings))
{
}

LR2021Error LR2021FSKDriver::init()
{
    if (!mySPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, LR2021_CS))
        return LR2021Error(LR2021_ERR_SPI_INIT_FAILED, 0);

    radio.irqDioNum = IRQ_PIN;

    Serial.print(F("Initializing ... "));

    int state = radio.beginGFSK(FREQ_FSK, BITRATE_FSK, FREQ_DEV_FSK, RX_BANDWIDTH_FSK, POWER, PREAMBLE_LENGTH, XTAL_MODE);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_FSK_INIT_FAILED, state);

    state = radio.setDataShaping(RADIOLIB_SHAPING_NONE);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_DATASHAPING, state);

    state = radio.fixedPacketLengthMode(PAYLOAD_SIZE_FSK);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_FSK_FIXED_PACKET_MD, state);

    state = radio.setSyncWord(SYNC_WORD_FSK, 4);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_SYNC_WORD_FAILED, state);

    state = radio.setCRC(2, 0xFFFF, 0x8005, false); // IBM CRC (2 bytes, initial 0xFFFF, polynomial 0x8005, non-inverted)
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_CRC_CONFIG_FAILED, state);

    state = radio.disableAddressFiltering();
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_ADDRESS_FILTERING, state);

    _instance = this;
    radio.setIrqAction(setFlag);

    setIRQ();

    return LR2021Error(LR2021_ERR_NONE, 0);
}

LR2021Error LR2021FSKDriver::transmit(uint8_t *data, size_t len)
{
    int state = radio.startTransmit(data, len);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_TX_TIMEOUT, state);

    const unsigned long timeout = 3000;
    unsigned long start = millis();

    while (true)
    {
        if (millis() - start > timeout)
            return LR2021Error(LR2021_ERR_TX_TIMEOUT, 0);
        if (!radioEvent)
            continue;
        radioEvent = false;
        radio.finishTransmit(); // powers down PA, resets state
        return LR2021Error(LR2021_ERR_NONE, 0);
    }
}

LR2021Error LR2021FSKDriver::receive(uint8_t *data, size_t len)
{
    int state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_RX_TIMEOUT, state);

    const unsigned long timeout = 3000;
    unsigned long start = millis();

    while (true)
    {
        if (millis() - start > timeout)
            return LR2021Error(LR2021_ERR_RX_TIMEOUT, 0);
        if (!radioEvent)
            continue;
        radioEvent = false;

        state = radio.readData(data, len);
        if (state == RADIOLIB_ERR_CRC_MISMATCH)
            return LR2021Error(LR2021_ERR_CRC_MISMATCH, 0);
        if (state != RADIOLIB_ERR_NONE)
            return LR2021Error(LR2021_ERR_GENERIC, state);

        return LR2021Error(LR2021_ERR_NONE, 0);
    }
}

// SetDioIrqConfig: opcode 0x01 0x15  (DS Table 6-46 / §5.7)
// TxDone   = bit 19
// RxDone   = bit 18
LR2021Error LR2021FSKDriver::setIRQ()
{
    uint32_t irqMask = (1UL << 19) | (1UL << 18);
    uint8_t setDioIrqCmd[] = {
        0x01, 0x15,
        IRQ_PIN,
        (uint8_t)((irqMask >> 24) & 0xFF),
        (uint8_t)((irqMask >> 16) & 0xFF),
        (uint8_t)((irqMask >> 8) & 0xFF),
        (uint8_t)(irqMask & 0xFF)};

    spiWrite(setDioIrqCmd, sizeof(setDioIrqCmd));

    return LR2021Error(LR2021_ERR_NONE, 0);
}

void LR2021FSKDriver::spiWrite(const uint8_t *cmd, size_t len)
{
    while (digitalRead(LR2021_BUSY))
        ;
    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(cmd, nullptr, len);
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();
}

void LR2021FSKDriver::spiTransfer(const uint8_t *txBuf, uint8_t *rxBuf, size_t len)
{
    while (digitalRead(LR2021_BUSY))
        ;
    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(txBuf, rxBuf, len);
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();
}

IRAM_ATTR void LR2021FSKDriver::setFlag()
{
    if (_instance)
    {
        _instance->radioEvent = true;
    }
}

// Might not be necessary
void LR2021FSKDriver::transmitCallSign()
{
    radio.variablePacketLengthMode();
    radio.startTransmit((uint8_t *)CALL_SIGN, strlen(CALL_SIGN));
    delay(500);
    radio.fixedPacketLengthMode(PAYLOAD_SIZE);
}
