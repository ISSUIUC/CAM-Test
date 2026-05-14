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

    // use 434 configureations for now.
    int state = radio.beginGFSK(FREQ_FSK, BITRATE_FSK_434, FREQ_DEV_FSK_434, RX_BANDWIDTH_FSK_434, POWER, PREAMBLE_LENGTH, XTAL_MODE);
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

LR2021Error LR2021FSKDriver::transmit(uint8_t *data, uint8_t len)
{
    radioEvent = false;
    // RadioTxFifo write opcode: 0x00 0x02
    static uint8_t cmd[PAYLOAD_SIZE_FSK + 2];
    cmd[0] = 0x00;
    cmd[1] = 0x02;
    memcpy(&cmd[2], data, len);
    spiWrite(cmd, len + 2);

    // SetTx with no timeout (continuous until TxDone IRQ)
    uint8_t setTxCmd[] = {0x02, 0x0D, 0x00, 0x00, 0x00, 0x00};
    spiWrite(setTxCmd, sizeof(setTxCmd));

    const unsigned long timeout = 3000;
    unsigned long start = millis();
    while (true)
    {
        if (millis() - start > timeout)
            return LR2021Error{LR2021_ERR_TX_TIMEOUT, 0};
        if (!radioEvent)
            continue;
        radioEvent = false;

        uint32_t irq = readIRQ(); // GetAndClearIrqStatus (same as FLRC)
        if (irq & (1UL << 19))    // TxDone bit 19
            return LR2021Error{LR2021_ERR_NONE, 0};
    }
}

LR2021Error LR2021FSKDriver::receive(uint8_t *data, uint8_t len)
{
    radioEvent = false;
    // Clear stale RX FIFO before arming  (opcode 0x01 0x1E)
    uint8_t clearRxCmd[] = {0x01, 0x1E};
    spiWrite(clearRxCmd, sizeof(clearRxCmd));

    // SetRx continuous (0xFFFFFF timeout = stay until RxDone)
    uint8_t setRxCmd[] = {0x02, 0x0C, 0xFF, 0xFF, 0xFF};
    spiWrite(setRxCmd, sizeof(setRxCmd));

    const unsigned long timeout = 3000;
    unsigned long start = millis();
    while (true)
    {
        if (millis() - start > timeout)
            return LR2021Error{LR2021_ERR_RX_TIMEOUT, 0};
        if (!radioEvent)
            continue;
        radioEvent = false;

        uint32_t irq = readIRQ();
        if (irq & (1UL << 18))
        {                          // RxDone bit 18
            if (irq & (1UL << 22)) // CrcError bit 22
                return LR2021Error{LR2021_ERR_CRC_MISMATCH, 0};

            // ReadRadioRxFifo opcode 0x00 0x01
            uint8_t txBuf[PAYLOAD_SIZE_FSK + 2];
            uint8_t rxBuf[PAYLOAD_SIZE_FSK + 2];
            txBuf[0] = 0x00;
            txBuf[1] = 0x01;
            memset(&txBuf[2], 0x00, len);
            spiTransfer(txBuf, rxBuf, len + 2);
            memcpy(data, &rxBuf[2], len); // data starts at byte 2

            return LR2021Error{LR2021_ERR_NONE, 0};
        }
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

/* ---- helpers ---*/

uint32_t LR2021FSKDriver::readIRQ()
{
    uint8_t irqCmd[] = {0x01, 0x17, 0x00, 0x00, 0x00, 0x00};
    uint8_t irqResp[6] = {0};
    spiTransfer(irqCmd, irqResp, sizeof(irqCmd));
    return (uint32_t)irqResp[2] << 24 | (uint32_t)irqResp[3] << 16 |
           (uint32_t)irqResp[4] << 8 | (uint32_t)irqResp[5];
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
    radio.variablePacketLengthMode(PAYLOAD_SIZE_FSK);
    radio.startTransmit((uint8_t *)CALL_SIGN, strlen(CALL_SIGN));
    delay(500);
    radio.fixedPacketLengthMode(PAYLOAD_SIZE_FSK);
    delay(10);
    setIRQ();
}
