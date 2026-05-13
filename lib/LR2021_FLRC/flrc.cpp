#include "flrc.h"

LR2021Driver::LR2021Driver()
    : mySPI(HSPI),
      spiSettings(SPI_SPEED, MSBFIRST, SPI_MODE0),
      radio(new Module(LR2021_CS, LR2021_GPIO9, LR2021_NRST, LR2021_BUSY, mySPI, spiSettings))
{
}

LR2021Driver *LR2021Driver::_instance = nullptr;

LR2021Error LR2021Driver::init()
{
    if (!mySPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, LR2021_CS))
    {
        // Serial.println("SPI failed wamp wamp");
        // while (true)
        // {
        //     delay(10);
        // }
        return LR2021Error(LR2021_ERR_SPI_INIT_FAILED, 0);
    }

    radio.irqDioNum = IRQ_PIN;

    Serial.print(F("Initializing ... "));
    int state = radio.beginFLRC(
        FREQ,
        BITRATE,
        RADIOLIB_LR2021_FLRC_CR_1_0,
        POWER,
        PREAMBLE_LENGTH,
        RADIOLIB_SHAPING_NONE,
        XTAL_MODE);

    if (state != RADIOLIB_ERR_NONE)
    {
        // Serial.print(F("failed, code "));
        // Serial.println(state);
        // while (true)
        // {
        //     delay(10);
        // }
        return LR2021Error(LR2021_ERR_FLRC_INIT_FAILED, state);
    }

    // NOTE HERE: I had to change the files lib/RadioLib/src/modules/LR2021/LR2021_config.cpp
    // to modify the function to accept a uint16_t PAYLOAD_SIZE variable
    // there was no change in preformance; however, once this fix was applied, which is confusing.
    state = radio.fixedPacketLengthMode(PAYLOAD_SIZE);
    if (state != RADIOLIB_ERR_NONE) // usually this passes so no printing
        return LR2021Error(LR2021_ERR_PKT_LEN_FAILED, state);

    state = radio.setSyncWord(SYNC_WORD, 4);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_SYNC_WORD_FAILED, state);

    state = radio.setCRC(0);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_CRC_CONFIG_FAILED, state);

    _instance = this;
    radio.setIrqAction(setFlag);

    setFIFO();
    setIRQ();

    return LR2021Error(LR2021_ERR_NONE, 0);
}

LR2021Error LR2021Driver::setFIFO()
{
    // Configure TxFifoLow IRQ so we know when to refill the FIFO during 511-byte TX
    // ConfigFifoIrq: opcode 0x01 0x1A
    //   byte 2: rxfifoirqenable = 0x00 (no Rx FIFO IRQs needed)
    //   byte 3: txfifoirqenable = 0x02 (enable TxFifoLow flag → triggers TxFifo IRQ)
    //   bytes 4-5: rxhighthreshold = 0x0000 (unused)
    //   bytes 6-7: txlowthreshold  = FIFO_LOW_THRESH (fire when TX FIFO level < 128)
    //   bytes 8-9: rxlowthreshold  = 0x0000 (unused)
    //   bytes 10-11: txhighthreshold = 0x0000 (unused)
    uint8_t configFifoCmd[] = {
        0x01, 0x1A, // ConfigFifoIrq opcode
        0x00,       // rxfifoirqenable: none
        0x02,       // txfifoirqenable: FifoLow (bit 1)
        0x00, 0x00, // rxhighthreshold (unused)
        (uint8_t)((FIFO_TX_LOW_THRESH >> 8) & 0xFF),
        (uint8_t)(FIFO_TX_LOW_THRESH & 0xFF), // txlowthreshold = 128
        0x00, 0x00,                           // rxlowthreshold (unused)
        0x00, 0x00                            // txhighthreshold (unused)
    };
    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(configFifoCmd, nullptr, sizeof(configFifoCmd));
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();

    return LR2021Error(LR2021_ERR_NONE, 0);
}

LR2021Error LR2021Driver::setIRQ()
{
    // SetDioIrqConfig: opcode 0x01 0x15
    // Dio = 9, Irq bits: TxDone = bit 19 (0x00080000), TxFifo = bit 17 (0x00020000)
    // Combined mask = 0x000A0000
    // DS Table 5-17: TxDone = bit 19, TxFifoIrq = bit 17 (Datasheet section 5.7)

    uint32_t irqMask = (1UL << 19) | (1UL << 17); // TxDone | TxFifoIrq
    uint8_t setDioIrqCmd[] = {
        0x01, 0x15,
        IRQ_PIN,
        (uint8_t)((irqMask >> 24) & 0xFF),
        (uint8_t)((irqMask >> 16) & 0xFF),
        (uint8_t)((irqMask >> 8) & 0xFF),
        (uint8_t)(irqMask & 0xFF)};

    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(setDioIrqCmd, nullptr, sizeof(setDioIrqCmd));
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();

    return LR2021Error(LR2021_ERR_NONE, 0);
}

LR2021Error LR2021Driver::transmit(uint8_t *data)
{
    txFIFOWriteChunkOne(data); // write first 255 bytes to fifo
    txSet();                   // turn on tx

    const unsigned long timeout = 3000; // 3 s safety timeout cuz emptying a 255 bit fifo shouldn't take that long
    unsigned long start = millis();

    while (true)
    {
        if (millis() - start > timeout)
            return LR2021Error(LR2021_ERR_TX_TIMEOUT, 0);

        if (!radioEvent)
            continue;

        radioEvent = false;

        uint32_t irqStatus = readIRQ();

        // TxFifoLow (bit 17): FIFO drained below threshold — write chunk 2
        if (irqStatus & (1UL << 17))
            txFIFOWriteChunkTwo();

        // TxDone (bit 19): full packet transmitted — we are done
        if (irqStatus & (1UL << 19))
            return LR2021Error(LR2021_ERR_NONE, 0);
    }

    return LR2021Error(LR2021_ERR_NONE, 0);
}

void LR2021Driver::txFIFOWriteChunkOne(uint8_t *data)
{
    static uint8_t cmd1[FIFO_CHUNK_1 + 2];
    cmd1[0] = 0x00;
    cmd1[1] = 0x02;
    memcpy(&cmd1[2], data, FIFO_CHUNK_1);
    while (digitalRead(LR2021_BUSY))
    {
    }
    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(cmd1, nullptr, sizeof(cmd1));
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();

    fifoRefillPtr = data + FIFO_CHUNK_1;
    fifoRefillLen = FIFO_CHUNK_2;
}

void LR2021Driver::txSet()
{
    uint8_t setTxCmd[] = {0x02, 0x0D, 0x00, 0x00, 0x00, 0x00};
    while (digitalRead(LR2021_BUSY))
    {
    }
    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(setTxCmd, nullptr, sizeof(setTxCmd));
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();
}

void LR2021Driver::txFIFOWriteChunkTwo()
{
    // fifoRefillPtr and fifoRefillLen set by txFIFOWriteChunkOne()
    static uint8_t cmd2[FIFO_CHUNK_2 + 2];
    cmd2[0] = 0x00;
    cmd2[1] = 0x02; // WriteRadioTxFifo opcode
    memcpy(&cmd2[2], fifoRefillPtr, fifoRefillLen);
    while (digitalRead(LR2021_BUSY))
    {
    }
    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(cmd2, nullptr, sizeof(cmd2));
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();
}

void LR2021Driver::rxSet()
{
}

void LR2021Driver::rxFIFODrainChunk(uint8_t *dst, uint16_t len)
{
}

uint32_t LR2021Driver::readIRQ()
{
    // Read and clear IRQ status
    // GetAndClearIrqStatus opcode: 0x01 0x17 (DS Table 6-48)
    // Response bytes [2:5] = 32-bit IRQ status, MSB first
    uint8_t irqCmd[6] = {0x01, 0x17, 0x00, 0x00, 0x00, 0x00};
    uint8_t irqResp[6] = {0};
    while (digitalRead(LR2021_BUSY))
    {
    }
    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(irqCmd, irqResp, sizeof(irqCmd));
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();

    return ((uint32_t)irqResp[2] << 24) | ((uint32_t)irqResp[3] << 16) | ((uint32_t)irqResp[4] << 8) | (uint32_t)irqResp[5];
}

LR2021Error LR2021Driver::receive(uint8_t *data, uint16_t len)
{
    return LR2021Error();
}

IRAM_ATTR void LR2021Driver::setFlag()
{
    if (_instance)
    {
        _instance->radioEvent = true;
    }
}

// Might not be necessary
void LR2021Driver::transmitCallSign()
{
    radio.variablePacketLengthMode();
    radio.startTransmit((uint8_t *)CALL_SIGN, strlen(CALL_SIGN));
    delay(500);
    radio.fixedPacketLengthMode(PAYLOAD_SIZE);
}
