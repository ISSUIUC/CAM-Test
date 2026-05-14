#include "radio.h"

LR2021FLRCDriver::LR2021FLRCDriver()
    : mySPI(HSPI),
      spiSettings(SPI_SPEED, MSBFIRST, SPI_MODE0),
      radio(new Module(LR2021_CS, LR2021_GPIO9, LR2021_NRST, LR2021_BUSY, mySPI, spiSettings))
{
}

LR2021FLRCDriver *LR2021FLRCDriver::_instance = nullptr;

LR2021Error LR2021FLRCDriver::init()
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

#ifdef IS_EAGLE
    int state = radio.beginFLRC(
        FREQ,
        BITRATE_FLRC,
        RADIOLIB_LR2021_FLRC_CR_1_0,
        POWER,
        PREAMBLE_LENGTH,
        RADIOLIB_SHAPING_NONE,
        XTAL_MODE);
#elifdef IS_CAM
    int state = radio.beginFLRC(
        FREQ,
        BITRATE_FLRC,
        RADIOLIB_LR2021_FLRC_CR_2_3,
        POWER,
        PREAMBLE_LENGTH,
        RADIOLIB_SHAPING_0_5,
        XTAL_MODE);
#endif

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

    state = radio.setSyncWord(SYNC_WORD_FLRC, 4);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_SYNC_WORD_FAILED, state);

    state = radio.setCRC(CRC_LENGTH);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_CRC_CONFIG_FAILED, state);

    _instance = this;
    radio.setIrqAction(setFlag);

    setFIFO();
    setIRQ();

    return LR2021Error(LR2021_ERR_NONE, 0);
}

// ConfigFifoIrq: opcode 0x01 0x1A
//   byte 2: rxfifoirqenable = 0x04 (FifoHigh — fires when RX FIFO > rxhighthreshold)
//   byte 3: txfifoirqenable = 0x02 (enable TxFifoLow flag → triggers TxFifo IRQ)
//   bytes 4-5: rxhighthreshold = FIFO_RX_HIGH_THRESH (unused)
//   bytes 6-7: txlowthreshold  = FIFO_TX_LOW_THRESH (fire when TX FIFO level < 128)
//   bytes 8-9: rxlowthreshold  = 0x0000 (unused)
//   bytes 10-11: txhighthreshold = 0x0000 (unused)
LR2021Error LR2021FLRCDriver::setFIFO()
{
    uint8_t configFifoCmd[] = {
        0x01, 0x1A,
        0x04, // rxfifoirqenable: FifoHigh (bit 2)
        0x02, // txfifoirqenable: FifoLow  (bit 1)
        (uint8_t)((FIFO_RX_HIGH_THRESH >> 8) & 0xFF),
        (uint8_t)(FIFO_RX_HIGH_THRESH & 0xFF), // rxhighthreshold = 200
        (uint8_t)((FIFO_TX_LOW_THRESH >> 8) & 0xFF),
        (uint8_t)(FIFO_TX_LOW_THRESH & 0xFF), // txlowthreshold  = 128
        0x00, 0x00,                           // rxlowthreshold  (unused)
        0x00, 0x00                            // txhighthreshold (unused)
    };
    spiWrite(configFifoCmd, sizeof(configFifoCmd));

    return LR2021Error(LR2021_ERR_NONE, 0);
}

// SetDioIrqConfig: opcode 0x01 0x15  (DS Table 6-46 / §5.7)
// TxDone   = bit 19, TxFifoIrq = bit 17 (TX — unchanged)
// RxDone   = bit 18 (packet reception completed)
// RxFifo   = bit  0 Rx FIFO high threshold reached - triggers mid-packet
LR2021Error LR2021FLRCDriver::setIRQ()
{
    uint32_t irqMask = (1UL << 19) | (1UL << 18) | (1UL << 17) | (1UL << 0);
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

LR2021Error LR2021FLRCDriver::transmit(uint8_t *data)
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
        {
            txFIFOWriteChunkTwo();
            clearFifoIrq(0x00, 0x02);
        }

        // TxDone (bit 19): full packet transmitted — we are done
        if (irqStatus & (1UL << 19))
            return LR2021Error(LR2021_ERR_NONE, 0);
    }

    return LR2021Error(LR2021_ERR_NONE, 0);
}

void LR2021FLRCDriver::txFIFOWriteChunkOne(uint8_t *data)
{
    static uint8_t cmd1[FIFO_TX_CHUNK_1 + 2];
    cmd1[0] = 0x00;
    cmd1[1] = 0x02;
    memcpy(&cmd1[2], data, FIFO_TX_CHUNK_1);
    while (digitalRead(LR2021_BUSY))
    {
    }
    spiWrite(cmd1, sizeof(cmd1));

    fifoRefillPtr = data + FIFO_TX_CHUNK_1;
    fifoRefillLen = FIFO_TX_CHUNK_2;
}

void LR2021FLRCDriver::txSet()
{
    uint8_t setTxCmd[] = {0x02, 0x0D, 0x00, 0x00, 0x00, 0x00};
    spiWrite(setTxCmd, sizeof(setTxCmd));
}

void LR2021FLRCDriver::txFIFOWriteChunkTwo()
{
    // fifoRefillPtr and fifoRefillLen set by txFIFOWriteChunkOne()
    static uint8_t cmd2[FIFO_TX_CHUNK_2 + 2];
    cmd2[0] = 0x00;
    cmd2[1] = 0x02; // WriteRadioTxFifo opcode
    memcpy(&cmd2[2], fifoRefillPtr, fifoRefillLen);

    spiWrite(cmd2, sizeof(cmd2));
}

uint32_t LR2021FLRCDriver::readIRQ()
{
    // Read and clear IRQ status
    // GetAndClearIrqStatus opcode: 0x01 0x17 (DS Table 6-48)
    // Response bytes [2:5] = 32-bit IRQ status, MSB first
    uint8_t irqCmd[6] = {0x01, 0x17, 0x00, 0x00, 0x00, 0x00};
    uint8_t irqResp[6] = {0};

    spiTransfer(irqCmd, irqResp, sizeof(irqCmd));

    return ((uint32_t)irqResp[2] << 24) | ((uint32_t)irqResp[3] << 16) | ((uint32_t)irqResp[4] << 8) | (uint32_t)irqResp[5];
}

void LR2021FLRCDriver::rxSet()
{
    // SetRx continuous mode: opcode 0x02 0x0C, timeout = 0xFFFFFF (DS §6.3.5, Table 6-11)
    // 0xFFFFFF = stay in Rx until host commands otherwise — device signals RxDone each packet
    uint8_t setRxCmd[] = {0x02, 0x0C, 0xFF, 0xFF, 0xFF};
    while (digitalRead(LR2021_BUSY))
        ;

    spiWrite(setRxCmd, sizeof(setRxCmd));
}

void LR2021FLRCDriver::rxFIFODrainChunk(uint8_t *dst, uint16_t len)
{
    // ReadRadioRxFifo: opcode 0x00 0x01 — direct read, data starts at byte 2 (DS Table 6-1)
    // Build a tx buffer of (2 + len) zeros; the chip streams data back on MISO starting at byte 2
    static uint8_t txBuf[258]; // 2 opcode + up to 256 data
    static uint8_t rxBuf[258];

    txBuf[0] = 0x00;
    txBuf[1] = 0x01;
    memset(&txBuf[2], 0x00, len);

    spiTransfer(txBuf, rxBuf, len + 2);

    memcpy(dst, &rxBuf[2], len); // bytes 0-1 are Stat[15:0], data starts at byte 2
}

bool LR2021FLRCDriver::rxGetFLRCPcktStatus(LR2021FlrcPktStatus *pkt_status)
{
    // GetFlrcPacketStatus: opcode 0x02 0x4B, 5 response bytes after Stat[15:0]
    // Full frame: 2 opcode + 5 dummy tx = 7 bytes total
    // Response layout (DS Table 18-9):
    //   resp[0-1] = Stat[15:0]      (ignored)
    //   resp[2]   = pkt_len[15:8]
    //   resp[3]   = pkt_len[7:0]
    //   resp[4]   = rssi_avg[8:1]   (MSBs)
    //   resp[5]   = rssi_sync[8:1]  (MSBs)
    //   resp[6]   = sw_num[7:4] | rfu[3] | rssi_avg(0)[2] | rfu[1] | rssi_sync(0)[0]
    uint8_t cmd[7] = {0x02, 0x4B, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t resp[7] = {0};

    spiTransfer(cmd, resp, sizeof(cmd));

    // Stat bytes are resp[0:1] — check for command error (bit 1 of Stat[7:0])
    if (resp[1] & 0x02)
        return false;

    uint16_t rssiAvgRaw = ((uint16_t)resp[4] << 1) | ((resp[6] >> 2) & 0x01);
    uint16_t rssiSyncRaw = ((uint16_t)resp[5] << 1) | (resp[6] & 0x01);

    pkt_status->packet_length_bytes = (uint16_t)(((uint16_t)resp[2] << 8) | resp[3]);
    pkt_status->rssi_avg_in_dbm = -(int16_t)resp[4];
    pkt_status->rssi_sync_in_dbm = -(int16_t)resp[5];
    pkt_status->rssi_avg_half_dbm = (resp[6] >> 2) & 0x01;
    pkt_status->rssi_sync_half_dbm = (resp[6] >> 0) & 0x01;
    pkt_status->syncword_index = resp[6] >> 4;

    return true;
}

LR2021Error LR2021FLRCDriver::receive(uint8_t *data, uint16_t len, LR2021FlrcPktStatus *pktStatus)
{
    // Reset RX state
    rxFifoHighFired = false;
    rxBytesRead = 0;

    // Clear any stale RX FIFO data before arming (§6.10.7, opcode 0x01 0x1E)
    uint8_t clearRxCmd[] = {0x01, 0x1E};
    while (digitalRead(LR2021_BUSY))
        ;

    spiWrite(clearRxCmd, sizeof(clearRxCmd));

    rxSet(); // SetRx continuous (0xFFFFFF timeout)

    const unsigned long timeout = 3000; // 3 s safety net
    unsigned long start = millis();

    while (true)
    {
        if (millis() - start > timeout)
            return LR2021Error(LR2021_ERR_RX_TIMEOUT, 0);

        if (!radioEvent)
            continue;

        radioEvent = false;

        uint32_t irqStatus = readIRQ(); // GetAndClearIrqStatus — clears all pending IRQs

        // RxFifo IRQ (bit 0): FIFO high threshold reached mid-packet
        // DS Table 5-17: bit 0 = RxFifo, §5.3.1
        if ((irqStatus & (1UL << 0)) && !rxFifoHighFired)
        {
            rxFifoHighFired = true;
            rxFIFODrainChunk(data, FIFO_RX_HIGH_THRESH);
            rxBytesRead = FIFO_RX_HIGH_THRESH;
            clearFifoIrq(0x04, 0x00);
        }

        // RxDone (bit 18): full packet received — drain whatever remains in FIFO
        // DS Table 5-17: bit 18 = RxDone
        if (irqStatus & (1UL << 18))
        {
            if (irqStatus & (1UL << 22))
                return LR2021Error(LR2021_ERR_CRC_MISMATCH, 0);

            if (pktStatus != nullptr)
                rxGetFLRCPcktStatus(pktStatus);

            uint16_t remaining = len - rxBytesRead;
            if (remaining > 0)
                rxFIFODrainChunk(data + rxBytesRead, remaining);

            return LR2021Error(LR2021_ERR_NONE, 0);
        }
    }
    return LR2021Error(LR2021_ERR_NONE, 0);
}

IRAM_ATTR void LR2021FLRCDriver::setFlag()
{
    if (_instance)
    {
        _instance->radioEvent = true;
    }
}

void LR2021FLRCDriver::clearFifoIrq(uint8_t rxFlags, uint8_t txFlags)
{
    const uint8_t cmd[] = {0x01, 0x14, rxFlags, txFlags};
    spiWrite(cmd, sizeof(cmd));
}

// Might not be necessary
void LR2021FLRCDriver::transmitCallSign()
{
    radio.variablePacketLengthMode();
    radio.startTransmit((uint8_t *)CALL_SIGN, strlen(CALL_SIGN));
    delay(500);
    radio.fixedPacketLengthMode(PAYLOAD_SIZE);
}

void LR2021FLRCDriver::spiWrite(const uint8_t *cmd, size_t len)
{
    while (digitalRead(LR2021_BUSY))
        ;
    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(cmd, nullptr, len);
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();
}

void LR2021FLRCDriver::spiTransfer(const uint8_t *txBuf, uint8_t *rxBuf, size_t len)
{
    while (digitalRead(LR2021_BUSY))
        ;
    mySPI.beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    mySPI.transferBytes(txBuf, rxBuf, len);
    digitalWrite(LR2021_CS, HIGH);
    mySPI.endTransaction();
}
