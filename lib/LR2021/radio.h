#pragma once

#ifndef LR2021_FLRC_DRIVER
#define LR2021_FLRC_DRIVER

#include <RadioLib.h>
#include <Arduino.h>
#include "../../src/pins.h"

static const uint16_t PAYLOAD_SIZE = 511;
static const uint8_t PAYLOAD_SIZE_FSK = RADIOLIB_LR2021_MAX_PACKET_LENGTH;
#define SPI_SPEED 16000000

static uint8_t SYNC_WORD_FLRC[] = {0x2D, 0x01, 0x4B, 0x1D};
static uint8_t SYNC_WORD_FSK[] = {0x01, 0x23, 0x45, 0x67,
                                  0x89, 0xAB, 0xCD, 0xEF};
static const char *CALL_SIGN = "KE2CNQ";
static const int IRQ_PIN = 9;

#define FREQ_915 915.0 // mhz
#define FREQ_434 434.0 // mhz

#define FREQ FREQ_434     // MHz
#define FREQ_FSK FREQ_434 // MHz
#define BITRATE_FLRC 2600 // Kbps -> reference table 18-1 on datahseet

#define BITRATE_FSK_434 1000
#define RX_BANDWIDTH_FSK_434 2222.22 // khz Table 11-2
#define FREQ_DEV_FSK_434 250.0       // khz

#define BITRATE_FSK_915 2000
#define RX_BANDWIDTH_FSK_915 1111.10 // khz Table 11-2
#define FREQ_DEV_FSK_915 500         // khz

#define POWER 22           // dBm Tx Power
#define PREAMBLE_LENGTH 24 // bit
#define XTAL_MODE 0        // tcxoVoltage argument, setting to 0 brings it to xtal mode
#define CRC_LENGTH 3       // bits, 0, 1, 2, 3

/* FIFO */
#define FIFO_TX_CHUNK_1 255    // bytes in first write (fills FIFO without overflow)
#define FIFO_TX_CHUNK_2 256    // bytes in second write (remainder of 511)
#define FIFO_TX_LOW_THRESH 128 // fire TxFifoLow IRQ when FIFO drops below this level

#define FIFO_RX_HIGH_THRESH 200 // fire RxFifo IRQ when RX FIFO fills above this

/* Error stuff */
#define LR2021_ERR_NONE 0
#define LR2021_ERR_GENERIC -6767
#define LR2021_ERR_SPI_INIT_FAILED -2001
#define LR2021_ERR_FLRC_INIT_FAILED -2002
#define LR2021_ERR_FSK_INIT_FAILED -2009
#define LR2021_ERR_SYNC_WORD_FAILED -2003
#define LR2021_ERR_CRC_CONFIG_FAILED -2004
#define LR2021_ERR_PKT_LEN_FAILED -2005
#define LR2021_ERR_TX_TIMEOUT -2006
#define LR2021_ERR_RX_TIMEOUT -2007
#define LR2021_ERR_CRC_MISMATCH -2008
#define LR2021_ERR_ADDRESS_FILTERING -2010
#define LR2021_ERR_DATASHAPING -2011
#define LR2021_ERR_FSK_FIXED_PACKET_MD -2012

struct LR2021Error
{
    int driverCode;
    int radioLibCode;

    bool ok() const { return driverCode == LR2021_ERR_NONE; }

    const char *stageStr() const
    {
        switch (driverCode)
        {
        case LR2021_ERR_NONE:
            return "OK";
        case LR2021_ERR_SPI_INIT_FAILED:
            return "SPI bus init failed";
        case LR2021_ERR_FSK_INIT_FAILED:
            return "radio.beginFSK failed";
        case LR2021_ERR_FLRC_INIT_FAILED:
            return "radio.beginFLRC() failed";
        case LR2021_ERR_SYNC_WORD_FAILED:
            return "radio.setSyncWord() failed";
        case LR2021_ERR_CRC_CONFIG_FAILED:
            return "radio.setCRC() failed";
        case LR2021_ERR_PKT_LEN_FAILED:
            return "radio.fixedPacketLengthMode() failed";
        case LR2021_ERR_TX_TIMEOUT:
            return "lr2021.transmit() tx timeout";
        case LR2021_ERR_RX_TIMEOUT:
            return "lr2021.receive() rx timeout";
        case LR2021_ERR_CRC_MISMATCH:
            return "lr2021.receive() crc mismatch";
        case LR2021_ERR_ADDRESS_FILTERING:
            return "radio.disableAddressFiltering(); failed";
        case LR2021_ERR_DATASHAPING:
            return "radio.setDataShaping(); failed";
        case LR2021_ERR_FSK_FIXED_PACKET_MD:
            return "radio.fixedPacketLengthMode(PAYLOAD_SIZE_FSK); failed";
        default:
            return "unknown error";
        }
    }
};

// Based off table 18-9
typedef struct
{
    uint16_t packet_length_bytes; // pkt_len[15:0]
    int16_t rssi_avg_in_dbm;      // -rssi_avg / 2 dBm (integer part)
    int16_t rssi_sync_in_dbm;     // -rssi_sync / 2 dBm (integer part)
    uint8_t rssi_avg_half_dbm;    // rssi_avg(0) — bit 2 of byte 6, adds 0.5 dBm
    uint8_t rssi_sync_half_dbm;   // rssi_sync(0) — bit 0 of byte 6, adds 0.5 dBm
    uint8_t syncword_index;       // sw_num[3:0] — bits [7:4] of byte 6
} LR2021FlrcPktStatus;

// Table 11 - 15

struct LR2021FskPktStatus
{
    uint16_t pktlen;
    float rssiAvg;  // dBm, e.g. -87.5
    float rssiSync; // dBm, e.g. -56.5
    uint8_t addrMatchBcast;
    uint8_t addrMatchNode;
    float lqi; // dB, 0.25 steps — >10 dB is good
};

class LR2021FSKDriver
{
private:
    SPIClass mySPI;
    SPISettings spiSettings;

    static LR2021FSKDriver *_instance;

    /* Flags */
    volatile bool radioEvent = false;
    static void setFlag();

    /* IRQ */
    uint32_t readIRQ();

    /* SPI */
    void spiWrite(const uint8_t *cmd, size_t len);
    void spiTransfer(const uint8_t *txBuf, uint8_t *rxBuf, size_t len);

    /* Other */
    LR2021FskPktStatus getFskPacketStatus();

public:
    LR2021 radio;
    LR2021Error setIRQ();
    LR2021Error init();

    LR2021FSKDriver();
    LR2021Error transmit(uint8_t *data, uint8_t len);
    LR2021Error transmitBurst(uint8_t **packets, int count, uint8_t len); // len should be fsk max size
    LR2021Error receive(uint8_t *data, uint8_t len, LR2021FskPktStatus *outStatus);

    void transmitCallSign();
};

// BAD BAD BAD BAD ASFJLADSFJ;LASDJFKASDLFJKLAS DO NOT USE PLEASE
class LR2021FLRCDriver
{
private:
    SPIClass mySPI;
    SPISettings spiSettings;

    static LR2021FLRCDriver *_instance;

    /* Flags */
    volatile bool radioEvent = false;
    static void setFlag();

    /* Tx stuff */
    uint8_t *fifoRefillPtr;
    uint16_t fifoRefillLen;

    void txFIFOWriteChunkOne(uint8_t *data);
    void txSet();
    void txFIFOWriteChunkTwo();

    /* Rx stuff */
    uint16_t rxBytesRead; // running count of bytes drained mid-packet
    bool rxFifoHighFired;

    void rxSet();
    void rxFIFODrainChunk(uint8_t *dst, uint16_t len);

    uint32_t readIRQ();
    void clearFifoIrq(uint8_t rxFlags, uint8_t txFlags);

    /* SPI */
    void spiWrite(const uint8_t *cmd, size_t len);
    void spiTransfer(const uint8_t *txBuf, uint8_t *rxBuf, size_t len);

public:
    LR2021 radio;

    LR2021FLRCDriver();
    LR2021Error init();
    LR2021Error setFIFO();
    LR2021Error setIRQ();
    LR2021Error transmit(uint8_t *data);
    LR2021Error receive(uint8_t *data, uint16_t len, LR2021FlrcPktStatus *pkt_status = nullptr);

    bool rxGetFLRCPcktStatus(LR2021FlrcPktStatus *pkt_status);

    void transmitCallSign();
};

#endif

/*
How TX fifo works:
511 bytes = maximum FLRC protocol payload, defined by the packet format (Section 18.2.1)
256 bytes = physical FIFO depth

1. We write 255 bytes into FIFO
2. We setTX so we start transmitting and drain the fifo
3. When fifo is < 128 bytes, we set the fifo low flag
4. Start writing 256 bytes as fifo is being drained
5. stop transmitting
*/

/*
How RX fifo works:
511 bytes = maximum FLRC protocol payload, defined by the packet format (Section 18.2.1)
256 bytes = physical FIFO depth

1. clear rx fifo, then setRX to arm continuous receive mode
2. Radio fills FIFO as packet arrives
3. When FIFO > 200 bytes, RxFifo high threshold IRQ fires
4. Drain first 200 bytes mid-packet to prevent overflow
5. RxDone IRQ fires when full packet is received
6. We drain remaining 311 bytes
*/