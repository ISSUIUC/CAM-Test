#pragma once

#ifndef LR2021_FLRC_DRIVER
#define LR2021_FLRC_DRIVER

#include <RadioLib.h>
#include <Arduino.h>
#include "../../src/pins.h"

static const uint16_t PAYLOAD_SIZE = 511;
#define SPI_SPEED 48000000

static uint8_t SYNC_WORD[] = {0x2D, 0x01, 0x4B, 0x1D};
static const char *CALL_SIGN = "KE2CNQ";
static const int IRQ_PIN = 9;

#define FREQ 434.0         // MHz
#define BITRATE 2600       // Kbps
#define POWER 22           // dBm Tx Power
#define PREAMBLE_LENGTH 24 // bit
#define XTAL_MODE 0        // tcxoVoltage argument, setting to 0 brings it to xtal mode

/* FIFO */
#define FIFO_TX_CHUNK_1 255    // bytes in first write (fills FIFO without overflow)
#define FIFO_TX_CHUNK_2 256    // bytes in second write (remainder of 511)
#define FIFO_TX_LOW_THRESH 128 // fire TxFifoLow IRQ when FIFO drops below this level

#define FIFO_RX_HIGH_THRESH 200 // fire RxFifo IRQ when RX FIFO fills above this

/* Error stuff */
#define LR2021_ERR_NONE 0
#define LR2021_ERR_SPI_INIT_FAILED -2001
#define LR2021_ERR_FLRC_INIT_FAILED -2002
#define LR2021_ERR_SYNC_WORD_FAILED -2003
#define LR2021_ERR_CRC_CONFIG_FAILED -2004
#define LR2021_ERR_PKT_LEN_FAILED -2005
#define LR2021_ERR_TX_TIMEOUT -2006
#define LR2021_ERR_RX_TIMEOUT -2007
#define LR2021_ERR_CRC_MISMATCH -2008

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
        default:
            return "unknown error";
        }
    }
};

class LR2021Driver
{
private:
    LR2021 radio;
    SPIClass mySPI;
    SPISettings spiSettings;

    static LR2021Driver *_instance;

    /* Flags */
    volatile bool radioEvent = false;
    static void setFlag();

    /* Tx stuff */
    uint8_t *fifoRefillPtr;
    uint16_t fifoRefillLen;

    void transmitCallSign();

    void txFIFOWriteChunkOne(uint8_t *data);
    void txSet();
    void txFIFOWriteChunkTwo();

    /* Rx stuff */
    uint16_t rxBytesRead; // running count of bytes drained mid-packet

    void rxSet();
    void rxFIFODrainChunk(uint8_t *dst, uint16_t len);

    uint32_t readIRQ();

public:
    LR2021Driver();
    LR2021Error init();
    LR2021Error setFIFO();
    LR2021Error setIRQ();
    LR2021Error transmit(uint8_t *data);
    LR2021Error receive(uint8_t *data, uint16_t len);
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