#pragma once

#include <RadioLib.h>
#include <Arduino.h>
#include "radio.h"

#ifndef LR2021_FSK_DRIVER
#define LR2021_FSK_DRIVER

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
    SPIClass *_spi;
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
    LR2021 *radio;
    LR2021Error setIRQ();
    LR2021Error init(SPIClass &spi);

    LR2021FSKDriver();
    LR2021Error transmit(uint8_t *data, uint8_t len);
    LR2021Error transmitBurst(uint8_t **packets, int count, uint8_t len); // len should be fsk max size
    LR2021Error receive(uint8_t *data, uint8_t len, LR2021FskPktStatus *outStatus);

    void transmitCallSign();
};

#endif