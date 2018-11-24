/*
  MIT License

  Copyright (c) 2018 Antonio Alexander Brewer (tonton81) - https://github.com/tonton81
  Thanks goes to Mike for all bug reports and testing :)

  Designed and tested for PJRC Teensy (3.2, 3.5, 3.6).

  Forum link : https://forum.pjrc.com/threads/54323-IMCTFD-Improved-Microchip-CAN-Teensy-FlexData-Library?styleid=1

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#if !defined(_IMCTFD_H_)
#define _IMCTFD_H_

#include "Arduino.h"
#include <SPI.h>
#include "circular_buffer.h"

#define IMCTFD_RX_BUFSIZE 16

/* SPI Instruction Set */
#define cINSTRUCTION_RESET			0x00
#define cINSTRUCTION_READ			0x03
#define cINSTRUCTION_READ_CRC       0x0B
#define cINSTRUCTION_WRITE			0x02
#define cINSTRUCTION_WRITE_CRC      0x0A
#define cINSTRUCTION_WRITE_SAFE     0x0C

/* Register Addresses */
#define cREGADDR_CiCON  	0x000
#define cREGADDR_CiNBTCFG	0x004
#define cREGADDR_CiDBTCFG	0x008
#define cREGADDR_CiTDC  	0x00C

#define cREGADDR_CiTBC      0x010
#define cREGADDR_CiTSCON    0x014
#define cREGADDR_CiVEC      0x018
#define cREGADDR_CiINT      0x01C
#define cREGADDR_CiINTFLAG      cREGADDR_CiINT
#define cREGADDR_CiINTENABLE    (cREGADDR_CiINT+2)

#define cREGADDR_CiRXIF     0x020
#define cREGADDR_CiTXIF     0x024
#define cREGADDR_CiRXOVIF   0x028
#define cREGADDR_CiTXATIF   0x02C

#define cREGADDR_CiTXREQ    0x030
#define cREGADDR_CiTREC     0x034
#define cREGADDR_CiBDIAG0   0x038
#define cREGADDR_CiBDIAG1   0x03C

#define cREGADDR_CiTEFCON   0x040
#define cREGADDR_CiTEFSTA   0x044
#define cREGADDR_CiTEFUA    0x048
#define cREGADDR_CiFIFOBA   0x04C

#define cREGADDR_CiFIFOCON  0x050
#define cREGADDR_CiFIFOSTA  0x054
#define cREGADDR_CiFIFOUA   0x058
#define CiFIFO_OFFSET       (12)

#define cREGADDR_CiTXQCON  0x050
#define cREGADDR_CiTXQSTA  0x054
#define cREGADDR_CiTXQUA   0x058

/* MCP2517 Specific */
#define cREGADDR_OSC        0xE00
#define cREGADDR_IOCON      0xE04
#define cREGADDR_CRC    	0xE08
#define cREGADDR_ECCCON  	0xE0C
#define cREGADDR_ECCSTA  	0xE10

#define cREGADDR_CiFLTCON   0x1D0
#define cREGADDR_CiFLTOBJ   0x1F0
#define cREGADDR_CiMASK     0x1F4

/* RAM addresses */
#define cRAM_SIZE       2048
#define cRAMADDR_START  0x400
#define cRAMADDR_END    (cRAMADDR_START+cRAM_SIZE)

//! CAN Operation Modes
typedef enum {
    CAN_NORMAL_MODE = 0x00,
    CAN_SLEEP_MODE = 0x01,
    CAN_INTERNAL_LOOPBACK_MODE = 0x02,
    CAN_LISTEN_ONLY_MODE = 0x03,
    CAN_CONFIGURATION_MODE = 0x04,
    CAN_EXTERNAL_LOOPBACK_MODE = 0x05,
    CAN_CLASSIC_MODE = 0x06,
    CAN_RESTRICTED_MODE = 0x07,
    CAN_INVALID_MODE = 0xFF
} CAN_OPERATION_MODE;

const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

typedef enum IMCTFD_CANFD_FIFO_CHANNELS {
  FIFO1 = 1,
  FIFO2,
  FIFO3,
  FIFO4,
  FIFO5,
  FIFO6,
  FIFO7,
  FIFO8,
  FIFO9,
  FIFO10,
  FIFO11,
  FIFO12,
  FIFO13,
  FIFO14,
  FIFO15,
  FIFO16,
  FIFO17,
  FIFO18,
  FIFO19,
  FIFO20,
  FIFO21,
  FIFO22,
  FIFO23,
  FIFO24,
  FIFO25,
  FIFO26,
  FIFO27,
  FIFO28,
  FIFO29,
  FIFO30,
  FIFO31,
  TXQ,
  TEF
} IMCTFD_CANFD_FIFO_CHANNELS;

typedef enum IMCTFD_CANFD_CHANNEL_PRIORITIES {
  PRIORITY_0 = 0,
  PRIORITY_1,
  PRIORITY_2,
  PRIORITY_3,
  PRIORITY_4,
  PRIORITY_5,
  PRIORITY_6,
  PRIORITY_7,
  PRIORITY_8,
  PRIORITY_9,
  PRIORITY_10,
  PRIORITY_11,
  PRIORITY_12,
  PRIORITY_13,
  PRIORITY_14,
  PRIORITY_15,
  PRIORITY_16,
  PRIORITY_17,
  PRIORITY_18,
  PRIORITY_19,
  PRIORITY_20,
  PRIORITY_21,
  PRIORITY_22,
  PRIORITY_23,
  PRIORITY_24,
  PRIORITY_25,
  PRIORITY_26,
  PRIORITY_27,
  PRIORITY_28,
  PRIORITY_29,
  PRIORITY_30,
  PRIORITY_31
} IMCTFD_CANFD_CHANNEL_PRIORITIES;

typedef enum IMCTFD_CANFD_FIFO_DEPTH {
  DEPTH_1 = 0,
  DEPTH_2,
  DEPTH_3,
  DEPTH_4,
  DEPTH_5,
  DEPTH_6,
  DEPTH_7,
  DEPTH_8,
  DEPTH_9,
  DEPTH_10,
  DEPTH_11,
  DEPTH_12,
  DEPTH_13,
  DEPTH_14,
  DEPTH_15,
  DEPTH_16,
  DEPTH_17,
  DEPTH_18,
  DEPTH_19,
  DEPTH_20,
  DEPTH_21,
  DEPTH_22,
  DEPTH_23,
  DEPTH_24,
  DEPTH_25,
  DEPTH_26,
  DEPTH_27,
  DEPTH_28,
  DEPTH_29,
  DEPTH_30,
  DEPTH_31,
  DEPTH_32
} IMCTFD_CANFD_FIFO_DEPTH;

typedef enum IMCTFD_CANFD_PLSIZE {
  PLSIZE_0 = 0,
  PLSIZE_1,
  PLSIZE_2,
  PLSIZE_3,
  PLSIZE_4,
  PLSIZE_5,
  PLSIZE_6,
  PLSIZE_7
} IMCTFD_CANFD_PLSIZE;

typedef enum IMCTFD_CANFD_TS {
  TIMESTAMP_OFF = 0,
  TIMESTAMP_ON
} IMCTFD_CANFD_TS;

typedef enum IMCTFD_RXTX {
  FIFO_RX = 0,
  FIFO_TX
} IMCTFD_RXTX;

typedef enum IMCTFD_FILTER_TYPE {
  NORMAL_FILTERING = 0,
  SID12,
  CATCH_ALL
} IMCTFD_FILTER_TYPE;

typedef enum WRITE_ACTION {
  TX_SEND = 0,
  TX_QUEUE
} IMCTFD_WRITE_ACTION;

typedef enum IMCTFD_CANFD_FILTERS {
  FILTER_0 = 0,
  FILTER_1,
  FILTER_2,
  FILTER_3,
  FILTER_4,
  FILTER_5,
  FILTER_6,
  FILTER_7,
  FILTER_8,
  FILTER_9,
  FILTER_10,
  FILTER_11,
  FILTER_12,
  FILTER_13,
  FILTER_14,
  FILTER_15,
  FILTER_16,
  FILTER_17,
  FILTER_18,
  FILTER_19,
  FILTER_20,
  FILTER_21,
  FILTER_22,
  FILTER_23,
  FILTER_24,
  FILTER_25,
  FILTER_26,
  FILTER_27,
  FILTER_28,
  FILTER_29,
  FILTER_30,
  FILTER_31
} IMCTFD_CANFD_FILTERS;

typedef struct CANFD_message_t {
  uint32_t id = 0;          // can identifier
  uint32_t timestamp = 0;   // time when message arrived
  uint8_t filthit = 0; // the filter that the message came in on
  uint8_t objID = 0; // internally used, to allow object tracking in a centralized circular buffer
  uint8_t len = 8;      // length of data
  uint8_t buf[64] = { 0 };       // data
  uint8_t fifo = 0;       // used to identify fifo reception
  uint8_t bus = 0;      // used to identify where the message came from when events() is used.
  uint8_t sequence = 0; // sequence of TX for TEF receipts (not same as seq)
  struct {
    bool extended = 0; // identifier is extended (29-bit)
    bool remote = 0;  // remote transmission request packet type
    bool overrun = 0; // message overrun
    bool reserved = 0;
    bool esi = 0;
    bool sid12 = 0;
  } flags;
  bool seq = 0;         // sequential frames
  bool fdf = 1;          // FD Frame; distinguishes between CAN and CAN FD formats
  bool rrs = 0;             // 12bit addressing in FD mode
  bool brs = 0;        // baud rate switching
} CANFD_message_t;

typedef void (*_FIFO_ptr)(const CANFD_message_t &msg); /* fifo / global callback */

class IMCTFD {

  public:
    IMCTFD(SPIClass& _port, uint8_t _miso, uint8_t _mosi, uint8_t _sck, uint8_t _cs, uint8_t _interrupt, uint32_t _mhz = 20000000);
    void onReceive(IMCTFD_CANFD_FIFO_CHANNELS channel, _FIFO_ptr handler); /* individual FIFO callback functions */
    void onReceive(_FIFO_ptr handler); /* global callback function */
    void begin();
    void setListenOnly(bool mode = 1);
    bool setOpMode(CAN_OPERATION_MODE mode, bool store = 1);
    CAN_OPERATION_MODE getOpMode();
    int8_t setBaudRate(uint32_t bitrate, uint32_t dataMultiplier = 0);
    void enableFIFOInterrupt(const IMCTFD_CANFD_FIFO_CHANNELS &channel, bool status = 1);
    void disableFIFOInterrupt(const IMCTFD_CANFD_FIFO_CHANNELS &channel) { enableFIFOInterrupt(channel, 0); }
    int write(const CANFD_message_t &msg, IMCTFD_CANFD_FIFO_CHANNELS channel = (IMCTFD_CANFD_FIFO_CHANNELS)0);
    int read(CANFD_message_t &msg, IMCTFD_CANFD_FIFO_CHANNELS channel = (IMCTFD_CANFD_FIFO_CHANNELS)0);
    void enableTimeStamping(uint16_t prescaler = 39, bool enable = 1, /* optional */ bool TSEOF = 0, bool TSRES = 0);
    void disableTimeStamping() { enableTimeStamping(0,0); }
    bool abortTransmission(const IMCTFD_CANFD_FIFO_CHANNELS channel = (IMCTFD_CANFD_FIFO_CHANNELS)0);
    bool configureTXQ(IMCTFD_CANFD_PLSIZE pls = PLSIZE_7, IMCTFD_CANFD_FIFO_DEPTH deep = DEPTH_6, IMCTFD_CANFD_CHANNEL_PRIORITIES priority = PRIORITY_15);
    bool configureTXQ(uint8_t dataLen, IMCTFD_CANFD_FIFO_DEPTH deep = DEPTH_6, IMCTFD_CANFD_CHANNEL_PRIORITIES priority = PRIORITY_15) { return configureTXQ((IMCTFD_CANFD_PLSIZE)conversionsDLC(6, dataLen, 1), deep, priority); }
    bool configureTEF(IMCTFD_CANFD_FIFO_DEPTH deep, IMCTFD_CANFD_TS ts = TIMESTAMP_ON);
    bool configureFIFO(IMCTFD_CANFD_FIFO_CHANNELS channel, IMCTFD_RXTX rx_tx, IMCTFD_CANFD_PLSIZE pls, IMCTFD_CANFD_FIFO_DEPTH deep, IMCTFD_CANFD_CHANNEL_PRIORITIES priority, IMCTFD_CANFD_TS ts);
    bool configureFIFO(IMCTFD_CANFD_FIFO_CHANNELS channel, IMCTFD_RXTX rx_tx, uint8_t dataLen, IMCTFD_CANFD_FIFO_DEPTH deep, IMCTFD_CANFD_CHANNEL_PRIORITIES priority, IMCTFD_CANFD_TS ts) { return configureFIFO(channel, rx_tx, (IMCTFD_CANFD_PLSIZE)conversionsDLC(6, dataLen, 1), deep, priority, ts); }
    uint16_t calculateRAM();
    void currentConfig();
    void test();
    void pinMode(uint8_t pin, uint8_t mode);
    bool digitalRead(uint8_t pin);
    void digitalWrite(uint8_t pin, uint8_t state);
    bool setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, IMCTFD_FILTER_TYPE type);
    bool setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id, IMCTFD_FILTER_TYPE type = NORMAL_FILTERING);
    bool setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, IMCTFD_FILTER_TYPE type = NORMAL_FILTERING);
    bool setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, uint32_t id2, IMCTFD_FILTER_TYPE type = NORMAL_FILTERING);
    bool setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, uint32_t id2, uint32_t id3, IMCTFD_FILTER_TYPE type = NORMAL_FILTERING);
    bool setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, IMCTFD_FILTER_TYPE type = NORMAL_FILTERING);
    bool setFIFOFilterRange(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, IMCTFD_FILTER_TYPE type = NORMAL_FILTERING);
    void currentFilters();
    void enhanceFilter(IMCTFD_CANFD_FILTERS filter);
    void disableFilter(IMCTFD_CANFD_FILTERS filter);
    void enableSPICRC(bool en = 1);
    void disableSPICRC() { enableSPICRC(0); }
    bool setQueue(IMCTFD_CANFD_FIFO_CHANNELS channel, IMCTFD_WRITE_ACTION action = TX_QUEUE);
    static Circular_Buffer<uint8_t, IMCTFD_RX_BUFSIZE, sizeof(CANFD_message_t)> rxBuffer; /* create an array buffer of struct size, IMCTFD_RX_BUFSIZE levels deep. */

    /* below should be :private, :public for development use */
    void enableECC(bool en = 1);
    void disableECC() { enableECC(0); }
    void configureCiCON(uint32_t value);
    bool _setBit(uint16_t address, uint8_t bit, bool value = 1);
    bool _getBit(uint16_t address, uint8_t bit);
    uint16_t calcCRC16(uint8_t* data, uint16_t size); /* crc calculations for the SPI transfers (if used(enableSPICRC())) */
    bool _readArrayCRC(uint16_t address, uint8_t *buffer, uint16_t size); /* reads an array of bytes from the chip at the specified address */
    void _writeArrayCRC(uint16_t address, uint8_t *buffer, uint16_t size); /* writes an array of bytes to the chip at the specified address */
    uint32_t _readWord(uint16_t address); /* reads a uint16_t to the chip at the specified address */
    void _writeWord(uint16_t address, uint32_t data); /* writes a uint32_t to the chip at the specified address */
    uint16_t _readHalfWord(uint16_t address); /* reads a uint16_t to the chip at the specified address */
    void _writeHalfWord(uint16_t address, uint32_t data); /* writes a uint16_t to the chip at the specified address */
    void initRAM(); /* clears the RAM of the chip */
    void _writeByte(uint16_t address, uint8_t txData); /* writes a byte to the chip at the specified address */
    uint8_t _readByte(uint16_t address); /* reads a byte to the chip at the specified address */
    void reset();
    void distribute(bool state = 1) { distribution = state; }
    static void IMCTFD_routines(); /* currently for callbacks via queues */
    static void process(); /* threading uses this static function to call the objects to process IMCTFD_isr() */

  private:
    SPIClass *port; /* SPI port used by the Object */
    SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE0); /* default SPISettings if omitted from constructor */
    uint8_t miso, mosi, sck, cs, interrupt, write_mode, read_mode; /* constructor updated values */
    uint8_t conversionsDLC(uint8_t cmd, uint8_t value, bool fdBit); /* multiple conversion DLC function */
    void _transfer(uint8_t *txData, uint8_t *rxData, uint16_t size); /* SPI communication */
    void _transfer(uint16_t address, uint8_t *txData, uint8_t *rxData, uint16_t size); /* SPI communication */
    uint32_t configured_fifos = 0; /* we keep track of configured fifos locally, for efficient scanning purposes */
    uint32_t tx_interrupts = 0; /* software toggling TX interrupt, due to limitation of MCP chip causes asserted pin to remain set after all transmissions end... */
    uint32_t tx_pending = 0; /* user has set the FIFO as queue only, and will later trigger it to send */
    uint32_t _currentBitrate = 500000; /* default bitrate if user did not set a baud rate */
    uint8_t _currentMultiplier = 4; /* default multiplier if user did not set a baud rate */
    void filterProcessing(uint8_t filter, uint8_t channel, uint32_t id, uint32_t mask);
    uint16_t enhancements[32] = { 0 }; // enhancements en (15), multi/range based (0)
    uint32_t id_enhancements[32][5] = { { 0 }, { 0 } }; // to store up to 5 id entries
    bool TXQconfigured = 0, TEFconfigured = 0; /* on POR, enabled by default, did user enable it? */
    static IMCTFD* _threadObjects[4]; /* we create a 4 object pointer array, used in static functions (process) */
    static uint8_t _totalObjects; /* total amount of objects instantiated in the class */
    static bool threading_started; /* one shot threading trigger handles multiple objects */
    uint8_t spi_port_value = 0; /* used to identify the spi port of multiple objects from static functions */
    _FIFO_ptr _FIFOhandlers[34] = { nullptr }; /* individual FIFO handlers, FIFO == 1-31, TXQ == 32, TEF == 33, so we need 34 total. 0 is used for automatic scans in read/write functions */
    _FIFO_ptr _globalhandler = nullptr; /* global mailbox handler */
    uint8_t currentObject = 0; /* specifies a value used for a specific lock for each object, incremented by _totalObjects in constructor */
    void IMCTFD_isr(); /* threading based emulated isr, based on pin state */
    uint32_t overflow_flags = 0;
    volatile int distribution = 0;
    void packet_distribution(CANFD_message_t &msg);
    bool init_filters = 0;
};

extern void IMCTFD_output(const CANFD_message_t &msg);
extern uint16_t IMCTFD_events();

#endif
