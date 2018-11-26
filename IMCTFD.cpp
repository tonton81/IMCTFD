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

#include "Arduino.h"
#include "IMCTFD.h"
#include "SPI.h"
#include <algorithm>

#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
#include "TeensyThreads.h"
Threads::Mutex SPI_LOCK[3];
#endif

#if defined(KINETISL)
IntervalTimer IMCTFD_iTimer;
#endif

IMCTFD* IMCTFD::_threadObjects[4] = { nullptr };
uint8_t IMCTFD::_totalObjects = 0;
bool IMCTFD::threading_started = 0;

Circular_Buffer<uint8_t, IMCTFD_RX_BUFSIZE, sizeof(CANFD_message_t)> IMCTFD::rxBuffer;

IMCTFD::IMCTFD(SPIClass& _port, uint8_t _miso, uint8_t _mosi, uint8_t _sck, uint8_t _cs, uint8_t _interrupt, uint32_t _mhz) {
  port = &_port;
  port->setMISO(miso = _miso);
  port->setMOSI(mosi = _mosi);
  port->setSCK(sck = _sck);
  port->setCS(cs = _cs);
  ::pinMode(cs, OUTPUT);
  ::digitalWriteFast(cs,HIGH); 
  port->begin();
  interrupt = _interrupt;
  ::pinMode(interrupt, INPUT);
  settings = SPISettings(((constrain(_mhz, 4000000UL, 20000000UL) != _mhz) ? 8000000UL : _mhz), MSBFIRST, SPI_MODE0);
  read_mode = cINSTRUCTION_READ;
  write_mode = cINSTRUCTION_WRITE;
  currentObject = _totalObjects;
  _threadObjects[_totalObjects++] = this;
  if ( port == &SPI ) spi_port_value = 0;
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL)
  else if ( port == &SPI1 ) spi_port_value = 1;
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else if ( port == &SPI2 ) spi_port_value = 2;
#endif
}

void IMCTFD::process() {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  while(1) {
#endif
    for ( uint8_t i = 0; i < _totalObjects; i++ ) {
      if ( _threadObjects[i] ) {
#if defined(KINETISL)
        if ( _threadObjects[i]->busy_flag ) return;
#endif
        if ( !digitalReadFast(_threadObjects[i]->interrupt)) {
          _threadObjects[i]->IMCTFD_isr();
        }
      }
    }
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    threads.yield();
  }
#else
  IMCTFD::IMCTFD_routines();
#endif
}

void __attribute__((weak)) IMCTFD_output(const CANFD_message_t &msg) {
}

uint16_t __attribute__((weak)) IMCTFD_events() {
  return 0;
}

void IMCTFD::IMCTFD_routines() {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  while(1) {
#endif
    if ( rxBuffer.size() ) {
      CANFD_message_t msg;
      uint8_t buf[sizeof(CANFD_message_t)];
      rxBuffer.pop_front(buf, sizeof(CANFD_message_t));
      memmove(&msg, buf, sizeof(CANFD_message_t));
      if ( _threadObjects[msg.objID]->_globalhandler ) _threadObjects[msg.objID]->_globalhandler(msg);
      if ( _threadObjects[msg.objID]->_FIFOhandlers[msg.fifo] ) _threadObjects[msg.objID]->_FIFOhandlers[msg.fifo](msg);
    }
    IMCTFD_events();
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    threads.yield();
  }
#endif
}

void IMCTFD::IMCTFD_isr() {
  CANFD_message_t msg;
  uint32_t _CiINT = _readWord(cREGADDR_CiINT);
/*
  Serial.print("INTERRUPT INTERRUPT INTERRUPT INTERRUPT INTERRUPT INTERRUPT INT:");
  Serial.print(_CiINT);
  Serial.print(" VEC: ");
  Serial.print(_readWord(cREGADDR_CiVEC));
Serial.print(" D0: ");
uint32_t d0 = _readWord(cREGADDR_CiBDIAG0);
Serial.print(d0);
Serial.print(" D1: ");
uint32_t d1 = _readWord(cREGADDR_CiBDIAG1);
Serial.print(d1);
*/
/*
static bool setOnce = 1;
if ( setOnce ) {
setOnce = 0;
if ( d0) _writeWord(cREGADDR_CiBDIAG0,0);
if ( d1) _writeWord(cREGADDR_CiBDIAG1,0);
}
*/
/*
if ( d0) {
 _writeWord(cREGADDR_CiBDIAG0,0);
 _writeWord(cREGADDR_CiBDIAG1,0);
}
  Serial.print(" Q: ");
  Serial.println(rxBuffer.size());
*/

//    Serial.print("FIFOSTA FIFO1: "); Serial.println(     (_readWord(cREGADDR_CiFIFOSTA + (3 * CiFIFO_OFFSET)) & 0x1F00) >> 8 );
//    Serial.print("FIFO UA FIFO1: "); Serial.println(     (_readWord(cREGADDR_CiFIFOUA + (3 * CiFIFO_OFFSET)))             );

  if ( _CiINT & 0xFFFF ) {
    if ( _CiINT & 0x8 ) { /* Operation Mode Change Interrupt Flag bit (bit19) */
      _CiINT &= ~(1UL << 3);
    }
    if ( _CiINT & 0x2000 ) { /* CAN Bus Error Interrupt Flag bit (bit29) */
      _CiINT &= ~(1UL << 13);
    }
    if ( _CiINT & 0x100 ) { /* ECC Error Interrupt Flag bit (bit24) */
      _CiINT &= ~(1UL << 8);
    }
    if ( _CiINT & 0x1000 ) { /* System Error Interrupt Flag bit (bit28) */
      _CiINT &= ~(1UL << 12);
    }
    if ( _CiINT & 0x4 ) { /* Time Base Counter Overflow Interrupt Flag bit (bit18) */
      _CiINT &= ~(1UL << 2);
    }
    if ( _CiINT & 0x8000 ) { /* Invalid Message Interrupt Enable bit (bit31) */
      _CiINT &= ~(1UL << 15);
    }
    if ( _CiINT & 0x4000 ) { /* Bus Wake Up Interrupt Flag bit (bit30) */
      _CiINT &= ~(1UL << 14);
    }
    
    if ( _CiINT & 0x800 ) { /* Receive Object Overflow Interrupt Flag bit (bit27) */
      /* bit27 does not need to be set to view the interrupt, just as well mightes well not assert interrupt pin.. */
      /* we only need to scan up to what bits are flagged */
      uint32_t _overflows = _readWord(cREGADDR_CiRXOVIF);
      overflow_flags &= ~(0xFFFFFFFF << (32 - __builtin_clz(_overflows))); /* we clear upper bits that have been cleared */
      for ( uint8_t i = 1; i < (32 - __builtin_clz(_overflows)); i++ ) { /* we compare last update to this to we notify user once */
        if ( _overflows & (1UL << i) ) {
          if ( !(overflow_flags & (1UL << i)) ) { /* notify user one time */
            /* let user know (event?) that FIFO(i) has overflowed on reception */
            overflow_flags |= (1UL << i);
          }
          continue;
        }
        else {
          overflow_flags &= ~(1UL << i);
        }
      }
    }
    else overflow_flags = 0;

    _writeHalfWord(cREGADDR_CiINT, _CiINT & 0xFFFF); /* deassert flags */
  }

  if ( _CiINT & 0x10 ) { /* We have a reception flag for an interrupt enabled TEF FIFO */
    uint16_t user_address = (((uint16_t)(cINSTRUCTION_READ << 12)) | (_readWord(cREGADDR_CiTEFUA) + cRAMADDR_START)) & 0xFFF;
    msg.len = conversionsDLC(2, _readByte(user_address + 4) & 0xF, 0);
    uint8_t buffer[12 + msg.len];
    _transfer(user_address, nullptr, buffer, 12 + msg.len);
    msg.id = ((uint32_t)buffer[0] | buffer[1] << 8 | buffer[2] << 16 | buffer[3] << 24);
    if ( msg.id & (1UL << 29) ) msg.id = (msg.id & 0x7FF) | (1UL << 11);
    else msg.id &= 0x1FFFFFFF;
    msg.timestamp = (uint32_t)buffer[8] | buffer[9] << 8 | buffer[10] << 16 | buffer[11] << 24;
    msg.flags.remote = buffer[4] & 0x20;
    msg.fdf = buffer[4] & 0x80;
    msg.flags.extended = buffer[4] & 0x10;
    msg.brs = buffer[4] & 0x40;
    msg.fifo = (IMCTFD_CANFD_FIFO_CHANNELS)TEF;
    msg.flags.esi = buffer[5] & 0x1;
    msg.sequence = ((buffer[5] & 0xFE) >> 1);
    msg.objID = currentObject;
    _setBit(cREGADDR_CiTEFCON, 8, 1); /* UINC */
    msg.flags.overrun = _setBit(cREGADDR_CiTEFSTA, 3, 0); /* clear overflow flag if set */
    uint8_t buf[sizeof(CANFD_message_t)];
    memmove(buf, &msg, sizeof(CANFD_message_t));
    rxBuffer.push_back(buf, sizeof(CANFD_message_t));
  }

  if ( _CiINT & 0x2 ) { /* We have a reception flag for an interrupt enabled RX FIFO */
    msg.fifo = (_readWord(cREGADDR_CiVEC) & 0x7F000000) >> 24;
    uint32_t fifoCon = _readWord(cREGADDR_CiFIFOCON + (msg.fifo * CiFIFO_OFFSET));
    uint16_t user_address = (((uint16_t)(cINSTRUCTION_READ << 12)) | (_readWord(cREGADDR_CiFIFOUA + (msg.fifo * CiFIFO_OFFSET)) + cRAMADDR_START)) & 0xFFF;
    msg.len = conversionsDLC(2, _readByte(user_address + 4) & 0xF, 0);
    uint8_t buffer[12 + msg.len];
    _transfer(user_address, nullptr, buffer, 12 + msg.len);
    msg.id = ((uint32_t)buffer[0] | buffer[1] << 8 | buffer[2] << 16 | buffer[3] << 24);
    if ( msg.id & (1UL << 29) ) msg.id = (msg.id & 0x7FF) | (1UL << 11);
    else msg.id &= 0x1FFFFFFF;
    msg.timestamp = (uint32_t)buffer[8] | buffer[9] << 8 | buffer[10] << 16 | buffer[11] << 24;
    msg.flags.remote = buffer[4] & 0x20;
    msg.brs = buffer[4] & 0x40;
    msg.flags.esi = buffer[5] & 0x1;
    msg.sequence = ((buffer[5] & 0xFE) >> 1);
    msg.fdf = buffer[4] & 0x80;
    msg.flags.extended = buffer[4] & 0x10;
    msg.filthit = (buffer[5] & 0xF8) >> 3;
    msg.objID = currentObject;
    memmove(&msg.buf[0], &buffer[0] + ((fifoCon & 0x20) ? 12UL : 8UL), msg.len);
    _setBit(cREGADDR_CiFIFOCON + (msg.fifo * CiFIFO_OFFSET), 8, 1); /* UINC */
    msg.flags.overrun = _setBit(cREGADDR_CiFIFOSTA + (msg.fifo * CiFIFO_OFFSET), 3, 0); /* clear overflow flag if set */
    uint8_t buf[sizeof(CANFD_message_t)];
    memmove(buf, &msg, sizeof(CANFD_message_t));
    if ( enhancements[msg.filthit] & 0x8000 ) { /* enhancement is enabled? */
      if ( enhancements[msg.filthit] & 0x1 ) { /* if it's range based */
        if ( msg.id >= id_enhancements[msg.filthit][0] && msg.id <= id_enhancements[msg.filthit][1] ) {
          rxBuffer.push_back(buf, sizeof(CANFD_message_t));
        }
      }
      else { /* if it's multi-id based */
        for ( uint8_t i = 0; i < ((enhancements[msg.filthit] & 0xE) >> 1); i++ ) {
          if ( id_enhancements[msg.filthit][i] == msg.id ) {
            rxBuffer.push_back(buf, sizeof(CANFD_message_t));
            break; /* stop scanning rest when found */
          }
        }
      }
    }
    else { /* if not enhanced */
      rxBuffer.push_back(buf, sizeof(CANFD_message_t));
    }
    if ( distribution ) packet_distribution(msg);
    IMCTFD_output(msg); /* allow external libraries to capture all interrupt frames */
  }
  
  if ( _CiINT & 0x1 ) { /* We have a transmission flag for an interrupt enabled TX FIFO */
    if ( (_readWord(cREGADDR_CiVEC) & 0x1F0000) < 32 ) { /* check if bits are related to channel specific only! */
      _setBit(cREGADDR_CiFIFOCON + ((( _readWord(cREGADDR_CiVEC) & 0x1F0000 ) >> 16) * CiFIFO_OFFSET), 2, 0);
    }
  }
}

void IMCTFD::packet_distribution(CANFD_message_t &msg) {
  uint8_t buf[sizeof(CANFD_message_t)], _filthit = msg.filthit, _fifoch = msg.fifo;
  uint32_t _mask = 0;
  for ( uint8_t filter = 0; filter < 32; filter++ ) { /* update associated filters */
    if ( (enhancements[filter] & 0x4000) && (((enhancements[filter] & 0x3F0) >> 4) != _fifoch) && (filter != _filthit) ) {
      _mask = _readWord(cREGADDR_CiMASK + ( filter * 8 ));
      _mask = (( _mask & 0x20000000 ) ? ((_mask & 0x7FF) | (1UL << 11)) : (_mask & 0x1FFFFFFF)); /* 12 bit standard or regular frames? */
      if ( (msg.id & _mask) == (id_enhancements[filter][0] & _mask) ) {
        if ( enhancements[msg.filthit] & 0x8000 ) { /* enhancement is enabled */
          if ( enhancements[filter] & 0x1 ) { /* if it's range based */
            if ( msg.id >= id_enhancements[filter][0] && msg.id <= id_enhancements[filter][1] ) {
              msg.fifo = (enhancements[filter] & 0x3F0) >> 4;
              msg.filthit = filter;
              memmove(buf, &msg, sizeof(CANFD_message_t));
              rxBuffer.push_back(buf, sizeof(CANFD_message_t));
              msg.fifo = _fifoch;
              msg.filthit = _filthit;
            }
          }
          else { /* if it's multi-id based */
            for ( uint8_t i = 0; i < ((enhancements[filter] & 0xE) >> 1); i++ ) {
              if ( id_enhancements[filter][i] == msg.id ) {
                msg.fifo = (enhancements[filter] & 0x3F0) >> 4;
                msg.filthit = filter;
                memmove(buf, &msg, sizeof(CANFD_message_t));
                rxBuffer.push_back(buf, sizeof(CANFD_message_t));
                msg.fifo = _fifoch;
                msg.filthit = _filthit;
                break;
              }
            }
          }
        }
        else {
          msg.fifo = (enhancements[filter] & 0x3F0) >> 4;
          msg.filthit = filter;
          memmove(buf, &msg, sizeof(CANFD_message_t));
          rxBuffer.push_back(buf, sizeof(CANFD_message_t));
          msg.fifo = _fifoch;
          msg.filthit = _filthit;
        }
      }
    }
  }
}

void IMCTFD::onReceive(IMCTFD_CANFD_FIFO_CHANNELS channel, _FIFO_ptr handler) {
  IMCTFD::_FIFOhandlers[channel] = handler;
}

void IMCTFD::onReceive(_FIFO_ptr handler) {
  IMCTFD::_globalhandler = handler;
}

void IMCTFD::enableSPICRC(bool en) {
  read_mode = ((en) ? cINSTRUCTION_READ_CRC : cINSTRUCTION_READ);
  write_mode = ((en) ? cINSTRUCTION_WRITE_CRC : cINSTRUCTION_WRITE);
}

void IMCTFD::enableECC(bool en) {
  _setBit(cREGADDR_ECCCON, 1, en);
}

int8_t IMCTFD::setBaudRate(uint32_t bitrate, uint32_t dataMultiplier) {
  if ( !dataMultiplier ) dataMultiplier = (2000000UL / bitrate);
  _currentBitrate = bitrate;
  _currentMultiplier = dataMultiplier;
  int32_t frequency = 40000000 ;
  uint8_t dSeg1 = 0, dSeg2 = 0;
  uint8_t dSJW = 0;
  uint16_t arbPhaseSeg1 = 0;
  uint8_t arbPhaseSeg2 = 0;
  uint8_t nSJW = 0;
  const uint16_t arbPhaseSeg1Max = 256;
  const uint8_t  arbPhaseSeg2Max = 128;
  const uint16_t dSeg1Max = 32;
  const uint8_t  dSeg2Max = 16;
  const uint32_t maxTQCount = arbPhaseSeg1Max + arbPhaseSeg2Max + 1;
  uint32_t BRP = 256;
  uint32_t smallestError = UINT32_MAX;
  uint32_t bestBRP = 1;
  uint32_t bestTQCount = 4;
  uint32_t TQCount = frequency / bitrate / BRP;
  while ((TQCount <= (arbPhaseSeg1Max + arbPhaseSeg2Max + 1)) && ( BRP )) {
    if ((TQCount >= 4) && (TQCount <= maxTQCount)) {
      const uint32_t error = frequency - bitrate * TQCount * BRP;
      if (error <= smallestError) {
        smallestError = error;
        bestBRP = BRP;
        bestTQCount = TQCount;
      }
    }
    if ((TQCount >= 3) && (TQCount < maxTQCount)) {
      const uint32_t error = bitrate * (TQCount + 1) * BRP - frequency;
      if ( error <= smallestError ) {
        smallestError = error;
        bestBRP = BRP;
        bestTQCount = TQCount + 1;
      }
    }
    TQCount = ( !--BRP ) ? (maxTQCount + 1) : (frequency / bitrate / BRP);
  }

  uint32_t tSeg2 = bestTQCount / 5;
  if ( !tSeg2 ) tSeg2 = 1;
  else if (tSeg2 > arbPhaseSeg2Max) tSeg2 = arbPhaseSeg2Max ;
  uint32_t tSeg1 = bestTQCount - tSeg2 - 1; // Sync Seg / ;
  if (tSeg1 > arbPhaseSeg1Max) {
    tSeg2 += tSeg1 - arbPhaseSeg1Max ;
    tSeg1 = arbPhaseSeg1Max ;
  }
  arbPhaseSeg1 = (uint16_t)tSeg1;
  arbPhaseSeg2 = (uint8_t)tSeg2;
  nSJW = arbPhaseSeg2 ; // Allways 1 <= SJW <= 128, and SJW <= arbPhaseSeg2

  if (dataMultiplier != 1) {
    uint32_t nominalTQCount = 1 + tSeg1 + tSeg2 ;
    uint32_t dataTQCount = nominalTQCount / dataMultiplier;
    uint32_t dataPS2 = dataTQCount / 5;
    if (dataPS2 == 0) dataPS2 = 1;
    else if (dataPS2 > dSeg2Max) dataPS2 = dSeg2Max;
    uint32_t dataPS1 = dataTQCount - dataPS2 - 1;
    if (dataPS1 > dSeg1Max) {
      dataPS2 += dataPS1 - dSeg1Max;
      dataPS1 = dSeg1Max;
    }
    dSeg1 = (uint8_t)dataPS1;
    dSJW = dSeg2 = (uint8_t)dataPS2;
  }

  if ( !setOpMode(CAN_CONFIGURATION_MODE) ) return 0;

  uint32_t _CiNBTCFG = (uint32_t)((bestBRP - 1) << 24) | ((arbPhaseSeg1 - 1) << 16 ) | ((arbPhaseSeg2 - 1) << 8 ) | ((nSJW -1) << 0);
  _writeWord(cREGADDR_CiNBTCFG, _CiNBTCFG);

  uint32_t _CiDBTCFG = (uint32_t)((bestBRP - 1) << 24) | ((dSeg1 - 1) << 16 ) | ((dSeg2 - 1) << 8 ) | ((dSJW -1) << 0);
  if ( bitrate < 500000UL && (bitrate * dataMultiplier) < 1000000UL ) _CiDBTCFG |= (1UL << 24);
  _writeWord(cREGADDR_CiDBTCFG, _CiDBTCFG);

  uint32_t _cREGADDR_CiTDC = (_readWord(cREGADDR_CiTDC) & 0xFFFC80FF) | (dSeg1 << 8) | (2UL << 16) | (1UL << 24);
  _writeWord(cREGADDR_CiTDC, _cREGADDR_CiTDC);

  if ( _readWord(cREGADDR_CiNBTCFG) == _CiNBTCFG && _readWord(cREGADDR_CiDBTCFG) == _CiDBTCFG  && _readWord(cREGADDR_CiTDC) == _cREGADDR_CiTDC ) {
    setOpMode(CAN_CONFIGURATION_MODE, 0);
    return 1;
  }

  setOpMode(CAN_CONFIGURATION_MODE, 0);
  return 0;
}

bool IMCTFD::setOpMode(CAN_OPERATION_MODE modeRequest, bool store) {
  static CAN_OPERATION_MODE restore_config = CAN_NORMAL_MODE;
  uint8_t data = _readByte(cREGADDR_CiCON + 3) & 0xF8;
  uint32_t timeout = 0;
  if ( store ) {
    restore_config = getOpMode();
    if ( restore_config != CAN_CONFIGURATION_MODE ) {
      _writeByte(cREGADDR_CiCON + 3, data | CAN_CONFIGURATION_MODE);
      timeout = millis();
      while ( getOpMode() != CAN_CONFIGURATION_MODE ) {
        if ( millis() - timeout > 1000 ) return 0;
      }
    }
    _writeByte(cREGADDR_CiCON + 3, data | modeRequest); /* set new mode */
    timeout = millis();
    while ( getOpMode() != modeRequest ) {
      if ( millis() - timeout > 1000 ) return 0;
    }
  }
  else {
    if ( restore_config == CAN_CONFIGURATION_MODE ) return 1; /* was previously in config mode */
    _writeByte(cREGADDR_CiCON + 3, data | restore_config);
    timeout = millis();
    while ( getOpMode() != restore_config ) {
      if ( millis() - timeout > 1000 ) return 0;
    }
  }
  return 1;
}

void IMCTFD::initRAM() {
  if ( !setOpMode(CAN_CONFIGURATION_MODE) ) return;
  uint8_t clearFIFOram[2048] = { 0 };
  std::fill( clearFIFOram, clearFIFOram + 2048, 0xFF );
  _transfer(((uint16_t)(cINSTRUCTION_WRITE << 12) | cRAMADDR_START), clearFIFOram, nullptr, 2048);
  setOpMode(CAN_CONFIGURATION_MODE, 0);
}

int IMCTFD::read(CANFD_message_t &msg, IMCTFD_CANFD_FIFO_CHANNELS channel) {
  uint32_t check_fifo_rx_msgs = _readWord(cREGADDR_CiRXIF);
  uint32_t fifoCon = 0;
  static uint8_t spread_reads = 1;
  bool found = 0;
  if ( !channel ) { /* spread out fifo scans for messages */
    for ( uint8_t i = 0; i < __builtin_popcount(configured_fifos) + 1; i++ ) {
      spread_reads++;
      if ( spread_reads > 31 ) spread_reads = 1;
      if ( !(configured_fifos & (1UL << spread_reads)) ) { /* skip unconfigured fifos */
        spread_reads = 0;
        continue;
      }
      fifoCon = _readWord(cREGADDR_CiFIFOCON + (spread_reads * CiFIFO_OFFSET));
      if ( (fifoCon & 0x80) ) continue; /* This is a TX FIFO! */
      if ( (check_fifo_rx_msgs & (1UL << spread_reads)) ) continue; /* make sure we're reading a non-interrupt FIFO */
      if ( !_getBit(cREGADDR_CiFIFOSTA + (spread_reads * CiFIFO_OFFSET), 0) ) continue; /* make sure it's not empty */
      found = 1;
      break;
    }
    if ( !found ) return 0;
    channel = (IMCTFD_CANFD_FIFO_CHANNELS)spread_reads;
  }
  else if ( channel == TEF ) {
    if ( !_getBit(cREGADDR_CiCON, 19) ) return 0; /* TEF is disabled */
    if ( _getBit(cREGADDR_CiTEFCON, 0) ) return 0; /* make sure it's not interrupt enabled to access */
    if ( !_getBit(cREGADDR_CiTEFSTA, 0) ) return 0; /* make sure it has at least 1 message to pull */
    uint16_t user_address = (((uint16_t)(cINSTRUCTION_READ << 12)) | (_readWord(cREGADDR_CiTEFUA) + cRAMADDR_START)) & 0xFFF;
    msg.len = conversionsDLC(2, _readByte(user_address + 4) & 0xF, 0);
    uint8_t buffer[12 + msg.len];
    _transfer(user_address, nullptr, buffer, 12 + msg.len);
    msg.id = ((uint32_t)buffer[0] | buffer[1] << 8 | buffer[2] << 16 | buffer[3] << 24);
    if ( msg.id & (1UL << 29) ) msg.id = (msg.id & 0x7FF) | (1UL << 12);
    else msg.id &= 0x1FFFFFFF;
    msg.timestamp = (uint32_t)buffer[8] | buffer[9] << 8 | buffer[10] << 16 | buffer[11] << 24;
    msg.flags.remote = buffer[4] & 0x20;
    msg.fdf = buffer[4] & 0x80;
    msg.fifo = (IMCTFD_CANFD_FIFO_CHANNELS)TEF;
    msg.flags.extended = buffer[4] & 0x10;
    msg.brs = buffer[4] & 0x40;
    msg.flags.esi = buffer[5] & 0x1;
    msg.sequence = ((buffer[5] & 0xFE) >> 1);
    _setBit(cREGADDR_CiTEFCON, 8, 1); /* UINC */
    msg.flags.overrun = _setBit(cREGADDR_CiTEFSTA, 3, 0); /* clear overflow flag if set */
    return 1;
  }
  else { /* channel specified */
    if ( !(configured_fifos & (1UL << channel)) ) return 0; /* skip unconfigured fifos */
    fifoCon = _readWord(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET));
    if ( (fifoCon & 0x80) ) return 0; /* This is a TX FIFO! */
    if ( (check_fifo_rx_msgs & (1UL << channel)) ) return 0; /* make sure we're reading a non-interrupt FIFO */
    if ( !_getBit(cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET), 0) ) return 0; /* make sure it's not empty */
  }
  msg.fifo = channel;
  uint16_t user_address = (((uint16_t)(cINSTRUCTION_READ << 12)) | (_readWord(cREGADDR_CiFIFOUA + (channel * CiFIFO_OFFSET)) + cRAMADDR_START)) & 0xFFF;
  msg.len = conversionsDLC(2, _readByte(user_address + 4) & 0xF, 0);
  uint8_t buffer[12 + msg.len];
  _transfer(user_address, nullptr, buffer, 12 + msg.len);
  msg.id = ((uint32_t)buffer[0] | buffer[1] << 8 | buffer[2] << 16 | buffer[3] << 24);
  if ( msg.id & (1UL << 29) ) msg.id = (msg.id & 0x7FF) | (1UL << 11);
  else msg.id &= 0x1FFFFFFF;
  msg.timestamp = (uint32_t)buffer[8] | buffer[9] << 8 | buffer[10] << 16 | buffer[11] << 24;
  msg.flags.remote = buffer[4] & 0x20;
  msg.fdf = buffer[4] & 0x80;
  msg.brs = buffer[4] & 0x40;
  msg.flags.esi = buffer[5] & 0x1;
  msg.sequence = ((buffer[5] & 0xFE) >> 1);
  msg.flags.extended = buffer[4] & 0x10;
  msg.filthit = (buffer[5] & 0xF8) >> 3;
  memmove(&msg.buf[0], &buffer[0] + ((fifoCon & 0x20) ? 12UL : 8UL), msg.len);
  _setBit(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), 8, 1); /* UINC */
  msg.flags.overrun = _setBit(cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET), 3, 0); /* clear overflow flag if set */
  return 1;
}

void IMCTFD::configureCiCON(uint32_t value) {
  /*
    bit 31-28 TXBWS<3:0> : Transmit Bandwidth Sharing bits
    bit 27 ABAT : Abort All Pending Transmissions bit
    bit 26-24 REQOP<2:0> : Request Operation Mode bits
    bit 23-21 OPMOD<2:0> : Operation Mode Status bits
    bit 20 TXQEN : Enable Transmit Queue bit
    bit 19 STEF : Store in Transmit Event FIFO bit
    bit 18 SERR2LOM : Transition to Listen Only Mode on System Error bit
    bit 17 ESIGM : Transmit ESI in Gateway Mode bit
    bit 16 RTXAT : Restrict Retransmission Attempts bit
    bit 15-13 Unimplemented
    bit 12 BRSDIS : Bit Rate Switching Disable bit
    bit 11 BUSY : CAN Module is Busy bit
    bit 10-9 WFT<1:0> : Selectable Wake-up Filter Time bits
    bit 8 WAKFIL : Enable CAN Bus Line Wake-up Filter bit
    bit 7 Unimplemented
    bit 6 PXEDIS : Protocol Exception Event Detection Disabled bit
    bit 5 ISOCRCEN: Enable ISO CRC in CAN FD Frames bit
    bit 4-0 DNCNT<4:0>: Device Net Filter Bit Number bits
  */
  if ( !setOpMode(CAN_CONFIGURATION_MODE) ) return;
  _writeWord(cREGADDR_CiCON, value);
  setOpMode(CAN_CONFIGURATION_MODE, 0);
}

bool IMCTFD::_readArrayCRC(uint16_t address, uint8_t *buffer, uint16_t size) {
  address = (address & 0xFFF) | (cINSTRUCTION_READ_CRC << 12); /* let user decide crc or not */
  uint8_t rx[size+5] = { 0 }, tx[size + 5] = { (uint8_t)(address >> 8), (uint8_t)(address), (((address & 0xFFF) >= 0x400 && (address & 0xFFF) <= 0xBFF) ? (uint8_t)(size >> 2) : (uint8_t)size) };
  uint16_t mcp2517fd_crc = 0, local_crc = 0;
  _transfer(tx, rx, size + 5);
  mcp2517fd_crc = (uint16_t)(rx[(size + 5) - 2] << 8) | rx[(size + 5) - 1];
  memmove(&rx[0], &tx[0], 3);
  local_crc = calcCRC16(rx, size + 3);
  if ( local_crc == mcp2517fd_crc ) {
    memmove(&buffer[0], &rx[0] + 3, size);
    return 1;
  }
  return 0;
}

uint8_t IMCTFD::_readByte(uint16_t address) {
  address = (address & 0xFFF) | (read_mode << 12); /* let user decide crc or not */
  if ( (address & 0x8000) ) { /* if CRC mode can run */
    uint8_t rx[4] = { 0 };
    _readArrayCRC((address & 0xFFF), rx, (((address & 0xFFF) >= 0x400 && (address & 0xFFF) <= 0xBFF) ? 4U : 1U));
    return rx[0];
  }
  uint8_t rx[3] = { 0 }, tx[3] = { (uint8_t)(address >> 8), (uint8_t)(address) };
  _transfer(tx, rx, 3);
  return rx[2];
}

void IMCTFD::_writeArrayCRC(uint16_t address, uint8_t *buffer, uint16_t size) {
  address = (address & 0xFFF) | (write_mode << 12); /* let user decide which type of CRC */
  if ( write_mode == cINSTRUCTION_WRITE_CRC || write_mode == cINSTRUCTION_WRITE_SAFE ) {
    uint8_t rx[size + 5] = { 0 }, tx[size + 5] = { (uint8_t)(address >> 8), (uint8_t)(address), (( (address & 0xFFF) >= 0x400 && (address & 0xFFF) <= 0xBFF ) ? (uint8_t)(size >> 2) : (uint8_t)size) };
    memmove(&tx[0] + 3, &buffer[0], size);
    uint16_t crc = calcCRC16(tx, (size + 5) - 2);
    tx[(size + 5) - 2] = crc >> 8;
    tx[(size + 5) - 1] = crc;
    _transfer(tx, rx, (size + 5));
    return;
  }
}

void IMCTFD::_writeWord(uint16_t address, uint32_t data) {
  address = (address & 0xFFF) | (write_mode << 12); /* let user decide which type of CRC */
  if ( write_mode == cINSTRUCTION_WRITE ) {
    uint8_t rx[6] = { 0 }, tx[6] = { ((uint8_t)((cINSTRUCTION_WRITE << 4) | ((address >> 8) & 0xF))), ((uint8_t)(address)), (uint8_t)(data), (uint8_t)(data >> 8), (uint8_t)(data >> 16), (uint8_t)(data >> 24) };
    _transfer(tx, rx, 6);
  }
  else if ( write_mode == cINSTRUCTION_WRITE_CRC || write_mode == cINSTRUCTION_WRITE_SAFE ) {
    uint8_t array[4] = { (uint8_t)(data), (uint8_t)(data >> 8), (uint8_t)(data >> 16), (uint8_t)(data >> 24) };
    _writeArrayCRC((address & 0xFFF), array, 4);
  }
}

void IMCTFD::_writeByte(uint16_t address, uint8_t txData) {
  address = (address & 0xFFF) | (write_mode << 12); /* let user decide crc or not */
  if ( write_mode == cINSTRUCTION_WRITE ) {
    uint8_t rx[3] = { 0 }, tx[3] = { ((uint8_t)((cINSTRUCTION_WRITE << 4) | ((address >> 8) & 0xF))), ((uint8_t)(address)), txData };
    _transfer(tx, rx, 3);
  }
  else if ( write_mode == cINSTRUCTION_WRITE_CRC || write_mode == cINSTRUCTION_WRITE_SAFE ) {
    uint8_t array[2] = { txData };
    _writeArrayCRC((address & 0xFFF), array, 1);
  }
}

uint32_t IMCTFD::_readWord(uint16_t address) {
  address = (address & 0xFFF) | (read_mode << 12); /* let user decide crc or not */
  if ( (address & 0x8000) ) { /* if CRC mode can run */
    uint8_t rx[4] = { 0 };
    _readArrayCRC((address & 0xFFF), rx, 4);
    return (uint32_t)(rx[0] << 0) | rx[1] << 8 | rx[2] << 16 | rx[3] << 24;
  }
  uint8_t rx[6] = { 0 }, tx[6] = { (uint8_t)(address >> 8), (uint8_t)(address) };
  _transfer(tx, rx, 6);
  return (uint32_t)(rx[2] << 0) | rx[3] << 8 | rx[4] << 16 | rx[5] << 24;
}

uint16_t IMCTFD::_readHalfWord(uint16_t address) {
  address = (address & 0xFFF) | (read_mode << 12); /* let user decide crc or not */
  if ( (address & 0x8000) ) { /* if CRC mode can run */
    uint8_t rx[4] = { 0 };
    _readArrayCRC((address & 0xFFF), rx, (((address & 0xFFF) >= 0x400 && (address & 0xFFF) <= 0xBFF) ? 4U : 2U));
    return (uint16_t)(rx[1] << 8) | rx[0];
  }
  uint8_t rx[4] = { 0 }, tx[4] = { (uint8_t)(address >> 8), (uint8_t)(address) };
  _transfer(tx, rx, 4);
  return (uint16_t)(rx[3] << 8) | rx[2];
}

void IMCTFD::_writeHalfWord(uint16_t address, uint32_t data) {
  if ( write_mode == cINSTRUCTION_WRITE ) {
    uint8_t rx[4] = { 0 }, tx[4] = { ((uint8_t)((cINSTRUCTION_WRITE << 4) | ((address >> 8) & 0xF))), ((uint8_t)(address)), (uint8_t)(data), (uint8_t)(data >> 8) };
    _transfer(tx, rx, 4);
  }
  else if ( write_mode == cINSTRUCTION_WRITE_CRC || write_mode == cINSTRUCTION_WRITE_SAFE ) {
    uint8_t array[2] = { (uint8_t)(data), (uint8_t)(data >> 8) };
    _writeArrayCRC((address & 0xFFF), array, 2);
  }
}

void IMCTFD::setListenOnly(bool mode) {
  ( mode ) ? setOpMode(CAN_LISTEN_ONLY_MODE) : setOpMode(CAN_NORMAL_MODE);
}

CAN_OPERATION_MODE IMCTFD::getOpMode() {
  return (CAN_OPERATION_MODE)((_readByte((read_mode << 12) | ((cREGADDR_CiCON + 2) & 0xFFF)) & 0xE0) >> 5);
}

void IMCTFD::reset() {
  /* The RESET instruction should only be issued after the device has entered Configuration mode. */
  if ( !setOpMode(CAN_CONFIGURATION_MODE) ) return;
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  Threads::Scope scope(SPI_LOCK[spi_port_value]);
#endif
  port->beginTransaction(settings);
  ::digitalWriteFast(cs,LOW);
  port->transfer16(cINSTRUCTION_RESET);
  ::digitalWriteFast(cs,HIGH);
  port->endTransaction();
}

uint16_t IMCTFD::calcCRC16(uint8_t* data, uint16_t size) {
  uint16_t init = 0xFFFF;
  uint8_t index = 0;
  while (size-- != 0) {
    index = ((uint8_t*) & init)[1] ^ *data++;
    init = (init << 8) ^ crc16_table[index];
  }
  return init;
}

void IMCTFD::enableFIFOInterrupt(const IMCTFD_CANFD_FIFO_CHANNELS &channel, bool status) {
  if ( channel == TXQ ) {
    tx_interrupts &= ~(1UL); /* enable interrupt handling */
    tx_interrupts |= ( (uint32_t)status );
    for ( uint8_t filter = 0; filter < 32; filter++ ) { /* update associated filters */
      if ( (channel | (1UL << 7)) == _readByte(cREGADDR_CiFLTCON + filter) ) {
        enhancements[filter] |= (1UL << 13);
        ( status ) ? enhancements[filter] |= (1UL) : enhancements[filter] &= ~(1UL);
      }
    }
    return;
  }
  if ( channel == TEF ) {
    _setBit(cREGADDR_CiTEFCON, 0, status);
    return;
  }
  if ( _getBit(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), 7) ) { /* if TX */
    tx_interrupts &= ~(1UL << channel); /* enable interrupt handling */
    tx_interrupts |= ( (uint32_t)status << channel );
    for ( uint8_t filter = 0; filter < 32; filter++ ) { /* update associated filters */
      if ( (channel | (1UL << 7)) == _readByte(cREGADDR_CiFLTCON + filter) ) {
        enhancements[filter] |= (1UL << 13);
        ( status ) ? enhancements[filter] |= (1UL << 14) : enhancements[filter] &= ~(1UL << 14);
      }
    }
  }
  else { /* if RX */
    _setBit(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), 0, status);
    for ( uint8_t filter = 0; filter < 32; filter++ ) { /* update associated filters */
      if ( (channel | (1UL << 7)) == _readByte(cREGADDR_CiFLTCON + filter) ) {
        enhancements[filter] &= ~(1UL << 13);
        ( status ) ? enhancements[filter] |= (1UL << 14) : enhancements[filter] &= ~(1UL << 14);
      }
    }
  }
}

bool IMCTFD::_setBit(uint16_t address, uint8_t bit, bool value) {
  uint8_t address_increment = map(bit, 0, 31, 0, 3);
  uint8_t _read = _readByte((address & 0xFFF) + address_increment);
  _writeByte((address & 0xFFF) + address_increment, ( _read & ~(1U << (bit - (8 * address_increment)))) | (value << (bit - (8 * address_increment))));
  return (_read & (1U << (bit - (8 * address_increment))));
}

bool IMCTFD::_getBit(uint16_t address, uint8_t bit) {
  uint8_t address_increment = map(bit, 0, 31, 0, 3);
  return (_readByte((address & 0xFFF) + address_increment) & (1U << (bit - (8 * address_increment))));
}

uint8_t IMCTFD::conversionsDLC(uint8_t cmd, uint8_t value, bool fdBit) {
  if ( cmd < 3 ) { /* data length input, correct with padding if necessary to get a valid DLC */
    const uint8_t buffer[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    if ( cmd == 2 ) return buffer[value];
    if ( fdBit && value > 64 ) return 64;
    if ( !fdBit && value > 8 ) return 8;
    if ( cmd == 1 ) {
      if ( fdBit ) for ( uint8_t i = 9; i < 16; i++ ) if ( buffer[i] == value ) return i;
      return value;
    }
    uint8_t count = 16;
    while ( count-- ) {
      if ( fdBit && value <= 8 ) return 8;
      if ( value <= buffer[count] ) continue;
      if ( value > 8 && value < 12 ) return buffer[count];
      return buffer[count+1];
    }
  }
  else if ( cmd < 5 ) { /* DLC <--> PLSIZE */
    const uint8_t buffer[8] = { 8, 12, 16, 20, 24, 32, 48, 64 };
    if ( cmd == 4 ) return buffer[value];
    if ( fdBit && value > 64 ) return 7;
    if ( !fdBit && value > 8 ) return 0;
    for ( uint8_t i = 0; i < 8; i++ ) if ( value <= buffer[i] ) return i;
  }
  else if ( cmd == 6 ) { /* length offset to PLSIZE, to include DLC 8 */
    if ( value <= 8 ) value = 8;
    else {
      const uint8_t buffer[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
      if ( fdBit && value > 64 ) value = 64;
      if ( !fdBit && value > 8 ) value = 8;
      uint8_t count = 16;
      while ( count-- ) {
        if ( fdBit && value <= 8 ) value = 8;
        if ( value <= buffer[count] ) continue;
        if ( value > 8 && value < 12 ) value = buffer[count];
        value = buffer[count+1];
      }
    }
    const uint8_t buffer[8] = { 8, 12, 16, 20, 24, 32, 48, 64 };
    if ( fdBit && value > 64 ) return 7;
    if ( !fdBit && value > 8 ) return 0;
    for ( uint8_t i = 0; i < 8; i++ ) if ( value <= buffer[i] ) return i;
  }
  return -1;
}

int IMCTFD::write(const CANFD_message_t &msg, IMCTFD_CANFD_FIFO_CHANNELS channel) {
  uint8_t _DLC = conversionsDLC(0, msg.len, msg.fdf);
  uint32_t fifoCon = 0;
  static uint8_t spread_writes = 1;
  bool found = 0;
  if ( !channel ) { /* spread out fifo writes for messages */
    for ( uint8_t i = 0; i < __builtin_popcount(configured_fifos) + 1; i++ ) {
      spread_writes++;
      if ( spread_writes > 31 ) spread_writes = 1;
      if ( !(configured_fifos & (1UL << spread_writes)) ) { /* skip unconfigured fifos */
        spread_writes = 0;
        continue;
      }
      fifoCon = _readWord(cREGADDR_CiFIFOCON + (spread_writes * CiFIFO_OFFSET));
      if ( !(fifoCon & 0x80) ) continue; /* This is an RX FIFO! */
      if ( _DLC > conversionsDLC(4, (fifoCon & 0xE0000000) >> 29, msg.fdf) ) continue; /* make sure it fits hardware queue */
      if ( !_getBit(cREGADDR_CiFIFOSTA + (spread_writes * CiFIFO_OFFSET), 0) ) continue; /* make sure it's not full */
      if ( tx_pending & ( 1UL << spread_writes ) ) continue; /* this is a pending queue set by user, fifo/txq must be specifically called */
      found = 1;
      break;
    }
    if ( !found ) return 0;
    channel = (IMCTFD_CANFD_FIFO_CHANNELS)spread_writes;
  }
  else if ( channel == TXQ ) {
    if ( !_getBit(cREGADDR_CiCON, 20) ) return 0; /* TXQ is disabled */
    _setBit(cREGADDR_CiTXQSTA, 3, 0); /* clear overflow flag if set */
    uint32_t txqcon = _readWord(cREGADDR_CiTXQCON);
    if ( _DLC > conversionsDLC(4, (txqcon & 0xE0000000) >> 29, msg.fdf) ) return 0; /* make sure it fits hardware queue */
    uint32_t timeout = millis();
    while ( !_getBit(cREGADDR_CiTXQSTA, 0) ) { /* make sure it's not full */
      if ( millis() - timeout > 100 ) return 0;
    }
  }
  else if ( channel == TEF ) return 0; /* TX only */
  else { /* channel specified */
    if ( !(configured_fifos & (1UL << channel)) ) return 0; /* skip unconfigured fifos */
    fifoCon = _readWord(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET));
    if ( !(fifoCon & 0x80) ) return 0; /* This is an RX FIFO! */
    if ( _DLC > conversionsDLC(4, (fifoCon & 0xE0000000) >> 29, msg.fdf) ) return 0; /* make sure it fits hardware queue */
    uint32_t timeout = millis();
    while ( !_getBit(cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET), 0) ) { /* wait till timeout if it's full */
      if ( millis() - timeout > 100 ) return 0;
    }
  }
  uint16_t user_address = 0;
  if ( channel == TXQ ) user_address = _readWord(cREGADDR_CiTXQUA) + cRAMADDR_START;
  else {
    _setBit(cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET), 3, 0); /* clear overflow flag if set */
    user_address = (((uint16_t)(cINSTRUCTION_WRITE << 12)) | (_readWord(cREGADDR_CiFIFOUA + (channel * CiFIFO_OFFSET)) + cRAMADDR_START)) & 0xFFF;
  }
  uint32_t config = conversionsDLC(1, _DLC, msg.fdf) | msg.fdf << 7 | msg.brs << 6 | msg.flags.remote << 5 | msg.flags.extended << 4;
  config |= ((msg.sequence & 0x7F) << 9);
  uint8_t txBuffer[_DLC + 8] = { 0 };

  uint32_t id = msg.id;
  if ( msg.flags.extended ) id &= 0x1FFFFFFF;
  else if ( msg.flags.sid12 ) {
    if ( _getBit(cREGADDR_CiTDC, 24) ) id = (id & 0x7FF) | ((id & 0x800) ? (1UL << 29) : 0UL);
    else id &= 0x7FF;
  }
  else id &= 0x7FF;

  for ( uint8_t i = 0; i < 4; i++ ) {
    txBuffer[i] = id >> (i * 8);
    txBuffer[i + 4] = config >> (i * 8);
  }
  memmove(&txBuffer[0] + 8, &msg.buf[0], _DLC);
  for ( uint8_t i = 0; i < _DLC - msg.len; i++ ) txBuffer[_DLC + 7 - i] = 0xAA;
  _transfer(user_address, txBuffer, nullptr, _DLC + 8);
  if ( channel == TXQ ) {
    uint32_t txqcon = _readWord(cREGADDR_CiTXQCON);
    txqcon |= ((tx_interrupts & 1UL) ? (1UL << 2) : 0UL);
    txqcon |= (((tx_pending & 1UL) ? 1UL : 3UL ) << 8 );
    _writeWord(cREGADDR_CiTXQCON, txqcon);
  }
  else {
    fifoCon |= ((tx_interrupts & (1UL << channel)) ? (1UL << 2) : 0UL );
    fifoCon |= (((tx_pending & (1UL << channel)) ? 1UL : 3UL ) << 8 );
    _writeWord(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), fifoCon);
  }
  return 1;
}

bool IMCTFD::setQueue(IMCTFD_CANFD_FIFO_CHANNELS channel, IMCTFD_WRITE_ACTION action) {
  if ( channel == TXQ ) {
    tx_pending &= ~(1UL << 0);
    tx_pending |= ((uint32_t)action << 0);
    return 1;
  }
  if ( !_getBit(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), 7) ) return 0; /* this is an RX FIFO! */
  tx_pending &= ~(1UL << channel);
  tx_pending |= ((uint32_t)action << channel);
  _setBit(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), 9, !action);
  return 1;
}

void IMCTFD::_transfer(uint8_t *txData, uint8_t *rxData, uint16_t size) {
  {
    busy_flag = 1;
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    Threads::Scope scope(SPI_LOCK[spi_port_value]);
#endif
    port->beginTransaction(settings);
    ::digitalWriteFast(cs,LOW);
    for ( uint16_t i = 0; i < size; i++ ) (rxData) ? rxData[i] = port->transfer((txData) ? txData[i] : 0xFF) : port->transfer((txData) ? txData[i] : 0xFF); 
    ::digitalWriteFast(cs,HIGH);
    port->endTransaction();
    busy_flag = 0;
  }
}

void IMCTFD::_transfer(uint16_t address, uint8_t *txData, uint8_t *rxData, uint16_t size) {
  if ( rxData ) {
    address = (address & 0xFFF) | (read_mode << 12); /* let user decide crc or not */
    if ( (address & 0x8000) ) { /* if CRC mode can run */
      _readArrayCRC((address & 0xFFF), rxData, size);
      return;
    }
  }
  else {
    address = (address & 0xFFF) | (write_mode << 12); /* let user decide crc or not */
    if ( (address & 0x8000) ) { /* if CRC mode can run */
      _writeArrayCRC((address & 0xFFF), txData, size);
      return;
    }
  }
  {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    Threads::Scope scope(SPI_LOCK[spi_port_value]);
#endif
    busy_flag = 1;
    port->beginTransaction(settings);
    ::digitalWriteFast(cs,LOW);
    port->transfer16(address);
    for ( uint16_t i = 0; i < size; i++ ) (rxData) ? rxData[i] = port->transfer((txData) ? txData[i] : 0xFF) : port->transfer((txData) ? txData[i] : 0xFF); 
    ::digitalWriteFast(cs,HIGH);
    port->endTransaction();
    busy_flag = 0;
  }
}

void IMCTFD::enableTimeStamping(uint16_t prescaler, bool enable, /* optional */ bool TSEOF, bool TSRES) {
  if( prescaler > 1024 ) return;
  _writeWord(cREGADDR_CiTSCON, prescaler | (enable << 16) | (TSEOF << 17) | (TSRES << 18));
}

bool IMCTFD::abortTransmission(const IMCTFD_CANFD_FIFO_CHANNELS channel) {
  uint16_t tx_address = 0;
  if ( !channel ) { /* Abort all transmissions */
    _setBit(cREGADDR_CiCON, 27, 1);
    return 1;
  }
  if ( channel == 32 ) { /* TXQ channel */
    if ( _getBit(cREGADDR_CiCON, 20) == 0 ) return 0; /* TxQ disabled */
    tx_address = cREGADDR_CiTXQCON;
  }
  else { /* FIFO channels 1 - 31 */
    if ( _getBit(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), 7) == 0 ) return 0; /* TX(1) or RX(0) FIFO ? */
    tx_address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
  }
  _setBit(tx_address, 9, 0);
  return 1;
}

bool IMCTFD::configureTXQ(IMCTFD_CANFD_PLSIZE pls, IMCTFD_CANFD_FIFO_DEPTH deep, IMCTFD_CANFD_CHANNEL_PRIORITIES priority) {
  if ( !setOpMode(CAN_CONFIGURATION_MODE) ) return 0;
  _setBit(cREGADDR_CiCON, 20, 1); /* enable TXQ in CICON */
  uint32_t config = (uint32_t)deep << 24 | pls << 29 | constrain(priority - 1, 0, 31) << 16;
  _writeWord(cREGADDR_CiTXQCON, (_readWord(cREGADDR_CiTXQCON) & 0xE0FF00) | config);
  TXQconfigured = 1;
  return 1;
}

bool IMCTFD::configureTEF(IMCTFD_CANFD_FIFO_DEPTH deep, IMCTFD_CANFD_TS ts) {
  if ( !setOpMode(CAN_CONFIGURATION_MODE) ) return 0;
  _setBit(cREGADDR_CiCON, 19, 1); /* enable TEF in CICON */
  uint32_t config = (uint32_t)deep << 24 | (ts << 5);
  _writeWord(cREGADDR_CiTEFCON, (_readWord(cREGADDR_CiTEFCON) & 0xE0FF00) | config);
  TEFconfigured = 1;
  return 1;
}

bool IMCTFD::configureFIFO(IMCTFD_CANFD_FIFO_CHANNELS channel, IMCTFD_RXTX rx_tx, IMCTFD_CANFD_PLSIZE pls, IMCTFD_CANFD_FIFO_DEPTH deep, IMCTFD_CANFD_CHANNEL_PRIORITIES priority, IMCTFD_CANFD_TS ts) {
  if ( !setOpMode(CAN_CONFIGURATION_MODE) ) return 0;
  if ( constrain(channel, 1, 31) != channel ) return 0; /* invalid channel */
  uint32_t fifoCon = _readWord(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET)) & 0xE0FF00;
  fifoCon |= deep << 24 | pls << 29 | constrain(priority - 1, 0, 31) << 16 | (ts << 5) | (rx_tx << 7);
  if ( rx_tx ) fifoCon &= 0xFFFFFFDF; /* timestamping for TX FIFO doesn't exist */
  else fifoCon |= (1UL << 3); /* for RX, enable the overflow interrupt flag */
  _writeWord(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), fifoCon);
  setOpMode(CAN_CONFIGURATION_MODE, 0);
  configured_fifos |= (1UL << channel);
  for ( uint8_t filter = 0; filter < 32; filter++ ) { /* disable filters associated with this channel */
    if ( (channel | (1UL << 7)) == _readByte(cREGADDR_CiFLTCON + filter) ) _writeByte(cREGADDR_CiFLTCON + filter, 0);
  }
  return 1;
}

void IMCTFD::test() {
}

uint16_t IMCTFD::calculateRAM() {
  uint32_t ramSize = 0;
  uint32_t cicon = _readWord(cREGADDR_CiCON);
  if ( cicon & (1UL << 19) ) {
    uint32_t tefcon = _readWord(cREGADDR_CiTEFCON);
    bool timestamp = tefcon & (1UL << 5);
    uint8_t tefcon_deep = ((tefcon & 0x1F000000) >> 24) + 1UL;
    ramSize += ((((timestamp) ? 4 : 0) + 8) * tefcon_deep);
  }
  if ( cicon & (1UL << 20) ) {
    uint32_t txqcon = _readWord(cREGADDR_CiTXQCON);
    uint8_t dataSize = conversionsDLC(4, (txqcon & 0xE0000000) >> 29, 1);
    uint8_t txq_deep = ((txqcon & 0x1F000000) >> 24) + 1UL;
    ramSize += ((dataSize + 8) * txq_deep);
  }
  for ( uint8_t i = 1; i < 32; i++ ) {
    if ( configured_fifos & ( 1UL << i ) ) {
      uint32_t fifo = _readWord(cREGADDR_CiFIFOCON + (i * CiFIFO_OFFSET));
      uint8_t dataSize = conversionsDLC(4, (fifo & 0xE0000000) >> 29, 1);
      bool timestamp = fifo & (1UL << 5);
      uint8_t fifo_deep = ((fifo & 0x1F000000) >> 24) + 1UL;
      ramSize += ((((timestamp) ? 4 : 0) + dataSize + 8) * fifo_deep);
    }
  }
  return ramSize;
}

void IMCTFD::currentConfig() {
  uint32_t cicon = _readWord(cREGADDR_CiCON), txqcon = _readWord(cREGADDR_CiTXQCON), tefcon = _readWord(cREGADDR_CiTEFCON), ramSize = 0;
  if ( cicon & ( 1UL << 19 ) ) {
    Serial.println("\t##########################################");
    Serial.println("\t#\t*** TEF Configuration ***\t #");
    Serial.println("\t##########################################");
    Serial.printf("\t#\t    %-0s", "FIFO Size: "); Serial.printf("%-2u", ((tefcon & 0x1F000000) >> 24) + 1U); Serial.println("\t\t #");
    Serial.printf("\t#\t    %-0s", "TimeStamp: "); Serial.printf("%-8s", ((tefcon & 0x20) ? "Enabled" : "Disabled")); Serial.println("\t\t #");
    Serial.printf("\t#\t    %-0s", "Interrupt: "); Serial.printf("%-8s", ((tefcon & 0x1) ? "Enabled" : "Disabled")); Serial.println("\t\t #");
    Serial.printf("\t#\t    %-0s", "RAM usage: "); Serial.printf("%4u", ((((tefcon & 0x20) ? 4 : 0) + 8) * (((tefcon & 0x1F000000) >> 24) + 1UL))); Serial.println(" (Bytes)\t #");
    Serial.println("\t##########################################\n");
    ramSize += ((((tefcon & 0x20) ? 4 : 0) + 8) * (((tefcon & 0x1F000000) >> 24) + 1UL));
  }

  if ( cicon & ( 1UL << 20 ) ) {
    Serial.println("\t##########################################");
    Serial.println("\t#\t*** TXQ Configuration ***\t #");
    Serial.println("\t##########################################");
    Serial.printf("\t#\t    %-0s", "Payload Size: "); Serial.printf("%-2u", (conversionsDLC(4, (txqcon & 0xE0000000) >> 29, 1))); Serial.println("\t\t #");
    Serial.printf("\t#\t    %-0s", "FIFO Size: "); Serial.printf("%-2u", (((txqcon & 0x1F000000) >> 24) + 1U)); Serial.println("\t\t #");
    Serial.printf("\t#\t    %-0s", "Priority: "); Serial.printf("%-2u", (((txqcon & 0x1F0000) >> 16) + 1U)); Serial.println("\t\t #");
    Serial.printf("\t#\t    %-0s", "Interrupt: "); Serial.printf("%-8s", ((txqcon & 0x1) ? "Enabled" : "Disabled")); Serial.println("\t\t #");
    Serial.printf("\t#\t    %-0s", "RAM usage: "); Serial.printf("%4u", (((conversionsDLC(4, (txqcon & 0xE0000000) >> 29, 1) + 8) * (((txqcon & 0x1F000000) >> 24) + 1UL)))); Serial.println(" (Bytes)\t #");
    Serial.println("\t##########################################\n");
    ramSize += ((conversionsDLC(4, (txqcon & 0xE0000000) >> 29, 1) + 8) * (((txqcon & 0x1F000000) >> 24) + 1UL));
  }

  if ( configured_fifos ) {
    Serial.println("\t##########################################");
    Serial.println("\t#\t*** FIFO Configuration ***\t #");
    Serial.println("\t##########################################");
    for ( uint8_t i = 0; i < 32; i++ ) {
      if ( configured_fifos & ( 1UL << i) ) {
        uint32_t fifocon = _readWord(cREGADDR_CiFIFOCON + (i * CiFIFO_OFFSET));
        Serial.printf("\t#\t    %-0s", "FIFO"); Serial.print(i); Serial.printf("%-18s", (( fifocon & 0x80 ) ? ": Transmit FIFO" : ": Receive FIFO")); Serial.println(" \t #");
        Serial.printf("\t#\t    %-0s", "Payload Size: "); Serial.printf("%-2u", conversionsDLC(4, (fifocon & 0xE0000000) >> 29, 1)); Serial.println("\t\t #");
        Serial.printf("\t#\t    %-0s", "FIFO Size: "); Serial.printf("%-2u", (((fifocon & 0x1F000000) >> 24) + 1U)); Serial.println("\t\t #");
        if ( fifocon & 0x80 ) {
          Serial.printf("\t#\t    %-0s", "Priority: "); Serial.printf("%-2u", (((fifocon & 0x1F0000) >> 16) + 1U)); Serial.println("\t\t #");
          Serial.printf("\t#\t    %-0s", "Interrupt: "); Serial.printf("%-8s", ((tx_interrupts & (1UL << i)) ? "Enabled" : "Disabled")); Serial.println("\t\t #");
        }
        else {
          Serial.printf("\t#\t    %-0s", "TimeStamp: "); Serial.printf("%-8s", ((fifocon & 0x20) ? "Enabled" : "Disabled")); Serial.println("\t\t #");
          Serial.printf("\t#\t    %-0s", "Interrupt: "); Serial.printf("%-8s", ((fifocon & 0x1) ? "Enabled" : "Disabled")); Serial.println("\t\t #");
        }
        Serial.printf("\t#\t    %-0s", "RAM usage: "); Serial.printf("%4u", ((((fifocon & 0x20) ? 4 : 0) + conversionsDLC(4, (fifocon & 0xE0000000) >> 29, 1) + 8) * (((fifocon & 0x1F000000) >> 24) + 1U))); Serial.println(" (Bytes)\t #");
        ramSize += (((fifocon & 0x20) ? 4 : 0) + conversionsDLC(4, (fifocon & 0xE0000000) >> 29, 1) + 8) * (((fifocon & 0x1F000000) >> 24) + 1U);
        Serial.println("\t##########################################");
      }
    }
    Serial.println();
  }

  if ( ramSize ) {
    Serial.println("\t##########################################");
    Serial.printf("\t#    %-0s", "Total RAM capacity: 2048"); Serial.println(" (Bytes)\t #");
    Serial.printf("\t#    %-0s", "Total RAM usage:    "); Serial.printf("%4u", ramSize); Serial.println(" (Bytes)\t #");
    if ( ramSize <= 2048 ) {
      Serial.printf("\t#    %-0s%4u", "RAM left available: ", (2048 - ramSize)); Serial.println(" (Bytes)\t #");
    }
    else {
      Serial.printf("\t#    %-0s%4u", "RAM overflowed by:  ", abs(2048 - ramSize)); Serial.println(" (Bytes)\t #");
    }
    Serial.println("\t##########################################\n");
  }
}

bool IMCTFD::setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, IMCTFD_FILTER_TYPE type) {
  filterProcessing(filter, channel, 0, 0); /* enable single filter to accept all extended and standard ids. */
  id_enhancements[filter][0] = 0;
  enhancements[filter] |= (1UL << 1); /* specify 1 ID (3 bits) */
  enhancements[filter] |= (channel << 4); /* specify channel assigned (6 bits) */
  return 1;
}

bool IMCTFD::setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id, IMCTFD_FILTER_TYPE type) {
  if ( type == SID12 ) {
    if ( _getBit(cREGADDR_CiTDC, 24) && ( id <= 0xFFF ) && ( id & 0x800 ) ) {
      filterProcessing(filter, channel, ((id & 0x7FF) | (1UL << 29)), 0x600007FF);
    }
    else return 0;
  }
  else if ( type == NORMAL_FILTERING ) {
    filterProcessing(filter, channel, id | (( id > 0x7FF ) ? (1UL << 30) : 0UL), (( id > 0x7FF ) ? 0x1FFFFFFF : 0x7FF) | (1UL << 30));
  }
  id_enhancements[filter][0] = id;
  enhancements[filter] |= (1UL << 1); /* specify 1 ID (3 bits) */
  enhancements[filter] |= (channel << 4); /* specify channel assigned (6 bits) */
  return 1;
}

bool IMCTFD::setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, IMCTFD_FILTER_TYPE type) {
  if ( type == SID12 ) {
    if ( _getBit(cREGADDR_CiTDC, 24) && ( id0 <= 0xFFF ) && ( id0 & 0x800 ) && ( id1 & 0x800 ) ) {
      uint8_t match_flags = ( id0 <= 0xFFF ) + ( id1 <= 0xFFF );
      if ( match_flags > 0 && match_flags < 2 ) return 0;
      uint32_t mask = ((id0 | id1) ^ (id0 & id1)) ^ 0xFFF;
      mask = ((mask & 0x7FF) | (1UL << 29));
      filterProcessing(filter, channel, ((id0 & 0x7FF) | (1UL << 29)), mask | (1UL << 30));
    }
    else return 0;
  }
  else if ( type == NORMAL_FILTERING ) {
    uint8_t match_flags = ( id0 > 0x7FF ) + ( id1 > 0x7FF );
    if ( match_flags > 0 && match_flags < 2 ) return 0;
    uint32_t mask = ((id0 | id1) ^ (id0 & id1)) ^ (( id0 > 0x7FF ) ? 0x1FFFFFFF : 0x7FF);
    filterProcessing(filter, channel, id0 | (( id0 > 0x7FF ) ? (1UL << 30) : 0UL), mask | (1UL << 30));
  }
  id_enhancements[filter][0] = id0;
  id_enhancements[filter][1] = id1;
  enhancements[filter] |= (2UL << 1); /* specify 2 IDs (3 bits) */
  enhancements[filter] |= (channel << 4); /* specify channel assigned (6 bits) */
  return 1;
}

bool IMCTFD::setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, uint32_t id2, IMCTFD_FILTER_TYPE type) {
  if ( type == SID12 ) {
    if ( _getBit(cREGADDR_CiTDC, 24) && ( id0 <= 0xFFF ) && ( id0 & 0x800 ) && ( id1 & 0x800 ) && ( id2 & 0x800 ) ) {
      uint8_t match_flags = ( id0 <= 0xFFF ) + ( id1 <= 0xFFF ) + ( id2 <= 0xFFF );
      if ( match_flags > 0 && match_flags < 2 ) return 0;
      uint32_t mask = ((id0 | id1 | id2 ) ^ (id0 & id1 & id2)) ^ 0xFFF;
      mask = ((mask & 0x7FF) | (1UL << 29));
      filterProcessing(filter, channel, ((id0 & 0x7FF) | (1UL << 29)), mask | (1UL << 30));
    }
    else return 0;
  }
  else if ( type == NORMAL_FILTERING ) {
      uint8_t match_flags = ( id0 > 0x7FF ) + ( id1 > 0x7FF ) + ( id2 > 0x7FF );
      if ( match_flags > 0 && match_flags < 3 ) return 0;
      uint32_t mask = ((id0 | id1 | id2) ^ (id0 & id1 & id2)) ^ (( id0 > 0x7FF ) ? 0x1FFFFFFF : 0x7FF);
      filterProcessing(filter, channel, id0 | (( id0 > 0x7FF ) ? (1UL << 30) : 0UL), mask | (1UL << 30));
  }
  id_enhancements[filter][0] = id0;
  id_enhancements[filter][1] = id1;
  id_enhancements[filter][2] = id2;
  enhancements[filter] |= (3UL << 1); /* specify 3 IDs (3 bits) */
  enhancements[filter] |= (channel << 4); /* specify channel assigned (6 bits) */
  return 1;
}

bool IMCTFD::setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, uint32_t id2, uint32_t id3, IMCTFD_FILTER_TYPE type) {
  if ( type == SID12 ) {
    if ( _getBit(cREGADDR_CiTDC, 24) && ( id0 <= 0xFFF ) && ( id0 & 0x800 ) && ( id1 & 0x800 ) && ( id2 & 0x800 ) && ( id3 & 0x800 ) ) {
      uint8_t match_flags = ( id0 <= 0xFFF ) + ( id1 <= 0xFFF ) + ( id2 <= 0xFFF ) + ( id3 <= 0xFFF );
      if ( match_flags > 0 && match_flags < 2 ) return 0;
      uint32_t mask = ((id0 | id1 | id2 | id3) ^ (id0 & id1 & id2 & id3)) ^ 0xFFF;
      mask = ((mask & 0x7FF) | (1UL << 29));
      filterProcessing(filter, channel, ((id0 & 0x7FF) | (1UL << 29)), mask | (1UL << 30));
    }
    else return 0;
  }
  else if ( type == NORMAL_FILTERING ) {
    uint8_t match_flags = ( id0 > 0x7FF ) + ( id1 > 0x7FF ) + ( id2 > 0x7FF ) + ( id3 > 0x7FF );
    if ( match_flags > 0 && match_flags < 4 ) return 0;
    uint32_t mask = ((id0 | id1 | id2 | id3 ) ^ (id0 & id1 & id2 & id3)) ^ (( id0 > 0x7FF ) ? 0x1FFFFFFF : 0x7FF);
    filterProcessing(filter, channel, id0 | (( id0 > 0x7FF ) ? (1UL << 30) : 0UL), mask | (1UL << 30));
  }
  id_enhancements[filter][0] = id0;
  id_enhancements[filter][1] = id1;
  id_enhancements[filter][2] = id2;
  id_enhancements[filter][3] = id3;
  enhancements[filter] |= (4UL << 1); /* specify 4 IDs (3 bits) */
  enhancements[filter] |= (channel << 4); /* specify channel assigned (6 bits) */
  return 1;
}

bool IMCTFD::setFIFOFilter(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, IMCTFD_FILTER_TYPE type) {
  if ( type == SID12 ) {
    if ( _getBit(cREGADDR_CiTDC, 24) && ( id0 <= 0xFFF ) && ( id0 & 0x800 ) && ( id1 & 0x800 ) && ( id2 & 0x800 ) && ( id3 & 0x800 ) && ( id4 & 0x800 ) ) {
      uint8_t match_flags = ( id0 <= 0xFFF ) + ( id1 <= 0xFFF ) + ( id2 <= 0xFFF ) + ( id3 <= 0xFFF ) + ( id4 <= 0xFFF );
      if ( match_flags > 0 && match_flags < 2 ) return 0;
      uint32_t mask = ((id0 | id1 | id2 | id3 | id4) ^ (id0 & id1 & id2 & id3 & id4)) ^ 0xFFF;
      mask = ((mask & 0x7FF) | (1UL << 29));
      filterProcessing(filter, channel, ((id0 & 0x7FF) | (1UL << 29)), mask | (1UL << 30));
    }
    else return 0;
  }
  else if ( type == NORMAL_FILTERING ) {
      uint8_t match_flags = ( id0 > 0x7FF ) + ( id1 > 0x7FF ) + ( id2 > 0x7FF ) + ( id3 > 0x7FF ) + ( id4 > 0x7FF );
      if ( match_flags > 0 && match_flags < 5 ) return 0;
      uint32_t mask = ((id0 | id1 | id2 | id3 | id4 ) ^ (id0 & id1 & id2 & id3 & id4)) ^ (( id0 > 0x7FF ) ? 0x1FFFFFFF : 0x7FF);
      filterProcessing(filter, channel, id0 | (( id0 > 0x7FF ) ? (1UL << 30) : 0UL), mask | (1UL << 30));
  }
  id_enhancements[filter][0] = id0;
  id_enhancements[filter][1] = id1;
  id_enhancements[filter][2] = id2;
  id_enhancements[filter][3] = id3;
  id_enhancements[filter][4] = id4;
  enhancements[filter] |= (5UL << 1); /* specify 5 IDs (3 bits) */
  enhancements[filter] |= (channel << 4); /* specify channel assigned (6 bits) */
  return 1;
}

bool IMCTFD::setFIFOFilterRange(IMCTFD_CANFD_FILTERS filter, IMCTFD_CANFD_FIFO_CHANNELS channel, uint32_t id0, uint32_t id1, IMCTFD_FILTER_TYPE type) {
  if ( type == SID12 ) {
    if ( _getBit(cREGADDR_CiTDC, 24) && ( id0 <= 0xFFF ) && ( id0 & 0x800 ) && ( id1 & 0x800 ) ) {
      uint8_t match_flags = ( id0 <= 0xFFF ) + ( id1 <= 0xFFF );
      if ( match_flags > 0 && match_flags < 2 ) return 0;
      uint32_t stage1 = id0, stage2 = id0;
      if ( id0 >= id1 || !id1 ) return 0; /* don't play around... */
      for ( uint32_t i = id0 + 1; i <= id1; i++ ) {
        stage1 |= i; stage2 &= i;
      }
      uint32_t mask = ( stage1 ^ stage2 ) ^ 0xFFF;
      mask = ((mask & 0x7FF) | (1UL << 29));
      filterProcessing(filter, channel, ((id0 & 0x7FF) | (1UL << 29)), mask | (1UL << 30));
    }
    else return 0;
  }
  else if ( type == NORMAL_FILTERING ) {
      uint8_t match_flags = ( id0 > 0x7FF ) + ( id1 > 0x7FF );
      if ( match_flags > 0 && match_flags < 2 ) return 0;
      uint32_t stage1 = id0, stage2 = id0;
      if ( id0 >= id1 || !id1 ) return 0; /* don't play around... */
      for ( uint32_t i = id0 + 1; i <= id1; i++ ) {
        stage1 |= i; stage2 &= i;
      }
      uint32_t mask = ( stage1 ^ stage2 ) ^ (( id0 > 0x7FF ) ? 0x1FFFFFFF : 0x7FF);
      filterProcessing(filter, channel, id0 | (( id0 > 0x7FF ) ? (1UL << 30) : 0UL), mask | (1UL << 30));
  }
  id_enhancements[filter][0] = id0;
  id_enhancements[filter][1] = id1;
  enhancements[filter] |= 1UL; /* enable bit0 to specify range based */
  enhancements[filter] |= (2UL << 1); /* specify 2 IDs (3 bits) */
  enhancements[filter] |= (channel << 4); /* specify channel assigned (6 bits) */
  return 1;
}

void IMCTFD::filterProcessing(uint8_t filter, uint8_t channel, uint32_t id, uint32_t mask) {
  if ( !init_filters ) {
    init_filters = 1;
    for ( uint8_t i = 0; i < 32; i++ ) _writeByte(cREGADDR_CiFLTCON + i, 0); /* start fresh */
  }
  enhancements[filter] = 0; /* reset enhancement bits */
  if ( channel < 32 ) {
    if ( _getBit(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), 7) ) { /* tx? */
      if ( tx_interrupts & (1UL << channel) ) enhancements[filter] |= (1UL << 14);
      enhancements[filter] |= (1UL << 13);
    }
    else { /* rx */
      if ( _getBit(cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET), 0) ) enhancements[filter] |= (1UL << 14); /* interrupt? */
    }
  }
  _writeByte(cREGADDR_CiFLTCON + filter, 0); /* we must disable a filter before we change any FIFO pointers */
  _writeWord(cREGADDR_CiFLTOBJ + ( filter * 8 ), id);
  _writeWord(cREGADDR_CiMASK + ( filter * 8 ), mask);
  _writeByte(cREGADDR_CiFLTCON + filter, 0x80 + channel ); /* enable the filter on that fifo channel */
}

void IMCTFD::enhanceFilter(IMCTFD_CANFD_FILTERS filter) {
  enhancements[filter] |= (1UL << 15);
}

void IMCTFD::disableFilter(IMCTFD_CANFD_FILTERS filter) {
  enhancements[filter] = 0; /* clear enhancements */
  _writeByte(cREGADDR_CiFLTCON + filter, 0); /* disable filter */
}

void IMCTFD::currentFilters() {
  Serial.println("\t##########################################");
  Serial.println("\t#\t*** Filter Configuration ***\t #");
  Serial.println("\t##########################################");

  bool found = 0;
  for ( uint32_t filter = 0, fltcon = 0, mask = 0, id = 0; filter < 32; filter++ ) { 
    fltcon = _readByte(cREGADDR_CiFLTCON + filter);
    if ( fltcon & 0x80 ) {
      found = 1;
      id = _readWord(cREGADDR_CiFLTOBJ + ( filter * 8 )) & 0x1FFFFFFF;
      mask = _readWord(cREGADDR_CiMASK + ( filter * 8 )) & 0x1FFFFFFF;
      Serial.printf("\t#\t    %-0s", "Filter_"); Serial.printf("%-2u", filter); Serial.println("\t\t\t #");
      Serial.printf("\t#\t    %s%s", ((id > 0x7FF) ? "Extended ID" : "Standard ID"), "\t\t\t #\n");
      Serial.printf("\t#\t    %s%-10X%s", "Mask: 0x" , mask, "\t\t #\n");
      Serial.printf("\t#\t    %s%-2u%s", "Assigned to: FIFO", fltcon & 0x1F, "\t\t #\n");
      Serial.printf("\t#\t    %s%-2s%s", "Enhancement: ", ((enhancements[filter] & 0x8000) ? "Enabled " : "Disabled"), "\t #\n");
      if (enhancements[filter] & 0x8000) {
        Serial.printf("\t#\t       * %s%s", ((enhancements[filter] & 0x1) ? "Range based    " : "Multi-ID based "), "\t #\n");
        if ( enhancements[filter] & 0x1 ) {
          Serial.printf("\t#\t          From: 0x%-10X%s", id_enhancements[filter][0], "\t #\n");
          Serial.printf("\t#\t            To: 0x%-10X%s", id_enhancements[filter][1], "\t #\n");
        }
        else {
          for ( uint8_t i = 0; i < ((enhancements[filter] >> 1) & 0x7); i++ ) {
            Serial.printf("\t#\t          ID %u%s%-10X%s", i + 1, ": 0x", id_enhancements[filter][i], "\t #\n");
          }
        }
      }
      Serial.println("\t##########################################");
    }
  }
  if ( !found ) {
    Serial.println("\t#\t    No Filters Configured\t #");
    Serial.println("\t##########################################\n\n");
  }
  else Serial.println('\n');
}

void IMCTFD::pinMode(uint8_t pin, uint8_t mode) {
  uint32_t iocon = _readWord(cREGADDR_IOCON);
  pin = constrain(pin, 0, 1);
  if ( mode == INPUT ) {
    iocon |= (1UL << (24 + pin)) | 1UL << pin;
  }
  else if ( mode == OUTPUT ) {
    iocon &= ~(1UL << pin);
    iocon |= (1UL << (24 + pin));
  }
  _writeWord(cREGADDR_IOCON, iocon);
}

bool IMCTFD::digitalRead(uint8_t pin) {
  return _getBit(cREGADDR_IOCON, (16 + constrain(pin, 0, 1)));
}

void IMCTFD::digitalWrite(uint8_t pin, uint8_t state) {
  _setBit(cREGADDR_IOCON, 8 + constrain(pin, 0, 1), state);
}

void IMCTFD::begin() {
  if ( !setOpMode(CAN_CONFIGURATION_MODE) ) return;
  uint32_t tefcon = _readWord(cREGADDR_CiTEFCON);
  uint32_t txqcon = _readWord(cREGADDR_CiTXQCON);
  enableTimeStamping();
  uint32_t citscon = _readWord(cREGADDR_CiTSCON);
  uint32_t iocon = _readWord(cREGADDR_IOCON);

  if ( !init_filters ) {
    init_filters = 1;
    for ( uint8_t i = 0; i < 32; i++ ) _writeByte(cREGADDR_CiFLTCON + i, 0); /* start fresh */
  }

  uint32_t backup_filters[72] = { 0 }; /* backup current set user filters */
  uint16_t count = 72;
  uint16_t start = 0x2EC;
  while ( count-- ) {
    backup_filters[count] = _readWord(start);
    start -= 4;
  }
  
  uint8_t total_fifos = __builtin_popcount(configured_fifos);
  uint32_t registered_fifos[total_fifos + 1]; /* index 0 is not used */
  for ( uint8_t i = 1; i < total_fifos + 1; i++ ) { /* store current FIFO configuration */
    registered_fifos[i] = _readWord(cREGADDR_CiFIFOCON + (i * CiFIFO_OFFSET));
  }

/*
•  Reset the MCP25xxFD
•  Configure the Oscillator and CLKO pin
•  Configure the I/O pins
•  Configure the CAN Control register
•  Configure the Bit Time registers
•  Configure the TEF, TXQ, TX and RX FIFOs
*/

  reset();
  _writeWord(cREGADDR_ECCCON, _readWord(cREGADDR_ECCCON) | 1UL); /* enable ECC */
  initRAM(); /* initialize RAM */
  _writeWord(cREGADDR_CiINT, 0); /* disable interrupts */
  _writeWord(cREGADDR_OSC, 0x460); /* start oscillator */

  /*
     In order to optimize RAM usage, the application should start configuring the RAM with the TEF,
     followed  by  the  TXQ,  and  continue  with  FIFO  1, FIFO  2,  FIFO  3  and  so  on.  In  case  a  user
     application requires TEF, TXQ, and 16 additional FIFOs, it should configure TEF, TXQ, followed
     by FIFO 1 through FIFO 16. It is not necessary to configure the unused FIFOs 17 through 31.
  */

  uint32_t por_cicon = _readWord(cREGADDR_CiCON) & 0xFFE7FFFF; /* power on reset clear STEF + TXQ flags */

  if ( TEFconfigured ) { /* STEF */
    por_cicon |= (1UL << 19);
    _writeWord(cREGADDR_CiCON, por_cicon); /* write back if TEF was set */
    _writeWord(cREGADDR_CiTEFCON, tefcon); /* restore TEF */
  }
  else _writeWord(cREGADDR_CiCON, por_cicon); /* TEF was not used */

  if ( TXQconfigured ) { /* TXQ */
    por_cicon |= (1UL << 20);
    _writeWord(cREGADDR_CiCON, por_cicon); /* write back if TXQ was set */
    _writeWord(cREGADDR_CiTXQCON, txqcon); /* restore TXQ */
  }
  else _writeWord(cREGADDR_CiCON, por_cicon); /* TXQ was not used */

  _writeWord(cREGADDR_CiTSCON, citscon); /* timestamp register */
  _writeWord(cREGADDR_IOCON, iocon); /* restore GPIO settings */

  for ( uint8_t i = 1; i < total_fifos + 1; i++ ) { /* restore FIFOs */
    _writeWord(cREGADDR_CiFIFOCON + (i * CiFIFO_OFFSET), registered_fifos[i]);
  }

  count = 72; /* begin restoring filters */
  start = 0x2EC;
  while ( count-- ) {
    _writeWord(start, backup_filters[count]);
    start -= 4;
  }

  if ( _currentBitrate && _currentMultiplier ) {
    setBaudRate(_currentBitrate, _currentMultiplier);
    setOpMode(CAN_NORMAL_MODE);
  }
  else {
    setBaudRate(500000, 4);
    setOpMode(CAN_LISTEN_ONLY_MODE);
  }

  _writeWord(cREGADDR_CiINT, 0xF30F0000); /* enable interrupts */

  if ( !threading_started ) {
    threading_started = 1;
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    uint8_t id = threads.addThread(IMCTFD::process);
    threads.setTimeSlice(id, 1000);
    id = threads.addThread(IMCTFD::IMCTFD_routines);
    threads.setTimeSlice(id, 20);
#endif
#if defined(KINETISL)
    IMCTFD_iTimer.begin(IMCTFD::process, 25000);
    IMCTFD_iTimer.priority(128);
#endif
  }
}
