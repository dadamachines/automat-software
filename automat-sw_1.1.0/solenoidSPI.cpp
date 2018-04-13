/*
   Copyright (c) 2016, DADAMACHINES
   Author: Tobias Münzer
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.

    3. Neither the name of DADAMACHINES nor the names of its contributors may be used
       to endorse or promote products derived from this software without
       specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "solenoidSPI.h"

/*! The constructor takes two parameters.  The first is an SPI class
    pointer.  This is the address of an SPI object (either the default
    SPI object on the Arduino, or an object made using the DSPIx classes
    on the chipKIT).  The second parameter is the chip select pin number
    to use when communicating with the shift registers.

    Example:


        SOLSPI mySolenoids(&SPI, A1);

*/
#ifdef __PIC32MX__
SOLSPI:: SOLSPI(DSPI *spi, uint8_t cs) {
#else
SOLSPI:: SOLSPI(SPIClass *spi, uint8_t cs) {
#endif
  _spi = spi;
  _cs = cs;
}

#ifdef __PIC32MX__
SOLSPI::SOLSPI(DSPI &spi, uint8_t cs) {
#else
SOLSPI::SOLSPI(SPIClass &spi, uint8_t cs) {
#endif
  _spi = &spi;
  _cs = cs;
}

void SOLSPI::begin() {
  outputState = 0;
  _spi->begin();
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, LOW);
  sendState();        //switch off all outputs
}

void SOLSPI::sendState()  {
  digitalWrite(_cs, LOW);
  _spi->transfer(outputState & 0xFF);
  _spi->transfer(outputState >> 8);
  digitalWrite(_cs, HIGH);
}

void SOLSPI::setOutput(uint8_t num) {
  outputState |= (1 << translatePinNumber(num));
  sendState();
}

void SOLSPI::clearOutput(uint8_t num) {
  outputState &= ~(1 << translatePinNumber(num));
  sendState();
}

void SOLSPI::singlePin(uint8_t num, bool on_or_off) {
  uint16_t backup_outputState = outputState;
  for (int i = 0 ; i < 12 ; i++) {
    if (num == i && on_or_off) {
      backup_outputState |= (1 << translatePinNumber(i));
    } else {
      backup_outputState &= ~(1 << translatePinNumber(i));
    }
  }
  if (backup_outputState != outputState)  {
    outputState=backup_outputState;
    sendState();
  }
}

uint8_t  SOLSPI::translatePinNumber(uint8_t n)  {
  const uint8_t nums[12] = {15, 13, 12, 11, 7, 3, 14, 10, 9, 6, 5, 4};
  if (n > 11)
    return 8;
  else
    return nums[n];
}


