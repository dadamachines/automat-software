/*
 * Copyright (c) 2016, DADAMACHINES
 * Author: Tobias Münzer
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of DADAMACHINES nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _SOLSPI_H
#define _SOLSPI_H

#if (ARDUINO >= 100) 
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

#ifdef __PIC32MX__
#include <DSPI.h>
#else
#include <SPI.h>
#endif

class SOLSPI {
    private:
#ifdef __PIC32MX__
        DSPI *_spi; /*! This points to a valid SPI object created from the chipKIT DSPI library. */
#else
        SPIClass *_spi; /*! This points to a valid SPI object created from the Arduino SPI library. */
#endif
        
    uint8_t _cs;    /*! Chip select pin */
    uint16_t outputState;
    uint8_t translatePinNumber(uint8_t pin);
    void sendState(void);
    
    public:
#ifdef __PIC32MX__
         SOLSPI(DSPI *spi, uint8_t cs);
         SOLSPI(DSPI &spi, uint8_t cs);
#else
         SOLSPI(SPIClass *spi, uint8_t cs);
         SOLSPI(SPIClass &spi, uint8_t cs);
#endif
        void begin();
       
        void setOutput(uint8_t addr); 
        void clearOutput(uint8_t addr);
        void singlePin(uint8_t num, bool on_or_off); // set a single pin an clear all other

};
#endif


