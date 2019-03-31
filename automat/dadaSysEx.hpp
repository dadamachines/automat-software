/*
   Copyright (c) 2018, DADAMACHINES
   Author: Justin Pedro
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


#ifndef _DADASYSEX_HPP
#define _DADASYSEX_HPP

#pragma once

extern void mapFixedDurationConfig();
byte dadaSysEx::sysexOutArr[dadaSysEx::SYSEX_CONFIG_LEN];
byte dadaSysEx::UsbSysExBuffer[dadaSysEx::MAX_SYSEX_MESSAGE_SIZE];

bool dadaSysEx::handleSysEx(byte * arr, unsigned len)
{
   if(len > 1 && (*arr == SYSEX_START))
   {
      arr++;
      // ignore the sysex framing
   }
  
   if (len < SYSEX_CONFIG_LEN)
   {
      if (len != SYSEX_GET_CONFIG_LEN)
      {
         return false;
      }
   }

   if (*arr++ != 0) {
     return false;
   }

   if (getIntFromArray(arr) != SYSEX_CONFIG_HEADER)
   {
       return false;
   }
   arr += sizeof(int);

   if (len == SYSEX_GET_CONFIG_LEN)
   {
       if (getIntFromArray(arr) == SYSEX_CONFIG_GET_CONFIG)
       {
         // provide a small delay in case they need to get ready to receive this data
         delay(200);
         saveConfigToSysEx();
         return true;
       }
       if (getIntFromArray(arr) == SYSEX_CONFIG_GET_VERSION)
       {
         // provide a small delay in case they need to get ready to receive this data
         delay(200);
         sendVersionToSysEx();
         return true;
       }
       return false;
   }
   
   if (getIntFromArray(arr) != SYSEX_CONFIG_PINS)
   {
       return false;
   }
   arr += sizeof(int);

   dataCFG* dataP = (dataCFG*) arr;

   if (hasConfigChanged(cfgData, dataP))
   {
      // avoid writing to Flash unless there is a need
      copyConfig(dataP, cfgData);
      nvStore.write(*cfgData);
   }
   arr += sizeof(dataCFG);
    
   if (getIntFromArray(arr) != SYSEX_CONFIG_VELOCITY)
   {
       return false;
   }
   arr += sizeof(int);

   velocityCFG* veloP = (velocityCFG*) arr;
   bool programChanged = false;
   if (hasConfigChanged(&(cfgProgram->velocityConfig), veloP))
   {
      // avoid writing to Flash unless there is a need
      copyConfig(veloP, &(cfgProgram->velocityConfig));
      programChanged = true;
   }

   arr += sizeof(velocityCFG);

   if (getIntFromArray(arr) != SYSEX_CONFIG_GATE)
   {
       return false;
   }
   arr += sizeof(int);

   gateCFG* gateP = (gateCFG*) arr;
   decodeForSysex(gateP);
   if (hasConfigChanged(&(cfgProgram->gateConfig), gateP))
   {
      // avoid writing to Flash unless there is a need
      copyConfig(gateP, &(cfgProgram->gateConfig));
      programChanged = true;
      mapFixedDurationConfig();
   }

   // I know this line is not really needed, but I don't want it forgotten when we extend this method
   arr += sizeof(gateCFG);

   if (programChanged) 
   {
      programStore.write(*cfgProgram);
   }
   
   return true;
}

bool dadaSysEx::handleSysExUSBPacket(midiEventPacket_t rx)
{
    bool ret = false;
    byte b;

    for(int i = 1; i < 4; ++i) {
      switch(i) {
        case 1:
          b = rx.byte1;
        break;
        case 2:
          b = rx.byte2;
        break;
        case 3:
          b = rx.byte3;
        break;
      }

      if (b == SYSEX_END) {
        UsbSysExBuffer[UsbSysExCursor++] = b;
        
        ret = handleSysEx(UsbSysExBuffer, UsbSysExCursor);
        UsbSysExCursor = 0;
        break;
      } else if (((b & 0x80) == 0) || (b == SYSEX_START)) {
        if (b == SYSEX_START) {
            UsbSysExCursor = 0;
        }
         UsbSysExBuffer[UsbSysExCursor++] = b;

         if (UsbSysExCursor >= MAX_SYSEX_MESSAGE_SIZE) {
           // Something went wrong with message, abort
           UsbSysExCursor = 0;
           break;
         }
      }
    }

    return ret;
}

void dadaSysEx::saveConfigToSysEx()
{
   byte* outP = &sysexOutArr[0];

   *outP++ = SYSEX_START;

   *outP++ = 0;

   outP = putIntToArray(outP, SYSEX_CONFIG_HEADER);

   outP = putIntToArray(outP, SYSEX_CONFIG_PINS);

   dataCFG* dataP = (dataCFG*) outP;
   copyConfig(cfgData, dataP);
   sanitizeForSysex(dataP);
   outP += sizeof(dataCFG);
    
   outP = putIntToArray(outP, SYSEX_CONFIG_VELOCITY);

   velocityCFG* veloP = (velocityCFG*) outP;
   copyConfig(&(cfgProgram->velocityConfig), veloP);
   sanitizeForSysex(veloP);
   outP += sizeof(velocityCFG);

   outP = putIntToArray(outP, SYSEX_CONFIG_GATE);

   gateCFG* gateP = (gateCFG*) outP;
   copyConfig(&(cfgProgram->gateConfig), gateP);
   encodeForSysex(gateP);
   outP += sizeof(gateCFG);

   *outP = SYSEX_END;

   // the midi2.send function probably doesn't do anything with the current hardware, but I'm leaving it in for completeness
   if (midi2 != NULL) {
       midi2->sendSysEx(SYSEX_CONFIG_LEN, sysexOutArr, true);
   }
   MidiUSB_sendSysEx(sysexOutArr, SYSEX_CONFIG_LEN);
}

void dadaSysEx::sanitizeForSysex(velocityCFG* veloP)
{
  for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
  {
    if(veloP->velocityProgram[i] < MIN_PROGRAM || veloP->velocityProgram[i] > MAX_PROGRAM)
    {
      veloP->velocityProgram[i] = ALWAYS_ON_PROGRAM;
    }
  }
}

void dadaSysEx::encodeForSysex(gateCFG* gateP)
{ // we need to avoid having the high bit set in any byte
  for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
  {
    if(gateP->durationConfiguration[i] < 0)
    {
      gateP->durationConfiguration[i] = 0;
    }
    else if(gateP->durationConfiguration[i] > 16383)
    { // this value can only be 14 bits max
      gateP->durationConfiguration[i] = 16383;
    }
    int lowerPart = gateP->durationConfiguration[i] & 0x007F;
    int upperPart = gateP->durationConfiguration[i] & 0x3F80;
    gateP->durationConfiguration[i] = lowerPart | (upperPart << 1);
  }
}

void dadaSysEx::decodeForSysex(gateCFG* gateP)
{
  for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
  {
    int lowerPart = gateP->durationConfiguration[i] & 0x007F;
    int upperPart = gateP->durationConfiguration[i] & 0x7F00;
    gateP->durationConfiguration[i] = lowerPart | (upperPart >> 1);
  }
}

void dadaSysEx::sendVersionToSysEx()
{
   byte* outP = &sysexOutArr[0];

   *outP++ = SYSEX_START;

   *outP++ = 0;

   outP = putIntToArray(outP, SYSEX_VERSION_HEADER);

   outP = putIntToArray(outP, SYSEX_FIRMWARE_VERSION);

   *outP = SYSEX_END;

   // the midi2.send function probably doesn't do anything with the current hardware, but I'm leaving it in for completeness
   if (midi2 != NULL) {
       midi2->sendSysEx(SYSEX_VERSION_LEN, sysexOutArr, true);
   }
   MidiUSB_sendSysEx(sysexOutArr, SYSEX_VERSION_LEN);
}

void dadaSysEx::sanitizeForSysex(dataCFG* dataP)
{
     // we need to avoid any values > 127 for sysex
     for (int j = 0; j < 8; ++j)
     {
       dataP->alignfiller[j] = 0;
     }
     
     for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
     {
       if(dataP->midiChannels[i] < 0 || dataP->midiChannels[i] > 127)
       {
         dataP->midiChannels[i] = MIDI_CHANNEL_OMNI;
       }
       
       if(dataP->midiPins[i] < 0 || dataP->midiPins[i] > 127)
       {
         dataP->midiPins[i] = 0;
       }
     }
}
  
bool dadaSysEx::hasConfigChanged(dataCFG* config1, dataCFG* config2)
{
    for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
    {
      if(config1->midiChannels[i] != config2->midiChannels[i])
      {
        return true;
      }
      if(config1->midiPins[i] != config2->midiPins[i])
      {
        return true;
      }
    }

    return false;
}
  
inline void dadaSysEx::copyConfig(dataCFG* src, dataCFG* dest)
{
    for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
    {
      dest->midiChannels[i] = src->midiChannels[i];
      dest->midiPins[i] = src->midiPins[i];
    }

    for (int j = 0; j < 8; ++j)
    {
      dest->alignfiller[j] = 0;
    }
}
    
bool dadaSysEx::hasConfigChanged(velocityCFG* config1, velocityCFG* config2)
{
    for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
    {
      if(config1->velocityProgram[i] != config2->velocityProgram[i])
      {
        return true;
      }
    }

    return false;
}
  
inline void dadaSysEx::copyConfig(velocityCFG* src, velocityCFG* dest)
{
    for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
    {
      dest->velocityProgram[i] = src->velocityProgram[i];
    }
}

bool dadaSysEx::hasConfigChanged(gateCFG* config1, gateCFG* config2)
{
    for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
    {
      if(config1->durationConfiguration[i] != config2->durationConfiguration[i])
      {
        return true;
      }
    }

    return false;
}
  
inline void dadaSysEx::copyConfig(gateCFG* src, gateCFG* dest)
{
    for (int i = 0; i < OUTPUT_PINS_COUNT; ++i)
    {
      dest->durationConfiguration[i] = src->durationConfiguration[i];
    }
}
  
inline void dadaSysEx::MidiUSB_sendSysEx(byte *data, size_t len)
{
  byte midiData[4];
  const byte *pData = data;
  int bytesRemaining = len;

  while (bytesRemaining > 0) {
      switch (bytesRemaining) {
      case 1:
          midiData[0] = 5;
          midiData[1] = *pData;
          midiData[2] = 0;
          midiData[3] = 0;
          bytesRemaining = 0;
          break;
      case 2:
          midiData[0] = 6;
          midiData[1] = *pData++;
          midiData[2] = *pData;
          midiData[3] = 0;
          bytesRemaining = 0;
          break;
      case 3:
          midiData[0] = 7;
          midiData[1] = *pData++;
          midiData[2] = *pData++;
          midiData[3] = *pData;
          bytesRemaining = 0;
          break;
      default:
          midiData[0] = 4;
          midiData[1] = *pData++;
          midiData[2] = *pData++;
          midiData[3] = *pData++;
          bytesRemaining -= 3;
          break;
      }
      MidiUSB.write(midiData, 4);
      delay(1);
  }
}

inline int dadaSysEx::getIntFromArray(byte* arr)
{
  int ret = (arr[0] & 0x0FF) << 24;
  ret |= (arr[1] & 0x0FF)  << 16;
  ret |= (arr[2] & 0x0FF)  << 8;
  ret |= (arr[3] & 0x0FF) ;
  return ret;
}
  
inline byte* dadaSysEx::putIntToArray(byte* arr, int in)
{
  arr[0] = in >> 24;
  arr[1] = (in >> 16) & 0x0FF;
  arr[2] = (in >> 8) & 0x0FF;
  arr[3] = in & 0x0FF;

  return arr + sizeof(int);
}
#endif
