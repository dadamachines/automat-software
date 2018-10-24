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


#ifndef _DADASYSEX_H
#define _DADASYSEX_H

#include <MIDI.h>
#include <MIDIUSB.h>


class dadaSysEx {

  private:
    // 'dA' = H64H41 which is way above the currently allocated Sysex manufacturer IDs so this shouldn't conflict with any existing IDs
    static const int SYSEX_CONFIG_HEADER = 'dAdA';
    static const int SYSEX_VERSION_HEADER = 'dAdV';
    static const int SYSEX_CONFIG_PINS = 'pins';
    static const int SYSEX_CONFIG_VELOCITY = 'velo';
    static const int SYSEX_CONFIG_GET_CONFIG = 'getc';
    static const int SYSEX_CONFIG_GET_VERSION = 'getv';
    static const int SYSEX_CONFIG_LEN = 3 + (sizeof (int) * 3) + sizeof(dataCFG) + sizeof(velocityCFG);
    static const int SYSEX_GET_CONFIG_LEN = 3 + (sizeof (int) * 2);
    static const int SYSEX_VERSION_LEN = 3 + (sizeof (int) * 2);
    static const int MAX_SYSEX_MESSAGE_SIZE = 128;
  
    static byte sysexOutArr[SYSEX_CONFIG_LEN];
    static byte UsbSysExBuffer[MAX_SYSEX_MESSAGE_SIZE];

    dataCFG * cfgData;
    velocityCFG * cfgVelocity;
    MIDI_NAMESPACE::MidiInterface<HardwareSerial>* midi2;
    int UsbSysExCursor;
  
  public:
    
    dadaSysEx(dataCFG * mynv, velocityCFG* velnv, MIDI_NAMESPACE::MidiInterface<HardwareSerial>* midiIn) {
      cfgData = mynv;
      cfgVelocity = velnv;
      midi2 = midiIn;
      UsbSysExCursor = 0;
    };

    inline bool handleSysEx(byte * arr, unsigned len);

    inline bool handleSysExUSBPacket(midiEventPacket_t rx);
 
    inline void saveConfigToSysEx();
    
    inline static void sanitizeVelocityConfig(velocityCFG* veloP) {
      sanitizeForSysex(veloP);
    }

    bool inSysExReceive() {
      return UsbSysExCursor > 0;
    }

protected:
  inline void sendVersionToSysEx();

  inline static void sanitizeForSysex(dataCFG* dataP);
  inline static void sanitizeForSysex(velocityCFG* veloP);
  
  inline static bool hasConfigChanged(dataCFG* config1, dataCFG* config2);
  inline static bool hasConfigChanged(velocityCFG* config1, velocityCFG* config2);
  
  inline static void copyConfig(dataCFG* src, dataCFG* dest);
  inline static void copyConfig(velocityCFG* src, velocityCFG* dest);
      
  inline static void MidiUSB_sendSysEx(byte *data, size_t len);
  
private:
  inline static int getIntFromArray(byte* arr);
  
  inline static byte* putIntToArray(byte* arr, int in);
};

#endif
