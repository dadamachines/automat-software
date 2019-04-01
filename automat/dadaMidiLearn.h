/*
   Copyright (c) 2017, DADAMACHINES
   Author: Sven Braun
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


#ifndef _DADALERN_H
#define _DADALERN_H

class dadaMidiLearn {
  public:

    bool      active;
    int8_t    counter;
    uint8_t   mode;   // 0 = basic, 1 = advanced
    dataCFG * nv;
    dadaMidiLearn(dataCFG * mynv) {
      nv = mynv;
      active = false;
      mode = 0;
      counter = 0;
      loadEEPROM();
    };
    
    void begin(uint8_t learnMode) {
      counter = 0;
      mode = learnMode;
      active = true;
      clearMap();
    };

    void clearMap() {
      for (int i = 0 ; i < OUTPUT_PINS_COUNT ; i++) {
        nv->midiChannels[i] = 0;  // no midi filter
        nv->midiNotes[i]=128;      // invalid note
      }
    };

    void noteOn(byte ch, byte note, byte velocity) {
      if (!active) return ;

      // nv->midiChannel = ch;

      if (mode == 0) {
        for (byte i = 0 ; i < OUTPUT_PINS_COUNT ; i++) {
          nv->midiNotes[i] = note + i;
          nv->midiChannels[i] = ch;
        }
        active = false;
        saveEEPROM();
        return;
      }

      if (mode == 1) {
        nv->midiNotes[counter] = note;
        nv->midiChannels[counter] = ch;
        counter++;
        if (counter >= OUTPUT_PINS_COUNT) {
          active = false;
          saveEEPROM();
        }
        return;
      }

    };

    void noteOff(byte ch, byte note, byte velocity) {
      // nothing todo
    };

    // load & save stuff for eeprom addr=0 .. midiChannel, addr=1-128 mapping table
    void saveEEPROM() {
      nvStore.write(nvData);
      statusLED.blink(8, 14, 16);
    };

    void loadEEPROM() {
      nvData = nvStore.read();
      if((nvData.midiChannels[0] & 0xFF) == 0xFF && (nvData.midiNotes[0] & 0xFF) == 0xFF) {
        // EEPROM data is not initialized.   Load defaults
        loadDefaults();
      } else {
        bool allZero = true;

        for(int i = 0; i < OUTPUT_PINS_COUNT; ++i) {
          if (nvData.midiNotes[i] != 0) {
            allZero = false;
            break;
          }
        }

        if(allZero) {
          loadDefaults();
        }
      }

      statusLED.blink(1, 3, 32);
    };

    void loadDefaults() {
       const byte note = 36; // C2 GM MIDI Bass Drum
       for (byte i = 0 ; i < OUTPUT_PINS_COUNT ; i++) {
         nv->midiNotes[i] = note + i;
         nv->midiChannels[i] = MIDI_CHANNEL_OMNI;
      }
    }

};

#endif
