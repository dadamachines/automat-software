// EEPROM replacement Lib find in "Manage Libraries"and here https://github.com/cmaglie/FlashStorage
/*

   Testplan:
   - Midi Speed
   - Connect both Din + USB and send a lot of data
   - Learn simple (single press button -  root node + chromatic up
   - Learn advanced (double press button - assign all notes in sequence
   -

*/

#include <FlashAsEEPROM.h>
#include <MIDI.h>
#include <MIDIUSB.h>
#include <SPI.h>
#include <OneButton.h>

// constants
const int OUTPUT_PINS_COUNT = 12;                       //= sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]);
const int LEARN_MODE_PIN = 38;                          // pin for the learn mode switch
const int SHIFT_REGISTER_ENABLE = 27;                   // Output enable for shiftregister ic
const int ACTIVITY_LED = 13;                            // activity led is still on D13 which is connected to PA17 > which means Pin 9 on MKRZero

// NV Data
typedef struct {
  byte   midiChannels[12];                                // 1-16 or 0 for any
  byte   midiPins[12];                                    // midi notes
  byte   alignfiller[8];                                  // for eeprom support
} dataCFG;
dataCFG nvData;

FlashStorage(nvStore, dataCFG);

#include "solenoidSPI.h"
SOLSPI solenoids(&SPI, 30);                             // PB22 Pin in new layout is Pin14 on MKRZero

#include "dadaStatusLED.h"
dadaStatusLED statusLED(ACTIVITY_LED);                    // led controller

#include "dadaMidiLearn.h"                              // learn class

// Objects
OneButton button(LEARN_MODE_PIN, true);                 // 38 Pin in new layout is Pin 38 used for SD Card on MKRZero

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midi2);   // DIN Midi Stuff
dadaMidiLearn midiLearn(&nvData);                       // lern class + load/save from eeprom

void setup() {
  Serial1.begin(31250);                                 // set up MIDI baudrate
  pinMode(SHIFT_REGISTER_ENABLE, OUTPUT);               // enable Shiftregister
  digitalWrite(SHIFT_REGISTER_ENABLE, LOW);
  pinMode(ACTIVITY_LED, OUTPUT);                        // pin leds to output
  pinMode(LEARN_MODE_PIN, INPUT_PULLUP);
  button.attachDoubleClick(doubleclick);                // register button for learnmodes
  button.attachClick(singleclick);                      // register button for learnmodes
  solenoids.begin();                                    // start shiftregister

  midi2.setHandleNoteOn(handleNoteOn);                  // add Handler for Din MIDI
  midi2.setHandleNoteOff(handleNoteOff);
  midi2.begin(MIDI_CHANNEL_OMNI);
  // init();
  statusLED.blink(20, 30, 32);
}

void loop() {
  midi2.read();
  button.tick();
  statusLED.tick();

  // handle blinking port on learning in advanced mode
  if(midiLearn.active) {
    if(midiLearn.mode==1) {
        solenoids.singlePin(midiLearn.counter,statusLED._state );
    }
  }




  // now handle usb midi and merge with DinMidi callbacks
  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();
    if (rx.header != 0) {
      switch (rx.byte1 & 0xF0) {
        case 0x90:  // note on
          if (rx.byte3 != 0)
            handleNoteOn(1 + (rx.byte1 & 0xF), rx.byte2, rx.byte3);
          else
            handleNoteOff(1 + (rx.byte1 & 0xF), rx.byte2, rx.byte3);
          break;
        case 0x80: // note off
          handleNoteOff(1 + (rx.byte1 & 0xF), rx.byte2, rx.byte3);
          break;
      }
    }
  } while (rx.header != 0);
}

/************************************************************************************************************************************************/
/************************************************************************************************************************************************/
/************************************************************************************************************************************************/
/************************************************************************************************************************************************/
/************************************************************************************************************************************************/
/************************************************************************************************************************************************/



void handleNoteOn(byte channel, byte note, byte velocity) {
  midiLearn.noteOn(channel, note, velocity);

  if (midiLearn.active) {
    return;
  }

  statusLED.blink(1, 2, 1);

  
  for (int i = 0 ; i < 12 ; i++) {
    if (nvData.midiPins[i] == note) {
      if (nvData.midiChannels[i] == channel || nvData.midiChannels[i] == 0) {
        solenoids.setOutput(i);
      }
    }
  }

}

void handleNoteOff(byte channel, byte note, byte velocity) {
  midiLearn.noteOff(channel, note, velocity);

  if (midiLearn.active) {
    return;
  }
  
  statusLED.blink(1, 2, 1);
  
  for (int i = 0 ; i < 12 ; i++) {
    if (nvData.midiPins[i] == note) {
      if (nvData.midiChannels[i] == channel || nvData.midiChannels[i] == 0) {
        solenoids.clearOutput(i);
      }
    }
  }
}

// Advanced Learn
void doubleclick() {
  statusLED.blink(20, 20, -1);  // LED Settings (On Time, Off Time, Count)
  midiLearn.begin(1);
}

// Simple Learn
void singleclick(void)  {
  statusLED.blink(10, 0, -1); // LED Settings (On Time, Off Time, Count)
  midiLearn.begin(0);
}


