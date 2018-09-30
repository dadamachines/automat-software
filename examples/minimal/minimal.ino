/* 
 This is a minimal sketch meant for your own experiments. 
 It defaults to accept midi on all channels from note C-2 (midi note 0) to B-1 (midi-note 11).
*/

#include <MIDI.h>
#include <MIDIUSB.h>
#include <SPI.h>

// constants
const int OUTPUT_PINS_COUNT = 12;                       //= sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]);
const int LEARN_MODE_PIN = 38;                          // pin for the learn mode switch
const int SHIFT_REGISTER_ENABLE = 27;                   // Output enable for shiftregister ic
const int ACTIVITY_LED = 13;                            // activity led is still on D13 which is connected to PA17 > which means Pin 9 on MKRZero

#include "solenoidSPI.h"
SOLSPI solenoids(&SPI, 30);                             // PB22 Pin in new layout is Pin14 on MKRZero

#include "dadaStatusLED.h"
dadaStatusLED statusLED(ACTIVITY_LED);                  // led controller

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midi2);   // DIN Midi Stuff

void setup() {
  Serial1.begin(31250);                                 // set up MIDI baudrate
  pinMode(SHIFT_REGISTER_ENABLE, OUTPUT);               // enable Shiftregister
  digitalWrite(SHIFT_REGISTER_ENABLE, LOW);
  pinMode(ACTIVITY_LED, OUTPUT);                        // pin leds to output
  pinMode(LEARN_MODE_PIN, INPUT_PULLUP);
  solenoids.begin();                                    // start shiftregister

  midi2.setHandleNoteOn(handleNoteOn);                  // add Handler for Din MIDI
  midi2.setHandleNoteOff(handleNoteOff);
  midi2.begin(MIDI_CHANNEL_OMNI);
  
  statusLED.blink(20, 30, 32);
}

void loop() {
  midi2.read();
  statusLED.tick();

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

void handleNoteOn(byte channel, byte note, byte velocity) {
  statusLED.blink(1, 2, 1);
  
  for (int i = 0 ; i < 12 ; i++) {
    if (note == i) {
      solenoids.setOutput(i);
    }
  }
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  statusLED.blink(1, 2, 1);
  
  for (int i = 0 ; i < 12 ; i++) {
    if (note == i) {
      solenoids.clearOutput(i);
    }
  }
}
