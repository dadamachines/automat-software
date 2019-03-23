// EEPROM replacement Lib find in "Manage Libraries"and here https://github.com/cmaglie/FlashStorage

#include <FlashAsEEPROM.h>
#include <MIDI.h>
#include <MIDIUSB.h>
#include <SPI.h>
#include <OneButton.h>
#include <Wire.h>

// constants
const int SYSEX_FIRMWARE_VERSION = 0x01000403;          // = version 1.4.3

const int OUTPUT_PINS_COUNT = 12;                       //= sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]);
const int LEARN_MODE_PIN = 38;                          // pin for the learn mode switch
const int SHIFT_REGISTER_ENABLE = 27;                   // Output enable for shiftregister ic
const int ACTIVITY_LED = 13;                            // activity led is still on D13 which is connected to PA17 > which means Pin 9 on MKRZero

const int ALWAYS_ON_PROGRAM = 0;                        // The index of the default always on program
const int QUADRATIC_PROGRAM = 1;                        // The index of the quadratic one pulse program
const int INVERSE_QUADRATIC_PROGRAM = 2;                // The index of the inverse quadratic one pulse program
/* programs 3 to 5 are reserved by the PWMManager */
const int FIXED_GATE_PROGRAM = 6;                       // The index of the one-pulse program with a configured gate duration
const int MIN_PROGRAM = 0;                              // The index of the minimum valid program
const int MAX_PROGRAM = 6;                              // The index of the maximum valid program

enum {
  MIDI_CC_MOD_WHEEL = 1,
  MIDI_CC_ALL_NOTES_OFF = 123
};

// i2c constants
// TODO: this is the temporary i2c address same as the TELEXo Teletype Expander,
// so we can mimic its teletype API. Future address will be 0xDA.
const int AUTOMAT_ADDR = 0x60;
const int I2C_SET = 0;

// NV Data
typedef struct {
  byte   midiChannels[OUTPUT_PINS_COUNT];                 // 1-16 or 0 for any
  byte   midiPins[OUTPUT_PINS_COUNT];                     // midi notes
  byte   alignfiller[8];                                  // for eeprom support   (Justin: I don't think this is needed. 32-bit alignment should be enough)
} dataCFG;
dataCFG nvData;

typedef struct {
  byte   velocityProgram[OUTPUT_PINS_COUNT];
} velocityCFG;

typedef struct {
  short  durationConfiguration[OUTPUT_PINS_COUNT];
} gateCFG;

typedef struct {
  velocityCFG velocityConfig;
  gateCFG     gateConfig;
} programCFG;
programCFG programData;

FlashStorage(nvStore, dataCFG);
FlashStorage(programStore, programCFG);

const int MAX_MIDI_CHANNEL = 16;

int pwm_countdown[OUTPUT_PINS_COUNT];                     // This is the total number of loops left where we will execute a PWM
const int NO_COUNTDOWN = 14401;                           // A special value to indicate that we are not using a PWM countdown
const int COUNTDOWN_START = 14400;                        // Maximum number of loops where we apply the PWM
const float LOOP_TIME_FACTOR = 64.0f;                     // The number of loops ms per ms according to my cheap oscilliscope

int gateDuration[OUTPUT_PINS_COUNT];                      // This is the total number of loops configured for this one-shot trigger


#include "solenoidSPI.h"
SOLSPI solenoids(&SPI, 30);                             // PB22 Pin in new layout is Pin14 on MKRZero

#include "PWMManager.h"
#include "PWMManager.hpp"


#include "dadaStatusLED.h"
dadaStatusLED statusLED(ACTIVITY_LED);                    // led controller

#include "dadaMidiLearn.h"                              // learn class

#include "dadaSysEx.h"
#include "dadaSysEx.hpp"

// Objects
OneButton button(LEARN_MODE_PIN, true);                 // 38 Pin in new layout is Pin 38 used for SD Card on MKRZero

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midi2);   // DIN Midi Stuff
dadaMidiLearn midiLearn(&nvData);                       // lern class + load/save from eeprom
dadaSysEx sysex(&nvData, &programData, &midi2);

void setup() {
  Serial1.begin(31250);                                 // set up MIDI baudrate
  pinMode(SHIFT_REGISTER_ENABLE, OUTPUT);               // enable Shiftregister
  digitalWrite(SHIFT_REGISTER_ENABLE, LOW);
  pinMode(ACTIVITY_LED, OUTPUT);                        // pin leds to output
  pinMode(LEARN_MODE_PIN, INPUT_PULLUP);
  button.attachDoubleClick(doubleclick);                // register button for learnmodes
  button.attachClick(singleclick);                      // register button for learnmodes
  button.setPressTicks(3000);                           // set a long press to be three seconds
  button.attachLongPressStart(longButtonPress);         // register button for sysex transmission
  solenoids.begin();                                    // start shiftregister

  Wire.begin(AUTOMAT_ADDR);                             // join i2c bus
  Wire.onReceive(receiveI2CEvent);                      // register event

  midi2.setHandleProgramChange(handleProgramChange);
  midi2.setHandleNoteOn(handleNoteOn);                  // add Handler for Din MIDI
  midi2.setHandleNoteOff(handleNoteOff);
  midi2.setHandleSystemExclusive(handleSysEx);
  midi2.setHandleControlChange(handleControlChange);
  midi2.setHandlePitchBend(handlePitchBend);
  midi2.begin(MIDI_CHANNEL_OMNI);
  // init();

  programData = programStore.read();
  // if uninitialized, this value should be read as -1
  sysex.sanitizeVelocityConfig(&(programData.velocityConfig));  
  mapFixedDurationConfig();

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

  for(int i = 0 ; i < OUTPUT_PINS_COUNT ; i++){
    int velocity_program = programData.velocityConfig.velocityProgram[i];

    switch (velocity_program)
    {
      case FIXED_GATE_PROGRAM:
      case QUADRATIC_PROGRAM:
      case INVERSE_QUADRATIC_PROGRAM:
        {
        // new single pulse width via velocity
          if(pwm_countdown[i] > 1 ){
              if(pwm_countdown[i] < NO_COUNTDOWN){
                pwm_countdown[i]--;
              }
              continue;
          }
          if(pwm_countdown[i] > 0) {
            solenoids.clearOutput(i);
            pwm_countdown[i]=0;
          }
        }
        break;

#if PWM_SUPPORT
      case PWM_PROGRAM:
      case PWM_MOTOR_PROGRAM:
      case HUM_MOTOR_PROGRAM:
        PWMManager::handlePinLoop(i, velocity_program);
        break;
  #endif
      case ALWAYS_ON_PROGRAM:
      default:
        break;
    }
  }

  // now handle usb midi and merge with DinMidi callbacks
  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();
      if (rx.header != 0)
      {
        if (sysex.inSysExReceive()) {
           if (sysex.handleSysExUSBPacket(rx)) {
              statusLED.blink(20, 10, 8); // LED Settings (On Time, Off Time, Count)
           }
        } else {
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
            case 0xB0: // control change
              handleControlChange(1 + (rx.byte1 & 0xF), rx.byte2, rx.byte3);
              break;
            case 0xC0: // program change
              handleProgramChange(1 + (rx.byte1 & 0xF), rx.byte2);
              break;
            case 0xE0: // pitch wheel
            {
              int p14bit;
              p14bit = (unsigned short)rx.byte3 & 0x7F;
              p14bit <<= 7;
              p14bit |= (unsigned short)rx.byte2 & 0x7F;
              handlePitchBend(1 + (rx.byte1 & 0xF), p14bit - 8192);
              break;
            }
            case SYSEX_START: // SystemExclusive
              if (sysex.handleSysExUSBPacket(rx)) {
                 statusLED.blink(20, 10, 8); // LED Settings (On Time, Off Time, Count)
              }
              break;
          }
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

void handleProgramChange(byte channel, byte patch) {

  if (patch > MAX_PROGRAM || patch < MIN_PROGRAM) {
      patch = ALWAYS_ON_PROGRAM;
  }

  bool configChanged = false;

  for (int pin = 0; pin < OUTPUT_PINS_COUNT; ++pin) {
    if ((nvData.midiChannels[pin] == channel) || (nvData.midiChannels[pin] == MIDI_CHANNEL_OMNI)) {     
      if (programData.velocityConfig.velocityProgram[pin] != patch) {
        programData.velocityConfig.velocityProgram[pin] = patch;
        configChanged = true;
      }
    }
  }

  if (configChanged) {
     programStore.write(programData);
  }

  statusLED.blink(2, 1, 2); // LED Settings (On Time, Off Time, Count)
}

void handleNoteOn(byte pin, byte velocity) {
    solenoids.setOutput(pin);

    int velocity_program = programData.velocityConfig.velocityProgram[pin];

    switch (velocity_program) 
    {
        case FIXED_GATE_PROGRAM:
          pwm_countdown[pin] = gateDuration[pin]; // set velocity timer
          break;
        case QUADRATIC_PROGRAM:  // strategy from 1.1.0  quadratic
          pwm_countdown[pin] = velocity * velocity; // set velocity timer
          break;
        case INVERSE_QUADRATIC_PROGRAM: // inverse quadratic
          if (velocity < 120)
          {
             velocity = 120 - velocity;
             pwm_countdown[pin] = COUNTDOWN_START - ((velocity * velocity) * 7/ 8);
          }
          else
          {
            pwm_countdown[pin] = NO_COUNTDOWN;
          }
          break;
#if PWM_SUPPORT
        case PWM_PROGRAM: // true pwm
        case PWM_MOTOR_PROGRAM: // continuous PWM
        case HUM_MOTOR_PROGRAM:
          PWMManager::handleNoteOn(velocity_program, pin, velocity);
          break;
#endif
        case ALWAYS_ON_PROGRAM:
        default: // no velocity control
         break;
    }
}

void handleNoteOn(byte channel, byte note, byte velocity) {
  midiLearn.noteOn(channel, note, velocity);

  if (midiLearn.active) {
    return;
  }

  statusLED.blink(1, 2, 1);

  for (int i = 0 ; i < OUTPUT_PINS_COUNT ; i++) {
    if (nvData.midiPins[i] == note) {
      if (nvData.midiChannels[i] == channel || nvData.midiChannels[i] == MIDI_CHANNEL_OMNI) {
        handleNoteOn(i, velocity);
      }
#if PWM_SUPPORT
    } else if (programData.velocityConfig.velocityProgram[i] == HUM_MOTOR_PROGRAM && nvData.midiChannels[i] == channel) {
        PWMManager::handleHumNoteOn(i, note);
        handleNoteOn(i, velocity);
#endif
    }
  }
}

void handleNoteOff(byte pin) {
  solenoids.clearOutput(pin);
  pwm_countdown[pin] = 0;
#if PWM_SUPPORT
  PWMManager::handleNoteOff(pin);
#endif
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  midiLearn.noteOff(channel, note, velocity);

  if (midiLearn.active) {
    return;
  }
  
  statusLED.blink(1, 2, 1);
  
  for (int i = 0 ; i < OUTPUT_PINS_COUNT ; i++) {
    if (nvData.midiPins[i] == note) {
      if (nvData.midiChannels[i] == channel || nvData.midiChannels[i] == MIDI_CHANNEL_OMNI) {
        handleNoteOff(i);
      }
#if PWM_SUPPORT
    } else if ((programData.velocityConfig.velocityProgram[i] == HUM_MOTOR_PROGRAM)
                && (nvData.midiChannels[i] == channel)) {
        if (PWMManager::handleHumNoteOff(i, note)) {
          handleNoteOff(i);
        }
#endif
    }
  }
}

void handleControlChange(byte channel, byte number, byte value) {
  if (number == MIDI_CC_MOD_WHEEL) {
    handleModWheel(channel, value);
  } else if (number == MIDI_CC_ALL_NOTES_OFF) {
    handleAllNotesOff();
  }
}

void handleModWheel(byte channel, byte mod) {
#if PWM_SUPPORT
  PWMManager::handleModWheel(channel, mod);
#endif
}

void handleAllNotesOff() {
    for (int i = 0 ; i < OUTPUT_PINS_COUNT ; i++) {
        handleNoteOff(i);
    }

#if PWM_SUPPORT
  PWMManager::handleAllNotesOff();
#endif
}


void handlePitchBend(byte channel, int bend) {
#if PWM_SUPPORT
  PWMManager::handlePitchBend(channel, bend);
#endif
}

void receiveI2CEvent(int len)
{
  int r = Wire.read();
  statusLED.blink(1, 2, 1);

  switch (r) {
    case I2C_SET:
      if (len = 3) {
        char pin = Wire.read();
        char velocity = Wire.read();
        if (velocity > 0) {
          handleNoteOn(pin, 127);
        } else {
          handleNoteOff(pin);
        }
      }
      break;
  }
}

// Advanced Learn
void doubleclick() {
  statusLED.blink(10, 10, -1);  // LED Settings (On Time, Off Time, Count)
  midiLearn.begin(1);
}

// Simple Learn
void singleclick(void)  {
  statusLED.blink(10, 0, -1); // LED Settings (On Time, Off Time, Count)
  midiLearn.begin(0);
}

void longButtonPress(void)  {
  sysex.saveConfigToSysEx();
  statusLED.blink(8, 4, 4); // LED Settings (On Time, Off Time, Count)
}

void handleSysEx(byte * arr, unsigned len) {
  if(sysex.handleSysEx(arr, len)) {
       statusLED.blink(20, 10, 8); // LED Settings (On Time, Off Time, Count)
  }
}

void mapFixedDurationConfig() {
  for(int i = 0; i < OUTPUT_PINS_COUNT; ++i) {
      if(programData.velocityConfig.velocityProgram[i] == FIXED_GATE_PROGRAM) {
        float duration = programData.gateConfig.durationConfiguration[i];
        // limit valid values to 1 to 2000 ms
        if (duration < 1) {
          duration = 1;
        } else if (duration > 2000) {
          duration = 2000;
        }
        gateDuration[i] = (duration * LOOP_TIME_FACTOR) + 0.5f;
      } else {
        gateDuration[i] = 0;
      }
  }
}

