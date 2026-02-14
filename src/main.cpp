#include <Arduino.h>

#include <FlashAsEEPROM.h>
#include <climits>
#include <MIDI.h>
#include <MIDIUSB.h>
#include <OneButton.h>
#include <SPI.h>
#include <Wire.h>

// This comment is read by the build script:
// V2DEVICE_METADATA("com.dadamachines.automat", 3, "dadamachines:samd:automat");

#include "automatConstants.h"

// Forward declarations (required for PlatformIO .cpp builds; Arduino IDE auto-generates these)
void handleNoteOn(byte pin, byte velocity);
void handleNoteOn(byte channel, byte note, byte velocity);
void handleNoteOff(byte pin);
void handleNoteOff(byte channel, byte note, byte velocity);
void handleClock();
void handleStart();
void handleStop();
void handleContinue();
void handleControlChange(byte channel, byte number, byte value);
void handleModWheel(byte channel, byte mod);
void handleAllNotesOff();
void handlePitchBend(byte channel, int bend);
void handleSysEx(byte *arr, unsigned len);
void handleProgramChange(byte channel, byte patch);
void handleMinConfig(byte pin, int val, int power);
void handleMaxConfig(byte pin, int val, int power);
void receiveI2CEvent(int len);
void requestI2CEvent();
void doubleclick();
void singleclick();
void longButtonPress();
void mapFixedDurationConfig();
void initMaxMinMap();
void initMaxMinMap(int pin, int min_range, int max_range, int power);

const int I2C_ADDR     = 0xDA;
const int I2C_SET      = 0; // prepared set of Output Pin and Velocity
const int I2C_MIDI_SET = 1; // MIDI Event set Chanel/Note/Velocity

// NV Data
typedef struct {
  byte midiChannels[OUTPUT_PINS_COUNT]; // 1-16 or 0 for any
  byte midiNotes[OUTPUT_PINS_COUNT];    // midi notes
} dataCFG;
dataCFG nvData;

typedef struct {
  byte velocityProgram[OUTPUT_PINS_COUNT];
  uint16_t min_milli[OUTPUT_PINS_COUNT];
  uint16_t max_milli[OUTPUT_PINS_COUNT];
  int8_t curve_power[OUTPUT_PINS_COUNT];
} velocityCFG;

typedef struct {
  short durationConfiguration[OUTPUT_PINS_COUNT];
} gateCFG;

typedef struct {
  velocityCFG velocityConfig;
  gateCFG gateConfig;
} programCFG;
programCFG programData;

FlashStorage(nvStore, dataCFG);
FlashStorage(programStore, programCFG);

unsigned long milli_stop[OUTPUT_PINS_COUNT]; // time at which to stop note for program 0
int loop_countdown[OUTPUT_PINS_COUNT];       // This is the total number of loops left
// where we will execute a PWM
int gateDuration[OUTPUT_PINS_COUNT]; // This is the total number of loops
// configured for this one-shot trigger
int32_t max_min_map[OUTPUT_PINS_COUNT][128];

#include "solenoidSPI.h"
SOLSPI solenoids(&SPI, 30);

#include "PWMManager.h"
#include "PWMManager.hpp"

#include "dadaStatusLED.h"
dadaStatusLED statusLED(ACTIVITY_LED);

#include "dadaMidiLearn.h"

#include "dadaSysEx.h"
#include "dadaSysEx.hpp"

OneButton button(LEARN_MODE_PIN, true);

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midi2); // DIN Midi Stuff
dadaMidiLearn midiLearn(&nvData);                     // lern class + load/save from eeprom
dadaSysEx sysex(&nvData, &programData, &midi2);

#if SIS_SUPPORT
unsigned int clockcount     = (unsigned int)0xFFFFFFFF;
unsigned int lastclockcount = (unsigned int)0xFFFFFFFF;
byte clockstate             = 0;
byte lastclockstate         = 0;
#endif

void setup() {
  Serial1.begin(31250);                         // set up MIDI baudrate
  pinMode(SHIFT_REGISTER_ENABLE, OUTPUT);       // enable Shiftregister
  digitalWrite(SHIFT_REGISTER_ENABLE, LOW);     //
  pinMode(ACTIVITY_LED, OUTPUT);                // pin leds to output
  pinMode(LEARN_MODE_PIN, INPUT_PULLUP);        //
  button.attachDoubleClick(doubleclick);        // register button for learnmodes
  button.attachClick(singleclick);              // register button for learnmodes
  button.setPressTicks(3000);                   // set a long press to be three seconds
  button.attachLongPressStart(longButtonPress); // register button for sysex transmission
  solenoids.begin();                            // start shiftregister

  Wire.begin(I2C_ADDR);            // join i2c bus
  Wire.onReceive(receiveI2CEvent); // register event
  Wire.onRequest(requestI2CEvent);

  midi2.setHandleProgramChange(handleProgramChange);
  midi2.setHandleNoteOn(handleNoteOn); // add Handler for Din MIDI
  midi2.setHandleNoteOff(handleNoteOff);
  midi2.setHandleSystemExclusive(handleSysEx);
  midi2.setHandleControlChange(handleControlChange);
  midi2.setHandlePitchBend(handlePitchBend);
  midi2.setHandleClock(handleClock);
  midi2.setHandleStart(handleStart);
  midi2.setHandleStop(handleStop);
  midi2.setHandleContinue(handleContinue);
  midi2.begin(MIDI_CHANNEL_OMNI);

  programData = programStore.read();
  // if uninitialized, this value should be read as -1
  sysex.sanitizeVelocityConfig(&(programData.velocityConfig));
  mapFixedDurationConfig();
  initMaxMinMap();

  statusLED.blink(20, 30, 32);
}

void loop() {
  midi2.read();
  button.tick();
  statusLED.tick();
  unsigned long now = 0;

  // handle blinking port on learning in advanced mode
  if (midiLearn.active) {
    if (midiLearn.mode == 1) {
      solenoids.singlePin(midiLearn.counter, statusLED._state);
    }
  }

  for (int i = 0; i < OUTPUT_PINS_COUNT; i++) {
    int velocity_program = programData.velocityConfig.velocityProgram[i];

    switch (velocity_program) {
      case MAX_MIN_PROGRAM:
        if (now == 0) {
          now = millis();
        }
        if (milli_stop[i] > 0 && milli_stop[i] < now) {
          solenoids.clearOutput(i);
          milli_stop[i] = 0;
        }
        break;
      case FIXED_GATE_PROGRAM:
      case QUADRATIC_PROGRAM:
      case INVERSE_QUADRATIC_PROGRAM: {
        // new single pulse width via velocity
        if (loop_countdown[i] > 1) {
          if (loop_countdown[i] < NO_COUNTDOWN) {
            loop_countdown[i]--;
          }
          continue;
        }
        if (loop_countdown[i] > 0) {
          solenoids.clearOutput(i);
          loop_countdown[i] = 0;
        }
      } break;

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
    if (rx.header != 0) {
      if (sysex.inSysExReceive()) {
        if (sysex.handleSysExUSBPacket(rx)) {
          statusLED.blink(20, 10, 8); // LED Settings (On Time, Off Time, Count)
        }
      } else {
        switch (rx.byte1 & 0xF0) {
          case 0x90: // note on
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
            switch (rx.byte1) {
              case 0xF0:
                if (sysex.handleSysExUSBPacket(rx)) {
                  statusLED.blink(20, 10,
                                  8); // LED Settings (On Time, Off Time, Count)
                }
                break;

              case 0xF8: {
                handleClock();
                break;
              }

              case 0xFA: {
                handleStart();
                break;
              }

              case 0xFB: {
                handleContinue();
                break;
              }

              case 0xFC: {
                handleStop();
                break;
              }
            }
            break;
        }
      }
    }
  } while (rx.header != 0);
}

void handleProgramChange(byte channel, byte patch) {

  if (patch > MAX_PROGRAM || patch < MIN_PROGRAM) {
    patch = MIN_PROGRAM;
  }

  bool configChanged = false;

  for (int pin = 0; pin < OUTPUT_PINS_COUNT; ++pin) {
    if (nvData.midiChannels[pin] == channel) {
      if (programData.velocityConfig.velocityProgram[pin] != patch) {
        programData.velocityConfig.velocityProgram[pin] = patch;
        configChanged                                   = true;
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

  switch (velocity_program) {
    case MAX_MIN_PROGRAM: {
      int8_t max_milli = programData.velocityConfig.max_milli[pin];
      if (max_milli == MAX_MIN_INFINITE) {
        milli_stop[pin] = ULONG_MAX;
      } else {
        milli_stop[pin] = millis() + max_min_map[pin][velocity];
      }
      break;
    }

    case FIXED_GATE_PROGRAM:
      loop_countdown[pin] = gateDuration[pin]; // set velocity timer
      break;

    case QUADRATIC_PROGRAM:                      // strategy from 1.1.0  quadratic
      loop_countdown[pin] = velocity * velocity; // set velocity timer
      break;

    case INVERSE_QUADRATIC_PROGRAM: // inverse quadratic
      if (velocity < 120) {
        velocity            = 120 - velocity;
        loop_countdown[pin] = COUNTDOWN_START - ((velocity * velocity) * 7 / 8);
      } else {
        loop_countdown[pin] = NO_COUNTDOWN;
      }
      break;

#if PWM_SUPPORT
    case PWM_PROGRAM:       // true pwm
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

  for (int i = 0; i < OUTPUT_PINS_COUNT; i++) {
    if (nvData.midiNotes[i] == note) {
      if (nvData.midiChannels[i] == channel || nvData.midiChannels[i] == MIDI_CHANNEL_OMNI) {
        handleNoteOn(i, velocity);
      }
#if PWM_SUPPORT
    } else if (programData.velocityConfig.velocityProgram[i] == HUM_MOTOR_PROGRAM &&
               nvData.midiChannels[i] == channel) {
      PWMManager::handleHumNoteOn(i, note);
      handleNoteOn(i, velocity);
#endif
    }
  }
}

void handleNoteOff(byte pin) {
  solenoids.clearOutput(pin);
  loop_countdown[pin] = 0;
  milli_stop[pin]     = 0;
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

  for (int i = 0; i < OUTPUT_PINS_COUNT; i++) {
    if (nvData.midiNotes[i] == note) {
      if (nvData.midiChannels[i] == channel || nvData.midiChannels[i] == MIDI_CHANNEL_OMNI) {
        handleNoteOff(i);
      }
#if PWM_SUPPORT
    } else if ((programData.velocityConfig.velocityProgram[i] == HUM_MOTOR_PROGRAM) &&
               (nvData.midiChannels[i] == channel)) {
      if (PWMManager::handleHumNoteOff(i, note)) {
        handleNoteOff(i);
      }
#endif
    }
  }
}

void handleClock() {
#if SIS_SUPPORT
  if (clockcount != (unsigned int)0xFFFFFFFF) {
    ++clockcount;
  } else {
    clockcount = 0;
  }
#endif
}

void handleStart() {
#if SIS_SUPPORT
  if (clockcount != (unsigned int)0xFFFFFFFF) {
    clockstate = 1;
  }
#endif
}

void handleContinue() {
#if SIS_SUPPORT
  if (clockcount != (unsigned int)0xFFFFFFFF) {
    clockstate = 2;
  }
#endif
}

void handleStop() {
#if SIS_SUPPORT
  if (clockcount != (unsigned int)0xFFFFFFFF) {
    clockstate = 4;
  }
#endif
}

void handleControlChange(byte channel, byte number, byte value) {
  switch (number) {
    case MIDI_CC_MOD_WHEEL:
      handleModWheel(channel, value);
      break;
    case MIDI_CC_ALL_NOTES_OFF:
      handleAllNotesOff();
      break;
  }
}

void handleModWheel(byte channel, byte mod) {
#if PWM_SUPPORT
  PWMManager::handleModWheel(channel, mod);
#endif
}

void handleAllNotesOff() {
  for (int i = 0; i < OUTPUT_PINS_COUNT; i++) {
    handleNoteOff(i);
  }

#if PWM_SUPPORT
  PWMManager::handleAllNotesOff();
#endif
}

void handleMinConfig(byte pin, int val, int power) {
  if (pin < OUTPUT_PINS_COUNT && programData.velocityConfig.velocityProgram[pin] == MAX_MIN_PROGRAM) {
    programData.velocityConfig.min_milli[pin]   = val;
    programData.velocityConfig.curve_power[pin] = power;
    if (programData.velocityConfig.max_milli[pin] < programData.velocityConfig.min_milli[pin]) {
      programData.velocityConfig.max_milli[pin] = programData.velocityConfig.min_milli[pin];
    }
    initMaxMinMap(pin,
                  programData.velocityConfig.min_milli[pin],
                  programData.velocityConfig.max_milli[pin],
                  programData.velocityConfig.curve_power[pin]);
    statusLED.blink(1, 2, 1);
  }
}

void handleMaxConfig(byte pin, int val, int power) {
  if (pin < OUTPUT_PINS_COUNT && (programData.velocityConfig.velocityProgram[pin] == MAX_MIN_PROGRAM)) {
    programData.velocityConfig.max_milli[pin]   = val;
    programData.velocityConfig.curve_power[pin] = power;
    if (programData.velocityConfig.max_milli[pin] < programData.velocityConfig.min_milli[pin]) {
      programData.velocityConfig.min_milli[pin] = programData.velocityConfig.max_milli[pin];
    }
    initMaxMinMap(pin,
                  programData.velocityConfig.min_milli[pin],
                  programData.velocityConfig.max_milli[pin],
                  programData.velocityConfig.curve_power[pin]);
    statusLED.blink(1, 2, 1);
  }
}

void handlePitchBend(byte channel, int bend) {
#if PWM_SUPPORT
  PWMManager::handlePitchBend(channel, bend);
#endif
}

void requestI2CEvent() {
#if SIS_SUPPORT
  if ((lastclockcount == clockcount) && (lastclockstate == clockstate)) {
    Wire.write(127);
  } else {
    byte msg[5];
    msg[0] = clockstate;
    msg[1] = clockcount >> 24;
    msg[2] = (clockcount >> 16) & 0x0FF;
    msg[3] = (clockcount >> 8) & 0x0FF;
    msg[4] = clockcount & 0x0FF;

    Wire.write(msg, 5);
    lastclockcount = clockcount;
    lastclockstate = clockstate;
  }
#endif
}

void receiveI2CEvent(int len) {
  int r = Wire.read();
  statusLED.blink(1, 2, 1);

  switch (r) {
    case I2C_SET: // received prepared set of OutputPin and Velocity (=0)
      if (len == 3) {
        uint8_t pin      = Wire.read();
        uint8_t velocity = Wire.read();
        if (pin < OUTPUT_PINS_COUNT) {
          if (velocity > 0) {
            handleNoteOn(pin, velocity);
          } else {
            handleNoteOff(pin);
          }
        }
      }
      break;
    case I2C_MIDI_SET: // received MIDI Event set containing up to 3 Bytes (=1)
      if (len == 4) {
        Wire.read(); // status byte (consumed but not used)
        uint8_t byte2 = Wire.read();
        uint8_t byte3 = Wire.read();

#if SIS_SUPPORT
        uint8_t pin      = byte2 - 1;
        uint8_t velocity = byte3;
        if (pin < OUTPUT_PINS_COUNT) {
          if (velocity > 0) {
            handleNoteOn(pin, velocity);
          } else {
            handleNoteOff(pin);
          }
        }
#endif
      }
      break;
  }
}

// Advanced Learn
void doubleclick() {
  statusLED.blink(10, 10, -1); // LED Settings (On Time, Off Time, Count)
  midiLearn.begin(1);
}

// Simple Learn
void singleclick(void) {
  statusLED.blink(10, 0, -1); // LED Settings (On Time, Off Time, Count)
  midiLearn.begin(0);
}

void longButtonPress(void) {
  sysex.saveConfigToSysEx();
  statusLED.blink(8, 4, 4); // LED Settings (On Time, Off Time, Count)
}

void handleSysEx(byte *arr, unsigned len) {
  if (sysex.handleSysEx(arr, len)) {
    statusLED.blink(20, 10, 8); // LED Settings (On Time, Off Time, Count)
  }
}

void mapFixedDurationConfig() {
  for (int i = 0; i < OUTPUT_PINS_COUNT; ++i) {
    if (programData.velocityConfig.velocityProgram[i] == FIXED_GATE_PROGRAM) {
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

void initMaxMinMap() {
  for (int pin = 0; pin < OUTPUT_PINS_COUNT; ++pin) {
    int min_range = programData.velocityConfig.min_milli[pin];
    int max_range = programData.velocityConfig.max_milli[pin];
    int power     = programData.velocityConfig.curve_power[pin];
    initMaxMinMap(pin, min_range, max_range, power);
  }
}

void initMaxMinMap(int pin, int min_range, int max_range, int power) {
  if ((power & 0x10) != 0) {
    power = (power & 0x0F) * -1;
  }
  if (max_range == MAX_MIN_INFINITE) {
    for (int i = 0; i < 128; i++) {
      max_min_map[pin][i] = ULONG_MAX;
    }
    return;
  }

  if (min_range < 1) {
    min_range = 1;
  }

  int range    = ((float)(max_range - min_range)) + 0.5f;
  int base_val = min_range;

  float fraction, y;

  for (int i = 0; i < 128; i++) {
    // Map the input range of 0..127 to a value between 0..1.

    if (power < 0) {
      fraction = ((float)(127 - i)) / 126.0;
      y        = 1 - pow(fraction, -power);
    } else {
      fraction = ((float)i) / 127.f;

      // Map 0..1 to 0..1, but let it grow exponentially.
      y = pow(fraction, power);
    }

    // Convert to a value between 0..1000. We add the base
    // value to assure that we produce growing values; otherwise
    // the first numbers in the sequence would be rounded to the
    // same values.
    int v = (y * range) + base_val;

    /*
    // Round 500..1000 in 10 steps increment.
    if (v >= 500) {
      v -= v % 10;
    // Round 150..499 in 5 steps increment.
    } else if (v >= 150) {
      v -= v % 5;
    }
    */

    max_min_map[pin][i] = v;
  }
}
