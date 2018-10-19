// EEPROM replacement Lib find in "Manage Libraries"and here https://github.com/cmaglie/FlashStorage

#include <FlashAsEEPROM.h>
#include <MIDI.h>
#include <MIDIUSB.h>
#include <SPI.h>
#include <OneButton.h>
#include <Wire.h>

// constants
const int OUTPUT_PINS_COUNT = 12;                       //= sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]);
const int LEARN_MODE_PIN = 38;                          // pin for the learn mode switch
const int SHIFT_REGISTER_ENABLE = 27;                   // Output enable for shiftregister ic
const int ACTIVITY_LED = 13;                            // activity led is still on D13 which is connected to PA17 > which means Pin 9 on MKRZero

const int ALWAYS_ON_PROGRAM = 0;                        // The index of the default always on program
const int QUADRATIC_PROGRAM = 1;                        // The index of the quadratic one pulse program
const int INVERSE_QUADRATIC_PROGRAM = 2;                // The index of the inverse quadratic one pulse program
const int PWM_PROGRAM = 3;                              // The index of the pwm multi-pulse program
const int PWM_MOTOR_PROGRAM = 4;                        // The index of the pwm continous program
const int HUM_MOTOR_PROGRAM = 5;                        // The index of the making the motor hum to a note
const int MIN_PROGRAM = 0;                              // The index of the minimum valid program
const int MAX_PROGRAM = 5;                              // The index of the maximum valid program

const byte SYSEX_START = 0xF0;
const byte SYSEX_END = 0xF7;

// i2c constants
// TODO: this is the temporary i2c address same as the TELEXo Teletype Expander,
// so we can mimic its teletype API. Future address will be 0xDA.
const int AUTOMAT_ADDR = 0x60;
const int I2C_SET = 0;

// NV Data
typedef struct {
  byte   midiChannels[OUTPUT_PINS_COUNT];                                // 1-16 or 0 for any
  byte   midiPins[OUTPUT_PINS_COUNT];                                    // midi notes
  byte   alignfiller[8];                                  // for eeprom support   (Justin: I don't think this is needed. 32-bit alignment should be enough)
} dataCFG;
dataCFG nvData;

typedef struct {
  byte velocityProgram[OUTPUT_PINS_COUNT];
} velocityCFG;
velocityCFG velocityConfig;


FlashStorage(nvStore, dataCFG);
FlashStorage(velocityStore, velocityCFG);


int pwm_countdown[OUTPUT_PINS_COUNT];                     // This is the total number of loops left where we will execute a PWM
int pwm_phase[OUTPUT_PINS_COUNT];                         // This is a repeating counter of PHASE_LIMIT to 0
int pwm_kick[OUTPUT_PINS_COUNT];                          // An initial loop counter where we leave the output high to overcome inertia in the solenoid
int pwm_level[OUTPUT_PINS_COUNT];                         // A counter that indicates how many loop counts we should leave the output high for.
const int COUNTDOWN_CONT = 2147483647;                    // number of loops where we apply the PWM for continous mode > 10 days
const int COUNTDOWN_START = 14400;                        // Maximum number of loops where we apply the PWM
const int NO_COUNTDOWN = 14401;                           // A special value to indicate that we are not using a PWM countdown
const int PHASE_KICK = 64;                                // Number of loops where we leave the output high to overcome inertia in the solenoid

const int PHASE_LIMIT = 32;                               // The number of loop counts we use to execute a PWM cycle.   If this value is too large, the solendoids will emit an audible noise during PWM
                                                          // 64 = approximately 1 ms
const int DOWN_PHASE_MAX = 21;                            // The maximum number of loop counts where the output is held low for a PWM cycle   Values between 13 and 15 are acceptable for a PHASE_LIMIT of 32
const int LEVEL_MAX = 20;                                 // Maximum level value for PWM so 120 to 127 is equal to no PWM
const int VELOCITY_DIVISOR = 6;                           // Divide the velocity value (1-127) by this number to the the PWM level

const int MAX_MIDI_CHANNEL = 16;

int modWheel[MAX_MIDI_CHANNEL + 1] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int humNote[OUTPUT_PINS_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#include "humTiming.h"

#include "solenoidSPI.h"
SOLSPI solenoids(&SPI, 30);                             // PB22 Pin in new layout is Pin14 on MKRZero

#include "dadaStatusLED.h"
dadaStatusLED statusLED(ACTIVITY_LED);                    // led controller

#include "dadaMidiLearn.h"                              // learn class

#include "dadaSysEx.h"
#include "dadaSysEx.hpp"

// Objects
OneButton button(LEARN_MODE_PIN, true);                 // 38 Pin in new layout is Pin 38 used for SD Card on MKRZero

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midi2);   // DIN Midi Stuff
dadaMidiLearn midiLearn(&nvData);                       // lern class + load/save from eeprom
dadaSysEx sysex(&nvData, &velocityConfig, &midi2);

void setup() {
  Serial1.begin(31250);                                 // set up MIDI baudrate
  pinMode(SHIFT_REGISTER_ENABLE, OUTPUT);               // enable Shiftregister
  digitalWrite(SHIFT_REGISTER_ENABLE, LOW);
  pinMode(ACTIVITY_LED, OUTPUT);                        // pin leds to output
  pinMode(LEARN_MODE_PIN, INPUT_PULLUP);
  button.attachDoubleClick(doubleclick);                // register button for learnmodes
  button.attachClick(singleclick);                      // register button for learnmodes
  button.setPressTicks(2000);                           // set a long press to be two seconds
  button.attachLongPressStart(longButtonPress);         // register button for sysex transmission
  solenoids.begin();                                    // start shiftregister

  Wire.begin(AUTOMAT_ADDR);                             // join i2c bus
  Wire.onReceive(receiveI2CEvent);                      // register event

  midi2.setHandleProgramChange(handleProgramChange);
  midi2.setHandleNoteOn(handleNoteOn);                  // add Handler for Din MIDI
  midi2.setHandleNoteOff(handleNoteOff);
  midi2.setHandleSystemExclusive(handleSysEx);
  midi2.setHandleControlChange(handleControlChange);
  midi2.begin(MIDI_CHANNEL_OMNI);
  // init();

  velocityConfig = velocityStore.read();
  // if uninitialized, this value should be read as -1
  sysex.sanitizeVelocityConfig(&velocityConfig);  

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
    int velocity_program = velocityConfig.velocityProgram[i];

    switch (velocity_program)
    {
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

      case PWM_PROGRAM:
      case PWM_MOTOR_PROGRAM:
        {
          // repeating pulse width via velocity
          // If the user has used a very high velocity, we will bypass PWM
          // Also, we will set a limit of pwm_countdown time to not keep the PWM on 
          //  once the solenoid has made contact with the drum, etc.
          if((pwm_countdown[i] == 0) || (pwm_level[i] == 0)) {
              continue;
          }
          
          // First thing we are going to do is leave the solenoids for 'kick' time 
          // to get them moving and overcoming inertia
          if(pwm_kick[i] > 0) {
            pwm_kick[i]--;
            continue;    
          }
          
          // step through the PWM phase sequence
          pwm_phase[i]--;
          pwm_countdown[i]--;
      
          if ((pwm_phase[i] == 0) || (pwm_countdown == 0)) {
            // Restart the phase sequence with the output set high
            solenoids.setOutput(i);
      
            if (pwm_countdown == 0) {
              // we are done the PWM part of the note.   Leave the output high until note off.
              pwm_phase[i] = 0;              
            }
            else {
              // Restart the PWM counter
              pwm_phase[i] = PHASE_LIMIT;      
            }
          }
          else if (pwm_phase[i] == (DOWN_PHASE_MAX - pwm_level[i])) {
            // we are in the low part of the PWM cycle
            solenoids.clearOutput(i);
          }
        }
        break;

      case HUM_MOTOR_PROGRAM:
        {
          // repeating pulse width tuned to a pitch with modulation
          if(pwm_level[i] == 0) {
              continue;
          }

          // step through the PWM phase sequence
          pwm_phase[i]--;

          if ((pwm_phase[i] == 0) || (pwm_countdown == 0)) {
            // Restart the PWM counter
            pwm_phase[i] = calculateTotalHumPhase(i);
            pwm_level[i] = calculateLoHumPhase(i);
            if (pwm_level[i] > 0) {
              // Restart the phase sequence with the output set high
              solenoids.setOutput(i);
            }
          }
          else if (pwm_phase[i] == pwm_level[i]) {
            // we are in the low part of the PWM cycle
            solenoids.clearOutput(i);
          }
        }
        break;
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
      if (velocityConfig.velocityProgram[pin] != patch) {
        velocityConfig.velocityProgram[pin] = patch;
        configChanged = true;
      }
    }
  }

  if (configChanged) {
     velocityStore.write(velocityConfig);
  }

  statusLED.blink(2, 1, 2); // LED Settings (On Time, Off Time, Count)
}

void handleNoteOn(byte pin, byte velocity) {
    solenoids.setOutput(pin);

    int velocity_program = velocityConfig.velocityProgram[pin];

    switch (velocity_program) 
    {
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
        case PWM_PROGRAM: // true pwm
        case PWM_MOTOR_PROGRAM: // continuous PWM
          pwm_level[pin] = (velocity / VELOCITY_DIVISOR) + 1;
          if(pwm_level[pin] > LEVEL_MAX) {
            pwm_countdown[pin] = 0;
            pwm_phase[pin] = 0;
            pwm_kick[pin] = 0;
            pwm_level[pin] = 0;
          }
          else {
            pwm_countdown[pin] = (velocity_program == PWM_MOTOR_PROGRAM) ? COUNTDOWN_CONT : COUNTDOWN_START; 
            pwm_phase[pin] = PHASE_LIMIT;
            pwm_kick[pin] = PHASE_KICK;
          }
          break;
        case HUM_MOTOR_PROGRAM:
          if (pwm_level[pin] == 0) {
            pwm_phase[pin] = calculateTotalHumPhase(pin);
            pwm_level[pin] = calculateLoHumPhase(pin);
          }  // otherwise let it calculate the phase at the end of the cycle
          break;
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
    } else if (velocityConfig.velocityProgram[i] == HUM_MOTOR_PROGRAM && nvData.midiChannels[i] == channel) {
        humNote[i] = note;
        handleNoteOn(i, velocity);
    }
  }
}

void handleNoteOff(byte pin) {
  solenoids.clearOutput(pin);
  pwm_countdown[pin] = 0;
  pwm_kick[pin] = 0;
  pwm_phase[pin] = 0;
  pwm_level[pin] = 0;
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
    } else if ((velocityConfig.velocityProgram[i] == HUM_MOTOR_PROGRAM)
                && (nvData.midiChannels[i] == channel)
                && (humNote[i] == note)) {
        handleNoteOff(i);
        humNote[i] = 0;
    }
  }
}

void handleControlChange(byte channel, byte number, byte value) {
  if (number == 1) {
    handleModWheel(channel, value);
  }
}

void handleModWheel(byte channel, byte mod) {
  modWheel[channel] = mod;
  modWheel[MIDI_CHANNEL_OMNI] = mod;
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

int calculateTotalHumPhase(int pin) {
  return NOTE_PERIOD[humNote[pin]];
}

int calculateLoHumPhase(int pin) {
  int channel = nvData.midiChannels[pin];
  int note = humNote[pin];

  if (NOTE_PERIOD[note] == 0) {
    // the math should still result in 0, but just in case...
    return 0;
  }

  float hiPhase = NOTE_PHASE_SCALE[note] * modWheel[channel];
  int iHiPhase = MIN_NOTE_PHASE + (int) (hiPhase + 0.5f);

  if (iHiPhase > MAX_NOTE_PHASE[note]) {
    iHiPhase = MAX_NOTE_PHASE[note];
  }

  return NOTE_PERIOD[note] - iHiPhase;
}


