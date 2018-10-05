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

// i2c constants
// TODO: this is the temporary i2c address same as the TELEXo Teletype Expander,
// so we can mimic its teletype API. Future address will be 0xDA.
const int AUTOMAT_ADDR = 0x60;
const int I2C_SET = 0;

// NV Data
typedef struct {
  byte   midiChannels[12];                                // 1-16 or 0 for any
  byte   midiPins[12];                                    // midi notes
  byte   alignfiller[8];                                  // for eeprom support
} dataCFG;
dataCFG nvData;

int velocity_program = 0;
int pwm_countdown[12];                                    // This is the total number of loops left where we will execute a PWM
int pwm_phase[12];                                        // This is a repeating counter of PHASE_LIMIT to 0
int pwm_kick[12];                                         // An initial loop counter where we leave the output high to overcome inertia in the solenoid
int pwm_level[12];                                        // A counter that indicates how many loop counts we should leave the output high for.
const int COUNTDOWN_CONT = 2147483647;                    // number of loops where we apply the PWM for continous mode > 10 days
const int COUNTDOWN_START = 14400;                        // Maximum number of loops where we apply the PWM
const int NO_COUNTDOWN = 14401;                           // A special value to indicate that we are not using a PWM countdown
const int PHASE_KICK = 64;                               // Number of loops where we leave the output high to overcome inertia in the solenoid

const int PHASE_LIMIT = 32;                               // The number of loop counts we use to execute a PWM cycle.   If this value is too large, the solendoids will emit an audible noise during PWM
                                                          // 64 = approximately 1 ms
const int DOWN_PHASE_MAX = 21;                            // The maximum number of loop counts where the output is held low for a PWM cycle   Values between 13 and 15 are acceptable for a PHASE_LIMIT of 32
const int LEVEL_MAX = 20;                                 // Maximum level value for PWM so 120 to 127 is equal to no PWM
const int VELOCITY_DIVISOR = 6;                          // Divide the velocity value (1-127) by this number to the the PWM level

FlashStorage(nvStore, dataCFG);
FlashStorage(velocityStore, int);

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

  Wire.begin(AUTOMAT_ADDR);                             // join i2c bus
  Wire.onReceive(receiveI2CEvent);                      // register event

  midi2.setHandleProgramChange(handleProgramChange);
  midi2.setHandleNoteOn(handleNoteOn);                  // add Handler for Din MIDI
  midi2.setHandleNoteOff(handleNoteOff);
  midi2.begin(MIDI_CHANNEL_OMNI);
  // init();

  int veloFromFlash = velocityStore.read();
  // if uninitialized, this value should be read as -1
  if (veloFromFlash >= 0)
  {
    velocity_program = veloFromFlash;
  }
  
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

  if (velocity_program > 0 && velocity_program < 3) {
    // new single pulse width via velocity
    for(int i = 0 ; i < 12 ; i++){
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
  } else if ((velocity_program == 3) || (velocity_program == 4)) {
    // repeating pulse width via velocity
    for(int i = 0 ; i < 12 ; i++){
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
        case 0xC0: // program change
          handleProgramChange(1 + (rx.byte1 & 0xF), rx.byte2);
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

void handleProgramChange(byte channel, byte patch) {

  int prev_value = velocity_program;
  
  velocity_program = patch;
  if (velocity_program > 4 || velocity_program < 0) {
      velocity_program = 0;
  }

  if (velocity_program != prev_value) {
      velocityStore.write(velocity_program);
  }

  statusLED.blink(2, 1, 2); // LED Settings (On Time, Off Time, Count)
}

void handleNoteOn(byte pin, byte velocity) {
    solenoids.setOutput(pin);

    switch (velocity_program) 
    {
        case 1:  // strategy from 1.1.0  quadratic
          pwm_countdown[pin] = velocity * velocity; // set velocity timer
          break;
        case 2: // inverse quadratic
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
        case 3: // true pwm
        case 4: // continuous PWM
          pwm_level[pin] = (velocity / VELOCITY_DIVISOR) + 1;
          if(pwm_level[pin] > LEVEL_MAX) {
            pwm_countdown[pin] = 0;
            pwm_phase[pin] = 0;
            pwm_kick[pin] = 0;
            pwm_level[pin] = 0;
          }
          else {
            pwm_countdown[pin] = velocity_program == 4 ? COUNTDOWN_CONT : COUNTDOWN_START; 
            pwm_phase[pin] = PHASE_LIMIT;
            pwm_kick[pin] = PHASE_KICK;
          }
          break;
        default: // no velocity control == 0
         break;
    }
}

void handleNoteOn(byte channel, byte note, byte velocity) {
  midiLearn.noteOn(channel, note, velocity);

  if (midiLearn.active) {
    return;
  }

  statusLED.blink(1, 2, 1);

  for (int i = 0 ; i < 12 ; i++) {
    if (nvData.midiPins[i] == note) {
      if (nvData.midiChannels[i] == channel || nvData.midiChannels[i] == 0) {
        handleNoteOn(i, velocity);
      }
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
  
  for (int i = 0 ; i < 12 ; i++) {
    if (nvData.midiPins[i] == note) {
      if (nvData.midiChannels[i] == channel || nvData.midiChannels[i] == 0) {
        handleNoteOff(i);
      }
    }
  }
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
          handleNoteOn(pin, velocity);
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
