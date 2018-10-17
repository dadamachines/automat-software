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
const int MIN_PROGRAM = 0;                              // The index of the minimum valid program
const int MAX_PROGRAM = 4;                              // The index of the maximum valid program

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


const int MAX_MIDI_CHANNEL = 16;

typedef struct {
  byte velocityProgram[MAX_MIDI_CHANNEL + 1];
  byte alignfiller[3];                                     // for eeprom support
} velocityCFG;
velocityCFG velocityConfig;

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

FlashStorage(nvStore, dataCFG);
FlashStorage(velocityStore, velocityCFG);

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
  button.setPressTicks(2000);                           // set a long press to be two seconds
  button.attachLongPressStart(longButtonPress);         // register button for sysex transmission
  solenoids.begin();                                    // start shiftregister

  Wire.begin(AUTOMAT_ADDR);                             // join i2c bus
  Wire.onReceive(receiveI2CEvent);                      // register event

  midi2.setHandleProgramChange(handleProgramChange);
  midi2.setHandleNoteOn(handleNoteOn);                  // add Handler for Din MIDI
  midi2.setHandleNoteOff(handleNoteOff);
  midi2.setHandleSystemExclusive(handleSysEx);
  midi2.begin(MIDI_CHANNEL_OMNI);
  // init();

  velocityConfig = velocityStore.read();
  // if uninitialized, this value should be read as -1
  sanitizeForSysex(&velocityConfig);  
  statusLED.blink(20, 30, 32);
}

static int UsbSysExCursor = 0;
const int MAX_SYSEX_MESSAGE_SIZE = 128;
static byte UsbSysExBuffer[MAX_SYSEX_MESSAGE_SIZE];

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
    int channel = nvData.midiChannels[i];
    if (channel < 0 || channel > MAX_MIDI_CHANNEL)
    {
      channel = MIDI_CHANNEL_OMNI;
    }
    
    int velocity_program = velocityConfig.velocityProgram[channel];

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
        if (UsbSysExCursor > 0) {
           handleSysExUSBPacket(rx);
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
            case 0xC0: // program change
              handleProgramChange(1 + (rx.byte1 & 0xF), rx.byte2);
              break;
            case SYSEX_START: // SystemExclusive
              UsbSysExCursor = 0;
              handleSysExUSBPacket(rx);
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
  int prev_value = velocityConfig.velocityProgram[channel];
  
  if (patch > MAX_PROGRAM || patch < MIN_PROGRAM) {
      patch = ALWAYS_ON_PROGRAM;
  }
  velocityConfig.velocityProgram[channel] = patch;
  velocityConfig.velocityProgram[MIDI_CHANNEL_OMNI] = patch;

  if (velocityConfig.velocityProgram[channel] != prev_value) {
      velocityStore.write(velocityConfig);
  }

  statusLED.blink(2, 1, 2); // LED Settings (On Time, Off Time, Count)
}

void handleNoteOn(byte pin, byte velocity) {
    solenoids.setOutput(pin);

    int channel = nvData.midiChannels[pin];
    if (channel < 0 || channel > MAX_MIDI_CHANNEL)
    {
      channel = MIDI_CHANNEL_OMNI;
    }
    
    int velocity_program = velocityConfig.velocityProgram[channel];

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
  saveConfigToSysEx();
}

// 'd' = H64 which is part of the reserved Sysex manufacturer IDs so this shouldn't conflict with any existing IDs
const int SYSEX_CONFIG_HEADER = 'dAdA';
const int SYSEX_CONFIG_PINS = 'pins';
const int SYSEX_CONFIG_VELOCITY = 'velo';
const int SYSEX_CONFIG_GET_CONFIG = 'getc';
const int SYSEX_CONFIG_LEN = 2 + (sizeof (int) * 3) + sizeof(dataCFG) + sizeof(velocityCFG);
const int SYSEX_GET_CONFIG_LEN = 2 + (sizeof (int) * 2);

void handleSysEx(byte * arr, unsigned len)
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
         return;
      }
   }

   if (getIntFromArray(arr) != SYSEX_CONFIG_HEADER)
   {
       return;
   }
   arr += sizeof(int);

   if (len == SYSEX_GET_CONFIG_LEN)
   {
       if (getIntFromArray(arr) == SYSEX_CONFIG_GET_CONFIG)
       {
         saveConfigToSysEx();
       }
       return;
   }
   
   if (getIntFromArray(arr) != SYSEX_CONFIG_PINS)
   {
       return;
   }
   arr += sizeof(int);

   dataCFG* dataP = (dataCFG*) arr;

   if (hasConfigChanged(&nvData, dataP)) 
   {
      // avoid writing to Flash unless there is a need
      copyConfig(dataP, &nvData);
      nvStore.write(nvData);
   }
   arr += sizeof(dataCFG);
     
   if (getIntFromArray(arr) != SYSEX_CONFIG_VELOCITY)
   {
       return;
   }
   arr += sizeof(int);

   velocityCFG* veloP = (velocityCFG*) arr;
   if (hasConfigChanged(&velocityConfig, veloP)) 
   {
      // avoid writing to Flash unless there is a need
      copyConfig(veloP, &velocityConfig);
      velocityStore.write(velocityConfig);
   }
   // I know this line is not really needed, but I don't want it forgotten when we extend this method
   arr += sizeof(velocityCFG);

   statusLED.blink(20, 10, 8); // LED Settings (On Time, Off Time, Count)
}

static byte sysexOutArr[SYSEX_CONFIG_LEN];

void saveConfigToSysEx()
{   
   byte* outP = &sysexOutArr[0];

   *outP++ = SYSEX_START;

   outP = putIntToArray(outP, SYSEX_CONFIG_HEADER);

   outP = putIntToArray(outP, SYSEX_CONFIG_PINS);

   dataCFG* dataP = (dataCFG*) outP;
   copyConfig(&nvData, dataP);
   sanitizeForSysex(dataP);
   outP += sizeof(dataCFG);
     
   outP = putIntToArray(outP, SYSEX_CONFIG_VELOCITY);

   velocityCFG* veloP = (velocityCFG*) outP;
   copyConfig(&velocityConfig, veloP);
   sanitizeForSysex(veloP);
   outP += sizeof(velocityCFG);

   *outP = SYSEX_END;

   // the midi2.send function probably doesn't do anything with the current hardware, but I'm leaving it in for completeness 
   midi2.sendSysEx(SYSEX_CONFIG_LEN, sysexOutArr, true);
   MidiUSB_sendSysEx(sysexOutArr, SYSEX_CONFIG_LEN);
   
   statusLED.blink(8, 4, 4); // LED Settings (On Time, Off Time, Count)
}

void sanitizeForSysex(dataCFG* dataP)
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

bool hasConfigChanged(dataCFG* config1, dataCFG* config2)
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

void copyConfig(dataCFG* src, dataCFG* dest)
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

void sanitizeForSysex(velocityCFG* veloP)
{
  for (int i = 0; i <= MAX_MIDI_CHANNEL; ++i)
  {
    if(veloP->velocityProgram[i] < MIN_PROGRAM || veloP->velocityProgram[i] > MAX_PROGRAM)
    {
      veloP->velocityProgram[i] = ALWAYS_ON_PROGRAM;
    }
  }

  for (int j = 0; j < 3; ++j)
  {  
    veloP->alignfiller[j] = 0;
  }
}

bool hasConfigChanged(velocityCFG* config1, velocityCFG* config2)
{
  for (int i = 0; i <= MAX_MIDI_CHANNEL; ++i)
  {
    if(config1->velocityProgram[i] != config2->velocityProgram[i])
    {
      return true;
    }
  }

  return false;
}

void copyConfig(velocityCFG* src, velocityCFG* dest)
{
  for (int i = 0; i <= MAX_MIDI_CHANNEL; ++i)
  {
    dest->velocityProgram[i] = src->velocityProgram[i];
  }

  for (int j = 0; j < 3; ++j)
  {  
    dest->alignfiller[j] = 0;
  }
}

void handleSysExUSBPacket(midiEventPacket_t rx)
{
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
      
      handleSysEx(UsbSysExBuffer, UsbSysExCursor);
      UsbSysExCursor = 0;
      break;
    } else if (((b & 0x80) == 0) || (b == SYSEX_START)) {
       UsbSysExBuffer[UsbSysExCursor++] = b; 

       if (UsbSysExCursor >= MAX_SYSEX_MESSAGE_SIZE) {
         // Something went wrong with message, abort
         UsbSysExCursor = 0;
         break;
       }
    }
  }
}

void MidiUSB_sendSysEx(byte *data, size_t len)
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

int getIntFromArray(byte* arr)
{
    int ret = (arr[0] & 0x0FF) << 24;
    ret |= (arr[1] & 0x0FF)  << 16;  
    ret |= (arr[2] & 0x0FF)  << 8;  
    ret |= (arr[3] & 0x0FF) ;  
    return ret;
}

byte* putIntToArray(byte* arr, int in)
{
    arr[0] = in >> 24;
    arr[1] = (in >> 16) & 0x0FF;
    arr[2] = (in >> 8) & 0x0FF;
    arr[3] = in & 0x0FF;

    return arr + sizeof(int);
}



