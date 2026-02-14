/*
   Copyright (c) 2018, 2019 DADAMACHINES
   Author: Justin Pedro
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.

    3. Neither the name of DADAMACHINES nor the names of its contributors may be
   used to endorse or promote products derived from this software without
       specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#include "humTiming.h"

#if PWM_SUPPORT
int pwm_phase[OUTPUT_PINS_COUNT];      // This is a repeating counter of PHASE_LIMIT to 0
int pwm_kick[OUTPUT_PINS_COUNT];       // An initial loop counter where we leave the
                                       // output high to overcome inertia in the solenoid
int pwm_level[OUTPUT_PINS_COUNT];      // A counter that indicates how many loop
                                       // counts we should leave the output high for.
const int COUNTDOWN_CONT = 2147483647; // number of loops where we apply the PWM
                                       // for continous mode > 10 days
const int PHASE_KICK = 64;             // Number of loops where we leave the output high to
                                       // overcome inertia in the solenoid

const int PHASE_LIMIT = 32;      // The number of loop counts we use to execute a PWM cycle.   If this
                                 // value is too large, the solendoids will emit an audible noise during
                                 // PWM 64 = approximately 1 ms
const int DOWN_PHASE_MAX = 21;   // The maximum number of loop counts where the output is held low for a
                                 // PWM cycle   Values between 13 and 15 are acceptable for a PHASE_LIMIT
                                 // of 32
const int LEVEL_MAX        = 20; // Maximum level value for PWM so 120 to 127 is equal to no PWM
const int VELOCITY_DIVISOR = 6;  // Divide the velocity value (1-127) by this number to the the PWM level

int pitchBend[MAX_MIDI_CHANNEL + 1] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int modWheel[MAX_MIDI_CHANNEL + 1]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int humNote[OUTPUT_PINS_COUNT]      = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int pwm_countdown[OUTPUT_PINS_COUNT]; // This is the total number of loops left
                                      // where we will execute a PWM

int PWMManager::calculateTotalHumPhase(int pin) {
  int note = humNote[pin];
  int ret  = NOTE_PERIOD[note];

  if (ret != 0) {
    int channel = nvData.midiChannels[pin];

    if (pitchBend[channel] != 0) {
      float adjust = pitchBend[channel] / 65536.f;

      ret -= (ret * adjust);
      if (ret < MAX_NOTE_PHASE[note] + MIN_NOTE_PHASE) {
        ret = MAX_NOTE_PHASE[note] + MIN_NOTE_PHASE;
      }
    }
  }

  return ret;
}

int PWMManager::calculateLoHumPhase(int pin) {
  int channel = nvData.midiChannels[pin];
  int note    = humNote[pin];

  if (NOTE_PERIOD[note] == 0) {
    // the math should still result in 0, but just in case...
    return 0;
  }

  float hiPhase = NOTE_PHASE_SCALE[note] * modWheel[channel];
  int iHiPhase  = MIN_NOTE_PHASE + (int)(hiPhase + 0.5f);

  if (iHiPhase > MAX_NOTE_PHASE[note]) {
    iHiPhase = MAX_NOTE_PHASE[note];
  }

  return NOTE_PERIOD[note] - iHiPhase;
}

void PWMManager::handlePinLoop(int pin, int program) {
  switch (program) {
    case PWM_PROGRAM:
    case PWM_MOTOR_PROGRAM: {
      // repeating pulse width via velocity
      // If the user has used a very high velocity, we will bypass PWM
      // Also, we will set a limit of pwm_countdown time to not keep the PWM on
      //  once the solenoid has made contact with the drum, etc.
      if ((pwm_countdown[pin] == 0) || (pwm_level[pin] == 0)) {
        return;
      }

      // First thing we are going to do is leave the solenoids for 'kick' time
      // to get them moving and overcoming inertia
      if (pwm_kick[pin] > 0) {
        pwm_kick[pin]--;
        return;
      }

      // step through the PWM phase sequence
      pwm_phase[pin]--;
      pwm_countdown[pin]--;

      if ((pwm_phase[pin] == 0) || (pwm_countdown[pin] == 0)) {
        // Restart the phase sequence with the output set high
        solenoids.setOutput(pin);

        if (pwm_countdown[pin] == 0) {
          // we are done the PWM part of the note.   Leave the output high until
          // note off.
          pwm_phase[pin] = 0;
        } else {
          // Restart the PWM counter
          pwm_phase[pin] = PHASE_LIMIT;
        }
      } else if (pwm_phase[pin] == (DOWN_PHASE_MAX - pwm_level[pin])) {
        // we are in the low part of the PWM cycle
        solenoids.clearOutput(pin);
      }
    } break;

    case HUM_MOTOR_PROGRAM: {
      // repeating pulse width tuned to a pitch with modulation
      if (pwm_level[pin] == 0) {
        return;
      }

      // step through the PWM phase sequence
      pwm_phase[pin]--;

      if ((pwm_phase[pin] == 0) || (pwm_countdown[pin] == 0)) {
        // Restart the PWM counter
        pwm_phase[pin] = calculateTotalHumPhase(pin);
        pwm_level[pin] = calculateLoHumPhase(pin);
        if (pwm_level[pin] > 0) {
          // Restart the phase sequence with the output set high
          solenoids.setOutput(pin);
        }
      } else if (pwm_phase[pin] == pwm_level[pin]) {
        // we are in the low part of the PWM cycle
        solenoids.clearOutput(pin);
      }
    } break;
    default:
      break;
  }
}

void PWMManager::handleNoteOn(byte velocity_program, byte pin, byte velocity) {
  switch (velocity_program) {
    case PWM_PROGRAM:       // true pwm
    case PWM_MOTOR_PROGRAM: // continuous PWM
      pwm_level[pin] = (velocity / VELOCITY_DIVISOR) + 1;
      if (pwm_level[pin] > LEVEL_MAX) {
        pwm_countdown[pin] = 0;
        pwm_phase[pin]     = 0;
        pwm_kick[pin]      = 0;
        pwm_level[pin]     = 0;
      } else {
        pwm_countdown[pin] = (velocity_program == PWM_MOTOR_PROGRAM) ? COUNTDOWN_CONT : COUNTDOWN_START;
        pwm_phase[pin]     = PHASE_LIMIT;
        pwm_kick[pin]      = PHASE_KICK;
      }
      break;
    case HUM_MOTOR_PROGRAM:
      if (pwm_level[pin] == 0) {
        pwm_phase[pin] = calculateTotalHumPhase(pin);
        pwm_level[pin] = calculateLoHumPhase(pin);
      } // otherwise let it calculate the phase at the end of the cycle
      break;
    default:
      break;
  }
}

void PWMManager::handleModWheel(byte channel, byte mod) {
  modWheel[channel]           = mod;
  modWheel[MIDI_CHANNEL_OMNI] = mod;
}

void PWMManager::handlePitchBend(byte channel, int bend) {
  pitchBend[channel]           = bend;
  pitchBend[MIDI_CHANNEL_OMNI] = bend;
}

void PWMManager::handleNoteOff(byte pin) {
  pwm_countdown[pin] = 0;
  pwm_kick[pin]      = 0;
  pwm_phase[pin]     = 0;
  pwm_level[pin]     = 0;
}

bool PWMManager::handleHumNoteOff(byte pin, byte note) {
  if (humNote[pin] == note) {
    humNote[pin] = 0;
    return true;
  }

  return false;
}

void PWMManager::handleHumNoteOn(byte pin, byte note) {
  humNote[pin] = note;
}

void PWMManager::handleAllNotesOff() {
  for (int i = 0; i < OUTPUT_PINS_COUNT; ++i) {
    pwm_kick[i]  = 0;
    pwm_phase[i] = 0;
    pwm_level[i] = 0;
    humNote[i]   = 0;
  }
}

#endif
