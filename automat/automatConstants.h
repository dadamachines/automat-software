/*
 * Copyright (c) 2019, DADAMACHINES
 * Author: Tobias Münzer
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of DADAMACHINES nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _AUTOMAT_CONSTANTS_H
#define _AUTOMAT_CONSTANTS_H


#define AUTOMAT_MINI 0
#define SIS_SUPPORT 1

const int SYSEX_FIRMWARE_VERSION = 0x02000002;          // = version 2.0.2

#if AUTOMAT_MINI
const int OUTPUT_PINS_COUNT = 6;                       //= sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]);
#else 
const int OUTPUT_PINS_COUNT = 12;                       //= sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]);
#endif
const int LEARN_MODE_PIN = 38;                          // pin for the learn mode switch
const int SHIFT_REGISTER_ENABLE = 27;                   // Output enable for shiftregister ic
const int ACTIVITY_LED = 13;                            // activity led is still on D13 which is connected to PA17 > which means Pin 9 on MKRZero

const int MAX_MIN_PROGRAM = 0;                          // The index of the default max/min program
const int ALWAYS_ON_PROGRAM = 1;                        // The index of the always on program
const int QUADRATIC_PROGRAM = 2;                        // The index of the quadratic one pulse program
const int INVERSE_QUADRATIC_PROGRAM = 3;                // The index of the inverse quadratic one pulse program
const int FIXED_GATE_PROGRAM = 4;                       // The index of the one-pulse program with a configured gate duration
const int MIN_PROGRAM = 0;                              // The index of the minimum valid program

enum {
  MIDI_CC_MOD_WHEEL = 1,
  MIDI_CC_GENERAL_PURPOSE_1 = 16,
  MIDI_CC_GENERAL_PURPOSE_2 = 17,
  MIDI_CC_ALL_NOTES_OFF = 123
};


const int NO_COUNTDOWN = 14401;                           // A special value to indicate that we are not using a PWM countdown
const int COUNTDOWN_START = 14400;                        // Maximum number of loops where we apply the PWM
const float LOOP_TIME_FACTOR = 64.0f;                     // The number of loops ms per ms according to my cheap oscilliscope
const int MAX_MIN_INFINITE = 0;

const int MAX_MIDI_CHANNEL = 16;

// disable the PWM_SUPPORT by default.   Don't include it as part of the official releases
// If you turn this feature on and upload it to your automat, you assume all responsibility for any impact it may
// have on the automat hardware
#define PWM_SUPPORT 0

#if PWM_SUPPORT
/* programs 5 to 7 are reserved by the PWMManager */
const int PWM_PROGRAM = 5;                              // The index of the pwm multi-pulse program
const int PWM_MOTOR_PROGRAM = 6;                        // The index of the pwm continous program
const int HUM_MOTOR_PROGRAM = 7;                        // The index of the making the motor hum to a note

const int MAX_PROGRAM = 7;                              // The index of the maximum valid program
#else
const int MAX_PROGRAM = 4;                              // The index of the maximum valid program
#endif

#endif
