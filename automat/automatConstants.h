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


#define AUTOMAT_MINI 1
const int SYSEX_FIRMWARE_VERSION = 0x01000500;          // = version 1.5.0

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


#endif


