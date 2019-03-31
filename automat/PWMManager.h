/*
 * Copyright (c) 2018, 2019 DADAMACHINES
 * Author: Justin Pedro
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


#ifndef _PWMMANAGER_H
#define _PWMMANAGER_H

// disable the PWM_SUPPORT by default.   Don't include it as part of the official releases
// If you turn this feature on and upload it to your automat, you assume all responsibility for any impact it may
// have on the automat hardware
#define PWM_SUPPORT 0

class PWMManager {
public:
   static int calculateTotalHumPhase(int pin);
   static int calculateLoHumPhase(int pin);
   static void handleNoteOn(byte velocity_program, byte pin, byte velocity);
   static void handleNoteOff(byte pin);
   static void handlePinLoop(int pin, int program);
   static void handleModWheel(byte channel, byte mod);
   static void handlePitchBend(byte channel, int bend);
   static void handleHumNoteOn(byte pin, byte note);
   static bool handleHumNoteOff(byte pin, byte note);
   static void handleAllNotesOff();
};

const int PWM_PROGRAM = 5;                              // The index of the pwm multi-pulse program
const int PWM_MOTOR_PROGRAM = 6;                        // The index of the pwm continous program
const int HUM_MOTOR_PROGRAM = 7;                        // The index of the making the motor hum to a note
#endif


