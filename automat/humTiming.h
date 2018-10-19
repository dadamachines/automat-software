/*
   Copyright (c) 2018, DADAMACHINES
   Author: Justin Pedro
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.

    3. Neither the name of DADAMACHINES nor the names of its contributors may be used
       to endorse or promote products derived from this software without
       specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef _HUMTIMING_H
#define _HUMTIMING_H

/* These timings were experimentally derived from testing a single solenoid with a particular
    automat */

const int NOTE_PERIOD[128] = {
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,
    /*A0 */
    1374, 1296, 1222, 1154, 1090, 1029, 970,  916,  866,  817,  772,  727,
    /*A1 */
    687,  648,  611,  577,  545,  514,  485,  458,  433,  408,  386,  363,
    /*A2 */
    343,  325,  306,  289,  273,  257,  243,  229,  216,  204,  192,  182,
    /*A3 */
    171,  162,  153,  144,  136,  129,  122,  114,  109,  101,  97,   91,
    /*A4 */
    85,   80,   76,   71,   67,   63,   59,   57,   53,   50,   47,   45,
    /*A5 */
    41, 39, 37, 35,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0
};

const int MAX_NOTE_PHASE[128] = {
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,
    /*A0 */
    1099,  1037, 978,  923,  872,  823, 776,  733,  693,  654,  618,  582,  
    /*A1 */
    550,  518,  489,  462,  436,  411,  388,  366,  346,  326,  309,  290,  
    /*A2 */
    274,  260,  245,  231,  218,  206,  194,  183,  173,  163,  154,  146,  
    /*A3 */
    137,  130,  122,  115,  109,  103,  98,    91,   87,   81,   78,   73, 
    /*A4 */
    68,   64,   61,   57,   54,   50,   47,   46,   42,   39,   36,   34,
    /*A5 */
    30,   28,   26,   24,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0
};

const int MIN_NOTE_PHASE = 8;

const float NOTE_PHASE_SCALE[128] = {
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,
    /*A0 */
    10.67f,  10.06f, 9.47f,  8.94f,  8.43f,  7.95f,  7.49f,  7.06f,  6.67f,  6.28f,  5.93f,  5.57f,
    /*A1 */
    5.26f,   4.95f,  4.66f,  4.39f,  4.14f,  3.9f,   3.67f,  3.46f,  3.26f,  3.06f,  2.89f,  2.71f,
    /*A2 */
    2.55f,   2.41f,  2.26f,  2.13f,  2.f,    1.87f,  1.76f,  1.65f,  1.55f,  1.46f,  1.36f,  1.28f,
    /*A3 */
    1.2f,    1.13f,  1.06f,  0.98f,  0.92f,  0.87f,  0.81f,  0.75f,  0.71f,  0.65f,  0.61f,  0.57f,
    /*A4 */
    0.52f,   0.48f,  0.45f,  0.41f,  0.38f,  0.35f,  0.31f,  0.3f,   0.27f,  0.25f,  0.24f,  0.22f,
    /*A5 */
    0.2f,    0.18f,  0.17f,  0.16f,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0
};


#endif
