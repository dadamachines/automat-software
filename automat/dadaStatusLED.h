/*
   Copyright (c) 2017, DADAMACHINES
   Author: Sven Braun
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


#ifndef _DARALED13_H
#define _DARALED13_H

#define loopTime  200     // speed depend on CPU Frequency

class dadaStatusLED {
  public:
    int         _ledPin;
    long        _on_time;
    long        _off_time;
    int         _times;
    bool        _state;
    long        _countDown;

    dadaStatusLED(int pin) {
      _ledPin = pin;
      _times = 0;
      pinMode(_ledPin, OUTPUT);   // pin leds to output
    };

    void blink( long on_time, long off_time, int count = -1) {
      if (_times > 0) { // accept only requests wen finished
        return;
      }
      _on_time    =  on_time * loopTime;
      _off_time   =  off_time * loopTime;
      _times      =  count;
    };

    void tick() {       // call this in MainLoop
      if (_times == 0) {
        return;
      }

      _countDown--;
      if (_state) {
        if (_countDown < 0) {
          _state = ! _state;
          _countDown = _off_time;
          digitalWrite(_ledPin, LOW);
          if (_times > 0 ) {
            _times--;
          }
        }
      } else {
        if (_countDown < 0) {
          _state = ! _state;
          _countDown = _on_time;
          digitalWrite(_ledPin, HIGH);
        }
      }
    };

};

#endif
