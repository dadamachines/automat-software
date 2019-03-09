#include <SPI.h>
#include <MIDIUSB.h>
#include <limits.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))

enum {
  MIDI_STATUS_NOTE_OFF =  8,
  MIDI_STATUS_NOTE_ON =   9,
  MIDI_STATUS_CC =       11
};

enum {
  MIDI_CC_ALL_NOTES_OFF = 123
};

enum {
  PIN_LED_STATUS =            13,
  PIN_SHIFT_REGISTER_ENABLE = 27,
  PIN_SHIFT_REGISTER_LATCH =  30
};

struct {
  // The mapping of the port to the bit in the shift register.
  long bit;

  // Port configuration, the time range from which the trigger
  // duration is calculated based on the incoming velocity value.
  //
  // The range 0-126 spans 0-1s (x^3 curve)), 127 is inifinity.
  int8_t min;
  int8_t max;

  // The time the note started to play and its duration.
  unsigned long millis;
  unsigned long duration;

} Ports[] = {
  { .bit = 15, .min = 127, .max = 127 },
  { .bit = 13, .min = 127, .max = 127 },
  { .bit = 12, .min = 127, .max = 127 },
  { .bit = 11, .min = 127, .max = 127 },
  { .bit =  7, .min = 127, .max = 127 },
  { .bit =  3, .min = 127, .max = 127 },
  { .bit = 14, .min = 127, .max = 127 },
  { .bit = 10, .min = 127, .max = 127 },
  { .bit =  9, .min = 127, .max = 127 },
  { .bit =  6, .min = 127, .max = 127 },
  { .bit =  5, .min = 127, .max = 127 },
  { .bit =  4, .min = 127, .max = 127 },
};

void ShiftRegisterUpdate() {
  uint16_t bits = 0;

  for (long i = 0; i < ARRAY_SIZE(Ports); i++) {
    if (Ports[i].duration > 0)
      bits |= 1 << Ports[i].bit;
  }

  digitalWrite(PIN_SHIFT_REGISTER_LATCH, LOW);
  SPI.transfer(bits & 0xff);
  SPI.transfer(bits >> 8);
  digitalWrite(PIN_SHIFT_REGISTER_LATCH, HIGH);
}

void PlayNote(int8_t channel, int8_t note, bool on, int8_t velocity) {
  // Map all notes to the available channels.
  long port = note % ARRAY_SIZE(Ports);

  if (on) {
    unsigned long duration;

    if (Ports[port].min == 127) {
      // Special value defined as infinity.
      duration = ULONG_MAX;

    }  else {
      float min = (float)Ports[port].min / 126;
      float max = (float)Ports[port].max / 126;
      float range = max - min;
      float fraction = (float)velocity / 127;

      // A fraction of 1 second, the velocity mapped to the specified time range.
      duration = (float)1000 * powf(min + (range * fraction), 3);
    }

    Ports[port].millis = millis();
    Ports[port].duration = duration;

  } else
    Ports[port].duration = 0;

  ShiftRegisterUpdate();
}

void Timeout() {
  bool update = false;

  for (long i = 0; i < ARRAY_SIZE(Ports); i++) {
    if (Ports[i].duration == 0)
      continue;

    if (Ports[i].duration == ULONG_MAX)
      continue;

    if ((unsigned long)(millis() - Ports[i].millis) < Ports[i].duration)
      continue;

    Ports[i].duration = 0;
    update = true;
  }

  if (update)
    ShiftRegisterUpdate();
}

// Starting with C-2, note 0, every two notes configure the minumum
// and maximum time values of a port.
void ConfigurePort(int8_t note, int8_t value) {
  long port = note / 2;
  bool min = note % 2 == 0;

  if (port >= ARRAY_SIZE(Ports))
    return;

  // Special value defined as infinity.
  if (value == 127) {
    Ports[port].min = 127;
    Ports[port].max = 127;
    return;
  }

  if (min) {
    Ports[port].min = value;
    if (Ports[port].max < value)
      Ports[port].max = value;

  } else {
    Ports[port].max = value;
    if (Ports[port].min > value)
      Ports[port].min = value;
  }

  analogWrite(PIN_LED_STATUS, 255);
}

void Initialize() {
  for (long i = 0; i < ARRAY_SIZE(Ports); i++) {
    Ports[i].millis = 0;
    Ports[i].duration = 0;
  }

  ShiftRegisterUpdate();
}

void setup() {
  SPI.begin();
  pinMode(PIN_SHIFT_REGISTER_LATCH, OUTPUT);
  digitalWrite(PIN_SHIFT_REGISTER_LATCH, LOW);
  pinMode(PIN_SHIFT_REGISTER_ENABLE, OUTPUT);
  digitalWrite(PIN_SHIFT_REGISTER_ENABLE, LOW);

  pinMode(PIN_LED_STATUS, OUTPUT);
  analogWrite(PIN_LED_STATUS, 16);

  Initialize();
}

void loop() {
  Timeout();

  midiEventPacket_t rx = MidiUSB.read();
  int8_t channel = rx.byte1 & 15;
  int8_t command = rx.byte1 >> 4;

  if (command == 0)
    return;

  switch (command) {
    case MIDI_STATUS_NOTE_ON:
      // Configuration request on channel 16.
      if (channel == 15) {
        ConfigurePort(rx.byte2, rx.byte3);
        break;
      }

      PlayNote(channel, rx.byte2, true, rx.byte3);
      break;

    case MIDI_STATUS_NOTE_OFF:
      // Configuration request on channel 16.
      if (channel == 15)
        break;

      PlayNote(channel, rx.byte2, false, rx.byte3);
      break;

    case MIDI_STATUS_CC:
      switch (rx.byte2) {
        case MIDI_CC_ALL_NOTES_OFF:
          Initialize();
          break;

        default:
          return;
      }

    default:
      return;
  }
}
