#include "functions.h"

// ———————————————————————————— LED COLOR CONTROL ——————————————————————————— //
/**
 * @brief Sets the built-in 3-in-1 LED color or turns it off.
 * @param[in] pins array of pin numbers for red, green, blue LEDs
 * @param[in] mode (R)ed, (G)reen, (B)lue, or (O)ff
 **/
void setLED(const byte *pins, const char mode) {
  const byte r = 0;
  const byte g = 1;
  const byte b = 2;

  digitalWrite(pins[r], HIGH);
  digitalWrite(pins[g], HIGH);
  digitalWrite(pins[b], HIGH);

  switch (mode) {
    case 'R':
      digitalWrite(pins[r], LOW);
      break;

    case 'G':
      digitalWrite(pins[g], LOW);
      break;

    case 'B':
      digitalWrite(pins[b], LOW);
      break;
  }
}