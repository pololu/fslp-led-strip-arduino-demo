/* FslpLedStripDemo: Example Arduino sketch that shows
 * how to use an Interlink force-sensing linear potentiometer
 * (FSLP) to control an Addressable RGB LED strip from Pololu.
 *
 * This code requires the PololuLedStrip library, which is
 * available from:
 *
 *   https://github.com/pololu/pololu-led-strip-arduino
 *
 * To use this code, you will need to plug an Addressable RGB LED
 * strip from Pololu into pin 12.
 *
 * You will need an additional resistor and four connections from
 * the FSLP to the Arduino.  The FSLP's integration guide from
 * Interlink can be used to make the correct connections as
 * follows:
 *
 * - Sense line (SL) to pin A2
 * - Drive line 1 (D1) to pin 8
 * - Drive line 1 (D2) to pin A3
 * - the connection made at the bottom of resistor R0 to pin 9
 *
 * After uploading the sketch, the Arduino will control the LED
 * strip using position and pressure readings from the FSLP.  The
 * position reading will determine the number of LEDs that light
 * up, while the pressure reading will determine the color of the
 * LEDs.
 *
 * This sketch also reports the force and pressure readings over
 * serial so you can read them on your computer using a terminal
 * program such as the Serial Monitor in the Arduino IDE.
 *
 * Without connecting an LED strip, this sketch can still be used
 * to take readings from the FSLP and see them on your computer.
 */

#include <PololuLedStrip.h>
PololuLedStrip<12> ledStrip;
#define LED_COUNT 60
rgb_color colors[LED_COUNT];

// To measure position, the sense line must be connected to a
// pin capable of reading analog voltages.  For pressure,
// the sense line and drive line 2 must be connected to pins
// capable of reading analog voltages.  See the FSLP guide for
// more information.
const int fslpSenseLine = A2;
const int fslpDriveLine1 = 8;
const int fslpDriveLine2 = A3;
const int fslpBotR0 = 9;

void setup()
{
  Serial.begin(9600);
  delay(250);
}

void loop()
{
  int pressure, position;

  pressure = fslpGetPressure();

  if (pressure == 0)
  {
    // There is no detectable pressure, so measuring
    // the position does not make sense.
    position = 0;
  }
  else
  {
    position = fslpGetPosition();  // Raw reading, from 0 to 1023.
  }

  char report[80];
  sprintf(report, "pressure: %5d   position: %5d\n",
    pressure, position);
  Serial.print(report);

  // Scale the position reading to be from 0 to the
  // number of LEDs.
  int adjustedPosition = (int32_t)position * LED_COUNT / 1000;

  // Scale the pressure reading to be from 0 to 255.
  int adjustedPressure = pressure * 8 / 10;
  if (adjustedPressure > 255)
  {
    adjustedPressure = 255;
  }

  for(int i = 0; i < LED_COUNT; i++)
  {
    if (i < adjustedPosition)
    {
      // Light up the LEDs with indices lower than the adjustedLocation.
      // Set the color of the LEDs based on the adjustedPressure
      colors[i] = (rgb_color){ adjustedPressure, 0, 255 - adjustedPressure };
    }
    else
    {
      // Turn off the other LEDs.
      colors[i] = (rgb_color){ 0, 0, 0 };
    }
  }

  // Update the LED strip with values from the above for loop.
  ledStrip.write(colors, LED_COUNT);

  delay(20);
}

// This function follows the steps described in the FSLP
// integration guide to measure the position of a force on the
// sensor.  The return value of this function is proportional to
// the physical distance from drive line 2, and it is between
// 0 and 1023.  This function does not give meaningful results
// if fslpGetPressure is returning 0.
int fslpGetPosition()
{
  // Step 1 - Clear the charge on the sensor.
  pinMode(fslpSenseLine, OUTPUT);
  digitalWrite(fslpSenseLine, LOW);

  pinMode(fslpDriveLine1, OUTPUT);
  digitalWrite(fslpDriveLine1, LOW);

  pinMode(fslpDriveLine2, OUTPUT);
  digitalWrite(fslpDriveLine2, LOW);

  pinMode(fslpBotR0, OUTPUT);
  digitalWrite(fslpBotR0, LOW);

  // Step 2 - Set up appropriate drive line voltages.
  digitalWrite(fslpDriveLine1, HIGH);
  pinMode(fslpBotR0, INPUT);
  pinMode(fslpSenseLine, INPUT);

  // Step 3 - Wait for the voltage to stabilize.
  delayMicroseconds(10);

  // Step 4 - Take the measurement.
  analogReset();
  return analogRead(fslpSenseLine);
}

// This function follows the steps described in the FSLP
// integration guide to measure the pressure on the sensor.
// The value returned is usually between 0 (no pressure)
// and 500 (very high pressure), but could be as high as
// 32736.
int fslpGetPressure()
{
  // Step 1 - Set up the appropriate drive line voltages.
  pinMode(fslpDriveLine1, OUTPUT);
  digitalWrite(fslpDriveLine1, HIGH);

  pinMode(fslpBotR0, OUTPUT);
  digitalWrite(fslpBotR0, LOW);

  pinMode(fslpSenseLine, INPUT);

  pinMode(fslpDriveLine2, INPUT);

  // Step 2 - Wait for the voltage to stabilize.
  delayMicroseconds(10);

  // Step 3 - Take two measurements.
  analogReset();
  int v1 = analogRead(fslpDriveLine2);
  analogReset();
  int v2 = analogRead(fslpSenseLine);

  // Step 4 - Calculate the pressure.
  // Detailed information about this formula can be found in the
  // FSLP Integration Guide.
  if (v1 == v2)
  {
    // Avoid dividing by zero, and return maximum reading.
    return 32 * 1023;
  }
  return 32 * v2 / (v1 - v2);
}

// Performs an ADC reading on the internal GND channel in order
// to clear any voltage that might be leftover on the ADC.
// Only works on AVR boards and silently fails on others.
void analogReset()
{
#if defined(ADMUX)
#if defined(ADCSRB) && defined(MUX5)
    // Code for the ATmega2560 and ATmega32U4
    ADCSRB |= (1 << MUX5);
#endif
    ADMUX = 0x1F;

    // Start the conversion and wait for it to finish.
    ADCSRA |= (1 << ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
#endif
}

