// Wet compass instrument for flight simulator
// Copyright Andrew Errington Jan 2019

/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <EEPROM.h>

// The wet compass is essentially a disc with compass markings around
// the perimeter driven by a stepper motor. An optical sensor is
// arranged to send a signal at a certain point during rotation to
// establish an index point. An offset from this point to the 'North'
// marking is stored in EEPROM.

// The current heading (1-360 degrees) is sent serially to the Arduino
// and software calculates the number of steps and direction to
// drive the stepper motor so that the heading is indicated on the
// edge of the compass disc.

// Opto sensor input to detect index position
const int pin_index = 12;

// Stepper motor control pins
const int pin_A = 8;
const int pin_B = 9;
const int pin_C = 10;
const int pin_D = 11;

// The on-board LED
const int pin_LED = 13;

// The calibration button
const int pin_calibrate = 7;

// There are 8 steps in a stepper motor cycle
int step_index = 0;
int step_drive[8] = { 0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09 };

// Keep track of our position
const int steps_per_rev = 4096;  // Specific to this model stepper
int cur_step_pos = 0;  // Keeps track of steps around one revolution

// Offset for North
// This is the number of steps needed to get from the index position
// sensed by the optical sensor to the N position.
// The offset is stored as a 16-bit number in EEPROM
int north_offset;


void setup()
{
  pinMode(pin_index, INPUT);
  pinMode(pin_calibrate, INPUT_PULLUP);

  pinMode(pin_A, OUTPUT);
  pinMode(pin_B, OUTPUT);
  pinMode(pin_C, OUTPUT);
  pinMode(pin_D, OUTPUT);

  pinMode(pin_LED, OUTPUT);

  Serial.begin(115200);

  // Use the on-board LED to indicate the state of the optical sensor
  digitalWrite(pin_LED, digitalRead(pin_index));

  // If the index indicator is set then move positively until it's clear
  while (digitalRead(pin_index)==1)
    do_steps(1);

  // Now rotate negatively until the index indicator is set
  while (digitalRead(pin_index)==0)
    do_steps(-1);

  // Now we are at zero
  cur_step_pos = 0;
  
  delay(500);

  // Get our calibration offset for North from EEPROM
  byte offset_low = EEPROM.read(0);
  byte offset_high = EEPROM.read(1);

  north_offset = offset_high << 8 | offset_low;

  if (north_offset > steps_per_rev)
    north_offset = 0;

  // Go to North
  do_steps(north_offset);
  
  delay(500);

}

void loop() {
  
  int delta_steps;
  int new_step_pos;
  
  while (Serial.available() > 0)
  {
    // look for the next valid integer. It's the degree heading (1-360)
    long new_heading = Serial.parseInt();
    
    // Calculate the new stepper position
    new_step_pos = ((new_heading * steps_per_rev) / 360) + north_offset;
  
    delta_steps = delta(new_step_pos, cur_step_pos);
    
    do_steps(delta_steps);
  }

  if (digitalRead(pin_calibrate) == 0)
  {
    // Wait for button to be released
    while(digitalRead(pin_calibrate) == 0)
      delay(10);

    // Calibration button was pushed. Move to index zero position
    delta_steps = delta(0, cur_step_pos);
    do_steps(delta_steps);
    
    delay(2000);
    
    // While the button is not pressed, move positively, slowly
    // User must press button when North is approaching
    while (digitalRead(pin_calibrate) == 1)
    {
      do_steps(1);
      delay(4);
    }
    
    // Wait for button to be released
    while(digitalRead(pin_calibrate) == 0)
      delay(10);

    // While the button is not pressed, move positively, very slowly
    // User must press button when North is indicated
    while (digitalRead(pin_calibrate) == 1)
    {
      do_steps(1);
      delay(19);
    }
    
    // Wait for button to be released
    while(digitalRead(pin_calibrate) == 0)
      delay(10);


    // The internal step counter is our North offset    
    Serial.println(cur_step_pos);
    
    north_offset = cur_step_pos;
    
    EEPROM.write(0, north_offset & 0xff);
    EEPROM.write(1, north_offset >> 8);
    
  }
  
}

int delta(int to_step_pos, int from_step_pos)
{
  return ((to_step_pos + (steps_per_rev / 2) - from_step_pos) % steps_per_rev) - (steps_per_rev / 2);
}

void do_steps(int steps)
{

  if (steps == 0) return;

  for (int i = 0; i < abs(steps); i++)
  {  
    // Generate a step for the motor
    if (steps > 0)
    {
      // Generate a forward step
      step_index += 1;
      
      cur_step_pos += 1;
  
    }
    else
    {
      // Generate a backward step
      step_index -= 1;
      
      cur_step_pos -= 1;
  
    }
    
    cur_step_pos = (cur_step_pos + steps_per_rev) % steps_per_rev;
    step_index = (step_index + (sizeof(step_drive)/sizeof(step_drive[0]))) % 8;
    
    digitalWrite(pin_A, step_drive[step_index] & 0x01);
    digitalWrite(pin_B, step_drive[step_index] & 0x02);
    digitalWrite(pin_C, step_drive[step_index] & 0x04);
    digitalWrite(pin_D, step_drive[step_index] & 0x08);

    delay(1);

    digitalWrite(pin_LED, digitalRead(pin_index));

  }
  
}
