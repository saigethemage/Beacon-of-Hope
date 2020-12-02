/************************************************************************
 * This program interfaces with a Miuzei SG90 9G Micro Servo Motor and a
 * Gikfun 12x12x7.3 mm Tactile Push Button to control a payload release
 * mechanism.
 * 
 * The servo motor uses pulse width modulation to communicate with a 
 * PJRC Teensy 3.2. Pins 3-6, 9-10, or 20-13 can be used to interface.
 * 
 * Written by Saige Martinez for CNM Ingenuity's Deep Dive Coding 
 * Internet of Things bootcamp.
 ***********************************************************************/

#include <Servo.h>

Servo servo; // Create servo object.

const int BUTTON = 2; // Variable to store button pin.
const int SERVO = 20; // Variable to store servo pin.

bool buttonState = false; // Variable to handle button state.
bool buttonStateOld = false; // Variable to handle previous button state.
bool systemState = false; // Variable to handle mechanism state.

void setup() {
  pinMode(BUTTON, INPUT_PULLDOWN); // Initialize button.
  servo.attach(SERVO); // Attach the servo on pin 20 to servo object.
}

void loop() {
  buttonState = digitalRead(BUTTON); // Read button pin.
  if(buttonState != buttonStateOld) {
    if(buttonState) {
      systemState = !systemState; // Change mechanism state.
    }
    buttonStateOld = buttonState;
  }
  if(systemState) {
    servo.write(45); // Set servo position to open.
    delay(15);
  }
  else {
    servo.write(0); // Set servo position to mechanism close.
    delay(15);
  }
}
