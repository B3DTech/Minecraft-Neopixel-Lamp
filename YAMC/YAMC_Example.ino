/******************************************************************************
Yet Another Minecraft Cube (YAMC) Example Sketch

Written by: Phillip Fisk (Dershum)

This example sketch contains 3 modes for the Cube. To change the mode, 
  press the rotary encoder, and the cube will blink white 1, 2, or 3
  times depending on the mode its entering.

MODE_SHIFT - this mode fades between each color defined in the color seequence.
  Turning the encoder will speed up or slow down the speed of the color
  shifting. 

MODE_TWINKLE - this mode displays a solid color from the color sequence,
  subtly increasing/decreasing the color of each pixel, causing a 
  twinkily effect. Turning the enocder lets you select a different color.

MODE_RAVE - get out your glow stick! This mode choses a random color and 
  flashes that color on a random pixel, picks another random color, flashes
  it on a pixel, wash, rinse, repeat. Turn the encoder to speed up or
  slow down the speed of the flashes.

MODE_SOLID - set a single color, always on.

Special thanks to Adafruit for all their awesome sample code and products - 
  without which I would have never built this!

Also special thanks to Ben Buxton for his Rotary Encoder library which 
  I've modified for use with this example sketch and library. For his 
  original sketch please visit 
  http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html

MIT license, all text above must be included in any redistribution
******************************************************************************/

#include <Adafruit_NeoPixel.h>
#include <YAMC_RGB.h>
#include <YAMC_Timer.h>
#include <YAMC_Matrix.h>
#include <rotary.h>

#define DATA_PIN      6          // Neopixel data pin
#define ENC_PIN_A     5          // Rotary Encoder Pin A
#define ENC_PIN_B     3          // Rotary Encoder Pin B
#define ENC_PIN_X     4          // Rotary Encoder Push-button pin

#define NUM_PIXELS    5          // Number of pixels in the matrix
#define SHIFT_DELAY   20         // Default Delay between steps in a color shift (smaller is faster)
#define HOLD_DELAY    1000       // Default Amount of time to hold at a color (milliseconds)
#define RAVE_DELAY    50         // Default Number of millis the color will be displayed on the pixel during rave mode
#define SEQUENCE_SIZE 8          // Number of colors in the sequence
#define BRIGHTNESS    255        // Sets the intensity of the neopixels. 0 is off, 255 is brightest

#define ENC_MIN       0          // Encoder minimum value (for speed control)
#define ENC_MAX       20         // Encoder max value (for speed control)
#define ENC_DEBOUNCE_DELAY 20    // Encoder push-button debounce delay (millis)
#define MAX_MULTIPLIER 3         // Multiplier for the speed changes in the cube, based on encoder position

// Modes for the lamp. Press the encoder button to change modes
#define MODE_NORMAL   0          // Normal operation will shift through the defined matrix colors in order
#define MODE_TWINKLE  1          // Pick a color using the encoder from the color sequence, and it'll twinkle on that color
#define MODE_RAVE     2          // Get out your glow stick!
#define MODE_SOLID    3          // Pick a single solid color.


// YAMC object(s)
YAMC_Matrix matrix = YAMC_Matrix(NUM_PIXELS, DATA_PIN);

// Colors used by the matrix. (GRB, not RGB)
YAMC_RGB sequence[SEQUENCE_SIZE] = {
  YAMC_RGB(0x0094D3),        // Purple
  YAMC_RGB(0x0000FF),        // Blue
  YAMC_RGB(0xFF00FF),        // Cyan
  YAMC_RGB(0xFF0000),        // Green
  YAMC_RGB(0xFFFF00),        // Yellow
  YAMC_RGB(0x00FF00),        // Red
  YAMC_RGB(0x00FFFF),        // Magenta
  YAMC_RGB(0xA5FF00)         // Orange
};

// Global Timers
YAMC_Timer shift_timer = YAMC_Timer(SHIFT_DELAY);
YAMC_Timer hold_timer = YAMC_Timer(HOLD_DELAY);
YAMC_Timer rave_timer = YAMC_Timer(RAVE_DELAY);

// Rotary encoder
Rotary rotary = Rotary(ENC_PIN_A, ENC_PIN_B, ENC_PIN_X);
short enc_min = ENC_MIN;           // Set the initial minimum for the encoder
short enc_max = ENC_MAX;           // Set the initial maximum for the encoder
short enc_pos = ENC_MAX / 2;       // Set the initial encoder position to the middle of the range

// Other globals
short curr_rgb = -1;
short next_rgb = 0;
short num_steps = 0;
char lamp_mode = MODE_NORMAL;      // Default mode that the lamp starts in

void setup() {
  matrix.begin();
  matrix.setBrightness(BRIGHTNESS);

  // init the random seed
  randomSeed(analogRead(0));

  // set the initial matrix state to black
  matrix.setMatrix(0, 0, 0);
  matrix.paintMatrix();

  // set the initial timers
  setTimers();
}

// Main program loop, controls which mode the lamp is in.
void loop() {
  switch (lamp_mode) {
    case MODE_NORMAL:
      normalMode();
      break;
    case MODE_TWINKLE:
      twinkleMode();
      break;
    case MODE_RAVE:
      raveMode();
      break;
    case MODE_SOLID:
      solidMode();
    default:
      normalMode();
      break;
  }
}

void setTimers() {
  // Calculate the "speed" multiplier based on the encoder position
  double multiplier = ( (double(ENC_MAX) - double(enc_pos) ) / double(ENC_MAX) ) * double(MAX_MULTIPLIER);

  // Only use half of the multiplier for the shift timer - otherwise it gets too slow
  shift_timer.setLength(double(SHIFT_DELAY) * (multiplier * 0.5));

  // Set the hold/rave timers
  hold_timer.setLength(double(HOLD_DELAY) * multiplier);
  rave_timer.setLength(double(RAVE_DELAY) * multiplier);
}

// Checks the encoder for state changes
// Returns TRUE if the state has changed, FALSE if it hasn't
boolean checkEncoder() {
  // handle changes in the encoder
  volatile unsigned char val = rotary.process();
  boolean enc_changed = false;

  // if the encoder has been turned, check the direction
  if (val) {
    if (val == rotary.clockwise()) {
      // if encoder has been turned clockwise, increment the position
      if (enc_pos + 1 < enc_max) {
        enc_pos++;
        // indicate that the encoder has changed
        enc_changed = true;
      }
    }
    
    if (val == rotary.counterClockwise()) {
      // if encoder has been turned clockwise, decrement the position
      if (enc_pos > enc_min) {
        enc_pos--;
        // indicate that the encoder has changed
       enc_changed = true;
      }
    }
  }

  // Check to see if the button has been pressed.
  // Passes in a debounce delay
  // After the button press, blink the cube to acknowledge the button press
  if (rotary.buttonPressedReleased(ENC_DEBOUNCE_DELAY)) {
    switch (lamp_mode) {
      case MODE_NORMAL:
        lamp_mode = MODE_TWINKLE;     // Switch to Twinkle mode - settings below affect Twinkle Mode
        enc_pos = 0;                  // Set the encoder to the zero position (first color)
        enc_max = SEQUENCE_SIZE;      // Reset the max econder position to the max color in the sequence
        enc_changed = true;
        blinkMatrix(2);
        break;
      case MODE_TWINKLE:
        lamp_mode = MODE_RAVE;        // Switch to rave mode - settings below affect Rave mode
        enc_pos = 0;                  // Reset the encoder position to slowest
        enc_max = ENC_MAX;            // Reset the max encoder position
        enc_changed = true;
        blinkMatrix(3);
        break;
      case MODE_RAVE:
        lamp_mode = MODE_SOLID;      // Switch to solid color mode - Settings below affect Solid Mode
        enc_pos = 0;                 // Set the encoder to the zero position (first color)
        enc_max = SEQUENCE_SIZE;     // Reset the max encoder position to the max color in the sequence
        enc_changed = true;
        blinkMatrix(1);
        break;
      case MODE_SOLID:               
        lamp_mode = MODE_NORMAL;     // Switch to Normal mode - Setting below affect Normal Mode
        enc_pos = ENC_MAX / 2;       // Reset the encoder position to the middle
        enc_max = ENC_MAX;           // Reset the max encoder position
        enc_changed = true;
        blinkMatrix(4);
        break;
      default:  // should never get here. Blink like mad if we do.
        lamp_mode = MODE_NORMAL;
        enc_changed = true;
        blinkMatrix(10);
    }
  }

  // Check to see if the button has been pressed and held for 3 seconds
  if (rotary.buttonPressedHeld(1000)) {
    sleepMode();
  }

  return enc_changed;
}

void sleepMode() {
  // Turn off the LEDs
  matrix.setBrightness(0);
  matrix.paintMatrix();
  
  // Wait until the encoder is pressed to turn back on
  while (true) {
    if (rotary.buttonPressedHeld(500)) {
      break;
    }

    // wait to check it again
    delay(10);
  }

  // The encoder is probably being held down, wait for it to be released
  while (rotary.readButton() == rotary.BUTTON_PRESSED) {
    delay(1);
  }

  // Turn the LEDs back on
  matrix.setBrightness(BRIGHTNESS);

  // Go to loop() with the lamp in its previous state (restart the mode)
  loop();
}


void normalMode() {
  // if we're in normal mode, and reached the end of the sequence, loop around
  // otherwise go to the next color in the sequence
  //curr_rgb = (curr_rgb + 1 >= SEQUENCE_SIZE ? 0 : curr_rgb + 1);
  if (curr_rgb + 1 >= SEQUENCE_SIZE) {
    curr_rgb = 0;
  }
  else {
    curr_rgb++;
  }

  // get the next rgb based on looping through the sequence
  //next_rgb = (curr_rgb + 1 < SEQUENCE_SIZE ? curr_rgb + 1 : 0);
  if (curr_rgb + 1 < SEQUENCE_SIZE) {
    next_rgb = curr_rgb + 1;
  }
  else {
    next_rgb = 0;
  }

  // calcuate the number of steps between the target color and the current one
  num_steps = sequence[curr_rgb].calcMaxDiff(sequence[next_rgb]);
  short s = 0;

  // Start the shift timer. This timer determines the speed that the shift
  // between colors occurs. A lower shift delay results in a faster 
  // shift between colors.
  shift_timer.startTimer();

  // Shift between colors  
  while (s < num_steps) {
    // check to see if the shift timer has expired. 
    if (shift_timer.isExpired()) {
      // if so, shift to the next step
      matrix.shiftStep(sequence[curr_rgb], sequence[next_rgb], s, num_steps);        
      // update the pixels with the new color
      matrix.paintMatrix();
      // increment the shift counter
      s++;
      // start the timer again
      shift_timer.startTimer();
    }

    // Check the rotary encoder
    if (checkEncoder()) {
      // Encoder has changed

      if (lamp_mode != MODE_NORMAL) {
        // Mode has changed, break out
        return;
      }

      // position has changed, update the timers
      setTimers();
    }
  } // end color shift

  // hold at the current color
  hold_timer.startTimer();
  
  while (!hold_timer.isExpired()) {
    if (checkEncoder()) {
      // Encoder state has changed
      if (lamp_mode != MODE_NORMAL) {
        // Mode has changed, break out
        return;
      }

      // Encoder position has changed, update the timers
      setTimers();
    }
  } // end color hold
}

void raveMode() {
  int random_pixel = -1;
  int prev_pixel = -1;
  int random_color = -1;
  int prev_color = -1;
  
  // while we're in rave mode  
  while (lamp_mode == MODE_RAVE) {

    // set all pixels to black
    matrix.setMatrix(0, 0, 0);
    matrix.paintMatrix();
    
    // pixck a random pixel, keep trying until we get one that's different
    while (random_pixel == prev_pixel) {
      random_pixel = random(NUM_PIXELS);
    }

    // pick a random ccolor from the sequence, keep trying until we get one that's different
    while (random_color == prev_color) {
      random_color = random(SEQUENCE_SIZE);
    }

    // save the pixel and color for the next go-round    
    prev_pixel = random_pixel;
    prev_color = random_color; 
  
    // assign the color to the pixel and flash
    matrix.setPixel(sequence[random_color], random_pixel);
    matrix.paintMatrix();
    
    // hold the color
    rave_timer.startTimer(); 

    while (!rave_timer.isExpired()) {
      // check the encoder
      if (checkEncoder()) {
        // if the mode has changed, break out
        if (lamp_mode != MODE_RAVE) {
          return;
        }

        // encoder position has changed, update the timers
        setTimers();
      }
    }
  }
}

void twinkleMode() {
  // when entering this mode, reset the matrix and repaint
  matrix.setMatrix(sequence[enc_pos]);
  matrix.paintMatrix();

  // init the timers and state vars
  YAMC_Timer twinkle_timer[NUM_PIXELS];  
  int pulse_direction[NUM_PIXELS];
  int curr_intensity[NUM_PIXELS];
  int min_intensity = 0;
  int pulse_step = 5;

  // init the twinkle settings
  for (int i = 0; i < NUM_PIXELS; i++) {
    twinkle_timer[i] = YAMC_Timer(random(10,30));
    pulse_direction[i] = pulse_step * (random(0,1) == 0 ? -1 : 1);
    curr_intensity[i] = random(min_intensity, BRIGHTNESS);
  }

  // start the timers
  for (int p = 0; p < NUM_PIXELS; p++) {
    twinkle_timer[p].startTimer();
  }
  
  while (lamp_mode == MODE_TWINKLE) {
    // check the encoder
    if (checkEncoder()) {

      // if the mode has changed, break out
      if (lamp_mode != MODE_TWINKLE) {
        // before we break out, set all the intensities back to the default
        matrix.setBrightness(BRIGHTNESS);
        break;
      }
  
      // if the encoder position has changed, change the color
      if (enc_pos >= ENC_MIN && enc_pos < SEQUENCE_SIZE) {
        matrix.setMatrix(sequence[enc_pos]);
        matrix.paintMatrix();
      }
    }

    // Check each pixel to see if it's expired
    // if so, adjust the brightness of the pixel
    for (int p = 0; p < NUM_PIXELS; p++) {
      // if the pixel's timer is expired, flip the direction
      if (twinkle_timer[p].isExpired()) {
        if (curr_intensity[p] + pulse_direction[p] <= min_intensity) {
          curr_intensity[p] = min_intensity;
          pulse_direction[p] *= -1;
        }
        else if (curr_intensity[p] + pulse_direction[p] > 255) {
          curr_intensity[p] = BRIGHTNESS;
          pulse_direction[p] *= -1;
        }
  
        matrix.setBrightness(curr_intensity[p] += pulse_direction[p], p);
        matrix.paintMatrix();
        twinkle_timer[p].startTimer(random(10,30));
      }
    }
  }
}

void solidMode() {
  matrix.setMatrix(sequence[enc_pos]);
  matrix.paintMatrix();

  // init the timers and state vars
   
  while (lamp_mode == MODE_SOLID) {
    // check the encoder
    if (checkEncoder()) {

      // if the mode has changed, break out
      if (lamp_mode != MODE_SOLID) {
        // before we break out, set all the intensities back to the default
        matrix.setBrightness(BRIGHTNESS);
        break;
      }
  
      // if the encoder position has changed, change the color
      if (enc_pos >= ENC_MIN && enc_pos < SEQUENCE_SIZE) {
        matrix.setMatrix(sequence[enc_pos]);
        matrix.paintMatrix();
      }
    }
  }
}

void blinkMatrix(short num_blinks) {
  // blinks num_blinks times
  // this function blocks input until the blink sequence is complete
  
  // loop through num_blinks
  for (int n = 0; n < num_blinks; n++) {
    // save the matrix state before we blink
    matrix.saveMatrix();
    delay(100);

    // flash white and wait a moment
    matrix.setMatrix(255, 255, 255);
    matrix.paintMatrix();
    delay(100);
    
    // return back to the original matrix color
    matrix.restoreMatrix();
    matrix.paintMatrix();
  }
}

