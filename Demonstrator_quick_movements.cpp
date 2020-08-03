// Demonstrator Pod Fast Movements Code
// Use the uncommented version for actual arduino upload!
// This version makes the cart move back and forth along the track
// 11-11-19

//##########################################         BACKGROUND INFO          ####################################################################
// This code sends a pwm signal through each coil
// A PWM signal is a voltage/current that pulses (turns off and on) at a frequency
// If the pin is set to a higher pwm, we can change the ammount of magnetic field from the coils (aka stronger magnetic force)
// Each coil's pwm freqency can be changed (decreased/increased) by moivng up/down elements of a sine array so that the cart gets pushed along
// Each coils have a "phase" that is an ofset of a sine wave to push the cart along (PHASE1 is element 1 of sine, PHASE2 is shifted 24 ammount of elements, etc...)
// Use shift(); function to shift the array(and pwm) down as the cart moves along the track and to get the new PWM value to set the coils to.
// The drive(); function is used to write that calulated new pwm value to each coil. (includes error calculation from the rotary encoder and correction for the pwm to be correct)

// The Rotary Encoder is used as a PID system. 
// A PID system automatically applies accurate and responsive correction to the control (drive();) function
// It is used to scale PWM values so the coils/magnets move at the correct speed based on the location
// We can put in a wanted location (Setpoint value), an input value (measured distance from rotary encoder) and an outputed value will be used to drive the motor correctly to that position
//#################################################################################################################################################

// Must Include this PID library (incuded in sharepoint)
// It has PID tools that is used to calculate an error/correction values
#include <PID_v1.h>

// Defines our arduino pin numbers. (Based on circuit layout)
const int encoder0PinA = 2 ;
const int encoder0PinB = 3 ;

const int EN1 = 5;
const int EN2 = 6;
const int EN3 = 7;
const int IN1 = 9;
const int IN2 = 10;
const int IN3 = 11;

int directionPin = 4;
int stepPin = 12;

volatile int encoder0Pos = 0; //This is the value of the position of the coils

// Setting up encoder object (myPID)
//   - Setpoint is a desired value (a location on the track)
//   - Input is the measured position
//   - Output is the calculated value (scalar multiplier to increase/decrease pwm frequency)
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

// SPWM (Sine Wave) This is a pwm value for the pins. (255 is 32khz or max frequency)
int pwmSin[72] = {127, 138, 149, 160, 170, 181, 191, 200, 209, 217, 224, 231, 237, 242, 246, 250, 252, 254, 254, 254, 252, 250, 246, 242, 237, 231, 224, 217, 209, 200, 191, 181, 170, 160, 149, 138, 127, 116, 105, 94, 84, 73, 64, 54, 45, 37, 30, 23, 17, 12, 8, 4, 2, 0, 0, 0, 2, 4, 8, 12, 17, 23, 30, 37, 45, 54, 64, 73, 84, 94, 105, 116 };

// These are the 3 possible phases
// count[] is helpfull for shifiting pwnSin[] by one phase
enum phase {PHASE1, PHASE2, PHASE3};
int count[] = {0,24,48};
int sDistance; // This is a shift distance, the ammount of elements to shift by to get to the correct pwm 

// These are our 3 magnet orientations and the correct pwm for each position
int *pos_U = pwmSin;
int *pos_V = pwmSin + 24;
int *pos_W = pwmSin + 48;

// length of track (how much to travel)
const double distance = 27; 
double step_position = 0;

int test;

// Converted to mm value of position on track
double position_in_mm = (encoder0Pos) / 40.00;

// This sets up our pins and initializes the position of the coils to be aligned with the magnets
void setup() {
  
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  // These interupts catch each pulse from the rotary encoder.
  // Each pulse will call doEncoderA/B function
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  // Increase PWM frequency to 32 kHz
  setPwmFrequency(IN1); 
  setPwmFrequency(IN2);
  setPwmFrequency(IN3);
  
  // Set Pin Modes
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(EN1, OUTPUT); 
  pinMode(EN2, OUTPUT); 
  pinMode(EN3, OUTPUT); 
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  digitalWrite(EN3, HIGH);

  // Align our coils to the magnets by setting correct pwm values to each coil
  analogWrite(IN1,*pos_U);
  analogWrite(IN2,*pos_V);
  analogWrite(IN3,*pos_W); 
  delay(2000);
  analogWrite(IN1,0);
  analogWrite(IN2,0);
  analogWrite(IN3,0); 

  //Initailize position/step_position to 0
  encoder0Pos = 0 ; 
  step_position = 0;

  // Our input (actual location) and expected (setpoint) should be zero
  Input = encoder0Pos / 40.00 ;
  Setpoint = step_position;
  
  //Setup our PID object
  myPID.SetOutputLimits(-1000, 1000); 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetTunings(15,0,0.4); //val_1,0,val_
 
  unsigned long previousMillis = 0;   
}

//Used for timing intervals
unsigned long previousMillis = 0;
const long interval = 500;

int i = 0;

// Fucntion completed each clock cycle
void loop() {
int positions[2] = { -100.0 , 0};

// This section calculates how much to drive the motor based on the PID error calulation
// Input (actual location) is calcuated from encoderPosition
Input = encoder0Pos / 40.00;
// Calculated error value is stored in Output
myPID.Compute();
// Drive our motor based on the output of the calculation
drive(Output);

// The new setpoint (wanted value) is either end of the track (100 or -100)
Setpoint = positions[i];
// Update current timing
unsigned long currentMillis = millis();

// Each interval cycle switches i from 0 to 1 or 1 to 0
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (i < 1) {
      i++;
    } else {
      i = 0;
    } 
  }
  
}

// This function actually drives the motor.
// scale_factor is a value to multply our pwm by in order to decrease our pwm based on how much we need to move
void drive(double scale_factor){
// It devides scale_factor by 1000 since our limits of output are -1000 to 1000, and we need to decrease our pwm value (multiply by a value less than one)
 scale_factor = scale_factor/1000;


if(scale_factor > 0.00){

// shiftDistance is used to shift the array the correct ammount of elements
// count[0] should always equal calculate()'s return value
// otherwise change (encoder0position) in setup();
sDistance = (-count[0] + calculate() + 24);

scale_factor = (scale_factor - 0.00) * (1.0 - 0.1) / (1.00 - 0.00) + 0.1;
//Shifts the pwmSin pointer by  to the correct element
  shift(&pos_U, sDistance , 72, PHASE1); 
  shift(&pos_V, sDistance, 72, PHASE2); 
  shift(&pos_W, sDistance, 72, PHASE3);
  
  //Write the pwm frequency to the pins multiplied by the scale_factor from the encoder
  analogWrite(IN1,*pos_U * scale_factor );
  analogWrite(IN2,*pos_V * scale_factor );
  analogWrite(IN3,*pos_W * scale_factor); 
 
return;
}

// This section handles a negative scale factor
// It does the same thing as the statement before, just handles negatives
if(scale_factor < 0.00){
double  temp = -scale_factor;
temp = (temp - 0.00) * (1.0 - 0.1) / (1.00 - 0.00) + 0.1;

sDistance =- ( count[0] - calculate() + 24) ; // +24
  shift(&pos_U, sDistance , 72, PHASE1); 
  shift(&pos_V, sDistance, 72, PHASE2); 
  shift(&pos_W, sDistance, 72, PHASE3);

  analogWrite(IN1,*pos_U * temp );
  analogWrite(IN2,*pos_V * temp );
  analogWrite(IN3,*pos_W * temp);  
return;
}

if(scale_factor == 0.00)
return;

}

// A function that calculates the three phaseshift values (how many elements to shift our sine wave array by)
// it returns the 3rd phaseshift (should be equal to count[0])
int calculate(){
  //re-find the position in mm
 double position_in_mm = (encoder0Pos) / 40.00;
 int multiple = position_in_mm / distance;
 double phaseshift2 = position_in_mm-(multiple*distance);
 // Serial.println(phaseshift2);
double phaseshift3 = (phaseshift2) * (72) / (distance - 0) + 0;
 //Serial.println(phaseshift3);
 return phaseshift3;
 
}

// Function that shifts the pwm sin wave by a number of elements
void shift(int **pwm_sin , int shift_distance , int array_size, phase phase_number){
  //If we are shifting by one whole sin cycle, we will not change values (sin(0) is the same as sin(360) etc)
  if(shift_distance == array_size)
  return;
  
  
  if(shift_distance > 0){
  if(count[phase_number] + shift_distance < array_size){
    //shift the pwm array and the count array by shift distance
  *pwm_sin = *pwm_sin + shift_distance;
   count[phase_number] += shift_distance ;
  }
  else  //Handles if shift is beyond the end of array
  {
    int temp =count[phase_number] + shift_distance - array_size;
    *pwm_sin = *pwm_sin - count[phase_number];
    *pwm_sin = *pwm_sin + temp;
    count[phase_number] = temp;
  }
  return; 
  }

  //Handles negative shift values.
  // Recursively calls shift(); with a correct value
  if(shift_distance < 0){
    int temp_distance = array_size + shift_distance;
    shift(pwm_sin, temp_distance , array_size, phase_number); 
  }

  if(shift_distance = 0 );
  return;
}
// ENCODER-INTERRUPT
// This function is called when a pulse from the encoder is measured 
// The encoder pulses 400 times per full rotaion, so this is used to measure the location of the motor
void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}

// This fucntion sets the pmw frequency of the board.
// PWM frequency is the rate which the pin switches off/on
// This function sets the frequency to 32 kHz to make it high enough to not effect the load (our coils).
void setPwmFrequency(int pin) {
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | 0x01;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | 0x01;
    }
  }
  else if(pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
  }
  
}