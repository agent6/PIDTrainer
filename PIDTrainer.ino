/*
  Arduino PID Controller with Serial Commands for Processing GUI
  
  This sketch controls a motor via a PID loop using encoder feedback.
  It listens for serial commands to:
    • Enable/disable the PID controller ("PID:ON" / "PID:OFF")
    • Set the setpoint in degrees ("SP:<value>")
    • Update PID gains ("Kp:<value>", "Ki:<value>", "Kd:<value>")
    • Set the integral accumulation threshold ("KIThresh:<value>")
    • Reset the encoder tick count ("ResetEnc")
  
  The Arduino sends data lines (every ~20ms) in the format:
      DATA,<millis>,<encoderTicks>,<targetTicks>
  so that the Processing GUI can graph the encoder value.
  
  Wiring:
    Motor Driver:
      D9  -> PWM
      D8  -> AIN2
      D7  -> AIN1
    Encoder:
      D2  -> Encoder Channel A (with interrupt)
      D3  -> Encoder Channel B (with interrupt)
  
  Encoder resolution: 1600 ticks per 360° rotation.
*/

////////////////////
// CONSTANTS & CONFIGURATION

// Motor Driver Pins
const int PWM_PIN    = 9;       // PWM output
const int DIR_PIN1   = 7;       // Motor direction control pin 1 (AIN1)
const int DIR_PIN2   = 8;       // Motor direction control pin 2 (AIN2)

// Encoder Pins
const int ENC_PIN_A  = 2;       // Encoder channel A
const int ENC_PIN_B  = 3;       // Encoder channel B

// Encoder resolution: 1600 ticks per 360° rotation
const long TICKS_PER_REV    = 1600;
const float TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;

// Default PID gains and threshold (modifiable via serial commands)
float KP = 2.0;          // Proportional gain
float KI = 0.1;          // Integral gain
float KD = 0.5;          // Derivative gain
int   KI_ACCUM_THRESHOLD = 20;  // Integral accumulates only if |error| <= threshold (in ticks)

// Global setpoint (in degrees, set via serial)
float setpointDegrees = 0;  // Default setpoint

// PID enable flag (controlled via serial command)
bool pidEnabled = false;

////////////////////
// GLOBAL VARIABLES

volatile long encoderTicks = 0;    // Encoder tick count (updated in ISR)
volatile int lastEncoded = 0;        // Last encoder state for quadrature decoding

// PID internal variables:
float previousError = 0;
float integral = 0;

////////////////////
// INTERRUPT SERVICE ROUTINE FOR ENCODER

void updateEncoder() {
  int MSB = digitalRead(ENC_PIN_A); // Read encoder channel A
  int LSB = digitalRead(ENC_PIN_B); // Read encoder channel B
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  // Quadrature decoding:
  if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) {
    encoderTicks++;
  } else if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
    encoderTicks--;
  }
  lastEncoded = encoded;
}

////////////////////
// SETUP

void setup() {
  Serial.begin(115200);

  // Configure motor driver pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  
  // Configure encoder pins with pull-ups
  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);
  
  // Initialize encoder state
  lastEncoded = (digitalRead(ENC_PIN_A) << 1) | digitalRead(ENC_PIN_B);
  
  // Attach interrupts for encoder channels
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), updateEncoder, CHANGE);
}

////////////////////
// LOOP

void loop() {
  // Process any serial commands from Processing
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("PID:")) {
      if (command.indexOf("ON") != -1) {
        pidEnabled = true;
        integral = 0;
        previousError = 0;
        Serial.println("PID,ON");
      } else if (command.indexOf("OFF") != -1) {
        pidEnabled = false;
        // Stop the motor
        digitalWrite(DIR_PIN1, LOW);
        digitalWrite(DIR_PIN2, LOW);
        analogWrite(PWM_PIN, 0);
        Serial.println("PID,OFF");
      }
    }
    else if (command.startsWith("SP:")) {
      setpointDegrees = command.substring(3).toFloat();
    }
    else if (command.startsWith("Kp:")) {
      KP = command.substring(3).toFloat();
    }
    else if (command.startsWith("Ki:")) {
      KI = command.substring(3).toFloat();
    }
    else if (command.startsWith("Kd:")) {
      KD = command.substring(3).toFloat();
    }
    else if (command.startsWith("KIThresh:")) {
      KI_ACCUM_THRESHOLD = command.substring(9).toInt();
    }
    else if (command.startsWith("ResetEnc")) {
      noInterrupts();
      encoderTicks = 0;
      interrupts();
    }
  }
  
  // Calculate target in ticks from setpoint (in degrees)
  long targetTicks = setpointDegrees * TICKS_PER_DEGREE;
  
  // Get current encoder tick count atomically
  long currentTicks;
  noInterrupts();
  currentTicks = encoderTicks;
  interrupts();
  
  // If PID is enabled, run PID control
  if (pidEnabled) {
    float error = targetTicks - currentTicks;
    
    // Conditional (integral separation): accumulate integral only if |error| is within threshold
    if (abs(error) <= KI_ACCUM_THRESHOLD) {
      integral += error;
    }
    
    float derivative = error - previousError;
    float output = KP * error + KI * integral + KD * derivative;
    previousError = error;
    
    // Constrain output to PWM limits (-255 to 255)
    output = constrain(output, -255, 255);
    
    // Drive motor based on PID output
    if (output > 0) {
      digitalWrite(DIR_PIN1, HIGH);
      digitalWrite(DIR_PIN2, LOW);
      analogWrite(PWM_PIN, (int)output);
    } else if (output < 0) {
      digitalWrite(DIR_PIN1, LOW);
      digitalWrite(DIR_PIN2, HIGH);
      analogWrite(PWM_PIN, (int)(-output));
    } else {
      digitalWrite(DIR_PIN1, LOW);
      digitalWrite(DIR_PIN2, LOW);
      analogWrite(PWM_PIN, 0);
    }
  }
  else {
    // If PID is disabled, ensure motor is off
    digitalWrite(DIR_PIN1, LOW);
    digitalWrite(DIR_PIN2, LOW);
    analogWrite(PWM_PIN, 0);
  }
  
  // Send data line for graphing: "DATA,<millis>,<encoderTicks>,<targetTicks>"
  Serial.print("DATA,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(currentTicks);
  Serial.print(",");
  Serial.println(targetTicks);
  
  //delay(20);  // Loop delay ~20ms
}
