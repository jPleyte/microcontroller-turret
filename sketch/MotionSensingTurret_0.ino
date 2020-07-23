#include <Servo.h>

// manual=true, auto=false
boolean manualOperationMode = false;

// Manual fire button
const int MANUAL_FIRE_BUTTON_IN = 10;

/* Turret panning variables */
Servo turretPanServo;
const int TURRET_PAN_SERVO_PIN_OUT = 3; // ~3
const int TURRET_PAN_POT_PIN_IN = A0;
const int TURRENT_PAN_SERVO_CENTER = 90;

// This is the amount of error in the POT.read value. It is neccessary to prevent controller from 
// thinking you've adjusted the POT from 90 degrees to 89 degrees (for example) even though you haven't
// touched anything. 
const int TURRET_PAN_SERVO_POT_TOLLERANCE = 2;

int turretPotVal;
int turretPotAngle;
const int TURRET_PAN_SERVO_MIN = 1;
const int TURRET_PAN_SERVO_MAX = 179;
int turretPanServoCurrentAngle;
int turretPanServoPreviousAngle;


/* Firing motor variables */
const int FIRING_MOTOR_RELAY_PIN = 4;
const int FIRING_MOTOR_WARMUP_DELAY = 1000;

/* Firing actuator variables */
const int FIRING_ACTUATOR_RELAY_ONE_PIN = 6;
const int FIRING_ACTUATOR_RELAY_TWO_PIN = 7;
// This controls how long the relay stays extended/retracted
const int ACTUATOR_RESPONSE_DELAY = 250;

/* PIR motion sensor variables */
const int PIR_ANALOG_PIN_IN = A3;   // PIR analog output on A3; currently unused
const int PIR_DIGITAL_PIN_IN = 12;  // PIR digital output on D12

/**
 * Microcontroller setup
 */
void setup() {
  Serial.begin(9600);

  // Initialise and center the turret pan servo
  initialiseTurrentPanServo();

  pinMode(MANUAL_FIRE_BUTTON_IN, INPUT);
  
  // Setup control of the firing motors 
  pinMode(FIRING_MOTOR_RELAY_PIN, OUTPUT);
  digitalWrite(FIRING_MOTOR_RELAY_PIN, LOW);

  // Set up the output pins which control the linear actuator
  pinMode(FIRING_ACTUATOR_RELAY_ONE_PIN, OUTPUT);
  firingRelaysOff();
  pinMode(FIRING_ACTUATOR_RELAY_TWO_PIN, OUTPUT);
  firingRelaysOff();  

  // Set up the PIR input pins
  pinMode(PIR_ANALOG_PIN_IN, INPUT);
  pinMode(PIR_DIGITAL_PIN_IN, INPUT);

  Serial.print("Operation mode: ");
  if(manualOperationMode) {
    Serial.println("manual");
  } else {
    Serial.println("auto");
  }
  
  Serial.println("Setup complete.");
}

/**
 * Main loop
 */
void loop() {
  if(manualOperationMode) {
    swivelTestTurrenPanServo();
  } else {
    monitorTurretPanServoPot();
  }
  
  if(isManualFireButtonPressed()) {
    fire();
  } else if(motionDetected()) {
    fire();
  }
  
  // Serial.println("Main loop complete.");
  delay(5);
}

/**
 * Attach microcontroller to servo and center at 90 degrees.
 */
void initialiseTurrentPanServo() {
  turretPanServo.attach(TURRET_PAN_SERVO_PIN_OUT);
  delay(3000);
  turretPanServo.write(TURRENT_PAN_SERVO_CENTER);
  delay(3000);
  turretPanServoCurrentAngle = TURRENT_PAN_SERVO_CENTER;
  turretPanServoPreviousAngle = turretPanServoCurrentAngle;
  Serial.print("Turrent pan servo is centered at ");
  Serial.println(turretPanServoCurrentAngle);
}

/**
 * Read the POT which controls panning of the gun servo
 */
void monitorTurretPanServoPot() {
  // Read the POT which pans the turret
  int turretPanPotVal = analogRead(TURRET_PAN_POT_PIN_IN);
  int turretPanAngle = map(turretPanPotVal, 0, 1023, TURRET_PAN_SERVO_MIN, TURRET_PAN_SERVO_MAX);
  if(turretPotHasBeenAddjusted(turretPanAngle)) {
    Serial.print("Panning turret pan servo to angle ");
    Serial.println(turretPanAngle);
    turretPanServoCurrentAngle = turretPanAngle;
    turretPanServoPreviousAngle = turretPanServoCurrentAngle;
    turretPanServo.write(turretPanServoCurrentAngle);
    delay(5);    
  }
}

/**
 * Compare the current servo POT position with it's previous position and return true if it is 
 * significantly different than the previous position. 
 */
boolean turretPotHasBeenAddjusted(int newPotPanAngle) {
  if(newPotPanAngle != turretPanServoCurrentAngle) {
    if(newPotPanAngle > turretPanServoCurrentAngle + TURRET_PAN_SERVO_POT_TOLLERANCE) {
      return true;
    }
    if (newPotPanAngle < turretPanServoCurrentAngle - TURRET_PAN_SERVO_POT_TOLLERANCE) {
      return true;
    }

    return false;
  }
}

/**
 * Turn on the firing motors
 */
void firingMotorRelayOn() {
  digitalWrite(FIRING_MOTOR_RELAY_PIN, HIGH);
}

/**
 * Turn off the firing motors
 */
void firingMotorRelayOff() {
  digitalWrite(FIRING_MOTOR_RELAY_PIN, LOW);
}


/**
 * I am using a two channel relay which has jumpers that allows a high OR low signal to turn either relay on or off.  
 * This is different from other relay modules where high-high or low-low means both off; and high-low turns 
 * relay 1 on and relay 2 off; and low-high turns relay 2 on and relay 1 off. Since this can be dangerous to
 * get wrong i have written these three firingRelay* functions to make sure i don't put the relays in a crossed state. 
 * Relay One input low turns relay one on
 * Relay One input high turns relay one off
 * Relay Two input low turns relay two off
 * Relay Two input high turns relay two on
 * 
 */
 void firingRelayOneOn() {
  // Make sure relay two is off
  digitalWrite(FIRING_ACTUATOR_RELAY_TWO_PIN, LOW);
  delay(ACTUATOR_RESPONSE_DELAY);

  // Turn relay one on
  digitalWrite(FIRING_ACTUATOR_RELAY_ONE_PIN,LOW);
  delay(ACTUATOR_RESPONSE_DELAY); 
}

void firingRelaysOff() {
   // Turn relay one off
  digitalWrite(FIRING_ACTUATOR_RELAY_ONE_PIN, HIGH);
  delay(ACTUATOR_RESPONSE_DELAY);

  // Turn relay two off
  digitalWrite(FIRING_ACTUATOR_RELAY_TWO_PIN,LOW);
  delay(ACTUATOR_RESPONSE_DELAY); 
}

void firingRelayTwoOn() {
  // Make sure relay one is off
  digitalWrite(FIRING_ACTUATOR_RELAY_ONE_PIN, HIGH);
  delay(ACTUATOR_RESPONSE_DELAY);

  // Turn relay two on
  digitalWrite(FIRING_ACTUATOR_RELAY_TWO_PIN, HIGH);
  delay(ACTUATOR_RESPONSE_DELAY);  
}

/**
 * Return true if motion is detected by the PIR
 */
boolean motionDetected() {
  // The OpenPIR's digital output is active high
  const int motionStatus = digitalRead(PIR_DIGITAL_PIN_IN);

  // If motion is detected, return true
  if (motionStatus == HIGH) {
    return true;
  } else {
    return false;
  }
}

void fire() {
  Serial.println("Fire!");  

  // Turn on firing motors and delay for a moment to let them get up to speed
  firingMotorRelayOn();
  delay(FIRING_MOTOR_WARMUP_DELAY);


  // TODO keep track of ammunition
  
  firingRelayOneOn();
  firingRelaysOff();
  firingRelayTwoOn();
  firingRelaysOff();

  // Turn off firing motors
  firingMotorRelayOff();
}

/**
 * Detect user pressing the manual fire button
 */
boolean isManualFireButtonPressed() {
  if(digitalRead(MANUAL_FIRE_BUTTON_IN) == HIGH) {
    Serial.println("Manual fire button pressed");
    return true;
  }

  return false;
}

/**
 * This code isn't used, but it is useful for testing. It pans the servo back and forth between min and max position. 
 */
void swivelTestTurrenPanServo() {
  // Start at current position (centered) and pan to TURRET_PAN_SERVO_MAX
  for(turretPanServoCurrentAngle = turretPanServoCurrentAngle; turretPanServoCurrentAngle <= TURRET_PAN_SERVO_MAX; turretPanServoCurrentAngle += 1) {
      Serial.print("Panning turrent pan servo to angle ");
      Serial.println(turretPanServoCurrentAngle);
      turretPanServo.write(turretPanServoCurrentAngle);
      delay(50);
  }

  // Start at TURRET_PAN_SERVO_MAX and pan to TURRET_PAN_SERVO_MIN
  for(turretPanServoCurrentAngle = TURRET_PAN_SERVO_MAX; turretPanServoCurrentAngle >= TURRET_PAN_SERVO_MIN; turretPanServoCurrentAngle -= 1) {
    turretPanServo.write(turretPanServoCurrentAngle);
    Serial.print("Panning turrent pan servo to angle ");
    Serial.println(turretPanServoCurrentAngle);
    delay(50);
  }
}
