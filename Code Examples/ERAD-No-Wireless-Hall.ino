/*
 * ERAD2 example program for ESP32S3-based wireless BLDC motor driver. Check it out at roboticworx.io!
 * This program is just a basic example with no wireless stuff to move a motor around a little.
*/

#include <SimpleFOC.h>

// Define the BLDC motor pins
// If you're using the same ERAD2 PCB these don't need to change.
#define BLDC_PWM_UH_GPIO 7
#define BLDC_PWM_UL_GPIO 8
#define BLDC_PWM_VH_GPIO 9
#define BLDC_PWM_VL_GPIO 10
#define BLDC_PWM_WH_GPIO 11
#define BLDC_PWM_WL_GPIO 12

#define DRV0_CSN_PIN 14
#define DRV1_SCK_PIN 13

#define HALLU_PIN 33
#define HALLV_PIN 34
#define HALLW_PIN 35

#define ENABLE_PIN 17

#define BLINK_PIN 38

#define POLE_PAIRS 2 // CHANGE THIS: Number of permanent magnets in motor divided by 2

unsigned long previousMillis = 0;
bool blinkState = true;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(BLDC_PWM_UH_GPIO, BLDC_PWM_UL_GPIO, BLDC_PWM_VH_GPIO, BLDC_PWM_VL_GPIO, BLDC_PWM_WH_GPIO, BLDC_PWM_WL_GPIO);

// Hall sensor instance
HallSensor sensor = HallSensor(HALLU_PIN, HALLV_PIN, HALLW_PIN, POLE_PAIRS);

// Interrupt routine intialisation
// Channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

// Angle set point variable
float target_angle = 0; // Default angle
float offset = 10; // Change angle used for testing
int count = 0; // Counter used for testing

// Instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() 
{
  // Use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  //motor.useMonitoring(Serial);
  
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(BLINK_PIN, OUTPUT);
  pinMode(DRV0_CSN_PIN, OUTPUT);
  pinMode(DRV1_SCK_PIN, OUTPUT);

  // Set TMC6200 drive configuration (medium strength)
  digitalWrite(DRV1_SCK_PIN, LOW);
  digitalWrite(DRV0_CSN_PIN, HIGH);
  
  // Toggle reset to clear pending errors
  digitalWrite(ENABLE_PIN, LOW);
  delay(250);
  digitalWrite(ENABLE_PIN, HIGH);

  // Initialize sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);

  // Iink the motor to the sensor
  motor.linkSensor(&sensor);

  // Driver config
  // Power supply voltage [V]
  driver.voltage_power_supply = 24; // Change to whatever
  driver.init();

  // Link the motor and the driver
  motor.linkDriver(&driver);

  // Aligning voltage [V]
  motor.voltage_sensor_align = 2; // START SMALL THEN INCREASE IF NEEDED. 2 is what I used in my ML5010 demo, 3 for the robotic arm. SHOULD BE SMALL!

  // Index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // Set motion control loop to be used
  motor.controller = MotionControlType::angle; // Can also change to velocity, but angle allows holding at a location

  // contoller configuration
  // default parameters in defaults.h

  // Velocity PI controller parameters
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;

  // Default voltage_power_supply
  motor.voltage_limit = 3; // START SMALL THEN INCREASE IF NEEDED. 3 is what I used in my demos. SHOULD BE SMALL!

  // Jerk control using voltage voltage ramp
  // Default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 100;

  // Velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // Angle P controller
  motor.P_angle.P = 20;
  // Maximal velocity of the position control
  motor.velocity_limit = 150;

  // Initialize motor
  motor.init();
  // Align sensor and start FOC
  motor.initFOC();

  // Add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  //Serial.println(F("Set the target angle using serial terminal:"));
  delay(1000);
}

void loop() {
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 500) // Every 500ms
  {
    digitalWrite(BLINK_PIN, blinkState); // Toggle LED
    blinkState = !blinkState;

    target_angle = target_angle + offset; // Change target angle every 500ms by "offset" for testing/demo
    count++; // Keep count of how many changes in offset for testing
    //Serial.println(target_angle);
    previousMillis = currentMillis;
  }

  if (count >= 18) // Go the other way after 18 changes (FOR TESTING/DEMO)
  {
    offset = -offset;
    count = 0;
  }

  // Main FOC algorithm function
  // The faster you run this function the better.
  motor.loopFOC();

  // Motion control function
  // Velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  if (motor.controller == MotionControlType::angle) { 
    float c = _PI_3/motor.pole_pairs;
    target_angle = floor(target_angle / c + 0.5f) * c;
  }
  motor.move(target_angle);
  // The HallSensor angle only changes in whole steps, so if the target is inbetween two of them, 
  // the motor will oscillate back and forth, never able to get the sensor exactly equal to target.

  // Function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!! Comment out if not needed.
  // motor.monitor();

  // user communication
  command.run();
}