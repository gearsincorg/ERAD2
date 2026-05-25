/*
 * ERAD2 example program for ESP32S3-based wireless BLDC motor driver. Check it out at roboticworx.io!
 * This program takes an angle received from the ERAD Sender example and then moves to it.
*/

#include <SimpleFOC.h>
#include <esp_now.h>
#include <WiFi.h>

#define MAX_SPEED_MPS  2.0
#define RADIUS_M       0.0837
#define MPS_to_RPS    11.967
#define FAILSAFE_MS 1000

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

#define POLE_PAIRS 15 // CHANGE THIS: Number of permanent magnets in motor divided by 2

unsigned long  previousMillis  = 0;
bool           blinkState      = true;
uint32_t       lastTime        = 0;
bool           enabled         = false;
float          target_rps      = 0;

typedef struct struct_message {
  float v_mps;
} struct_message;

// Create a struct_message to hold the incoming data
struct_message incomingData;

// Callback function that runs when data is received for ESP-NOW wireless
void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));  // Copy the received data into the struct

  float speed_mps = incomingData.v_mps ;
  if (speed_mps > MAX_SPEED_MPS) {
    speed_mps = MAX_SPEED_MPS;
  } else if (speed_mps < -MAX_SPEED_MPS) {
    speed_mps = -MAX_SPEED_MPS;
  }

  // convert Meters/Sec to Radians/Sec
  target_rps = speed_mps * MPS_to_RPS;
  lastTime = millis();
}

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(BLDC_PWM_UH_GPIO, BLDC_PWM_UL_GPIO, BLDC_PWM_VH_GPIO, BLDC_PWM_VL_GPIO, BLDC_PWM_WH_GPIO, BLDC_PWM_WL_GPIO);

// Hall sensor instance
HallSensor sensor = HallSensor(HALLU_PIN, HALLV_PIN, HALLW_PIN, POLE_PAIRS);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}


// Instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_rps, cmd); }

void setup() 
{
  // Use monitoring with serial
  Serial.begin(115200);

  // Comment out if not needed
  // motor.useMonitoring(Serial);

  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);
  Serial.println("ESP-NOW Receiver");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback
  esp_now_register_recv_cb(onDataRecv);
  
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
  sensor.pullup = Pullup::USE_INTERN; //  <<< Phil
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);

  // Link the motor to the sensor
  motor.linkSensor(&sensor);

  // Driver Config
  // Power supply voltage [V]
  driver.voltage_power_supply = 12; 
  driver.init();

  // Link the motor and the driver
  motor.linkDriver(&driver);

  // Aligning voltage [V]
     // motor.voltage_sensor_align = 2; // START SMALL THEN INCREASE IF NEEDED. 2 is what I used in my ML5010 demo, 3 for the robotic arm. SHOULD BE SMALL!
  // Index search velocity [rad/s]
     // motor.velocity_index_search = 3;

  // Set motion control loop to be used
  motor.controller = MotionControlType::velocity; 

  // controller configuration
  // default parameters in defaults.h

  // Velocity PI controller parameters
  motor.PID_velocity.P = 0.3f;
  motor.PID_velocity.I = 1.0f;
  motor.PID_velocity.D = 0.0f;
  
  // Default voltage_power_supply
     // motor.voltage_limit = 3; // START SMALL THEN INCREASE IF NEEDED. 3 is what I used in my demos. SHOULD BE SMALL!

  // Jerk control using voltage voltage ramp
  // Default value is 300 volts per sec  ~ 0.3V per millisecond
     // motor.PID_velocity.output_ramp = 100;

  // Velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;
  motor.sensor_direction = Direction::CW; 
  motor.zero_electric_angle = 4.19; 

  // Initialize motor
  motor.init();
  // Align sensor and start FOC
  motor.initFOC();

  // Add target command T
  // command.add('T', doTarget, "target velocity");

  //Serial.println(F("Motor ready."));
  //Serial.println(F("Set the target angle using serial terminal:"));
  // delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();

  // check for loss of signal.
  if ((millis() - lastTime) > FAILSAFE_MS){
    target_rps = 0;
  }
  
  // Disable motor when target_velocity == 0;
  if (enabled) {
    if (target_rps == 0) {
      motor.disable();
      enabled = false;
    }
  } else {
    if (target_rps != 0) {
      motor.enable();
      enabled = true;
    }
  }

  if (currentMillis - previousMillis >= 250) // Every 250ms
  {
    digitalWrite(BLINK_PIN, blinkState); // Toggle LED
    blinkState = !blinkState;
    previousMillis = currentMillis;

    Serial.print(enabled ? "ENA " : "DIS ");
    Serial.print(target_rps);
    Serial.print(" ");
    Serial.println(sensor.getVelocity());
  }

  // Main FOC algorithm function
  // The faster you run this function the better
  motor.loopFOC();
  motor.move(target_rps);
 
  // Function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!! Comment out if not needed.
  // motor.monitor();

  // User communication
  // command.run();
}