#include "LineSensors.h"
#include "BumpSensors.h"
#include "Motors.h"
#include "Kinematics.h"
#include "Encoders.h"
#include "PID.h"
#include <PololuOLED.h>

// States
#define DELAY 0
#define LEADER 1
#define LEADER2 2
#define FOLLOWERDISCRETE 3
#define FOLLOWERDISCRETELPF 4
#define FOLLOWERCONTINUOUS 5
#define DISPLAYRESULTS 6
#define DEBUG 7

// Definitions
#define SPEED_EST_MS 10
#define FWD 00
#define REV 11
#define MAX_RESULTS 100
#define VARIABLES 3

// Initiations
Motors_c motors;
Kinematics_c pose;
LineSensors_c line_sensors;
BumpSensors_c bump_sensors;
PID_c speed_pid;

// OLED Display
PololuSH1106 display(1, 30, 0, 17, 13);

// Global Variables
bool startForwarding = false;
long lastE0, lastE1;
uint8_t savedUDIEN, savedUENUM, savedUEIENX0;
int state, followThreshold = 600, count = 0, results_index;
float MAXI_PWM = 50, MIN_PWM = 15, increment = 0, lpf_pwm, lpf_pwm_right, lpf_pwm_prev, lpf_pwm_right_prev, alpha = 0.7, results[MAX_RESULTS][VARIABLES], speedE0, speedE1;
unsigned long estimatedSpeedTS, estimatedSpeedTS1, currentTime, mission_complete_ts_leader, mission_complete_ts_follower, moveReverseTime, moveForwardTime, thresholdTime, startDelayTime;

void setup() {
  // Serial debug begins
  Serial.begin(115200);
  Serial.println(" *** READY *** ");

  // Change different states for the robot
  state = DELAY;

  // Initialise configurations
  speed_pid.initialise(1.0, 0.0, 0.0);
  pose.initialise(0, 0, 0);
  results_index = 0;
  setupEncoder0();
  setupEncoder1();
  lpf_pwm = 0.0;
  lastE0 = count_e0;
  lastE1 = count_e1;
  speedE0 = 0.0;
  speedE1 = 0.0;

  // Initiate timings
  currentTime = millis();
  mission_complete_ts_leader = millis() + 11000;
  mission_complete_ts_follower = millis() + 13000;
  moveReverseTime = millis() + 6000;
  estimatedSpeedTS = millis();
  estimatedSpeedTS1 = millis();
  thresholdTime = millis() + 2500;
}

void loop() {

  if (state == DELAY) {
    // Delay state to let us align the robot in a straight line before beginning the experiment
    displayLEDString("ALIGN");
    delay(2000);
    // Change this when flashing the robot with different "folllower" states or leader
    state = LEADER;   //LEADER  //FOLLOWERCONTINUOUS  //FOLLOWERDISCRETELPF  //FOLLOWERDISCRETE
  }

  else if (state == LEADER) {
    // Leader robot for moving backward for 3 seconds
    displayLEDString("LEADER");
    bump_sensors.emit_sensor_digital();
    float detection[2];
    float detection_speed[2];
    bool moveStatusRev = checkMovingRev();
    float leftSpeed = -30.0;
    float rightSpeed = -30.0;
    if (moveStatusRev) {
      state = LEADER2;
      startDelayTime = millis() + 1000;
      moveForwardTime = millis() + 5000;
    } else {
      motors.setPWM(leftSpeed, rightSpeed);
    }
    pose.update();
    calculateSpeed(detection_speed);
    float timeInSeconds = float(millis()) / 1000;
    storeResults(timeInSeconds, detection_speed[0], detection_speed[1]);
  }

  else if (state == LEADER2) {
    // Leader robot for moving forward for 3 seconds
    if (millis() >= startDelayTime) {
      displayLEDString("LEADER");
      bool moveStatusFwd = checkMovingFwd();
      float leftSpeed = 30.0;
      float rightSpeed = 30.0;
      if (moveStatusFwd) {
        if (millis() >= mission_complete_ts_leader) {
          state = DISPLAYRESULTS;
          motors.setPWM(0.0, 0.0);
        } else {
          motors.setPWM(leftSpeed, rightSpeed);
        }
      }
      pose.update();
      float detection_speed[2];
      calculateSpeed(detection_speed);
      float timeInSeconds = float(millis()) / 1000;
      storeResults(timeInSeconds, detection_speed[0], detection_speed[1]);
    } else {
      delay(100); // adding this to reduce filling up the "results" variable
      float detection_speed[2];
      calculateSpeed(detection_speed);
      float timeInSeconds = float(millis()) / 1000;
      storeResults(timeInSeconds, detection_speed[0], detection_speed[1]);
    }

  }

  else if (state == FOLLOWERDISCRETELPF) {
    // Follower robot for discrete speed control with a low pass filter
    displayLEDString("FOLLOWER");
    bump_sensors.receive_sensor_digital();
    Serial.print(bump_sensors.digitalreadings[1]);
    Serial.print(',');
    Serial.println(bump_sensors.digitalreadings[0]);
    int avgReading = (bump_sensors.digitalreadings[1] + bump_sensors.digitalreadings[0]) / 2;
    int error = avgReading - followThreshold;
    float detection[2];
    float detection_speed[2];
    const float FORWARD_PWM = 40.0;
    const float REVERSE_PWM = -40.0;
    const float STOP_PWM = 0.0;
    if (abs(error) < 2) { error = 0; }
    float target_pwm = STOP_PWM;
    if (error > 0) {
      target_pwm = FORWARD_PWM;
    } else if (error < 0 || bump_sensors.digitalreadings[0] > 3000) {
      target_pwm = REVERSE_PWM;
    }
    lpf_pwm = (alpha * target_pwm) + ((1.0 - alpha) * lpf_pwm);
    if (lpf_pwm > 0) {
      moveStraightLine(FWD, abs(lpf_pwm), abs(lpf_pwm), detection);
    } else if (lpf_pwm < 0) {
      moveStraightLine(REV, abs(lpf_pwm), abs(lpf_pwm), detection);
    } else {
      motors.setPWM(0.0, 0.0);
      lpf_pwm = (alpha * lpf_pwm) + ((1.0 - alpha) * STOP_PWM);  // 1
    }
    pose.update();
    calculateSpeed(detection_speed);
    float timeInSeconds = float(millis()) / 1000;
    storeResults(timeInSeconds, detection[0], detection[1]);
    if (millis() >= mission_complete_ts_follower) {
      state = DISPLAYRESULTS;
    }
  }

  else if (state == FOLLOWERDISCRETE) {
    // Follower robot for discrete speed control with a low pass filter
    displayLEDString("FOLLOWER");
    bump_sensors.receive_sensor_digital();
    int avgReading = (bump_sensors.digitalreadings[1] + bump_sensors.digitalreadings[0]) / 2;
    int error = avgReading - followThreshold;
    float detection[2];
    float detection_speed[2];
    float leftSpeed;
    float rightSpeed;
    if (error > 0) {
      leftSpeed = 30.0;
      rightSpeed = 30.0;
      motors.setPWM(leftSpeed, rightSpeed);
    } else if (error < 0 || bump_sensors.digitalreadings[0] > 3000) {
      leftSpeed = -30.0;
      rightSpeed = -30.0;
      motors.setPWM(leftSpeed, rightSpeed);
    } else {
      leftSpeed = 0.0;
      rightSpeed = 0.0;
      motors.setPWM(leftSpeed, rightSpeed);
    }
    pose.update();
    calculateSpeed(detection_speed);
    float timeInSeconds = float(millis()) / 1000;
    storeResults(timeInSeconds, detection_speed[0], detection_speed[1]);
    if (millis() >= mission_complete_ts_follower) {
      state = DISPLAYRESULTS;
    }
  }

  else if (state == FOLLOWERCONTINUOUS) {
    // Follower robot for continuous speed control by mapping the speed to the error of the bump sensor readings
    displayLEDString("FOLLOWER");
    bump_sensors.receive_sensor_digital();
    int avgReading = (bump_sensors.digitalreadings[1] + bump_sensors.digitalreadings[0]) / 2;
    int error = avgReading - followThreshold;
    int speed = constrain(map(abs(error), 0, 400, MIN_PWM, MAXI_PWM), MIN_PWM, MAXI_PWM);
    float detection[2];
    float detection_speed[2];
    if (abs(error) < 10) { error = 0; }
    if (error > 0) {
      moveStraightLine(FWD, speed, speed, detection);
    } else if (error < 0 || bump_sensors.digitalreadings[0] > 3000) {
      moveStraightLine(REV, speed, speed, detection);
    } else {
      motors.setPWM(0.0, 0.0);
    }
    pose.update();
    calculateSpeed(detection_speed);
    float timeInSeconds = float(millis()) / 1000;
    Serial.println(detection_speed[0]);
    storeResults(timeInSeconds, detection_speed[0], detection_speed[1]);
    if (millis() >= mission_complete_ts_follower) {
      state = DISPLAYRESULTS;
    }
  }

  else if (state == DISPLAYRESULTS) {
    // Display the results over serial after the experiment is complete
    displayLEDString("RESULTS");
    motors.setPWM(0, 0);
    int result;
    Serial.println(" Time, Threeshold, Speed\n");
    for (result = 0; result < MAX_RESULTS; result++) {
      Serial.print(result);
      Serial.print(",");
      Serial.print(results[result][0]);
      Serial.print(",");
      Serial.print(results[result][1]);
      Serial.print(",");
      Serial.print(results[result][2]);
      Serial.print("\n");
    }
    delay(5000);
  }

  else if (state == DEBUG) {
    // Debug state used for experimentations
    displayLEDString("LEADER");
    bump_sensors.emit_sensor_digital();
  }
}

float moveStraightLine(float dir, float left, float right, float detection[]) {
  unsigned long elapsedTime = millis() - estimatedSpeedTS;
  if (elapsedTime >= SPEED_EST_MS) {
    estimatedSpeedTS = millis();
    float countDiff = count_e0 - count_e1;
    float correction = speed_pid.update(10, countDiff);
    if (dir == REV) {
      float final_left_speed = -left - correction;
      float final_right_speed = -right - correction;
      detection[0] = final_left_speed;
      detection[1] = final_right_speed;
      motors.setPWM(final_left_speed, final_right_speed);
    } else if (dir == FWD) {
      float final_left_speed = left - correction;
      float final_right_speed = right - correction;
      detection[0] = final_left_speed;
      detection[1] = final_right_speed;
      motors.setPWM(final_left_speed, final_right_speed);
    }
  }
}

float calculateSpeed(float detection[]) {
  unsigned long elapsedTime = millis() - estimatedSpeedTS1;
  if (elapsedTime >= SPEED_EST_MS) {
    estimatedSpeedTS1 = millis();
    long count_difference_e0 = count_e0 - lastE0;
    long count_difference_e1 = count_e1 - lastE1;
    lastE0 = count_e0;
    lastE1 = count_e1;
    speedE0 = (float(count_difference_e0)) / SPEED_EST_MS;
    speedE1 = (float(count_difference_e1)) / SPEED_EST_MS;
    detection[0] = speedE0;
    detection[1] = speedE1;
  }
}

void displayLEDString(String words) {
  disableUSB();
  display.clear();
  display.gotoXY(0, 0);
  display.print(words);
  enableUSB();
}

void storeResults(float time, float left, float right) {
  if (results_index < MAX_RESULTS) {
    pose.update();
    results[results_index][0] = time;
    results[results_index][1] = left;
    results[results_index][2] = right;
    results_index++;
  }
}

void disableUSB() {
  savedUDIEN = UDIEN;
  UDIEN = 0;
  savedUENUM = UENUM;
  UENUM = 0;
  savedUEIENX0 = UEIENX;
  UEIENX = 0;
}

void enableUSB() {
  UENUM = 0;
  UEIENX = savedUEIENX0;
  UENUM = savedUENUM;
  UDIEN = savedUDIEN;
}

void stopMotor() {
  motors.setPWM(0.0, 0.0);
}

bool checkMovingRev() {
  if (millis() >= moveReverseTime) {
    motors.setPWM(0, 0);
    return true;
  } else {
    return false;
  }
}

bool checkMovingFwd() {
  if (millis() >= moveForwardTime) {
    motors.setPWM(0, 0);
    startForwarding = false;
    return true;
  } else {
    return false;
  }
}
