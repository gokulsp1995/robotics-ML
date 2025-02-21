

// These #include commands essentially "copy and paste"
// the above .h files (tabs above) into your code here.
#include "Motors.h"         // Labsheet 1
#include "PID.h"            // Labsheet 1 - Advanced
#include "LineSensors.h"    // Labsheet 2
#include "Magnetometer.h" // Labsheet 3
#include "Kinematics.h"     // Labsheet 4  maths for odometry/dead-reckoning
#include "Encoders.h"   // provide automatic counting of encoders // For encoder counts


// See Labsheet 0.
#define BUZZER_PIN 6

#define STATE_CALIBRATE 0  // give each behaviour a number
#define STATE_LEAVE_START 1
#define STATE_SEARCH_AREA 2
#define STATE_MAGNET_FOUND 3
#define STATE_TURN_TO_ORIGIN 4
#define STATE_GO_TO_ORIGIN 5
#define STATE_RETURN_TO_MAGNET 7
#define STATE_DEBUG 8

#define TURN_THRESHOLD 0.1

int state;          // this will track which STATE is active

float count;
float target_angle; //the angle to rotate to
float angle;
float distance_threshold = 1;
float distance;
float distance1;
float magnet_X;
float magnet_Y;
float left;
float right;
float theta_h;
float normal_angle;
float home_distance;
float x_origin;
float y_origin;
float theta_origin;

bool trial = false;
bool isTurningNow;
bool calibrationDone;
bool traverse;
bool isOnMagnet = false;
bool forward_moving = true;
bool moves = false;
bool magnet_detected;
bool a = false;
bool goingToStart;

bool updateMagPos = false;

unsigned long forwards_stop_duration;
unsigned long turn_stop_duration;
unsigned long startTime;
unsigned long beep_stop_duration;

//LCD
# include <PololuHD44780.h>
PololuHD44780 lcd(0, 1, 14, 17, 13, 30);
uint8_t savedUDIEN;
uint8_t savedUENUM;
uint8_t savedUEIENX0;
void disableLCD() {
  savedUDIEN = UDIEN;
  UDIEN = 0;
  savedUENUM = UENUM;
  UENUM = 0;
  savedUEIENX0 = UEIENX;
  UEIENX = 0;
}
void enableLCD() {
  UENUM = 0;
  UEIENX = savedUEIENX0;
  UENUM = savedUENUM;
  UDIEN = savedUDIEN;
}
//LCD
Motors_c motors;

LineSensors_c line_sensors;

Magnetometer_c magnetometer;

Kinematics_c pose;

void startBeep(unsigned long duration_ms) {
  analogWrite( BUZZER_PIN, 30 );
  beep_stop_duration = millis() + duration_ms;
}
void checkBeep() {
  if ( millis() >= beep_stop_duration) {
    analogWrite( BUZZER_PIN, 0);
  }
}

void setForwards(float left_pwm, float right_pwm, unsigned long duration_ms) {
  motors.setPWM( left_pwm, right_pwm);
  forwards_stop_duration = millis() + duration_ms;
  forward_moving = true;
}
void setTraverse(float left_pwm, float right_pwm, unsigned long distance_needed) {
  motors.setPWM( left_pwm, right_pwm);
  distance = pose.x + distance_needed;
  traverse = true;
}
bool checkTraverse() {
  if (fabs(distance - pose.x) < distance_threshold) {
    motors.setPWM( 0, 0 );
    return true;
    traverse = false;
  }
}
// Setting the left and right pwm directly
void setTurn(float left, float right, float target_angle ) {
  //  if (isTurningNow == false) {
  pose.update();
  motors.setPWM(left, right);
  angle = pose.theta + target_angle;
  isTurningNow = true;
}
void checkTurn() {
  pose.update();
  float ang_diff = angle - pose.theta;

  if ( fabs(ang_diff) < TURN_THRESHOLD ) { //float_abs
    motors.setPWM( 0, 0 );  // Stop robot
    //    angle = 0;
    isTurningNow = false;
  }
}
bool checkForwards() {
  if ( millis() >= forwards_stop_duration) {
    motors.setPWM( 0, 0 );
    //    setTurn(30, -20, 2000);
    return false;
  }
  else return true;
}
bool stateCalibrationFunc() {
  //Line sensor calibration with motor rotation
  pose.initialise(0, 0, 0);
  pose.update();
  setTurn( -30 , 30 , 4 * PI );
  while (isTurningNow == true) {
    checkTurn();
  }
  line_sensors.calibrateAlongRotation(3000);
  line_sensors.calcCalibratedADC();
  ////  Magnetometer Calibration
  magnetometer.calibrationOfMagnetometer(3000);
  //  return true;
  calibrationDone = true;
}
//const unsigned long debounceTime = 300;
//unsigned long lastLineTime = 0;
bool returned;
void setReturnToStart(float x, float y) {
  while (!returned) {
    pose.update();
    //direction from current (pose.x,pose.y) to (x,y)
    theta_h = atan2((y - pose.y), (x - pose.x));

    //normalize ang diff btw pose.theta, theta_h- making sure angle between -PI and PI
    normal_angle = atan2(sin(theta_h - pose.theta), cos(theta_h - pose.theta));

    distance1 = sqrt(sq(pose.x) + sq(pose.y));

    //  if (isTurningNow) {
    //    if (normal_angle < 0) {
    //      setTurn(30, -30, normal_angle);
    //    }
    //    else if (normal_angle > 0) {
    //      setTurn(-30, 30, normal_angle);
    //    }
    //    goingToStart = true;
    //  }

    if (normal_angle < 0) {
      setTurn(30, -30, normal_angle);
    }
    else if (normal_angle > 0) {
      setTurn(-30, 30, normal_angle);
    }
    while (isTurningNow) {
      checkTurn();
    }
    motors.setPWM(40, 40);

    pose.update();
    //    if (fabs(pose.x) <= distance_threshold && fabs(pose.y) <= distance_threshold) {
    //      motors.setPWM(0, 0);
    //      returned = true;
    //      state = 5;
    //
    //    }
    setTraverse(40, 40, distance1);
    while (traverse = true) {
      checkTraverse();
      state = STATE_RETURN_TO_MAGNET;
    }
  }
}
bool mag_returned;
void setReturnToMagnet(float x, float y) {
  while (!mag_returned) {
    pose.update();
    //direction from current (pose.x,pose.y) to (x,y)
    theta_h = atan2((y - pose.y ), (x - pose.x));

    //normalize ang diff btw pose.theta, theta_h- making sure angle between -PI and PI
    normal_angle = atan2(sin(theta_h - pose.theta), cos(theta_h - pose.theta));

    if (normal_angle < 0) {
      setTurn(30, -30, normal_angle);
    }
    else if (normal_angle > 0) {
      setTurn(-30, 30, normal_angle);
    }
    while (isTurningNow == true) {
      checkTurn();
    }
    //    motors.setPWM(40, 40);
    //    pose.update();
    //    if (fabs(pose.x) <= distance_threshold && fabs(pose.y) <= distance_threshold) {
    //      motors.setPWM(0, 0);
    //      mag_returned = true;
    //    }
  }
}


void center_stay() {
  if (pose.y > -235 && pose.x < 174) {
    setTurn(40, -30, PI);
    delay(20);
    //    while (isTurningNow == true) {
    //      checkTurn();
    //    }
  }
  else if (pose.y > -235 && pose.x > 174) {
    setTurn(-30, 40, PI);
    delay(20);
  }
}
void checkSearch() {
  pose.update();
  if (line_sensors.isOnLine(2) == true || line_sensors.isOnLine(3) == true || line_sensors.isOnLine(4) == true) {
    setTurn(-30,  30, 1.50);
    while (isTurningNow == true) {
      checkTurn();
    }
  }
  else if (line_sensors.isOnLine(2) == true && line_sensors.isOnLine(3) == true) {
    setTurn(-30,  30, 1.50);
    while (isTurningNow == true) {
      checkTurn();
    }
  }
  else if (line_sensors.isOnLine(3) == true && line_sensors.isOnLine(4) == true) {
    setTurn(-30,  30, 0.79);
    while (isTurningNow == true) {
      checkTurn();
    }
  }
  else if (line_sensors.isOnLine(2) == true && line_sensors.isOnLine(4) == true) {
    setTurn(-30,  30, 1.50);
    while (isTurningNow == true) {
      checkTurn();
    }
  }
  else if (line_sensors.isOnLine(2) == true && line_sensors.isOnLine(3) == true && line_sensors.isOnLine(4) == true) {
    setTurn(-30,  30, 0.79);
    while (isTurningNow == true) {
      checkTurn();
    }
  }
  else if (line_sensors.isOnLine(0) == true || line_sensors.isOnLine(1) == true || line_sensors.isOnLine(0) == true && line_sensors.isOnLine(1) == true || line_sensors.isOnLine(1) == true && line_sensors.isOnLine(2) == true || line_sensors.isOnLine(0) == true && line_sensors.isOnLine(2) == true || line_sensors.isOnLine(0) == true && line_sensors.isOnLine(1) == true && line_sensors.isOnLine(2) == true) {
    setTurn(30, -30, -1.50);
    while (isTurningNow == true) {
      checkTurn();
    }
  }
  else {
    setForwards(30, 30, 2000);
  }
}
bool checkSearchFunc() {
  checkSearch();
  center_stay();
  if (magnetometer.magnitude > magnetometer.threshold) {
    long int duration_ts = millis();
    startBeep(250);
    checkBeep();

    isOnMagnet = true;
  }
  else if ((magnetometer.magnitude < magnetometer.threshold)) {
    return true;
    isOnMagnet = false;
  }
}
bool magnetFound() {
  if (magnetometer.magnitude > magnetometer.threshold) {
    startBeep(250);
  }
  while (magnetometer.isOnMagnet() == true ) {
    motors.setPWM(0, 0);
    //    exit(0);
    return true;
    isOnMagnet = true;
  }
}
void setup() {
  // Update FSM.  Check which state we are currently
  // in.  States will transition themselves.
  state = STATE_CALIBRATE;

  motors.initialise();
  magnetometer.initialise();
  line_sensors.initialiseForADC();
  // These two functions are in encoders.h and
  // they activate the encoder sensors to
  // measure the wheel rotation.  You do not
  // need to change or update these.
  // These are used in Labsheet 4.
  // Encoder counts are counted automatically.
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);

  Serial.begin(9600);
  Serial.println(" *** READY *** ");
}
void loop() {
  // Instruct the kinematics to perform an update
  // of the robot position
  // It is recommended you schedule this to occur at
  // a fixed interval, such as 20ms.
  unsigned long duration_ts = 0;
  unsigned long duration_ms = millis();
  if (duration_ms - duration_ts > 20) {
    pose.update();
    duration_ts = duration_ms;
  }
  //LCD display
  disableLCD();
  //  lcd.clear();
  lcd.gotoXY(0, 0);
  //  lcd.print('m');
  //  lcd.print(':');
  //  lcd.print(magnetometer.magnitude);
  pose.update();
  lcd.print("Mag: "); lcd.print(magnetometer.magnitude);
  lcd.gotoXY(0, 1);
  lcd.print("State:");
  lcd.print(state);
  Serial.println(state);
  enableLCD();
  // Get latest readings.
  checkBeep();
  Serial.println("Calibrated value");
  magnetometer.calcCalibratedMag();
  if (magnetometer.magnitude > magnetometer.threshold) {
    magnet_detected = true;
  }
  Serial.println(line_sensors.calibrated[0]);
  Serial.println(line_sensors.calibrated[1]);
  Serial.println(line_sensors.calibrated[2]);
  Serial.println(line_sensors.calibrated[3]);
  Serial.println(line_sensors.calibrated[4]);
  Serial.println( magnetometer.magnitude );

  if ( state == STATE_CALIBRATE ) {
    //    startTime = millis();    // call calibration routine (non-blocking)
    pose.update();
    stateCalibrationFunc();
    // Should we move to the next state?
    if ( calibrationDone == true ) {

      setTurn( 30 , -30 , -0.79 );
      while (isTurningNow == true) {
        checkTurn();
      }
      setForwards(30, 30, 2800);
      //      motors.setPWM(30, 30);
      state = STATE_LEAVE_START;
    }
  }
  // Prepare any variables or functions
  //      setForwards(30, 30, 2000 );

  else if ( state == STATE_LEAVE_START ) {
    // Finished moving forwards? (non-blocking)
    //    setForwards(30, 30, 200 );
    if (moves == false) {
      checkForwards();
      moves = true;
    }
    if (!checkForwards()) {
      state = STATE_SEARCH_AREA;
    }
  }
  else if ( state == STATE_SEARCH_AREA ) {
    // continue search (non-blocking)
    //    checkSearch();
    checkForwards();
    //    motors.setPWM(0, 0);
    //    checkSearchFunc();
    bool status = checkSearchFunc();
    // Should we move to next state?
    if ( isOnMagnet == true ) {
      // move the state variable to next state
      motors.setPWM(0, 0);
      pose.update();
      if (!updateMagPos) {
        magnet_X = pose.x;
        magnet_Y = pose.y;
        updateMagPos = true;
      }
      state = STATE_TURN_TO_ORIGIN;
    }
  }
  else if (state == STATE_TURN_TO_ORIGIN) {  // GO_TO_ORIGIN

    setReturnToStart(0, 0);
    while (returned == true) {
      state = STATE_RETURN_TO_MAGNET;
    }
    //}
    //    while (goingToStart == true ) {
    //      checkReturnToStart(0, 0);
    //      isOnMagnet = false;
    //      state = STATE_RETURN_TO_MAGNET;
    //    }
    Serial.print("normal_angle: "); Serial.println(normal_angle);
    Serial.print("angle: "); Serial.println(angle);
    Serial.print("pose.theta: "); Serial.println(pose.theta);
    Serial.print("theta_h: "); Serial.println(theta_h);
  }

  else if ( state == STATE_RETURN_TO_MAGNET ) {
//    setReturnToStart(magnet_X, magnet_Y);
    setReturnToMagnet(magnet_X, magnet_Y);
    distance = sqrt(sq(magnet_X) + sq(magnet_Y));
    setTraverse(30, 30, distance);
    while (traverse = true) {
      checkTraverse();
    }
  }
} // End of FSM update
