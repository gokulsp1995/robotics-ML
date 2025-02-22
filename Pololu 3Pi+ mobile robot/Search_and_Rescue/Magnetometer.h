/***************************************
  ,        .       .           .     ,--,
  |        |       |           |       /
  |    ,-: |-. ,-. |-. ,-. ,-. |-     `.
  |    | | | | `-. | | |-' |-' |        )
  `--' `-` `-' `-' ' ' `-' `-' `-'   `-'
***************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3

class Magnetometer_c {

  public:

    // Instance of the LIS3MDL class used to
    // interact with the magnetometer device.
    LIS3MDL mag;

    // A place to store the latest readings
    // from the magnetometer
    float readings[ MAX_AXIS ];

    float maximum[ MAX_AXIS ];
    float minimum[ MAX_AXIS ];
    float range[MAX_AXIS];
    float offset[MAX_AXIS];
    float scaling[MAX_AXIS];

    float calibrated[MAX_AXIS];

    float threshold = 1.6;
    float magnitude;
    // Constructor, must exist.
    Magnetometer_c () {
      // Leave this empty.
      // If you put Wire.begin() into this function
      // it will crash your microcontroller.
    }

    // Call this function witin your setup() function
    // to initialise the I2C protocol and the
    // magnetometer sensor
    bool initialise() {

      // Start the I2C protocol
      Wire.begin();

      // Try to connect to the magnetometer
      if (!mag.init()) {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1);
      }
      mag.enableDefault();
    }


    // Function to update readings array with
    // latest values from the sensor over i2c
    void getReadings() {
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    } // End of getReadings()
    void calcCalibratedMag() {
      getReadings();
      for ( int axis = 0; axis < MAX_AXIS; axis++) {
        calibrated[axis] = (readings[axis] - offset[axis]) * scaling[axis];
      }
      magnitude = sqrt(sq(calibrated[0]) + sq(calibrated[1]) + sq(calibrated[2]));
    }
    void calibrationOfMagnetometer(unsigned long duration_ms) {
      for (int axis = 0 ; axis < MAX_AXIS; axis++) {
        maximum[axis] = 9999.9;
        minimum[axis] = -9999.9;
      }
      unsigned long duration_ts = millis();
      while ( millis() - duration_ts < duration_ms) {   //ie, calibration is taking place

        calcCalibratedMag();
        for ( int axis = 0; axis < MAX_AXIS; axis++) {
          if (readings[axis] < minimum[axis]) {
            minimum[axis] = readings[axis];
          }
          if (readings[axis] > maximum[axis]) {
            maximum[axis] = readings[axis];
          }
        }
      }
      //      offset and scaling
      for ( int axis = 0; axis < MAX_AXIS; axis++) {
        range[axis] = maximum[axis] - minimum[axis];

        offset[axis] = minimum[axis] + (range[axis] / 2.0); // Mid point of the readings measured
        scaling[axis] = 1.0 / (range[axis] / 2.0); //Normalising scaling factor
      }
    }
    bool isOnMagnet() {
      if ( magnitude > threshold ) {
        return true;
      }
      else {
        return false;
      }
    }

}; // End of Magnetometer_c class definition

#endif
