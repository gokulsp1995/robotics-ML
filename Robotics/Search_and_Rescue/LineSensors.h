/***************************************
  ,        .       .           .     ,-.
  |        |       |           |        )
  |    ,-: |-. ,-. |-. ,-. ,-. |-      /
  |    | | | | `-. | | |-' |-' |      /
  `--' `-` `-' `-' ' ' `-' `-' `-'   '--'
****************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#include "Motors.h"

extern Motors_c motors;

// We will use all 5 line sensors (DN1 - 5)
// and so define a constant here, rather than
// type '5' in lots of places.
#define NUM_SENSORS 5

// Pin definitions
// This time, we will use an array to store the
// pin definitions.  This is a bit like a list.
// This way, we can either loop through the
// list automatically, or we can ask for a pin
// by indexing, e.g. sensor_pins[0] is A11,
// sensors_pins[1] is A0.
const int sensor_pins[ NUM_SENSORS ] = { A11, A0, A2, A3, A4 };

// This is the pin used to turn on the infra-
// red LEDs.
#define EMIT_PIN 11


// Class to operate the linesensors.
class LineSensors_c {

  public:

    // Store your readings into this array.
    // You can then access these readings elsewhere
    // by using the syntax line_sensors.readings[n];
    // Where n is a value [0:4]
    float readings[ NUM_SENSORS ];

    // Variables to store calibration constants.
    // Make use of these as a part of the exercises
    // in labsheet 2.
    float minimum[ NUM_SENSORS ];
    float maximum[ NUM_SENSORS ];
    float scaling[ NUM_SENSORS ];
    float threshold = 3;

    float range[ NUM_SENSORS ];

    // Variable to store the calculated calibrated
    // (corrected) readings. Needs to be updated via
    // a function call, which is completed in
    // labsheet 2.
    float calibrated[ NUM_SENSORS ];

    // Constructor, must exist.
    LineSensor_c() {
      // leave this empty
    }

    // Refer to Labsheet 2: Approach 1
    // Fix areas marked ????
    // Use this function to setup the pins required
    // to perform an read of the line sensors using
    // the ADC.
    void initialiseForADC() {

      // Ensure that the IR LEDs are on
      // for line sensing
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );
      // Configure the line sensor pins
      // DN1, DN2, DN3, DN4, DN5.
      for ( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        pinMode( sensor_pins[ sensor ], INPUT_PULLUP );
      }
    } // End of initialiseForADC()

    void readSensorsADC() {
      // First, initialise the pins.
      // You need to complete this function (above).
      initialiseForADC();
      for ( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        readings[sensor] = analogRead(sensor_pins[ sensor ]);
      }
    } // End of readSensorsADC()

    // Use this function to apply the calibration values
    // that were captured in your calibration routine.
    // Therefore, you will need to write a calibration
    // routine (see Labsheet 2)

    void calcCalibratedADC() {
      // Get latest readings (raw values)
      readSensorsADC();
      // Apply calibration values, store in calibrated[]
      for ( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        calibrated[sensor] = ((readings[sensor] - minimum[sensor]) / range[sensor]);
//        if (calibrated[sensor] < 0) {
//          calibrated[sensor] = 0;
//        }
//        if (calibrated[sensor] > 1) {
//          calibrated[sensor] = 1;
//        }
      }
    } // End of calcCalibratedADC()
    //};
    void calibrateAlongRotation(unsigned long duration_ms ) {
      for ( int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        minimum[sensor] = 1023;
        maximum[sensor] = 0;
      }
      unsigned long duration_ts = millis();
      while ( millis() - duration_ts < duration_ms ) {
        readSensorsADC();

        for ( int sensor = 0; sensor < NUM_SENSORS; sensor++) {
          if (readings[sensor] < minimum[sensor]) {
            minimum[sensor] = readings[sensor];
          }
          if (readings[sensor] > maximum[sensor]) {
            maximum[sensor] = readings[sensor];
          }
        }
        delay(10);
      }
      //Updating scaling factor
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        range[sensor] = maximum[sensor] - minimum[sensor];
        if (range > 0) {
          scaling[sensor] = 1 / range[sensor];
        }
        else {
          scaling[sensor] = 1.0; //in case, division by zero
        }
      }
    }
    //  to check whether on the black line
    //    bool isOnLine( int which_sensor ) {
    //
    //      return calibrated[which_sensor] < threshold;
    //    }
    bool isOnLine( int sensor_num ){
      calcCalibratedADC();
      for ( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        if ( calibrated[sensor_num] > threshold ){
          return true;
        }
        else if (calibrated[sensor_num] < threshold){
          return false;
        }
      }
    }

    // Part of the Advanced Exercises for Labsheet 2
    void initialiseForDigital() {

      // Ensure that the IR LEDs are on
      // for line sensing
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );

    } // End of initialiseForDigital()

    // Part of the Advanced Exercises for Labsheet 2
    void readSensorsDigital() {
      //  ???
    } // End of readSensorsDigital()

}; // End of LineSensor_c class defintion



#endif
