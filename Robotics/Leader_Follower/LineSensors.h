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
#define NUM_SENSORS 5

const int sensor_pins[NUM_SENSORS] = { A11, A0, A2, A3, A4 };

#define EMIT_PIN 11

class LineSensors_c {

  public:
    LineSensors_c() {
    }

    float readings[NUM_SENSORS];
    float digitalreadings[NUM_SENSORS];
    float minimum[NUM_SENSORS];
    float maximum[NUM_SENSORS];
    float scaling[NUM_SENSORS];
    float calibrated[NUM_SENSORS];

    void initialiseForADC() {

      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        pinMode(sensor_pins[sensor], INPUT_PULLUP);
      }

    }  // End of initialiseForADC()

    void readSensorsADC() {
      initialiseForADC();
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        readings[sensor] = analogRead(sensor_pins[sensor]);
      }

    }  // End of readSensorsADC()

    void calcCalibratedADC() {
      readSensorsADC();
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        calibrated[sensor] = (readings[sensor] - minimum[sensor]) / scaling[sensor];
      }
    }  // End of calcCalibratedADC()

    bool LineDetected(int sensor_num) {
      float threshold = 0.6;
      calcCalibratedADC();
      if (calibrated[sensor_num] > threshold) {
        return true;
      } else {
        return false;
      }
    }

    void start_emitting() {
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);
      Serial.println("Emittingggg Analog....");
    }

    void initiate_receiving() {
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        pinMode(sensor_pins[sensor], INPUT_PULLUP);
      }
    }

    void start_receiving() {
      initiate_receiving();
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        readings[sensor] = analogRead(sensor_pins[sensor]);
      }
    }
    void initialiseForDigital() {
      pinMode(EMIT_PIN, INPUT);
      digitalWrite(EMIT_PIN, LOW);
    }

    void receive_sensor_digital() {
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        pinMode(sensor_pins[sensor], OUTPUT);
        digitalWrite(sensor_pins[sensor], HIGH);
        delayMicroseconds(10);
        pinMode(sensor_pins[sensor], INPUT);
        unsigned long start_time = micros();
        while (digitalRead(sensor_pins[sensor]) == HIGH) {
        }
        unsigned long end_time = micros();
        pinMode(EMIT_PIN, INPUT);
        unsigned long elapsed_time = end_time - start_time;
        digitalreadings[sensor] = elapsed_time;
      }
    }

    void emit_sensor_digital() {
      initialiseForDigital();
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);
      Serial.println("Emitting Line Digital");
    }

    void calcCalibratedDigital() {
      receive_sensor_digital();
      for ( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        calibrated[sensor] = ((digitalreadings[sensor]*10) - minimum[sensor]) / scaling[sensor];
      }
    }

};


#endif
