#ifndef _BUMPSENSORS_H
#define _BUMPSENSORS_H

#define NUM_BUMP_SENSORS 2

const int analog_sensor_pin = A6;
const int digital_sensor_pins[NUM_BUMP_SENSORS] = { 4 , 5 };


#define EMIT_PIN 11

class BumpSensors_c {

  public:
    BumpSensors_c() {
    }

    float readings[NUM_BUMP_SENSORS], readings_a, readings_analog, minimum[NUM_BUMP_SENSORS], maximum[NUM_BUMP_SENSORS], scaling[NUM_BUMP_SENSORS], calibrated[NUM_BUMP_SENSORS], digitalreadings[NUM_BUMP_SENSORS];


    void initialiseAnalog() {
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);
      pinMode(analog_sensor_pin, INPUT_PULLUP);
    }

    void start_emitting() {

      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, LOW);
      Serial.println("Emitting Analog....");
    }

    void initiate_receiving() {
      pinMode(analog_sensor_pin, INPUT_PULLUP);
    }

    void start_receiving() {
      initiate_receiving();
      readings_analog = analogRead(analog_sensor_pin);
    }
    void initialiseForDigital() {
      pinMode(EMIT_PIN, INPUT);
      digitalWrite(EMIT_PIN, LOW);
    }

    void receive_sensor_digital() {
      for (int sensor = 0; sensor < NUM_BUMP_SENSORS; sensor++) {
        pinMode(digital_sensor_pins[sensor], OUTPUT);
        digitalWrite(digital_sensor_pins[sensor], HIGH);
        delayMicroseconds(10);
        pinMode(digital_sensor_pins[sensor], INPUT);
        unsigned long start_time = micros();
        while (digitalRead(digital_sensor_pins[sensor]) == HIGH) {
        }
        unsigned long end_time = micros();
        pinMode(EMIT_PIN, INPUT);
        unsigned long elapsed_time = end_time - start_time;
        digitalreadings[sensor] = elapsed_time;
      }
    }

// 224, 220 -- 2ms
// 300, 292  -- 5ms
// 360, 352 -- 10ms

    void emit_sensor_digital() {
      initialiseForDigital();
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, LOW);
      Serial.println("Emitting Bump Digital");
    }

    void calcCalibratedDigital() {
      receive_sensor_digital();
      for ( int sensor = 0; sensor < NUM_BUMP_SENSORS; sensor++ ) {
        calibrated[sensor] = ((digitalreadings[sensor]*100) - minimum[sensor]) / scaling[sensor];
      }
    }

};
#endif
