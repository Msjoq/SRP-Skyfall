#include <Wire.h>
#include <MPU6050.h>
#include "BMP280_DEV.h"
#include "SparkFun_LIS331.h"

#include "functions.h"
#include "ISA.h"
#include "defines.h"

ISA atmo;
BiasCorrectingKalmanFilter kal;
BMP280_DEV bmp;
MPU6050 mpu(MPU);
LIS331 x1;
State state;

float pressure;
float mpuz;
unsigned long programtime = 0;
Values values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};


//init all sensors etc
void setup(void) {
  pinMode(ARM, INPUT);
  digitalWrite(ARM, HIGH); // enable pull up
  state = IDLE;
  Serial.begin(57600);
  Serial.println(" ");
  initMPU();
  initBMP();
  initLIS();

  calibrate();
}


void loop() {

  while (state == IDLE) {
          bmp.startForcedConversion();

          // as soon as we get new reading update
          if (bmp.getPressure(pressure)) {
            update_vals();
            transmit(values);

            // recalibrate pressure every so often to prevent the 0m point to drift.
            if ((int) (programtime / 40) % 24000 == 0) {
              calibrate();
            }

            // test transition to arm
            if (digitalRead(ARM)) {
              state = ARMED;
              Serial.println("Transition to arm!");
              delay(1000);
            }

          }

  }

  while (state == ARMED) {
        bmp.startForcedConversion();

        // as soon as we get new reading update (same as when idle)
        if (bmp.getPressure(pressure)) {
          update_vals();
          transmit(values);

          if ((int) (programtime / 40) % 24000 == 0) {
            calibrate();
          }

          if (!digitalRead(ARM)) {
            state = IDLE;
            Serial.println("Transition to idle!");
            delay(1000);
          }
        }
  }

  

  
  
}
