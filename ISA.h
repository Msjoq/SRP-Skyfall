//This file contains both a class to calculate the altitude from pressure, temp readings
//as well as a class with a Kalman filter. This filter takes accelerations and height readings
//and filters them to get a pretty accurate estimation of the vehicle's state, as well as its accuracy.
// Also compensates for any consistent bias in accelerometer readings, which is nice. 
//All of the concepts are shamelessly ripped from Maurits van Heijningen's post on slack,  
//just rewritten by me with arduino libraries.

#ifndef ISA_h
#define ISA_h

#include "Arduino.h"
#include <BasicLinearAlgebra.h>
#include <math.h>
#include "BMP280_DEV.h"


using namespace  BLA;

//this is defined in ISA.cpp
class ISA {
		public:
		  ISA();
		  void calibrate(float pressure, float temperature);
		  float height(float pressure) const;

    private:
      float base_pressure;
      float base_temperature;	
};

class BiasCorrectingKalmanFilter {

  private:
    //Stuff changin
    BLA::Matrix<4> x;
    BLA::Matrix<4, 4> P;

  //tunable:
    const float stddev_jerk;

    //enum
    typedef enum {
      IDLE = 0,
      ARMED = 1,
      ASCENT = 2,
      DESCENT = 3,
    } State;
    State state;

  public:
    BiasCorrectingKalmanFilter() : BiasCorrectingKalmanFilter(100.0f) { }
    BiasCorrectingKalmanFilter(float stddev_jerk) : stddev_jerk(stddev_jerk), state(IDLE) {
      this->x = x.Fill(0);
      BLA::Matrix<4> certainty = {10000.0f, 0.01f, 0.01f, 10.0f};
      //Multiply(certainty, certainty, certainty);
      this->P = this->P.Fill(0);
      this->P(0,0) = certainty(0) * certainty(0);
      this->P(1,1) = certainty(1) * certainty(1);
      this->P(2,2) = certainty(2) * certainty(2);
      this->P(3,3) = certainty(3) * certainty(3);
    }

    BLA::Matrix<4> get_x() {
      return this->x;
    }

    BLA::Matrix<4,4> get_P() {
      return this->P;
    }

    
    void predict(float dt) {
      // State trans
      BLA::Matrix<4,4> Phi = {1 , dt, dt * dt * 0.5, 0,
                              0,  1, dt, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

      //noise matrix
      BLA::Matrix<4, 4> Q = {0.16666666f * dt * dt * dt * this->stddev_jerk, 0, 0, 0,
                             0, 0.5f * dt * dt * this->stddev_jerk, 0, 0,
                             0, 0, dt * this->stddev_jerk, 0,
                             0, 0, 0, dt * 0.01f};

      Q = Q * Q;
      //multiply x with Phi
      this->x = Phi * this->x;

      // P = Phi
      BLA::Matrix<4,4> Phit = ~Phi;
      this->P = Phi * this->P * Phit + Q;
      
    }


    void correct(float meas_h, float stddev_h, float meas_a, float stddev_a) {
      BLA::Matrix<2,4> H = {1, 0, 0, 0,
                            0, 0, 1, 1};
      //observation noise matrix
      BLA::Matrix<2,2> R = {stddev_h * stddev_h, 0,
                            0, stddev_a * stddev_a};

      //observation vector
      BLA::Matrix<2> y = {meas_h, meas_a};

      //error
      BLA::Matrix<2> e = y - H * x;

      //gain estimate
      BLA::Matrix<4,2> Ht = ~H;
      BLA::Matrix<2,2> S = (H * this->P * Ht + R).Inverse();
      BLA::Matrix<4,2> K = this->P * Ht * S;

      this->x = this->x + K * e;
      BLA::Matrix<4,4> id = {1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1};
      this->P = (id - K * H) * this->P;
    }

    void advance_state_machine() {
      switch(this->state) {
        case IDLE:
            break;
        case ARMED:
            if (this->get_velocity() > 10.0f + 3.0f * this->get_velocity_accuracy()) {
              this->state = ASCENT;
            }
            break;
        case ASCENT:
            if (this->get_velocity() < 0.0f - 3.0f * this->get_velocity_accuracy()) {
              this->state = DESCENT;
            }
            break;
        case DESCENT:
            break;
      }
    }

    void update(float dt, float meas_h, float stddev_h, float meas_a, float stddev_a) {
        if (this->is_past_apogee()) {
          meas_a = -meas_a;  
        }
        this->predict(dt);
        this->correct(meas_h, stddev_h, meas_a, stddev_a);
        this->advance_state_machine();
    }

    State get_state() {
      return this->state;
    }

    void arm() {
      if (this->state == IDLE) {
          this->state = ARMED;
      }
    }

        // Resets and disarms the kalman filter.
    void reset() {
        this->state = IDLE;
    }

    // Returns true if liftoff has occurred.
    bool is_launched() {
        return this->state >= ASCENT;
    }

    // Returns true if the vehicle has passed apogee.
    bool is_past_apogee() {
        return this->state == DESCENT;
    }

    // Returns the estimated height of the vehicle.
    float get_height() {
        return this->x(0);
    }

    // Returns the estimated standard deviation of the estimated height of the vehicle.
    float get_height_accuracy() {
        return sqrt(this->P(0,0));
    }
    
    // Returns the estimated velocity of the vehicle.
    float get_velocity() {
        return this->x(1);
    }

    // Returns the estimated standard deviation of the estimated velocity of the vehicle.
    float get_velocity_accuracy() {
        return sqrt(this->P(1,1));
    }

    // Returns the estimated acceleration of the vehicle.
    float get_acceleration() {
        return this->x(2);
    }

    // Returns the estimated standard deviation of the estimated acceleration of the vehicle.
    float get_acceleration_accuracy() {
        return sqrt(this->P(2,2));
    }

    float get_accel_bias() {
      return this->x(3);
    }
};


#endif
