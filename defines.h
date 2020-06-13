#ifndef DEFINE_H
#define DEFINE_H


// I2C addresses
#define LIS 0x19
#define MPU 0x68
// Arm switch pin
#define ARM 17


//state enum
typedef enum {
  ERR = 0,
  ARMED = 1,
  ASCENT = 2,
  DESCENT = 3,
  DEPLOY = 4,
  LANDED = 5,
  IDLE = 6
} State;


// struct containing all current values
struct Values {
  float meas_h;
  float meas_a;
  float high_g;
  float kal_h;
  float kal_v;
  float kal_a;
  State state;
  uint16_t dt;
};


#endif /* DEFINE_H */
