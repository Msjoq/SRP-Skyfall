#include "ISA.h"
#include "defines.h"


// getting relevant variables from main sketch
extern BMP280_DEV bmp;
extern float pressure;
extern unsigned long programtime;
extern Values values;
extern State state;
extern ISA atmo;
extern MPU6050 mpu;
extern LIS331 x1;
extern BiasCorrectingKalmanFilter kal;


// initialises Pressuresensor to forced mode, quickest settings
void initBMP() {
    bmp.begin(FORCED_MODE, BMP280_I2C_ALT_ADDR);
    bmp.setPresOversampling(OVERSAMPLING_X1);  // Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16
    bmp.setTempOversampling(OVERSAMPLING_X1);  // Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16
    bmp.setIIRFilter(IIR_FILTER_OFF);  // Options are IIR_FILTER_OFF, _2, _4, _8, _16
}

// initializes accelerometer
void initMPU() {
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
}


// initializes high G accelerometer
void initLIS() {
  x1.setI2CAddr(LIS);
  x1.begin(LIS331::USE_I2C);
  x1.setODR(LIS331::DR_100HZ);
  x1.setFullScale(LIS331::LOW_RANGE);
  
}


// calibrate ISA to be 0m for average of past pressures
void calibrate() {
  Serial.println("Calibrating...");
  float tem;
  float init_temp = 0.0f;
  float init_pres = 0.0f;
  float init_samples = 30.0f;
  for (int i = 0; i < init_samples; i = i) {
    bmp.startForcedConversion();
    if (bmp.getTempPres(tem, pressure)) {
      //Serial.println(tem);
      init_temp = init_temp + tem / init_samples;
      init_pres = init_pres + pressure / init_samples;
      i++;
    }
  };
  atmo.calibrate(init_pres, init_temp);
  Serial.print("Calibrated pres: ");
  Serial.print(init_pres);
  Serial.print(" temp: ");
  Serial.println(init_temp);
}


// get latest values from sensors and write them to values struct
void update_vals() {
  
      float meas_h = atmo.height(pressure);
      float dt = (millis() - programtime) / 1000.0f;
      programtime = millis();
      float mpuz = (mpu.getAccelerationZ()) / -208.7665647f;
      int16_t highg;
      x1.readAxes(highg);

      // eye balled offset of sensor, not accurate 
      values.high_g = (x1.convertToG(100, highg) - 1.1) * 9.81;
  
      kal.update(dt, meas_h, 3.0f, mpuz, 0.08f);
      values.dt = (uint16_t) (dt * 1000);
      values.meas_h = meas_h;
      values.meas_a = mpuz;
      values.kal_h = kal.get_height();
      values.kal_v = kal.get_velocity();
      values.kal_a = kal.get_acceleration();
      values.state = state;
}

// converts floats to half precision floats, half as much data to send, thus we can send more values
uint16_t floattohalf(float f) 
{
    uint32_t t = *(uint32_t*)&f;
    // man bits = 10; but we keep 11 for rounding
    uint16_t man = (t & 0x007FFFFF) >> 12;
    int16_t  exp = (t & 0x7F800000) >> 23;
    bool     sgn = (t & 0x80000000);

    // handle 0
    if ((t & 0x7FFFFFFF) == 0)
    {
        return sgn?0x8000:0x0000;
    }
    // denormalized float32 does not fit in float16
    if (exp == 0x00)
    {
        return sgn?0x8000:0x0000;
    }
    // handle inf & NAN
    if (exp == 0x00FF)
    {
        if (man) return 0xFE00; // NAN
        return sgn?0xFC00:0x7C00; // -INF : INF
    }
    // normal numbers
    exp = exp - 127 + 15;
    if (exp > 30) // overflow does not fit => INF
    {
        return sgn?0xFC00:0x7C00;
    }
    if (exp < -38) // subnormal not possible => zero
    {
        return sgn?0x8000:0x0000;
    }
    if (exp <= 0) // subnormal
    {
        man >>= (exp+14);
        // rounding
        man++;
        man >>= 1;
        return sgn?0x8000:0x0000 | man;
    }

    exp <<= 10;
    man++;
    man >>= 1;
    return (sgn?0x8000:0x0000) | (exp | man);
}


// reverses floattohalf, just used for testing earlier, will only be implemented on receiver
//float halftofloat(uint16_t n)
//{
//    uint16_t sgn, man;
//    int exp;
//    double f;
//
//    sgn = (n & 0x8000) > 0;
//    exp = (n & 0x7C00) >> 10;
//    man = (n & 0x03FF);
//
//    // ZERO
//    if ((n & 0x7FFF) == 0)
//    {
//        return sgn?-0:0;
//    }
//    // NAN & INF
//    if (exp == 0x001F)
//    {
//        if (man == 0) return sgn?-INFINITY:INFINITY;
//        else return NAN;
//    }
//    // SUBNORMAL/NORMAL
//    if (exp == 0)  f = 0;
//    else           f = 1;
//    // PROCESS MANTISSE
//    for (int i=9; i>=0; i--)
//    {
//        f *= 2;
//        if (man & (1<<i)) f = f + 1;
//    }
//    f = f * pow(2.0, exp-25);
//    if (exp == 0)
//    {
//        f = f * pow(2.0, -13); // 5.96046447754e-8;
//    }
//    return sgn?-f:f;
//}



// transmit all values to Pi/Ground station
void transmit(Values vals) {

    // convert everything to half precision float saving bandwidth
    uint16_t half_meas_h = floattohalf(vals.meas_h);
    uint16_t half_meas_a = floattohalf(vals.meas_a);
    uint16_t half_high_g = floattohalf(vals.high_g);
    uint16_t half_kal_h = floattohalf(vals.kal_h);
    uint16_t half_kal_v = floattohalf(vals.kal_v);
    uint16_t half_kal_a = floattohalf(vals.kal_a);

    //byte containing state and time between readings.
    //first 3 bits: state
    // last 5 bits: time between readings
    byte statebyte = (vals.state << 5);
    statebyte = statebyte | vals.dt;
    
    byte buf[13] = {highByte(half_meas_h), lowByte(half_meas_h), highByte(half_meas_a), lowByte(half_meas_a),
                  highByte(half_high_g), lowByte(half_high_g), highByte(half_kal_h), lowByte(half_kal_h),
                  highByte(half_kal_v), lowByte(half_kal_v), highByte(half_kal_a), lowByte(half_kal_a),
                  statebyte};
    Serial.write(buf, 13);

    //new line is delimiter between messages
    Serial.println();

    //total bitrate: ~40Hz * 15bytes * 8 bits/byte = 4.8kbps
}
