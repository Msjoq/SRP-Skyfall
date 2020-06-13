
#include "Arduino.h"
#include <math.h>
#include "ISA.h"

ISA::ISA() {
  base_pressure = 101325.0;
  base_temperature = 288.15;
}

void ISA::calibrate(float pressure, float temperature) {
  base_pressure = pressure;
  base_temperature = temperature + 273.15;
}

float ISA::height(float pressure) const {
  const float gamma = 1.4f;
  const float dTdh = -0.0065f;
  const float g = 9.81f;
  const float Rspecific = 287.058f;

  // fraction = (P/P0)**(-R*dTdh/g)
  float fraction = pow(pressure / this->base_pressure, -(Rspecific * dTdh / g));
            // height = (fraction - 1) * (T0 / dTdh)
  return (fraction - 1.f) * (this->base_temperature / dTdh);
  
}
