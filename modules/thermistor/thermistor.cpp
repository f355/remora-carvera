#include "thermistor.h"

Thermistor::Thermistor(const int var_number, Pin* pin, const float beta, const int r0, const int t0,
                       const uint32_t thread_freq, volatile txData_t* tx_data)
    : Module(thread_freq, 1),  // slow module with 1Hz update
      output_var(&tx_data->vars[var_number]),
      adc(new AnalogIn(pin->as_input()->to_pin_name())),
      r0(r0),
      r1(0),
      r2(4700),
      j(1.0F / beta),
      k(1.0F / (t0 + 273.15F)) {
  // Take some readings to get the ADC up and running before moving on
  this->slow_update();
  this->slow_update();
}

// This is the workhorse routine that calculates the temperature
// using the Steinhart-Hart equation for thermistors
// https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
float Thermistor::getTemperature() const {
  const float adcValue = this->adc->read_u16();

  // resistance of the thermistor in ohms
  float r = this->r2 / ((65536.0F / adcValue) - 1.0F);

  if (this->r1 > 0.0F) {
    r = (this->r1 * r) / (this->r1 - r);
  }

  // use Beta value
  return 1.0F / (this->k + this->j * logf(r / this->r0)) - 273.15F;
}

void Thermistor::slow_update() {
  float temperaturePV = this->getTemperature();

  if (temperaturePV <= 0) {
    printf("Temperature sensor error, reading = %f\n", temperaturePV);
    temperaturePV = 999999;
  }

  *this->output_var = temperaturePV * 100;
}
