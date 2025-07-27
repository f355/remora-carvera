#include "thermistor.h"

Module *createThermistor(JsonObject module, PRUThread *thread, Comms *comms) {
  int pv = module["process_variable"];
  const char *pinSensor = module["pin"];
  float beta = module["beta"];
  int r0 = module["r0"];
  int t0 = module["t0"];

  // slow module with 1 hz update
  int updateHz = 1;
  return new Thermistor(comms->ptr_tx_data->process_variable[pv], thread->frequency, updateHz, pinSensor, beta, r0, t0);
}

Thermistor::Thermistor(volatile float &ptr_feedback, int32_t thread_freq, const int32_t slow_update_freq,
                       std::string pin_sensor, const float beta, const int r0, const int t0)
    : Module(thread_freq, slow_update_freq), ptr_feedback(&ptr_feedback), r0(r0) {
  // Thermistor math
  this->j = (1.0F / beta);
  this->k = (1.0F / (t0 + 273.15F));

  this->adc = new AnalogIn((new Pin(pin_sensor))->as_input()->to_pin_name());
  this->r1 = 0;
  this->r2 = 4700;

  // Take some readings to get the ADC up and running before moving on
  this->slow_update();
  this->slow_update();
}

// This is the workhorse routine that calculates the temperature
// using the Steinhart-Hart equation for thermistors
// https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
float Thermistor::get_temperature() const {
  const float adcValue = this->adc->read_u16();

  // resistance of the thermistor in ohms
  float r = this->r2 / ((65536.0F / adcValue) - 1.0F);

  if (this->r1 > 0.0F) r = (this->r1 * r) / (this->r1 - r);

  // use Beta value
  return (1.0F / (this->k + (this->j * logf(r / this->r0)))) - 273.15F;
}

void Thermistor::slow_update() {
  // check for disconnected temperature sensor
  if (float temperature_pv = this->get_temperature(); temperature_pv > 0) {
    *this->ptr_feedback = temperature_pv;
  } else {
    printf("Temperature sensor error, reading = %f\n", temperature_pv);
    // cout << "Temperature sensor error, pin " << this->pinSensor << " reading
    // = " << this->temperaturePV << endl;
    *this->ptr_feedback = 999;
  }
}
