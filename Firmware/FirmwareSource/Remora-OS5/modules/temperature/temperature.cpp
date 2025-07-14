#include "temperature.h"

Module* createTemperature(JsonObject module, PRUThread* thread, RemoraComms* comms)
{
    int pv = module["process_variable"];
    const char* pinSensor = module["pin"];
    float beta =  module["beta"];
    int r0 = module["r0"];
    int t0 = module["t0"];

    // slow module with 1 hz update
    int updateHz = 1;
    return new Temperature(comms->ptrTxData->processVariable[pv], thread->frequency, updateHz, pinSensor, beta, r0, t0);
}

Temperature::Temperature(volatile float &ptrFeedback, int32_t threadFreq, int32_t slowUpdateFreq, std::string pinSensor, float beta, int r0, int t0) :
  Module(threadFreq, slowUpdateFreq),
  ptrFeedback(&ptrFeedback),
  pinSensor(pinSensor),
    beta(beta),
    r0(r0),
    t0(t0)
{
    this->Sensor = new Thermistor(this->pinSensor, this->beta, this->r0, this->t0);
 
    // Take some readings to get the ADC up and running before moving on
    this->slowUpdate();
    this->slowUpdate();
    printf("Start temperature = %f\n", this->temperaturePV);
}

void Temperature::update()
{
  return;
}

void Temperature::slowUpdate()
{
    this->temperaturePV = this->Sensor->getTemperature();

    // check for disconnected temperature sensor
    if (this->temperaturePV > 0)
    {
        *(this->ptrFeedback) = this->temperaturePV;
    }
    else
    {
        printf("Temperature sensor error, pin %s reading = %f\n", this->pinSensor.c_str(), this->temperaturePV);
        //cout << "Temperature sensor error, pin " << this->pinSensor << " reading = " << this->temperaturePV << endl;
        *(this->ptrFeedback) = 999;
    }

}
