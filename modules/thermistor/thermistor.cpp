#include "thermistor.h"

Thermistor::Thermistor(int processVariable, Pin* pin, float beta, int r0, int t0, int32_t threadFreq, volatile txData_t* txData) :
    Module(threadFreq, 1), // slow module with 1Hz update
    ptrFeedback(&txData->processVariable[processVariable]),
    adc(new AnalogIn(pin->as_input()->to_pin_name())),
    r0(r0),
    r1(0),
    r2(4700),
    j(1.0F / beta),
    k(1.0F / (t0 + 273.15F))
{
    // Take some readings to get the ADC up and running before moving on
    this->slowUpdate();
    this->slowUpdate();
    printf("Start temperature = %f\n", this->temperaturePV);
}

// This is the workhorse routine that calculates the temperature
// using the Steinhart-Hart equation for thermistors
// https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
float Thermistor::getTemperature()
{
    float adcValue = this->adc->read_u16();
    float t;

    // resistance of the thermistor in ohms
    float r = this->r2 / ((65536.0F / adcValue) - 1.0F);


    if (this->r1 > 0.0F) r = (this->r1 * r) / (this->r1 - r);

    // use Beta value
    t= (1.0F / (this->k + (this->j * logf(r / this->r0)))) - 273.15F;

    return t;
}

void Thermistor::slowUpdate()
{
    this->temperaturePV = this->getTemperature();

    // check for disconnected temperature sensor
    if (this->temperaturePV > 0)
    {
        *(this->ptrFeedback) = this->temperaturePV;
    }
    else
    {
        printf("Temperature sensor error, reading = %f\n", this->temperaturePV);
        //cout << "Temperature sensor error, pin " << this->pinSensor << " reading = " << this->temperaturePV << endl;
        *(this->ptrFeedback) = 999;
    }

}
