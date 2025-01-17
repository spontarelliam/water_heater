
// Reprogramming of SETS water heater

#include <SPI.h>
//#include <SD.h>
#include "water_heater.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include "Adafruit_MAX31855.h"

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   4
#define MAXCS   7
#define MAXCLK  8

#define ONE_WIRE_BUS 4
#define MAXTEMP 60 // 135F

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

//double TEMPSETPOINT = 41.1; // 41=106F
double TEMPSETPOINT = 49; // winter target
double Tin;
double Tout2; // thermocouple
unsigned long lastTime;
double Q;
double PID_adj = 1.0;
float last_loop = 0;

//Specify the links and initial tuning parameters

double Kp=10, Ki=5, Kd=1; // not sure why this is good
//double Kp=.2, Ki=0.1, Kd=200; // used for months
//double Kp=.1, Ki=0.1, Kd=500; // also good


PID myPID(&Tout2, &PID_adj, &TEMPSETPOINT, Kp, Ki, Kd, DIRECT);

Heater heater_one;
Heater heater_two;
Heater heater_three;
Heater heater_four;


void setup(void) {
  Serial.begin(9600);

  heater_one.set_pin(3);
  heater_two.set_pin(5);
  heater_three.set_pin(6);
  heater_four.set_pin(9);

    delay(300);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }

  sensors.begin();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0.01, 2);
  
}


long ClassicalMethod(float T1, float T2){
    // Calculate Q in watts

    // Measure inlet flow rate, set Q
    // Q = m x cp x (T2 - T1)
    // 0.05 kg/s = 1 gpm
    float cp = 4180; // J / kg * K
    
    float q = 0.07 * cp * (T2 - T1);
    q = max(0, q);
    q = min(q, 20000);

/*
    Serial.print("q = ");
    Serial.print(q);
    Serial.print(", Tin = ");
    Serial.print(T1);
    Serial.print(", Tsetpt = ");
    Serial.print(TEMPSETPOINT);
    Serial.print(", PID_adj = ");
    Serial.print(PID_adj);
    Serial.print(", therm code: ");
    Serial.println(thermocouple.readError());
    */
    return q;
}


bool safety_check(float Tout){
  if (Tout >MAXTEMP){
    
    Serial.println("WARNING: Temp above limit, powering off");
    Serial.println(Tout);
    Serial.println(MAXTEMP);
    heater_one.set_power(0);
    heater_two.set_power(0);
    heater_three.set_power(0);
    heater_four.set_power(0);
    delay(3000);
    // Inhibit correction factor further
    return false;
  }
  return true;
}


void loop(void) {

  if (millis() - last_loop > 200){ // max31855 sampling frequency of 10hz max
      last_loop = millis();

  // Take measurements
  //Tout = exit_thermistor.get_temperature();
  Tin = 5; // guestimate in C
  Tout2 = thermocouple.readCelsius();
  Q = ClassicalMethod(Tin, TEMPSETPOINT);
  myPID.Compute();
  Q = Q * PID_adj;

  // power boost for first 3 sec
  if (millis() < 3000){
    Q = min(Q * 2.0, 20000);
  }

  // -- Debug printout --
  Serial.print("Time (ms) = ");
  Serial.print(millis());
  Serial.print(", Total Q (W)= ");
  Serial.print(Q);
  Serial.print(" , Tin = ");
  Serial.print(Tin);
  Serial.print(" , Tout = ");
  Serial.print(Tout2);
  Serial.print(" , Tsetpt = ");
  Serial.print(TEMPSETPOINT);
  Serial.print(" , PID_adj = ");
  Serial.println(PID_adj);

  // ---------------------

  if (safety_check(Tout2) == true){
        heater_one.set_power(Q/4);
        heater_two.set_power(Q/4);
        heater_three.set_power(Q/4);
        heater_four.set_power(Q/4);
  }

/*
  // Report heater power
  Serial.print("Heater power (W): ");
  Serial.print(heater_one.get_power());
  Serial.print(", ");
  Serial.print(heater_two.get_power());
  Serial.print(", ");
  Serial.print(heater_three.get_power());
  Serial.print(", ");
  Serial.println(heater_four.get_power());
  */
    
  }

}
