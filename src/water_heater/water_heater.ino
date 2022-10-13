// Reprogramming of SETS water heater

#include <SPI.h>
//#include <SD.h>
#include "water_heater.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4
#define MAXTEMP 140
#define TEMPSETPOINT 95
#define FUDGEFACTOR 0.6

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

float Tout;
float Tin;
float Tout2;
unsigned long lastTime;
double errSum, lastErr;
double kp, ki, kd;
double pid_output;

int demand;

Heater heater_one;
Heater heater_two;
Heater heater_three;
Heater heater_four;

Thermistor exit_thermistor;
Thermistor inlet_thermistor;
FlowMeter flowmeter;


void setup(void) {
  Serial.begin(9600);

  TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pins 3 and 11
  TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz on pins 5 and 6
  TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pins 9 and 10

  heater_one.set_pin(3);
  heater_two.set_pin(5);
  heater_three.set_pin(6);
  heater_four.set_pin(9);
  exit_thermistor.set_pin(A0);
  flowmeter.set_pin(2);
  attachInterrupt(0, flow, RISING); // Setup Interrupt attach to flow function
  sensors.begin();
}

void flow(){ // helper function for interrupt attachment
  flowmeter.flow();
}

int ClassicalMethod(float mdot, float T1, float T2){
    // Measure inlet flow rate, set Q
    // Q = m x cp x (T2 - T1)
    float cp = 4180; // J / kg * K
    float Q = FUDGEFACTOR * mdot * cp * (T2 - T1); 
    return Q;
}

void safety_check(float Tout){
  float Tout_F = Tout * 1.8 + 32;
  if (Tout_F >MAXTEMP){
    Serial.println("WARNING: Temp above limit, powering off");
    heater_one.set_power(0);
    heater_two.set_power(0);
    heater_three.set_power(0);
    heater_four.set_power(0);
    delay(2000);
  }
}

void loop(void) {

  // Take measurements
  Tout = exit_thermistor.get_temperature();
  sensors.requestTemperatures(); // Send the command to get temperatures
  Tin = sensors.getTempCByIndex(0);
  
  /* pid_output = Compute(); // compute pid demand */
  flowmeter.calc_flow_rate();

  float T_setpt_C = (TEMPSETPOINT - 32) / 1.8;
  float Q = ClassicalMethod(flowmeter.flow_rate, Tin, T_setpt_C);


  // -- Debug printout --
  Serial.print("Power = ");
  Serial.print(Q);
  Serial.print(", Flow = ");
  Serial.print(flowmeter.flow_rate);
  Serial.print(" , Tin = ");
  Serial.print(Tin);
  Serial.print(" , Actual Tout = ");
  Serial.println(Tout);
  // ---------------------

  safety_check(Tout);

  if (Q<=50){
    heater_one.set_power(0);
      heater_two.set_power(0);
      heater_three.set_power(0);
      heater_four.set_power(0);
  }
  else if (Q>50){
      heater_one.set_power(Q/4);
      heater_two.set_power(Q/4);
      heater_three.set_power(Q/4);
      heater_four.set_power(Q/4);
  }

/*
  // Report heater power
  Serial.print("Heater power: ");
  Serial.print(heater_one.get_power());
  Serial.print(", ");
  Serial.println(heater_two.get_power());
*/




  delay(100);
}
