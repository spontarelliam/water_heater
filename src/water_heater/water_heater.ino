// Reprogramming of SETS water heater

#include <SPI.h>
//#include <SD.h>
#include "water_heater.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4
#define MAXTEMP 140
#define TEMPSETPOINT 100

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
    float Q = mdot * cp * (T2 - T1); 
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



  /*  if (pid_output >= 100000){ // All on */
  /*      int  demand = map(pid_output, 100000, 200000, 0, 255); */
  /*      heater_one.set_power(demand); */
  /*      heater_two.set_power(demand); */
  /*      heater_three.set_power(demand); */
  /*      heater_four.set_power(demand); */
  /*  } */
  /*  else if (pid_output < 100000 && pid_output >= 50000){ */
  /*      int  demand = map(pid_output, 50000, 100000, 0, 255); */
  /*      heater_one.set_power(demand); */
  /*      heater_two.set_power(demand); */
  /*      heater_three.set_power(demand); */
  /*      heater_four.set_power(0); */
  /*  } */
  /*   else if (pid_output < 50000 && pid_output >= 25000){ */
  /*      int  demand = map(pid_output, 25000, 50000, 0, 255); */
  /*      heater_one.set_power(demand); */
  /*      heater_two.set_power(demand); */
  /*      heater_three.set_power(0); */
  /*      heater_four.set_power(0); */
  /*  } */
  /*  else if (pid_output < 25000 && pid_output >= 1000){ */
  /*      int  demand = map(pid_output, 1000, 25000, 0, 255); */
  /*      heater_one.set_power(demand); */
  /*      heater_two.set_power(0); */
  /*      heater_three.set_power(0); */
  /*      heater_four.set_power(0); */
  /*  } */
  /*  else if (pid_output < 1000 && pid_output >= 0){ */
  /*    heater_one.set_power(0); */
  /*    heater_two.set_power(0); */
  /*    heater_three.set_power(0); */
  /*    heater_four.set_power(0); */
  /*  } */
  /*  else{ */
  /*      Serial.println("OVERHEATED: All off"); */
  /*      heater_one.set_power(0); */
  /*      heater_two.set_power(0); */
  /*      heater_three.set_power(0); */
  /*      heater_four.set_power(0); */
  /*  } */


  /* if (heater_one.get_time_on() >= 3000 and heater_one.rate < 1){ */
  /*   Serial.println("WARNING heat on but Tout not increasing"); */

  /*   heater_one.set_power(0); */
  /*   heater_two.set_power(0); */
  /*   heater_three.set_power(0); */
  /*   heater_four.set_power(0); */

  /*   delay(10000); */
  /* } */
  /* if (exit_thermistor.temp > MAXTEMP){ */
  /*   Serial.println("WARNING temperature has exceeded maximum"); */
  /*   heater_one.set_power(0); */
  /*   heater_two.set_power(0); */
  /*   heater_three.set_power(0); */
  /*   heater_four.set_power(0); */
  /*   delay(10000); */
  /* } */
  /* // if no demand, reset PID errors */
  /* if (heater_one.power == 0){ */
  /*   errSum = 0; */
  /* } */



  /* myFile = SD.open("temps.log", FILE_WRITE); */
  /* if (myFile) { */
  /*   myFile.print(millis()); */
  /*   myFile.print(", "); */
  /*   myFile.print(pid_output); */
  /*   myFile.print(", "); */
  /*   myFile.print(exit_thermistor.temp); */
  /*   myFile.print(", "); */
  /*   myFile.print(heater_one.power); */
  /*   myFile.print(", "); */
  /*   myFile.print(heater_two.power); */
  /*   myFile.print(", "); */
  /*   myFile.print(heater_three.power); */
  /*   myFile.print(", "); */
  /*   myFile.println(heater_four.power); */
  /*   myFile.close(); */
  /* } */

  /*   Serial.print(pid_output); */
  /*   Serial.print(", F="); */
  /*   Serial.print(exit_thermistor.temp); */
  /*   Serial.print(", "); */
  /*   myFile.print(heater_one.power); */
  /*   myFile.print(", "); */
  /*   myFile.print(heater_two.power); */
  /*   myFile.print(", "); */
  /*   myFile.print(heater_three.power); */
  /*   myFile.print(", "); */
  /*   myFile.print(heater_four.power); */
  /*   Serial.print(", "); */
  /*   myFile.print(heater_one.get_time_on()); */
  /*   myFile.print(", "); */
  /*   myFile.print(heater_two.get_time_on()); */
  /*   myFile.print(", "); */
  /*   myFile.print(heater_three.get_time_on()); */
  /*   myFile.print(", "); */
  /*   myFile.println(heater_four.get_time_on()); */

  delay(100);
}
