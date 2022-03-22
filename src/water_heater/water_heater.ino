// Reprogramming of SETS water heater

#include <SPI.h>
#include <SD.h>
#include "water_heater.h"

#define MAXTEMP 140
#define TEMPSETPOINT 100

float Tout;
unsigned long lastTime;
double errSum, lastErr;
double kp, ki, kd;
File myFile;
double pid_output;

Heater heater_one;
Heater heater_two;
Heater heater_three;
Heater heater_four;

Thermistor thermistor;
FlowMeter flowmeter;


void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  SetTunings(850, 0.1, 0.1);

  heater_one.set_pin(3);
  heater_two.set_pin(5);
  heater_three.set_pin(6);
  heater_four.set_pin(9);
  thermistor.set_pin(A0);
  attachInterrupt(0, flow, RISING); // Setup Interrupt

  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  myFile = SD.open("temps.log", FILE_WRITE);
  if (myFile) {
    myFile.println("");
    myFile.println("-------  New  ---------");
    myFile.close();
  }

}

void flow(){ // helper function for interrupt attachment
  flowmeter.flow();
}

double Compute()
{
  double Output;
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);

   /*Compute all the working error variables*/
   double error = TEMPSETPOINT - Tout;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;

   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;

   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;

   Serial.print(Tout);
   Serial.print(", ");
   Serial.print(error);
   Serial.print(", ");
   Serial.print(errSum);
   Serial.print(", ");
   Serial.print(dErr);
   Serial.print(", ");
   Serial.println(Output);
   return Output;
}


void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}


void ClassicalMethod(float mdot, float T2){
    // Measure inlet flow rate, set Q
    // Q = m x cp x (T2 - T1)
    float cp = 4.18; // kJ / kg * K
    float T1 = 13; // Celsius.  no measurement currently, assuming temp
    float Q = mdot * cp * (T2 - T1);
    return Q;
}

void adjustment(float T2, float TEMPSETPOINT){
    float diff = T2 - TEMPSETPOINT;
    float adj_factor = diff / 1000;
    return adj_factor;
}

void loop(void) {

  Tout = thermistor.get_temperature();

  heater_one.append_temp(Tout);
  heater_two.append_temp(Tout);
  heater_three.append_temp(Tout);
  heater_four.append_temp(Tout);

  /* pid_output = Compute(); // compute pid demand */
  float Q = ClassicalMethod(flowmeter.flow_rate, TEMPSETPOINT);
  Serial.print(Q);
  Serial.print(" watts, Actual Tout = ");
  Serial.println(Tout);



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
  /* if (thermistor.temp > MAXTEMP){ */
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
  /*   myFile.print(thermistor.temp); */
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
  /*   Serial.print(thermistor.temp); */
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

  delay(300);
}
