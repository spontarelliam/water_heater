// Reprogramming of SETS water heater

#include <SPI.h>
//#include <SD.h>
#include "water_heater.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

#define ONE_WIRE_BUS 4
#define MAXTEMP 140
#define TEMPSETPOINT 95 // 35C = 95F
#define FUDGEFACTOR 0.3

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

float Tout; // thermistor
float Tin;
float Tout2; // digital temp sensor
unsigned long lastTime;
float Q;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Tout, &Q, &TEMPSETPOINT, Kp, Ki, Kd, DIRECT);

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
    float Cf = max(1 - (mdot / 0.05), 0.2);
    Cf = min(Cf, 0.7);
    Cf = 0.4;
    float q = Cf * mdot * cp * (T2 - T1);
    //Serial.println(Cf);
    return q;
}

void PID_method(){
input = Tout
    setpoint = 35
    output = power
}


bool safety_check(float Tout){
  float Tout_F = Tout * 1.8 + 32;
  if (Tout_F >MAXTEMP){
    Serial.println("WARNING: Temp above limit, powering off");
    heater_one.set_power(0);
    heater_two.set_power(0);
    heater_three.set_power(0);
    heater_four.set_power(0);
    delay(2000);
    return false;
  }
  return true;
}


void loop(void) {

  // Take measurements
  Tout = exit_thermistor.get_temperature();
  sensors.requestTemperatures(); // Send the command to get temperatures
  Tin = sensors.getTempCByIndex(0);

  flowmeter.calc_flow_rate();

  float T_setpt_C = (TEMPSETPOINT - 32) / 1.8;
  /* Q = ClassicalMethod(flowmeter.flow_rate, Tin, T_setpt_C); */
  myPID.Compute();


  // -- Debug printout --
  Serial.print("Power = ");
  Serial.print(Q);
  Serial.print(", Flow = ");
  Serial.print(flowmeter.flow_rate,4);
  Serial.print(" , Tin = ");
  Serial.print(Tin);
  Serial.print(" , Actual Tout = ");
  Serial.println(Tout);
  // ---------------------


  if (safety_check(Tout) == true){
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
  }



  // Report heater power
  Serial.print("Heater power: ");
  Serial.print(heater_one.get_power());
  Serial.print(", ");
  Serial.println(heater_two.get_power());





  delay(100);
}
