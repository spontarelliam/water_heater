
// Reprogramming of SETS water heater

#include <SPI.h>
//#include <SD.h>
#include "water_heater.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include "Adafruit_MAX31855.h"

// Default connection is using software SPI, but comment and uncomment one of
// the two examples below to switch between software SPI and hardware SPI:

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   4
#define MAXCS   7
#define MAXCLK  8



#define ONE_WIRE_BUS 4
#define MAXTEMP 135
#define FUDGEFACTOR 0.3

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

double TEMPSETPOINT = 106; 
double Tin;
double Tout; // thermistor
double Tout2; // thermocouple
unsigned long lastTime;
double Q;
double PID_adj = 1.0;
float last_loop = 0;


//Specify the links and initial tuning parameters
//double Kp=30, Ki=1.0, Kd=40; // thermistor
//double Kp=5, Ki=0.1, Kd=30;
double Kp=1, Ki=0.1, Kd=200;
PID myPID(&Tout2, &PID_adj, &TEMPSETPOINT, Kp, Ki, Kd, DIRECT);

Heater heater_one;
Heater heater_two;
Heater heater_three;
Heater heater_four;

Thermistor exit_thermistor;
FlowMeter flowmeter;


void setup(void) {
  Serial.begin(9600);
  
/*
  TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pins 3 and 11
  TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz on pins 5 and 6
  TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pins 9 and 10
*/
  heater_one.set_pin(3);
  heater_two.set_pin(5);
  heater_three.set_pin(6);
  heater_four.set_pin(9);
  exit_thermistor.set_pin(A0);
  flowmeter.set_pin(2);
  attachInterrupt(0, flow, RISING); // Setup Interrupt attach to flow function


    delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }

  sensors.begin();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0.1, 2.0);
  
}

void flow(){ // helper function for interrupt attachment
  flowmeter.flow();
}


long ClassicalMethod(float mdot, float T1, float T2){
    // Calculate Q in watts
    T1 = (T1 - 32) / 1.8; // Celsius
    T2 = (T2 - 32) / 1.8; // Celsius
    // Measure inlet flow rate, set Q
    // Q = m x cp x (T2 - T1)
    float cp = 4180; // J / kg * K
    
    float q = PID_adj * mdot * cp * (T2 - T1);

    Serial.print("q = ");
    Serial.print(q);
    Serial.print(", PID_adj = ");
    Serial.print(PID_adj);
    Serial.print(", therm code: ");
    Serial.print(thermocouple.readError());
    Serial.print(", mdot = ");
    Serial.print(mdot);
    Serial.print(", T1 = ");
    Serial.print(T1);
    Serial.print(", Tsetpt = ");
    Serial.println(T2);
    return q;
}


bool safety_check(float Tout){
  if (Tout >MAXTEMP){
    Serial.println("WARNING: Temp above limit, powering off");
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

  if (millis() - last_loop > 150){ // max31855 sampling frequency of 10hz max
      last_loop = millis();


  // Take measurements
  //Tout = exit_thermistor.get_temperature();
  Tin = 50; // guestimate in F
  Tout2 = thermocouple.readFahrenheit();

  flowmeter.calc_flow_rate();

 
  Q = ClassicalMethod(flowmeter.flow_rate, Tin, TEMPSETPOINT);
  
  if (flowmeter.flow_rate > 0.001){
    myPID.Compute();
  }
  Serial.print("PID values = ");
    Serial.print(Kp);
    Serial.print(" ");
    Serial.print(Ki);
    Serial.print(" ");
    Serial.println(Kd);

  // -- Debug printout --
  Serial.print("Time (ms) = ");
  Serial.print(millis());
  Serial.print(", Total Q = ");
  Serial.print(Q);
  Serial.print(", Flow = ");
  Serial.print(flowmeter.flow_rate,4);
  Serial.print(" , Tin = ");
  Serial.print(Tin);
  Serial.print(" , Tout = ");
  Serial.println(Tout2);

  Serial.print("Thermocouple read error: ");
  Serial.println(thermocouple.readError());
  // ---------------------



  if (safety_check(Tout2) == true){
    if (flowmeter.flow_rate < 0.001){
        heater_one.set_power(0);
        heater_two.set_power(0);
        heater_three.set_power(0);
        heater_four.set_power(0);
    }
    else{
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



    
  }

}
