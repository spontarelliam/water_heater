class FlowMeter{
public:
    int PIN;
    float flow_rate = 0; // kg/s
    float l_hour;
    volatile int flow_frequency; // Measures flow sensor pulsesunsigned
    unsigned long currentTime;
    unsigned long cloopTime = millis();

    void set_pin(int pin){
        PIN = pin;
        pinMode(pin, INPUT_PULLUP);
    }

    void flow(){ // Interrupt function
        flow_frequency++;
    }

    void calc_flow_rate(){
        currentTime = millis();

        if(currentTime >= (cloopTime + 1000))
        {
            cloopTime = currentTime; // Updates cloopTime
            // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
            l_hour = (flow_frequency * 60 / 4.65); // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour
            flow_frequency = 0; // Reset Counter
            //Serial.print(l_hour, DEC); // Print litres/hour
            //Serial.println(" L/hour");
            int rho = 997; // kg/m3  1L = .001m3
            flow_rate = l_hour / 1000 * rho / 3600; // kg/s
        }
    }
};


class Heater {       // The class
  public:             // Access specifier
    int PIN;
    bool on = false;
    float power; // in watts
    unsigned int time_on = 0;
    unsigned int start_time;
    float temp_history[10] = {0,0,0,0,0,0,0,0,0,0};
    float rate=1;

    void set_pin(int pin){
      PIN = pin;
      pinMode(PIN, OUTPUT);
    }

    void set_power(int watts){
        // set power in Watts
      power = watts;
      int setpoint = map(watts, 0, 5500, 0, 255);
      setpoint = min(setpoint, 255);
      //Serial.println(setpoint);
      analogWrite(PIN, setpoint);


      if (power > 0){
        // heater just turned on from off
	if (on == false){
	  start_time = millis();
	}
	on = true;
      }
      else{
	on = false;
      }
    }

    void get_state(){
      delay(1);
    }

  int get_time_on(){
    if (on == true){
      time_on = millis() - start_time;
    }
    else{
      time_on = 0;
    }
    return time_on;
  }

  void append_temp(float temp){
    for (int i = 0; i<=8; i++){
      temp_history[i] = temp_history[i+1];
      //Serial.println(T_history[i]);
    }
    temp_history[9] = temp;
  }

  int get_power(){
    return power;
  }
};




class Thermistor{
  public:
  int PIN;
  long THERMISTORNOMINAL = 120000;
  int TEMPERATURENOMINAL = 25;
  int BCOEFFICIENT = 3000; // The beta coefficient of the thermistor (usually 3000-4000)
  long SERIESRESISTOR = 100000;
  float temp;

  void set_pin(int pin){
    PIN = pin;
  }

  float get_temperature(){
    int NUMSAMPLES = 5;
    int samples[NUMSAMPLES];
    float average = 0;
    float steinhart;
    // take N samples in a row, with a slight delay
    for (int i=0; i< NUMSAMPLES; i++) {
      //samples[i] = 1023-analogRead(PIN);
      samples[i] = analogRead(PIN);
      delay(10);
    }
    // average all the samples out
    for (int i=0; i< NUMSAMPLES; i++) {
      average += samples[i];
    }
    average /= NUMSAMPLES;

    // convert the value to resistance
    average = 1023 / average - 1;
    average = SERIESRESISTOR / average;


    steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert absolute temp to C

    float tempc = steinhart;
    float tempf = steinhart * 1.8 + 32;
    return tempf;
  }
};
