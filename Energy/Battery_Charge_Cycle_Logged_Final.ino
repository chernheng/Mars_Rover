/*
Authors: Praveen THARMARAJAN, TAN Chern Heng, Nitu BARUA
*/

#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>
#include <SD.h>

#define ESP32 0 // set to 1 if Energy ever connects to ESP32
INA219_WE ina219; // this is the instantiation of the library for the current sensor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = 10;
unsigned int rest_timer;
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float ui_max = 1, ui_min = 0; //anti-windup limitation
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float Ts = 0.001; //1 kHz control frequency.
float current_ref = 0, error_amps; // Current Control
float current_measure;
float pwm_out = 120; //initialisation of duty cycle
float V_Bat;
float V_Bat1, V_Bat2, V_Bat3; // voltage of the 3 batteries configured in parallel
float SOC1,SOC2,SOC3;
boolean input_switch;
int state_num=0,next_state;
String dataString;
float voltage_pres = 0;
float current_pres = 0;
float power_pres = 0;
float power_prev = 0;
float voltage_prev = 0;
float delta = 0.001;
float Vol_Discharging [] = {3165.5,3161.5,3157.5,3149.6,3145.6,3141.6,3137.6,3125.7,3113.8,3097.9,3085.9,3062.1,3034.2,2998.4};
float Vol_Charging [] = {3547.2,3543.2,3539.3,3527.3,3523.4,3519.4,3507.5,3499.5,3487.6,3479.6,3463.7,3443.8,3423.9,3396.1};
int Battery_lv []=  {80,75,70,65,60,55,50,45,40,35,30,25,20,15};
float average_SOC, present_capacity,low_battery_capacity;
float range; //range that rover can travel in cm
float discharge_rate = 330; //discharge rate of battery in mA
int low_speed=6,medium_speed=10,high_speed=14; //speed in cm/s
float rover_speed = low_speed;
float remaining_range;
float SOH;
float discharge_time;
unsigned long lastTime = 0;
unsigned long sendingDelay = 1000;
char* fromESP[2] = {"0","0"};
int speed_ESP = 0;

void setup() {
  //Some General Setup Stuff

  Wire.begin(); // We need this for the i2c comms for the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  ina219.init(); // this initiates the current sensor
  Serial.begin(9600); // USB Communications
  #if ESP32
  Serial1.begin(9600);
  #endif
  //Check for the SD Card
  Serial.println("\nInitializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("* is a card inserted?");
    while (true) {} //It will stick here FOREVER if no SD is in on boot
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  if (SD.exists("BatCycle.csv")) { // Wipe the datalog when starting
    SD.remove("BatCycle.csv");
  }

  
  noInterrupts(); //disable all interrupts
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  //SMPS Pins
  pinMode(13, OUTPUT); // Using the LED on Pin D13 to indicate status
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  pinMode(6, OUTPUT); // This is the PWM Pin

  //LEDs on pin 7 and 8
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT);

  //Relays
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
  pinMode(3, OUTPUT);

  //Voltage Measurements
  pinMode(A3, INPUT);
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);

  //Discharge inputs
  pinMode(9, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  // TimerA0 initialization for 1kHz control-loop interrupt.
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm;

  // TimerB0 initialization for PWM output
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz

  interrupts();  //enable interrupts.
  analogWrite(6, pwm_out); //just a default state to start with
}

void loop() {
   if (loop_trigger == 1){ // FAST LOOP (1kHZ)
      state_num = next_state; //state transition
      V_Bat = analogRead(A0)*4/1.003; //check the battery voltage (1.003 is a correction for measurement error, you need to check this works for you)
      if ((V_Bat > 3700 || V_Bat < 2400 || V_Bat1 > 3700 || V_Bat1 < 2400 || V_Bat2 > 3700 || V_Bat2 < 2400 || V_Bat3 > 3700 || V_Bat3 < 2400)) { //Checking for Error states (just battery voltage for now)
          state_num = 5; //go directly to jail
          next_state = 5; // stay in jail
          digitalWrite(7,true); //turn on the red LED
          current_ref = 0; // no current
      }
      current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)
      error_amps = (current_ref - current_measure) / 1000; //PID error calculation
      pwm_out = pidi(error_amps); //Perform the PID controller calculation
      pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life
  }
  
  if (int_count == 1000) { // SLOW LOOP (1Hz)
    input_switch = digitalRead(2); //get the OL/CL switch status
    switch (state_num) { // STATE MACHINE (see diagram)
      case 0:{ // Start state (no current, no LEDs)
        current_ref = 0;
        if (input_switch == 1) { // if switch, move to charge
          next_state = 1;
          digitalWrite(8,true);
        } else { // otherwise stay put
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 1:{ // Charge state (750mA and a green LED)

            if (current_ref>750){
              current_ref=750;
            }else{

            //mppt algorithm based on P&O  
            current_pres = ina219.getCurrent_mA(); // sampling the inductor current, which is equal to the output current from the buck
            voltage_pres = analogRead(A0) * 4 / 1.003; //sample the output voltage which is equal to the output voltage from the buck
 
            power_pres = voltage_pres * current_pres;


            if (power_pres > power_prev)
            { if (voltage_pres > voltage_prev)
                  pwm_out = pwm_out + delta;
              else
                  pwm_out = pwm_out - delta;
              }
            else
              
            {  if (voltage_pres > voltage_prev)
                  pwm_out = pwm_out - delta;
                else
                  pwm_out = pwm_out + delta;
              }
              power_prev = power_pres;
              voltage_prev = voltage_pres;
            
              analogWrite(6, pwm_out);
            }
            
            //relay operations to measure voltages of each cell and charge cells
            digitalWrite(A6, HIGH); //relay of battery 1 asserted to measure battery 1 voltage as an 8 bit value
            V_Bat1 = analogRead(A1)*4/1.003; //battery 1 voltage measured in volts by multiplying by 4 (max external reference value) and dividing by the correction factor of 1.003
            digitalWrite(A6, LOW); //relay of battery 1 unasserted to charge battery 1
            delay(100);
            digitalWrite(A7, HIGH); //relay of battery 2 asserted to measure battery 2 voltage as an 8 bit value
            V_Bat2 = analogRead(A2)*4/1.003; ////battery 2 voltage measured in volts by multiplying by 4 (max external reference value) and dividing by the correction factor of 1.003
            digitalWrite(A7, LOW); //relay of battery 2 unasserted to charge battery 2
            delay(100);
            digitalWrite(3, HIGH); //relay of battery 3 asserted to measure battery 3 voltage as an 8 bit value
            V_Bat3 = analogRead(A3)*4/1.003; ////battery 3 voltage measured in volts by multiplying by 4 (max external reference value) and dividing by the correction factor of 1.003
            digitalWrite(3, LOW); //relay of battery 3 unasserted to charge battery 3

            SOC1 = SOC_Charging (V_Bat1);
            SOC2 = SOC_Charging (V_Bat2);
            SOC3 = SOC_Charging (V_Bat3);

            //battery balancing algorithm
            if(SOC1 == SOC2 == SOC3){
            digitalWrite(9, LOW); //discharge of battery 1 unasserted
            digitalWrite(4, LOW); //discharge of battery 2 unasserted
            digitalWrite(5, LOW); //discharge of battery 3 unasserted
          }else if((SOC1 == SOC2) < SOC3){
            digitalWrite(9, LOW); 
            digitalWrite(4, LOW); 
            digitalWrite(5, HIGH); 
          }else if((SOC1 == SOC3) < SOC2){
            digitalWrite(9, LOW);
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
          }else if(SOC1 < (SOC2 == SOC3)){
            digitalWrite(9, LOW);
            digitalWrite(4, HIGH);
            digitalWrite(5, HIGH);
          }else if(SOC1 < SOC2 < SOC3){
            digitalWrite(9, LOW);
            digitalWrite(4, HIGH);
            digitalWrite(5, HIGH);
          }else if(SOC1 < SOC3 < SOC2){
            digitalWrite(9, LOW);
            digitalWrite(4, HIGH);
            digitalWrite(5, HIGH);
          }else if(SOC1 > (SOC2 == SOC3)){
            digitalWrite(9, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(5, LOW);
          }else if((SOC1 == SOC3) > SOC2){
            digitalWrite(9, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(5, HIGH);
          }else if(SOC2 < SOC1 < SOC3){
            digitalWrite(9, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(5, HIGH);
          }else if((SOC1 == SOC2) > SOC3){
            digitalWrite(9, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
          }else if(SOC3 < SOC1 < SOC2){
            digitalWrite(9, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
          }else if(SOC2 < SOC3 < SOC1){
            digitalWrite(9, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(5, HIGH);
          }else if(SOC3 < SOC2 < SOC1){
            digitalWrite(9, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
          }

      
        if (V_Bat1 < 3600 && V_Bat2 < 3600 && V_Bat3 < 3600) { // if not charged, stay put
          next_state = 1;
          digitalWrite(8,true);          
        } else { // otherwise go to charge rest
          next_state = 2;
          digitalWrite(8,false);
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 2:{ // Charge Rest, green LED is off and no current
        current_ref = 0;
        if (rest_timer < 30) { // Stay here if timer < 30
          next_state = 2;
          digitalWrite(8,false);
          rest_timer++;
        } else { // Or move to discharge (and reset the timer)
          next_state = 3;
          digitalWrite(8,false);
          rest_timer = 0;
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;        
      }
      case 3:{ //Discharge state (-330mA and no LEDs)
         current_ref = -330;
            
            //relay operations to measure voltages of each cell and charge cells
            digitalWrite(A6, HIGH); //relay of battery 1 asserted to measure battery 1 voltage as an 8 bit value
            V_Bat1 = analogRead(A1)*4/1.003; //battery 1 voltage measured in volts by multiplying by 4 (max external reference value) and dividing by the correction factor of 1.003
            digitalWrite(A6, LOW); //relay of battery 1 unasserted to charge battery 1
            delay(100);
            digitalWrite(A7, HIGH); //relay of battery 2 asserted to measure battery 2 voltage as an 8 bit value
            V_Bat2 = analogRead(A2)*4/1.003; ////battery 2 voltage measured in volts by multiplying by 4 (max external reference value) and dividing by the correction factor of 1.003
            digitalWrite(A7, LOW); //relay of battery 2 unasserted to charge battery 2
            delay(100);
            digitalWrite(3, HIGH); //relay of battery 3 asserted to measure battery 3 voltage as an 8 bit value
            V_Bat3 = analogRead(A3)*4/1.003; ////battery 3 voltage measured in volts by multiplying by 4 (max external reference value) and dividing by the correction factor of 1.003
            digitalWrite(3, LOW); //relay of battery 3 unasserted to charge battery 3

            SOC1 = SOC_Discharging (V_Bat1);
            SOC2 = SOC_Discharging (V_Bat2);
            SOC3 = SOC_Discharging (V_Bat3);

            //battery balancing algorithm
            if(SOC1 == SOC2 == SOC3){
            digitalWrite(9, LOW); //discharge of battery 1 unasserted
            digitalWrite(4, LOW); //discharge of battery 2 unasserted
            digitalWrite(5, LOW); //discharge of battery 3 unasserted
          }else if((SOC1 == SOC2) < SOC3){
            digitalWrite(9, LOW); 
            digitalWrite(4, LOW); 
            digitalWrite(5, HIGH); 
          }else if((SOC1 == SOC3) < SOC2){
            digitalWrite(9, LOW);
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
          }else if(SOC1 < (SOC2 == SOC3)){
            digitalWrite(9, LOW);
            digitalWrite(4, HIGH);
            digitalWrite(5, HIGH);
          }else if(SOC1 < SOC2 < SOC3){
            digitalWrite(9, LOW);
            digitalWrite(4, HIGH);
            digitalWrite(5, HIGH);
          }else if(SOC1 < SOC3 < SOC2){
            digitalWrite(9, LOW);
            digitalWrite(4, HIGH);
            digitalWrite(5, HIGH);
          }else if(SOC1 > (SOC2 == SOC3)){
            digitalWrite(9, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(5, LOW);
          }else if((SOC1 == SOC3) > SOC2){
            digitalWrite(9, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(5, HIGH);
          }else if(SOC2 < SOC1 < SOC3){
            digitalWrite(9, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(5, HIGH);
          }else if((SOC1 == SOC2) > SOC3){
            digitalWrite(9, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
          }else if(SOC3 < SOC1 < SOC2){
            digitalWrite(9, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
          }else if(SOC2 < SOC3 < SOC1){
            digitalWrite(9, HIGH);
            digitalWrite(4, LOW);
            digitalWrite(5, HIGH);
          }else if(SOC3 < SOC2 < SOC1){
            digitalWrite(9, HIGH);
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
          }

          remaining_range = Range_est(SOC1,SOC2,SOC3, rover_speed, SOH);
          SOH = SOH_Capacity(discharge_rate, discharge_time);
         
         if (V_Bat1 > 2500 && V_Bat2 > 2500 && V_Bat3 > 2500) { // While not at minimum volts, stay here
           next_state = 3;
           digitalWrite(8,false);
         } else { // If we reach full discharged, move to rest
           next_state = 4;
           digitalWrite(8,false);
         }
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 4:{ // Discharge rest, no LEDs no current
        current_ref = 0;
        if (rest_timer < 30) { // Rest here for 30s like before
          next_state = 4;
          digitalWrite(8,false);
          rest_timer++;
        } else { // When thats done, move back to charging (and light the green LED)
          next_state = 1;
          digitalWrite(8,true);
          rest_timer = 0;
        }
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 5: { // ERROR state RED led and no current
        current_ref = 0;
        next_state = 5; // Always stay here
        digitalWrite(7,true);
        digitalWrite(8,false);
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(7,false);
        }
        break;
      }

      default :{ // Should not end up here ....
        Serial.println("Boop");
        current_ref = 0;
        next_state = 5; // So if we are here, we go to error
        digitalWrite(7,true);
      }
      
    }
    
    dataString = String(state_num) + "," + String(V_Bat) + "," + String(current_ref) + "," + String(current_measure) + "," + String(V_Bat1) + "," + String(V_Bat2) + "," + String(V_Bat3) + "," + String(SOC1) + "," + String(SOC2) + "," + String(SOC3) + "," + String(remaining_range); //build a datastring for the CSV file
    Serial.println(dataString); // send it to serial as well in case a computer is connected
    File dataFile = SD.open("BatCycle.csv", FILE_WRITE); // open our CSV file
    if (dataFile){ //If we succeeded (usually this fails if the SD card is out)
      dataFile.println(dataString); // print the data
    } else {
      Serial.println("File not open"); //otherwise print an error
    }
    dataFile.close(); // close the file
    int_count = 0; // reset the interrupt count so we dont come back here for 1000ms
  }
  #if ESP32 //Communication with ESP32
  if ((millis() - lastTime) > sendingDelay) {
    Serial1.println(SOC_Discharging(V_Bat));
    Serial1.println(Range_est(SOC1,SOC2,SOC3,rover_speed,SOH));
    lastTime= millis();
  }
  if ((Serial1.available() > 0)) {
    String msg;
    msg = Serial1.readString();
    char* cstr = new char [msg.length() + 1];
    strcpy(cstr, msg.c_str());
    char* ptr = strtok(cstr, "\n");
    speed_ESP = atoi(ptr);
    if (speed_ESP == 255){
      rover_speed = low_speed;
    } else if(speed_ESP == 500){
      rover_speed = mid_speed;
    } else if(speed_ESP == 700){
      rover_speed = high_speed;
    }
  }
  #endif
}

// Timer A CMP1 interrupt. Every 1000us the program enters this interrupt. This is the fast 1kHz loop
ISR(TCA0_CMP1_vect) {
  loop_trigger = 1; //trigger the loop when we are back in normal flow
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
}

float saturation( float sat_input, float uplim, float lowlim) { // Saturation function
  if (sat_input > uplim) sat_input = uplim;
  else if (sat_input < lowlim ) sat_input = lowlim;
  else;
  return sat_input;
}

float pidi(float pid_input) { // discrete PID function
  float e_integration;
  e0i = pid_input;
  e_integration = e0i;

  //anti-windup
  if (u1i >= ui_max) {
    e_integration = 0;
  } else if (u1i <= ui_min) {
    e_integration = 0;
  }

  delta_ui = kpi * (e0i - e1i) + kii * Ts * e_integration + kdi / Ts * (e0i - 2 * e1i + e2i); //incremental PID programming avoids integrations.
  u0i = u1i + delta_ui;  //this time's control output

  //output limitation
  saturation(u0i, ui_max, ui_min);

  u1i = u0i; //update last time's control output
  e2i = e1i; //update last last time's error
  e1i = e0i; // update last time's error
  return u0i;
}

int SOC_Discharging (float V_Bat){

    float V_diff = abs(Vol_Discharging[0]-V_Bat);
    int index = 0;
    float tmp;
    for (int i = 1; i<sizeof(Vol_Discharging);i++){ 
        tmp = abs(Vol_Discharging[i]-V_Bat);
        if (tmp<V_diff){
            index = i;
            V_diff = tmp;
        }        
    }
    return Battery_lv[index];
}

int SOC_Charging (float V_Bat){

    float V_diff = abs(Vol_Charging[0]-V_Bat);
    int index = 0;
    float tmp;
    for (int i = 1; i<sizeof(Vol_Charging);i++){ 
        tmp = abs(Vol_Charging[i]-V_Bat);
        if (tmp<V_diff){
            index = i;
            V_diff = tmp;
        }        
    }
    return Battery_lv[index];
}

float SOH_Capacity (float discharge_rate, float discharge_time){

        SOH = ((discharge_rate * discharge_time) / 3600) * 1700; // 1700mAh refers to the beginning of life capacity of the three cells in parallel in the battery
        return SOH;
}

float Range_est(float SOC1, float SOC2, float SOC3, int rover_speed, float SOH) {
    average_SOC = (SOC1+SOC2+SOC3)/3;
    present_capacity = (average_SOC/100)*1700*SOH; // 1700mAh refers to the beginning of life capacity of the three cells in parallel in the battery
    low_battery_capacity = (15/100)*1700*SOH;

    if (rover_speed==low_speed){
      range = ((present_capacity-low_battery_capacity)/discharge_rate)*3600*low_speed;
    }else if (rover_speed==medium_speed){
      range = ((present_capacity-low_battery_capacity)/discharge_rate)*3600*medium_speed;
    }else if (rover_speed==high_speed){
      range = ((present_capacity-low_battery_capacity)/discharge_rate)*3600*high_speed;
    }
    return range;
}
