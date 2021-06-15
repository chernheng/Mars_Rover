/*
Authors: Edwyn PEK, TAN Chern Heng, Nitu BARUA

Program adapted Yue Zhu (yue.zhu18@imperial.ac.uk) in July 2020.
pin6 is PWM output at 62.5kHz.
duty-cycle saturation is set as 2% - 98%
Control frequency is set as 1.25kHz. 
*/

#define SERIAL_DEBUG 0 // Getting rid of all the Serial Prints to improve program performance and speed,change this variable from 0 to 1 to re-enable.
#define PIC_QUAL 0
#include <SPI.h>
#include <INA219_WE.h>
#include <Wire.h>
#include <math.h>    

INA219_WE ina219; // this is the instantiation of the library for the current sensor

//**************************Controllers **************************//
float open_loop, closed_loop; // Duty Cycles
float vpd,vb,vref,iL,dutyref,current_mA; // Measurement Variables
unsigned int sensorValue0,sensorValue1,sensorValue2,sensorValue3;  // ADC sample values declaration
float ev=0,cv=0,ei=0,oc=0; //internal signals
float Ts=0.0008; //1.25 kHz control frequency. It's better to design the control period as integral multiple of switching period.
float kpv=0.05024,kiv=168,kdv=0; // voltage pid.
float u0v,u1v,delta_uv,e0v,e1v,e2v; // u->output; e->error; 0->this time; 1->last time; 2->last last time
float kpi=0.02512,kii=39.4,kdi=0; // current pid.
float u0i,u1i,delta_ui,e0i,e1i,e2i; // Internal values for the current controller
float uv_max=4, uv_min=0; //anti-windup limitation
float ui_max=1, ui_min=0; //anti-windup limitation
float kpp=0.072,kip=0.002,kdp=0; // position pid
float u0p,u1p,delta_up,e0p,e1p,e2p; // Internal values for the position controller
float x_threshold=0.05,x_errorcorrection=0.05,y_threshold=0.05; //tagged to accuracy of sensor
float current_limit = 1.0;
boolean Boost_mode = 0;
boolean CL_mode = 0;
double Setpoint, Input, Output;
volatile unsigned int loopTrigger;
unsigned int com_count=0;   // a variables to count the interrupts. Used for program debugging.
int maxsize=100;
float target_ydistance[100],target_angle[100];
float temp_y=0,temp_x=0; //variables to accumulate distance
float y_error=0,x_error=0;
//for communication with control
char* fromControl[3] = {"0","0","255"};
int instr_count = 0, angle_to_send = 0,direction_control = 0,value_control = 0,instruction_done =0,start_movement = 0;
char instr_buffer[10]; // for decoder
int vis = 0,speed_control = 0;
//**************************Controllers **************************//

//************************** Motor Constants **************************//   
int DIRRstate = LOW;              //initializing direction states
int DIRLstate = HIGH;

int DIRL = 20;                    //defining left direction pin
int DIRR = 21;                    //defining right direction pin

int pwmr = 5;                     //pin to control right wheel speed using pwm
int pwml = 9;                     //pin to control left wheel speed using pwm

boolean forward,backward,left,right,stopnow;
String instr;
char direct='s';

 int TURNSPEED=255;
 int FORWARDSPEED=255;
//*******************************************************************//

//*******OPTICAL PINS*************//

// these pins may be different on different boards
// this is for the uno
#define PIN_SS        10
#define PIN_MISO      12
#define PIN_MOSI      11
#define PIN_SCK       13

#define PIN_MOUSECAM_RESET     8
#define PIN_MOUSECAM_CS        7

#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17
//*******OPTICAL PINS*************//

//*******OPTICAL CONSTANTS*************//
int total_x1 =0,total_y1=0,last_x =0,last_y=0;
float total_y=0,total_x =0;
int distance_x=0,distance_y=0,tdistance = 0;
int a =0,b=0,x=0,y=0;
float x_ferror=0;

volatile byte movementflag=0;
volatile int xydat[2];

//defining structures of MD
struct MD
{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};

byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];
//*******OPTICAL CONSTANTS*************//

void vision(){
  direct = 's';
  value_control = 0;
  vis = 1;
}

void setup() {
   //********* Motor Pins Defining *********//
  pinMode(DIRR, OUTPUT);
  pinMode(DIRL, OUTPUT);
  pinMode(pwmr, OUTPUT);
  pinMode(pwml, OUTPUT);
  digitalWrite(pwmr, HIGH);       //setting right motor speed at maximum
  digitalWrite(pwml, HIGH);       //setting left motor speed at maximum
  //***********************//

  //Basic pin setups
  noInterrupts(); //disable all interrupts
  pinMode(13, OUTPUT);  //Pin13 is used to time the loops of the controller
  pinMode(3, INPUT_PULLUP); //Pin3 is the input from the Buck/Boost switch
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  // TimerA0 initialization for control-loop interrupt.
  TCA0.SINGLE.PER = 999; //1kHz
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //64 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm; 

  // TimerB0 initialization for PWM output
  pinMode(6, OUTPUT);
  TCB0.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz
  analogWrite(6,255); 

  interrupts();  //enable interrupts.
  Wire.begin(); // We need this for the i2c comms for the current sensor
  ina219.init(); // this initiates the current sensor
  Wire.setClock(700000); // set the comms speed for i2c

  pinMode(PIN_SS,OUTPUT);
  pinMode(PIN_MISO,INPUT);
  pinMode(PIN_MOSI,OUTPUT);
  pinMode(PIN_SCK,OUTPUT);
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(4),vision,FALLING);
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  Serial1.begin(9600);
  Serial.begin(115200);
  if(mousecam_init()==-1){
    Serial.println("Mouse cam failed to init");
    while(1);
  }  
}

void(*resetFunc) (void) = 0;
//end of set up
void loop() {
  interrupts();

  #if SERIAL_DEBUG
  Serial.println("--------VOID LOOP START--------");
  #endif

  if (vis==1){
    delay(200);
    vis = 0;
  }
  if(Serial1.available()>0){
    String msg;
    // delay(5000);  // Debug hook
    msg = Serial1.readString();

    #if SERIAL_DEBUG
    Serial.println("instruction received");
    Serial.print(msg);
    #endif

    // delay(5000);  // Debug hook

    char* cstr = new char [msg.length()+1];
    strcpy(cstr,msg.c_str());
    char* ptr = strtok(cstr, "\n");
    int i = 0;

    while(ptr){
      fromControl[i] = ptr;
      
      #if SERIAL_DEBUG
      Serial.println("Decoded:");
      Serial.println(fromControl[i]);
      #endif
      
      ptr = strtok(NULL, "\n");
      i++;
    }
    direction_control = atoi(fromControl[0]);
    value_control = atof(fromControl[1]);
    speed_control= atoi(fromControl[2]);
    #if SERIAL_DEBUG
    Serial.println(speed_control);
    #endif
    TURNSPEED = speed_control;
    FORWARDSPEED=speed_control;
    instruction_done =0;
    start_movement = 0;
    instr_count++;
    delete[] cstr;
    switch(direction_control){
      case 1:
        direct = 'f';
        break;
      case 2:
        direct = 'l';
        break;
      case 3:
        direct = 'r';
        break;
      case 4:
        direct = 's';
        break;
      #if SERIAL_DEBUG
      default:
        Serial.print("Error detected.");
        Serial.println(direction_control);
      #endif
    } 
  }


  if (instruction_done >5){
    if (direction_control == 2 || direction_control == 3){
      angle_to_send = -round(180*total_x/(15.8*3.14159265));
      Serial1.println(angle_to_send);//change to Serial1
      Serial1.flush();

      #if SERIAL_DEBUG
      Serial.println("Movement done");
      Serial.println("angle_to_send"+String(int(angle_to_send)));
      #endif
      resetFunc();

      #if SERIAL_DEBUG
      Serial.print("mousecam reset");
      #endif

      start_movement = 0;
      instruction_done = 0;
    } else if (direction_control == 1 || direction_control == 4) {
      Serial1.println(int(total_y));
      Serial1.flush();
      #if SERIAL_DEBUG
      Serial.println("this tells me movement is done");
      #endif
      resetFunc();

      #if SERIAL_DEBUG
      Serial.print("mousecam reset");
      #endif

      start_movement = 0;
      instruction_done = 0;
    }
  }
  sprintf(instr_buffer, "%03d%c%03d", instr_count, direct, int(value_control));
  #if SERIAL_DEBUG
  Serial.print(instr_buffer);
  #endif
  decoder(instr_buffer); // in the form of '000f020'
  
  movenow(direct);
 
  // delay(100); // Debug hook

  unsigned long currentMillis = millis();
  
 #if 0
  
  // if enabled this section grabs frames and outputs them as ascii art
  
  if(mousecam_frame_capture(frame)==0)
  {
    int i,j,k;
    for(i=0, k=0; i<ADNS3080_PIXELS_Y; i++) 
    {
      for(j=0; j<ADNS3080_PIXELS_X; j++, k++) 
      {
        Serial.print(asciiart(frame[k]));
        Serial.print(' ');
      }
      Serial.println();
    }
  }
  Serial.println();
  delay(250);
  
  #else
  
  // if enabled this section produces a bar graph of the surface quality that can be used to focus the camera
  // also drawn is the average pixel value 0-63 and the shutter speed and the motion dx,dy.
 
  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);

  // Ensure quality of pic
  #if PIC_QUAL
  Serial.print("Pic Qual: ");
  Serial.println(md.squal/4);
  #endif
  
  #if SERIAL_DEBUG
  for(int i=0; i<md.squal/4; i++) {
    Serial.print('*'); // Displays number of stars (quality)
  }
  Serial.print(' ');
  Serial.print((val*100)/351);
  Serial.print(' ');
  Serial.print(md.shutter); Serial.print(" (");
  Serial.print((int)md.dx); Serial.print(',');
  Serial.print((int)md.dy); Serial.println(')');

  // Serial.println(md.max_pix);
  #endif

    delay(100);   // allow sensor enough time to take a new measurement.

    distance_x = md.dx; //convTwosComp(md.dx);
    distance_y = md.dy; //convTwosComp(md.dy);

    total_x1 = (total_x1 + distance_x);
    total_y1 = (total_y1 + distance_y);

    total_x = (float)total_x1/157.48; //Conversion from counts per inch to cm (400 counts per inch)
    total_y = (float)total_y1/157.48; //Conversion from counts per inch to cm (400 counts per inch)

    #if SERIAL_DEBUG
    Serial.println("Angle to send = " + String(((180*total_x)/(15.8*3.14159265)),3));
    Serial.println("Distance_x1 = " + String(total_x,3));
    Serial.println("Distance_y1 = " + String(total_y,3));
    #endif

    last_x=total_x; //records previous distance
    last_y=total_y; //records previous distance

  #endif
  
    if(loopTrigger) { // This loop is triggered, it wont run unless there is an interrupt
    
      digitalWrite(13, HIGH);   // set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.
      
      // Sample all of the measurements and check which control mode we are in
      sampling();
      CL_mode = digitalRead(3); // input from the OL_CL switch
      Boost_mode = digitalRead(2); // input from the Buck_Boost switch

      if (Boost_mode){
        if (CL_mode) { //Closed Loop Boost
            pwm_modulate(1); // This disables the Boost as we are not using this mode
            
            #if SERIAL_DEBUG
            Serial.println("CL Boost");
            #endif
        }else{ // Open Loop Boost
            pwm_modulate(1); // This disables the Boost as we are not using this mode
            
            #if SERIAL_DEBUG
            Serial.println("OL Boost");
            #endif
        }
      }else{      
        if (CL_mode) { // Closed Loop Buck
            // The closed loop path has a voltage controller cascaded with a current controller. The voltage controller
            // creates a current demand based upon the voltage error. This demand is saturated to give current limiting.
            // The current loop then gives a duty cycle demand based upon the error between demanded current and measured
            // current
            current_limit = 3; // Buck has a higher current limit
            ev = vref - vb;  //voltage error at this time
        
            #if SERIAL_DEBUG
            Serial.println("vref=" + String(vref));
            Serial.println("vb=" + String(vb)); //output 
            #endif

            cv=pidv(ev);  //voltage pid
            cv=saturation(cv, current_limit, 0); //current demand saturation
            ei=cv-iL; //current error
            closed_loop=pidi(ei);  //current pid
            closed_loop=saturation(closed_loop,0.99,0.01);  //duty_cycle saturation
            pwm_modulate(closed_loop); //pwm modulation
            
            #if SERIAL_DEBUG
            Serial.println("pwm_input" + String(closed_loop)); // greater duty cycle, greater the value
            Serial.println("Closed Loop Buck");
            #endif

        }else{ // Open Loop Buck
            current_limit = 3; // Buck has a higher current limit
            oc = iL-current_limit; // Calculate the difference between current measurement and current limit
            if ( oc > 0) {
              open_loop=open_loop-0.001; // We are above the current limit so less duty cycle
            } else {
              open_loop=open_loop+0.001; // We are below the current limit so more duty cycle
            }
            open_loop=saturation(open_loop,dutyref,0.02); // saturate the duty cycle at the reference or a min of 0.01
            pwm_modulate(open_loop); // and send it out
            
            #if SERIAL_DEBUG
            Serial.println("Open Loop Buck");
            #endif
        }
      }
      // closed loop control path

      digitalWrite(13, LOW);   // reset pin13.
      loopTrigger = 0;
      delay(100);   // Needed to reset the loop trigger for next sampling iteration
    }
  }

//*****INSTRRUCTION DECODE******//

void decoder(String instr){
  int num = (instr.substring(0,3)).toInt();
  direct = instr[3];
  float value= (instr.substring(4)).toFloat();
  
  #if SERIAL_DEBUG
  Serial.println("direct=" + String(direct));
  #endif

  if(direct=='f'){
    forward=true;
    backward=false;
    left=false;
    right=false;
    stopnow=false;
    target_ydistance[num]=value;
  }
   if(direct=='b'){
    forward=false;
    backward=true;
    left=false;
    right=false;
    stopnow=false;
    target_ydistance[num]=-value;
  }
   if(direct=='l'){
    forward=false;
    backward=false;
    left=true;
    right=false;
    stopnow=false;
    target_angle[num]=value;
  }
   if(direct=='r'){
    forward=false;
    backward=false;
    left=false;
    right=true;
    stopnow=false;
    target_angle[num]=value;
  }
    if(direct=='s'){
    forward=false;
    backward=false;
    left=false;
    right=false;
    stopnow=true;
  }
}
  
 //*****INSTRRUCTION DECODE******//

//*******MOTOR FUNCTION*************//

void go_forward(){
          analogWrite(pwmr, FORWARDSPEED);       //speed setting
          analogWrite(pwml, FORWARDSPEED);       //speed setting
          DIRRstate = HIGH;
          DIRLstate = LOW;
          digitalWrite(DIRR, DIRRstate);        //direction setting
          digitalWrite(DIRL, DIRLstate);        //direction setting
          #if SERIAL_DEBUG
          Serial.println("forward");
          #endif
          
          xerrorcorrection();
          start_movement = 1;
  }

void go_backward(){
          analogWrite(pwmr, FORWARDSPEED);       //speed setting
          analogWrite(pwml, FORWARDSPEED);       //speed setting
          DIRRstate = LOW;
          DIRLstate = HIGH;
          digitalWrite(DIRR, DIRRstate);        //direction setting
          digitalWrite(DIRL, DIRLstate);        //direction setting
          
          #if SERIAL_DEBUG
          Serial.println("backward");
          #endif
          
          xerrorcorrection();
          start_movement = 1;
  }

void go_left(){
          analogWrite(pwmr, TURNSPEED);       //speed setting
          analogWrite(pwml, TURNSPEED);       //speed setting
          DIRRstate = HIGH;
          DIRLstate = HIGH;
          digitalWrite(DIRR, DIRRstate);     //direction setting
          digitalWrite(DIRL, DIRLstate);     //direction setting
          
          #if SERIAL_DEBUG
          Serial.println("left");
          #endif
          
          start_movement = 1;
  }

void go_right(){

          analogWrite(pwmr, TURNSPEED);       //speed setting
          analogWrite(pwml, TURNSPEED);       //speed setting
          DIRRstate = LOW;
          DIRLstate = LOW;
          digitalWrite(DIRR, DIRRstate);      //direction setting
          digitalWrite(DIRL, DIRLstate);      //direction setting
          
          #if SERIAL_DEBUG
          Serial.println("right");
          #endif
          
          start_movement = 1;
  }

void stop_rover(){
          analogWrite(pwmr, 0);       //setting right motor speed to 0
          analogWrite(pwml, 0);       //setting left motor speed to 0

          #if SERIAL_DEBUG
          Serial.println("stop");
          #endif

          if (start_movement == 1) {
            instruction_done++;
          }
  }
//*******MOTOR FUNCTION*************//

//*****VREF SETTINGS******//

void setyspeed(){ 
    vref = 5;
}

void setxspeed(){ 
  vref = 5;
}

//*****VREF SETTINGS******//

//*****POSITION CONTROL******//

void movenow(char direct){

 if(direct=='f'||direct=='b'){
  movedistance();
 }

  else if(direct=='l'||direct=='r'){
  moveangle();
 }
  else if(direct=='s'){
    stop_rover();
    vref=0;
    #if SERIAL_DEBUG
    Serial.println("stop instruction executed. Rover not moving");
    #endif
 }

  #if SERIAL_DEBUG
  else {
    Serial.println("No instruction given. Rover not moving");
  }
  #endif
}

void movedistance(){
  if(forward){
    temp_y=target_ydistance[instr_count];
    y_error=pidp(temp_y-total_y);
    
    #if SERIAL_DEBUG
    Serial.println("check instruction forewards");
    Serial.println("temp_y = "+(String)(temp_y));
    Serial.println("yerror = "+(String)(y_error));
    #endif

    if(y_error>y_threshold){
      go_forward();
      setyspeed();
      
    }
    else if(y_error<-y_threshold){
      go_backward();
      setyspeed();

    }
    else{      
      stop_rover();
      vref=0;
      #if SERIAL_DEBUG
      Serial.println("distance complete");
      #endif
    }
  }

    if(backward){
      temp_y=target_ydistance[instr_count];
      y_error=pidp(temp_y-total_y);
      
      #if SERIAL_DEBUG
      Serial.println("check instruction backwards");
      Serial.println("temp_y="+(String)(temp_y));
      Serial.println("yerror="+(String)(y_error));
      #endif
      
      if(y_error<y_threshold){
        go_backward();
        setyspeed();
      }
      else if(y_error>-y_threshold){
        go_forward();
        setyspeed();
      }
      else{      
        stop_rover();
        vref=0;
        
        #if SERIAL_DEBUG
        Serial.println("distance complete");
        #endif
      }
    }
    
    if(stopnow){
      #if SERIAL_DEBUG
      Serial.println("check instruction stop");
      #endif
      stop_rover();
      vref=0;
      #if SERIAL_DEBUG
      Serial.println("stopnow");
      #endif
    }
}

void moveangle(){
    if(left){
      temp_x=15.8*3.14159265*target_angle[instr_count]/180;
      x_error=pidp(temp_x-total_x);

      #if SERIAL_DEBUG
      Serial.println("check instruction left");
      Serial.println("temp_x ="+(String)(temp_x));
      Serial.println("xerror ="+(String)(x_error));
      #endif
      
      if(x_error>x_threshold){
        go_left();
        setxspeed();
      }
       else if(x_error<-x_threshold){
        go_right();
        setxspeed();
      }
      else{
        vref=0;
        stop_rover();

        #if SERIAL_DEBUG
        Serial.println("turn complete");
        #endif
      }
    }

    if(right){

      temp_x=-15.8*3.14159265*target_angle[instr_count]/180;
      x_error=pidp(temp_x-total_x);
      
      #if SERIAL_DEBUG
      Serial.println("check instruction right");
      Serial.println("temp_x"+(String)(temp_x));
      Serial.println("xerror="+(String)(x_error));
      #endif

      if(x_error>x_threshold){
        go_left();
        setxspeed();
      }
      else if(x_error<-x_threshold){
        go_right();
        setxspeed();
      }
      else{      
        stop_rover();
        vref=0;
        #if SERIAL_DEBUG
        Serial.println("turn complete");
        #endif
      }
    }

    if(stopnow){
      stop_rover();
      vref=0;
      #if SERIAL_DEBUG
      Serial.println("stopped");
      #endif
    }
}

void xerrorcorrection() {
  x_ferror=pidp(total_x-last_x);
  #if SERIAL_DEBUG
  Serial.println("x_ferror = "+(String)(x_ferror));
  #endif
  if (x_error >x_errorcorrection){
    analogWrite(pwmr, 0);
  }
  if (x_error <-x_errorcorrection){
    analogWrite(pwml, 0);
  }
}

// This is a PID controller for the position
float pidp(float pid_input){
  float e_integration;
  e0p = pid_input;
  e_integration=e0p;
  
  delta_up = kpp*(e0p-e1p) + kip*Ts*e_integration + kdp/Ts*(e0p-2*e1p+e2p); //incremental PID programming avoids integrations.
  u0p = u1p + delta_up;  //this time's control output
  
  u1p = u0p; //update last time's control output
  e2p = e1p; //update last last time's error
  e1p = e0p; // update last time's error
  return u0p;
}

//*******OPTICAL FUNCTION*************//
// functions to covert 2s complement to normal binary
int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }

// function to reset the mousecam
void mousecam_reset()
{
  digitalWrite(PIN_MOUSECAM_RESET,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET,LOW);
  delay(35); // 35ms from reset to functional
}

// function to initilize the mousecam
int mousecam_init(){
  pinMode(PIN_MOUSECAM_RESET,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);
  
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  
  mousecam_reset();
  
  int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
  if(pid != ADNS3080_PRODUCT_ID_VAL)
    return -1;

  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);

  return 0;
}


// write mousecam register
void mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}


// read register of mousecam
int mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(1);
  return ret;
}

// take in a pointer that points to a MD,  read the motion of MD
void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);      // disable CS of mousecam
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff)<<8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(5);
}

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(byte *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE,0x83);
  
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  
  SPI.transfer(ADNS3080_PIXEL_BURST);
  delayMicroseconds(50);
  
  int pix;
  byte started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for(count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
  {
    pix = SPI.transfer(0xff);
    delayMicroseconds(10);
    if(started==0)
    {
      if(pix&0x40)
        started = 1;
      else
      {
        timeout++;
        if(timeout==100)
        {
          ret = -1;
          break;
        }
      }
    }
    if(started==1)
    {
      pdata[count++] = (pix & 0x3f)<<2; // scale to normal grayscale byte range
    }
  }

  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(14);
  
  return ret;
}

char asciiart(int k)
{
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k>>4];
}

//*******OPTICAL FUNCTION*************//

// This subroutine processes all of the analogue samples, creating the required values for the main loop

//*******SMPS FUNCTION*************//

// Timer A CMP1 interrupt. Every 800us the program enters this interrupt. 
// This, clears the incoming interrupt flag and triggers the main loop.

ISR(TCA0_CMP1_vect){
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
  loopTrigger = 1;
  com_count++;
}

void sampling(){

  // Make the initial sampling operations for the circuit measurements
  
  sensorValue0 = analogRead(A0); //sample 
  
  //sensorValue2 = analogRead(A2); //sample Vref
  sensorValue3 = analogRead(A3); //sample Vpd
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts
  //vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  vpd = sensorValue3 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages
  
  if (Boost_mode == 1){
    iL = -current_mA/1000.0;
    dutyref = saturation(sensorValue2 * (1.0 / 1023.0),0.99,0.33);
  }else{
    iL = current_mA/1000.0;
    dutyref = sensorValue2 * (1.0 / 1023.0);
  }
  
}

float saturation( float sat_input, float uplim, float lowlim){ // Saturatio function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < lowlim ) sat_input=lowlim;
  else;
  return sat_input;
}

void pwm_modulate(float pwm_input){ // PWM function
  analogWrite(6,(int)(255-pwm_input*255)); 
}

// This is a PID controller for the voltage

float pidv( float pid_input){
  float e_integration;
  e0v = pid_input;
  e_integration = e0v;
 
  //anti-windup, if last-time pid output reaches the limitation, this time there won't be any intergrations.
  if(u1v >= uv_max) {
    e_integration = 0;
  } else if (u1v <= uv_min) {
    e_integration = 0;
  }

  delta_uv = kpv*(e0v-e1v) + kiv*Ts*e_integration + kdv/Ts*(e0v-2*e1v+e2v); //incremental PID programming avoids integrations.there is another PID program called positional PID.
  u0v = u1v + delta_uv;  //this time's control output

  //output limitation
  saturation(u0v,uv_max,uv_min);
  
  u1v = u0v; //update last time's control output
  e2v = e1v; //update last last time's error
  e1v = e0v; // update last time's error
  return u0v;
}

// This is a PID controller for the current

float pidi(float pid_input){
  float e_integration;
  e0i = pid_input;
  e_integration=e0i;
  
  //anti-windup
  if(u1i >= ui_max){
    e_integration = 0;
  } else if (u1i <= ui_min) {
    e_integration = 0;
  }
  
  delta_ui = kpi*(e0i-e1i) + kii*Ts*e_integration + kdi/Ts*(e0i-2*e1i+e2i); //incremental PID programming avoids integrations.
  u0i = u1i + delta_ui;  //this time's control output

  //output limitation
  saturation(u0i,ui_max,ui_min);
  
  u1i = u0i; //update last time's control output
  e2i = e1i; //update last last time's error
  e1i = e0i; // update last time's error
  return u0i;
}

//*******SMPS FUNCTION*************//


/*end of the program.*/
