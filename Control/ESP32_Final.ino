/*
Author: TAN Chern Heng

Documentation of Control program Logic
Phase 1: A 360 degrees rotation to scan the surroundings and take note of boundaries and the ping pong balls
  - Rotation is done via 10 degrees increment, then poll Vision and Drive to get obstacles found and the angle of rover
  - Rotation can stop either via hitting the 10 degree rotation or Vision does a wired interrupt to Drive, and Drive will automatically stop and send angle to Control
  - Send the data collected to command via URL
  - Repeat until rover does a full 360 rotation, then send 'done' to command and go into phase 2

Phase 2: Path-finding algorithm will be deployed on Command and the Rover will navigate to the specified object of interest on the field.
  - Command will send distance/angle command to control 1 at a time, and this will be communicated to drive
  - Once drive has completed the movement, the distance travelled or angle turned is sent back to command via control
  - This repeats until the 2nd last or last instruction, whichever one is an angle instruction.
    - After the angle instruction, the obj colour is sent to vision, and the vision should be able to see where the object is, and direct an angle correction
    - This angle correction is sent to Drive which will stop once Vision does a wire interrupt to both control and Drive, and the final angle turned will be sent to Command.
*/


#define RXD2_ARD 16
#define TXD2_ARD 17
#define VSPI_MISO   MISO
#define VSPI_MOSI   MOSI
#define VSPI_SCLK   SCK
#define VSPI_SS     SS

#define WIFI_Code 1
#define ROTATION 360
#define pi 3.141592
#define SERIAL_DEBUG 0
#define ENERGY 0 // set to 1 to run energy code

#include <WiFi.h>
#include <SPI.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <WebSocketClient.h>

WebSocketClient webSocketClient;
WiFiClient client;

//CHANGE THESE FIELDS
const char* ssid     = "WIFI_USERNAME";
const char* password = "WIFI_PASSWORD";
char path[] = "/ws/update/data/";
char host[] = "13.212.248.141";

static const int spiClk = 1000000; // 1 MHz
int phase = 0; // indicate which phase of the program

//Command Communication
unsigned long commandFetch = 0;
unsigned long commandFetchDelay = 1000;
int commandArr[6] = {0, 0, 0, 0, 0, 0}; // buffer to receive things from command
int start = 0;
int angle_comm = 0;
int distance_comm = 0;
int tag_comm = 0;
int drive_send_to_command = 0;
int vision_send_to_command = 0;
int send_to_command_p2 = 0;
int obj_colour_comm = 0;
int last_path = 0;
int speed_rover = 255;

//Drive
int send_to_drive_p1 = 0;
int send_to_drive_p2 = 0;
int angle_drive = 0;
int distance_drive = 0;
int direct = 0;
int sent_value = 0;
int drive_count = 0;


//Vision Communication
uint8_t arr_out[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t vis_distance[6] = {0, 0, 0, 0, 0, 0}; //to track all the received distances
uint8_t vis_color[6] = {0, 0, 0, 0, 0, 0}; // to track all the received colour
uint8_t vis_valid[6] = {0, 0, 0, 0, 0, 0}; // to track the index of the objects in the middle of the screen, hence is valid
uint8_t vis_correction[6] = {0, 0, 0, 0, 0, 0};// to track the correction of the object seen
uint8_t ball_correction[5] = {0,0,0,0,0};// track the correction fo reach ball for Phase 2
int obj_num = 0;//number of object received from vision
uint8_t obj_color = 0;
int num_valid = 0;//number of objects in the centre of the screen
int vis_p2 = 0;
int invalid_data = 0;
int target_correction = 255;

//Energy Communication
unsigned long energySend = 0;
unsigned long energySendDelay = 1000;
char* fromEnergy[2] = {"0","0"};
int battery = 80; // preset values
int range_estimation = 100; //preset values


//Running values
int total_rotation = 0;
int rotation_done = 0;

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;

void setup() {
  Serial.begin(115200);
  //SPI code
  vspi = new SPIClass(VSPI);
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();
  pinMode(VSPI_SS, OUTPUT); //VSPI SS

  //interrupt pin 15 for Arduino interrupt
  pinMode(21, INPUT);
  pinMode(15, INPUT);

//WIFI connection
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(1000);

  //Arduino communication to start Low power mode
  obj_color=7;
  visionComm(obj_color);
  Serial1.begin(9600, SERIAL_8N1, RXD2_ARD, TXD2_ARD);
  #if ENERGY
  Serial2.begin(9600, SERIAL_8N1, 2, 4); //11 and 13 port on PCB for Energy
  #endif

  if (client.connect(host, 8000)) {
    Serial.println("Connected");
    webSocketClient.path = path;
    webSocketClient.host = host;
    if (webSocketClient.handshake(client)) {
      Serial.println("Handshake successful");
    } else {
      Serial.println("Handshake failed.");
    }
  } else {
    Serial.println("Connection failed.");
  }
  
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();//reconnecting to previously connected WiFi if connection is lost
  }
  //getting start from server
  if (start == 0) {
    startServer();
    if (start == 0) {
      Serial.println("Not started");
      delay(500); 
      return; //program returns, so will repeat this in a loop if program not started
    } else if (start == 1) {
      phase = 1;// go into phase 1
      send_to_drive_p1 = 1; // send rotation instruction to drive for phase 1
      //reset FPGA
      obj_color=0;
      visionComm(obj_color);
      delay(100);
      obj_color=6;
      visionComm(obj_color);
      Serial.println("Starting the program");
    }
  }

  //sending done of phase 1 to server
  if (rotation_done == 1) {
    updateServerP1();
    doneServer();
    obj_color = 0;
    visionComm(obj_color);//sending 0 for vision to reset FPGA for phase 2
    rotation_done = 0;
    phase = 2; // go into phase2
    total_rotation = 0;
  }

  //Phase 1
  if (phase == 1) {
    //sending to command for phase 1
    if (drive_send_to_command == 1 && vision_send_to_command == 1) { // check if Control already received dataa from both Drive and Vision so that the information sent to Command is accurate
      if (num_valid != 0) { //there are valid objects in the centre of screen
        updateServerP1();
      }
      Serial.println("Going to next loop");
      send_to_drive_p1 = 1; // continue the next loop to send to drive
      drive_send_to_command = 0;
      vision_send_to_command = 0;
    }

    if (total_rotation < (ROTATION-10) && send_to_drive_p1) { //if total rotation is less than 350, will just send a right rotation of 10 degrees
      Serial1.println("3"); // 3 is right rotation
      Serial1.println("10");
      send_to_drive_p1 = 0;
      if (speed_rover != 255){
        Serial1.println(speed_rover);
      }
      Serial.println("Sending to Drive");
    } else if ((total_rotation >= (ROTATION-10)) && (total_rotation < ROTATION) && send_to_drive_p1) { //but if the difference is less than 10, just send the difference
      Serial1.println("3");
      Serial1.println((ROTATION+5) - total_rotation); //to make sure it hits 360 due to small errors in controller
      send_to_drive_p1 = 0;
      if (speed_rover != 255){
        Serial1.println(speed_rover);
      }
      Serial.println("Sending to Drive");
    }
    if ((Serial1.available() > 0)) {//receiving from Drive
      String msg;
      msg = Serial1.readString();
      char* cstr = new char [msg.length() + 1];
      strcpy(cstr, msg.c_str());
      char* ptr = strtok(cstr, "\n");
      angle_drive = atoi(ptr);
      Serial.print("Angle from Drive: ");
      Serial.println(angle_drive);
      total_rotation = total_rotation + angle_drive; // keep track of total rotation of the rover
      Serial.print("Total rotation so far: ");
      Serial.println(total_rotation);
      delete[] cstr;
      if (total_rotation >= ROTATION) {
        rotation_done = 1; // this triggers the update of the done server to move onto phase 2
      } else {
        drive_send_to_command = 1;//received from Drive, ready to send to command
      }
      obj_color = 6;
      visionComm(obj_color);//only read from vision once receive information from Drive, hence Vision do not need to interrupt Control
      vision_send_to_command = 1;//received from Vision, ready to send to vision
      delay(400);
    }
  }





  //Phase 2 code which does navigation
  if (phase == 2) {
    if ((millis() - commandFetch) > commandFetchDelay) { //retrieve instructions from command every second
      decodeServer();
      commandFetch = millis();
    }
    if (send_to_command_p2 == 1) { //sending to command, only updates when receives movement data from Drive
      updateServerP2();
      send_to_command_p2 = 0;
    }

    if (send_to_drive_p2 == 1) {
      if (angle_comm != 0) {//Setting the direction and the value to send to Drive
        if (angle_comm < 0) {
          direct = 2;
          angle_comm = -angle_comm;
        } else {
          direct = 3;
        }
        drive_count = 1; //angle sent
        sent_value = angle_comm;
      } else if (distance_comm != 0) {
        sent_value = distance_comm;
        direct = 1;
        drive_count = 2; // distance sent
      }
      Serial1.println(direct);
      Serial1.println(sent_value);
      if (speed_rover != 255){
        Serial1.println(speed_rover);
      }
      #if SERIAL_DEBUG
      Serial.println(direct);
      Serial.println(sent_value);
      Serial.println("Sent to drive");
      #endif
      send_to_drive_p2 = 0;
      sent_value = 0;
    }
    if ((Serial1.available() > 0)) {
      String msg;
      msg = Serial1.readString();
      if (msg.length() > 9) {
        Serial.println("Too many characters");
      } else {
        char* cstr = new char [msg.length() + 1];
        strcpy(cstr, msg.c_str());
        char* ptr = strtok(cstr, "\n");
        if (drive_count == 1) {
          angle_drive = atoi(ptr);
          Serial.println("From Drive, Angle Moved:");
          Serial.println(angle_drive);
          distance_drive = 0;
        } else if (drive_count == 2) {
          distance_drive = atoi(ptr);
          Serial.println("From Drive, Distance Moved:");
          Serial.println(distance_drive);
          angle_drive = 0;
        }
        send_to_command_p2 = 1;
        delete[] cstr;
        if (vis_p2 == 1) { //only 1 after every angle instruction to use vision to update the surroundings to Command to see the map
          vis_p2 = 0;
          if(last_path != 0){ //last path is taken from a field in Command, its 1 if its the 2nd last instruction and 2 if its the last instruction, 0 if its not any of the final instructions
            //This entire code is to engage vision to make adjustments after the final angle instruction, so that the rover will shift and adjust itself to face the object of interest 
            Serial.println("Engaging Vision");
            visionComm(obj_colour_comm);           
            Serial.print("Target Correction: ");
            Serial.println(target_correction);
            if (target_correction == 0) {
              Serial.println("Object is in the centre of the screen");
              angle_drive = total_rotation + angle_drive; //return angle_drive
            } else if (target_correction == 1) {
              Serial.println("Object is on the right");
              Serial1.println("3");
              Serial1.println("45"); //Preset adjustment, will use interrupt once the object is in the centre of screen
              vis_p2 = 1;//want to repeat this code to check if its on target
              total_rotation = total_rotation + angle_drive;
              send_to_command_p2 = 0;
            } else if (target_correction == 2) {
              Serial.println("Object is on the left");
              Serial1.println("2");
              Serial1.println("45");
              vis_p2 = 1;
              total_rotation = total_rotation + angle_drive;
              send_to_command_p2 = 0;
            } else {
               Serial.println("Object required not detected");
            }
          } else {
            //this code runs when it is not the final instructions
            obj_color = 6;//returns everything Vision sees
            visionComm(obj_color);//only read from vision if once receive information from Drive
          }
        }
      }
    }
  }

  //Energy code
 #if ENERGY
  if ((millis() - energySend) > energySendDelay) { //retrieve instructions from command every second
    energySend = millis();
    Serial2.println(speed_rover);
  }
  if ((Serial2.available() > 0)) {
    String msg;
    msg = Serial1.readString();
    char* cstr = new char [msg.length() + 1];
    strcpy(cstr, msg.c_str());
    char* ptr = strtok(cstr, "\n");
    int i = 0;
    while(ptr){
      fromEnergy[i] = ptr;
      ptr = strtok(NULL, "\n");
      i++;
    }
    battery  = atoi(fromEnergy[0]);
    range_estimation = atoi(fromEnergy[1]);
    delete[] cstr;
  }
#endif
}

/*
 * Vision_color : 0-> None/Invalid, 1->Red, 2->Green, 3->Blue, 4->Yellow, 5->Purple, 6-> Wall, all other numbers are invalid
 * correction: 0->Centre of screen, 1-> Right of screen, 2->Left of Screen
 * obj is the value to transfer to vision, sending 6 will get everything Vision sees, 1-5 will get Wall and only the object of the specified colour
 * vis_valid just holds the index of the items in the centre of the frame, which will be sent to Command
 */

void visionComm(uint8_t obj) {
  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  obj_num = 0;//track number of received items
  uint8_t tmp = 0;
  num_valid = 0;//track number of items to send to command, which is the number of items in the centre of screen
  target_correction = 255;//correction of interested object
  // Start data transfer
  invalid_data = 1;
  while(invalid_data == 1){
    for (int i = 0; i < 13; i++) {
      digitalWrite(VSPI_SS, LOW); //pull SS slow to prep other end for transfer
      if (i == 0) {
        tmp = obj;
        obj |= 0x80;//sending 1 as the MSB for the first transfer to align values
      } else {
        obj = tmp;
      }
      arr_out[i] = vspi->transfer(obj);
      digitalWrite(VSPI_SS, HIGH); //pull ss high to signify end of data transfer
      if (i != 0) {
        // Odd i: distance (y) values
        // Even i: msb's-correction, lsb's correction
        if (arr_out[i] == 0xFF) {
          invalid_data = 0;//end the loop
          break; //poll everything vision sees until encounter 255 which means no more items
        } else if((obj != 0) && (arr_out[i] == 0x00) && (obj!=7)){
          invalid_data = 1; //trigger re-read
          Serial.println("trigger re-read");
          break;
        }
        if (i == 12){
          invalid_data = 0; //end loop
        }
        if (i % 2) {
          vis_distance[obj_num] = arr_out[i];
        } else {
          vis_correction[obj_num] = arr_out[i] >> 6;
          vis_color[obj_num] = arr_out[i] & 0b0111;
          if(vis_color[obj_num] == 0){
            continue;
          }
          uint8_t prev_corr = ball_correction[vis_color[obj_num]-1];
          ball_correction[vis_color[obj_num]-1] = vis_correction[obj_num];
          if (vis_color[obj_num]==6){
            uint8_t r = vis_distance[obj_num] & 0b01111111; //Least significant 7 bits is the R
            uint8_t theta = arr_out[i] >> 3;
            vis_distance[obj_num] = wallDistFromRTheta(r,theta);
            vis_valid[num_valid] = obj_num; //hold the index of the valid object
            num_valid++;
          } else if (vis_correction[obj_num] == 0) {//putting the items in the middle of the screen into vis_valid
            vis_valid[num_valid] = obj_num; //hold the index of the valid object
            num_valid++;
          } else if ((ball_correction[vis_color[obj_num]-1]==2) && (prev_corr==1)) {
            vis_valid[num_valid] = obj_num; //hold the index of the valid object
            num_valid++;
          }
          if (obj != 6 && vis_color[obj_num] == obj) { //for phase 2, if the correct target object is found, log its correction
            target_correction = vis_correction[obj_num];
          }
          obj_num++;
        }
      }
      delay(50);
    }
  }
  vspi->endTransaction();
  #if SERIAL_DEBUG
  Serial.println("polling vision");
  for(int i =0;i<6;i++){
    if (vis_distance[i]==0){
      break;
    }
    Serial.print("Distance: ");
    Serial.println(vis_distance[i]);
    Serial.print("color: ");
    Serial.print(vis_color[i]);
    Serial.print(",  correction: ");
    Serial.println(vis_correction[i]);
  }
  #endif
}

uint8_t wallDistFromRTheta(uint8_t r, uint8_t theta_in){ // calculate the distance of a wall from R and theta
    int theta = theta_in*2 + 50;
    float dist = 0;  // Perform operations on this variable

    if (theta==90) return r;

    if (theta > 90) {
        theta -= 90;
        dist = (float)r * cos( (float)theta*pi/180 ) + (10.0 - (float)r * sin((float)theta*pi/180) )*(tan((float)theta*pi/180));
    } else {
        dist = (((float)r/cos((float)theta*pi/180))-10.0) * tan((float)(90-theta)*pi/180);
    }

    dist = 240 + dist*2;    // Y-value on screen
    
    if (dist > 435) return 33;
    if (dist < 265) return 254;
    int res = 7920/(dist-239);
    return res;
}

void reconnect() {
  Serial.println("Reconnecting to WiFi...");
  WiFi.disconnect();
  WiFi.reconnect();
  delay(1000);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void battery_check(){
  if (battery<=15){
    //setting vision to Low power mode
    obj_color=7;
    visionComm(obj_color);
    while (1){
    if ((Serial2.available() > 0)) {
      String msg;
      msg = Serial1.readString();
      char* cstr = new char [msg.length() + 1];
      strcpy(cstr, msg.c_str());
      char* ptr = strtok(cstr, "\n");
      battery  = atoi(ptr);
      delete[] cstr;
    }
    if (battery >= 80){
      break;
    }
    delay(10000);
    }
  }
}

void updateServerP1() { 
  String json;
  for (int i = 0; i < num_valid; i++) { //only send what the rover sees in the middle of the screen, can send multiple things as well
    if(client.connected()){
      json = "{\"obj_color\":" + String(vis_color[vis_valid[i]]) + ",\"angle\":" + String(total_rotation) + ",\"distance\":" + String(vis_distance[vis_valid[i]]) + ",\"battery\":" + String(battery) + ",\"range_estimate\":" + String(range_estimation) + "}";
      String json2 = "{\"type\": \"receive\", \"text\": "+String(json)+"}";
      webSocketClient.sendData(json2);
      Serial.println("Data Sent!");
    } else {
      Serial.println("Reconnecting");
      int temp = client.connect(host,8000);
      while (temp !=1){
        temp = client.connect(host,8000);
      }
      int temp2 = webSocketClient.handshake(client);
      while(temp2!=1){
        temp2 = webSocketClient.handshake(client);
      }
      json = "{\"obj_color\":" + String(vis_color[vis_valid[i]]) + ",\"angle\":" + String(total_rotation) + ",\"distance\":" + String(vis_distance[vis_valid[i]]) + ",\"battery\":" + String(battery) + ",\"range_estimate\":" + String(range_estimation) + "}";
      String json2 = "{\"type\": \"receive\", \"text\": "+String(json)+"}";
      webSocketClient.sendData(json2);
      Serial.println("Data Sent!");
    }
  }
  #if ENERGY
  battery_check(); //check battery after updating server
  #endif
}


void updateServerP2() { 
  String json;
  if(client.connected()){
    if(num_valid !=0){
      for (int i = 0; i < num_valid; i++) {//sending everything it sees as it moves along
        json = "{\"obj_color\":" + String(vis_color[vis_valid[i]]) + ",\"angle\":" + String(angle_drive) + ",\"distance\":" + String(vis_distance[vis_valid[i]]) + ",\"battery\":80,\"range_estimate\":100}";
        String json2 = "{\"type\": \"receive\", \"text\": "+String(json)+"}";
        webSocketClient.sendData(json2);
        Serial.println("Data Sent!");
      }
    }
    //describing rover's movement
    json = "{\"obj_color\":0,\"angle\":" + String(angle_drive) + ",\"distance\":" + String(distance_drive) + ",\"battery\":80,\"range_estimate\":100}";
    String json2 = "{\"type\": \"receive\", \"text\": "+String(json)+"}";
    webSocketClient.sendData(json2);
    Serial.println("Data Sent!");
  } else {
    Serial.println("Reconnecting");
    int temp = client.connect(host,8000);
    while (temp !=1){
      temp = client.connect(host,8000);
    }
    int temp2 = webSocketClient.handshake(client);
    while(temp2!=1){
      temp2 = webSocketClient.handshake(client);
    }
    if(num_valid !=0){
      for (int i = 0; i < num_valid; i++) {//sending everything it sees as it moves along
        json = "{\"obj_color\":" + String(vis_color[vis_valid[i]]) + ",\"angle\":" + String(angle_drive) + ",\"distance\":" + String(vis_distance[vis_valid[i]]) + ",\"battery\":80,\"range_estimate\":100}";
        String json2 = "{\"type\": \"receive\", \"text\": "+String(json)+"}";
        webSocketClient.sendData(json2);
        Serial.println("Data Sent!");
      }
    }
    json = "{\"obj_color\":0,\"angle\":" + String(angle_drive) + ",\"distance\":" + String(distance_drive) + ",\"battery\":80,\"range_estimate\":100}";
    String json2 = "{\"type\": \"receive\", \"text\": "+String(json)+"}";
    webSocketClient.sendData(json2);
    Serial.println("Data Sent!");
  }
  #if ENERGY
  battery_check(); //check battery after updating server
  #endif
}


void doneServer() { //updating done to server when phase 1 is complete
  String json;
  if(client.connected()){
    json = "{\"done\":1}";
    String json2 = "{\"type\": \"receive\", \"text\": "+String(json)+"}";
    webSocketClient.sendData(json2);
    Serial.println("Data Sent!");
  } else {
    Serial.println("Reconnecting");
    int temp = client.connect(host,8000);
    while (temp !=1){
      temp = client.connect(host,8000);
    }
    int temp2 = webSocketClient.handshake(client);
    while(temp2!=1){
      temp2 = webSocketClient.handshake(client);
    }
    json = "{\"done\":1}";
    String json2 = "{\"type\": \"receive\", \"text\": "+String(json)+"}";
    webSocketClient.sendData(json2);
    Serial.println("Data Sent!");
  }
}

void startServer() { //pinging the start server to see when to start the program
  String data;
  if(client.connected()){
    webSocketClient.getData(data);
    if (data.length() > 0) {
      Serial.print("Received data: ");
      if (data.length() <5){
        Serial.println("oops");
        return;
      }
      JSONVar myObject = JSON.parse(data);

      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
      // myObject.keys() can be used to get an array of all the keys in the object
      JSONVar keys = myObject.keys();
      JSONVar data2 = myObject[keys[1]];
      String jsonString = JSON.stringify(data2);

      JSONVar myObject2 = JSON.parse(jsonString);
      Serial.println(jsonString);

      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject2) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
      // myObject.keys() can be used to get an array of all the keys in the object
      JSONVar keys2 = myObject2.keys();
      JSONVar value = myObject2[keys2[0]];
      Serial.println((int)value);
      start = (int)value;
    }
  } else {
    Serial.println("Reconnecting");
    int temp = client.connect(host,8000);
    while (temp !=1){
      temp = client.connect(host,8000);
    }
    int temp2 = webSocketClient.handshake(client);
    while(temp2!=1){
      temp2 = webSocketClient.handshake(client);
    }
  }
}

void decodeServer() { // decode from the main URL to get the instructions
  String data;
  if(client.connected()){
    webSocketClient.getData(data);
    if (data.length() > 0) {
      Serial.print("Received data: ");
      if (data.length() <5){
        Serial.println("oops");
        return;
      }
      JSONVar myObject = JSON.parse(data);

      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
      // myObject.keys() can be used to get an array of all the keys in the object
      JSONVar keys = myObject.keys();
      JSONVar data2 = myObject[keys[1]];
      String jsonString = JSON.stringify(data2);

      JSONVar myObject2 = JSON.parse(jsonString);
      Serial.println(jsonString);

      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject2) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
      // myObject.keys() can be used to get an array of all the keys in the object
      JSONVar keys2 = myObject2.keys();
      for (int i = 0; i < keys2.length(); i++) {
        JSONVar value = myObject2[keys2[i]];
        Serial.println(int(value));
        commandArr[i] = int(value);
      }
      tag_comm = commandArr[0];
      angle_comm = commandArr[1];
      distance_comm = commandArr[2];
      obj_colour_comm = commandArr[3];
      speed_rover = commandArr[4];
      last_path = commandArr[5];
      send_to_drive_p2 = 1;//receiving data
      if (angle_comm != 0) {//if it is an angle instruction, engage vision to update the map
        vis_p2 = 1;
      }
      drive_count = 0;
      Serial.println(drive_count);
    }
  } else {
    Serial.println("Reconnecting");
    int temp = client.connect(host,8000);
    while (temp !=1){
      temp = client.connect(host,8000);
    }
    int temp2 = webSocketClient.handshake(client);
    while(temp2!=1){
      temp2 = webSocketClient.handshake(client);
    }
  }
  Serial.println("Received from server");
}
