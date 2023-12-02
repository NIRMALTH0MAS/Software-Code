
#include <WiFi.h>
#define NUM_SENSORS 16
#include "HX711.h"
const int LOADCELL_DOUT_PIN = 50;
const int LOADCELL_SCK_PIN = 51;
const int limitSwitch1 = A0;
const int limitSwitch2 = A1;
const int limitSwitch3 = A2;
const int limitSwitch4 = A3;
const int curtainSensor = A4;
const int rightEdgedetect = 1;
const int leftEdgedetect = 52;
const int rearEdgedetect = 53;
const int motor1RelayPin1 = A5;
const int motor1RelayPin2 = A6;
const int motor2RelayPin1 = A7;
const int motor2RelayPin2 = 0;
const int capacity = 200;
HX711 scale;
float weight;
float calibration_factor = 20.775;
const char* ssid = "QPR";
const char* password = "ABCDEFGH";
int curtainSensorStatus = 1;
int limitSwitch1status = 1;
int limitSwitch2status = 1;
int limitSwitch3status = 1;
int limitSwitch4status = 1;
bool rightEdgeStatus, rearEdgeStatus, leftEdgeStatus;
bool edgeDetectionDone = false;
int FrontbayDoorStatus = 0; // Door 1 status (0 for closed, 1 for open)
int RearbayDoorStatus = 0; // Door 2 status (0 for closed, 1 for open)
WiFiServer wifiServer(80);
int RelayPins[NUM_SENSORS] = {4, 7, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49};
int TriggerPins[NUM_SENSORS] = {2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47};
int EchoPins[NUM_SENSORS] = {3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48};
float Time[NUM_SENSORS] = {0}; 
float Distance[NUM_SENSORS] = {0}; 
float sensor_val[NUM_SENSORS];
float Empty_Tray = 625;
float Object_Height = 0;
float maxValue = 0;
float minValue = 0;
unsigned long customOpenDuration = 1500;  // Default custom open duration
unsigned long customCloseDuration = 1300;
unsigned long safetyCurtainDelay = 100;

// ----------------------------------------------------------------------------------------------------------
String BayFrontDoorOpen(unsigned long customOpenDuration);
String BayFrontDoorClose(unsigned long customCloseDuration);
String BayRearDoorOpen(unsigned long customOpenDuration);
String BayRearDoorClose(unsigned long customCloseDuration);
unsigned long weightloopstarttime;
unsigned long weightloopendtime;
unsigned long heightloopstarttime;
unsigned long heightloopendtime;

// ---------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();
  scale.tare();
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(RelayPins[i], OUTPUT);
    digitalWrite(RelayPins[i], HIGH);
    pinMode(TriggerPins[i], OUTPUT);
    pinMode(EchoPins[i], INPUT);
  }
  
  pinMode(motor1RelayPin1, OUTPUT);
  pinMode(motor1RelayPin2, OUTPUT);
  pinMode(motor2RelayPin1, OUTPUT);
  pinMode(motor2RelayPin2, OUTPUT);
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);
  pinMode(curtainSensor, INPUT_PULLUP);
  pinMode(rightEdgedetect, INPUT_PULLUP);
  pinMode(leftEdgedetect, INPUT_PULLUP);
  pinMode(rearEdgedetect, INPUT_PULLUP);
  
  digitalWrite(motor1RelayPin1, LOW);
  digitalWrite(motor1RelayPin2, LOW);
  digitalWrite(motor2RelayPin1, LOW);
  digitalWrite(motor2RelayPin2, LOW);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  wifiServer.begin();
}

// ------------------------------------------------------------DOOR Operation---------------------------------------------------
void parseStringAndInteger(String command, String& str, int& value) {
  int separatorIndex = command.indexOf(':');
  if (separatorIndex != -1) {
    str = command.substring(0, separatorIndex);
    value = command.substring(separatorIndex + 1).toInt();
  } else {
    // Handle the case where the received command doesn't contain a string and an integer.
    // You can add error handling code here.
    str = "";
    value = 0;
  }
}

String BayFrontDoorOpen(unsigned long customOpenDuration  = customOpenDuration) {
  unsigned long FrontdoorOpenStartTime = 0;
  digitalWrite(motor1RelayPin1, HIGH);
  digitalWrite(motor1RelayPin2, LOW);
  
  FrontdoorOpenStartTime = millis(); // Record the start time
  String status = "FrontdoorOpenStartTime" + String(FrontdoorOpenStartTime);
 while (true) {
    // Check if the limit switch is pressed
    if (digitalRead(limitSwitch1) == LOW) {
      break; // Exit the loop if the limit switch is pressed
    }
    
    // Check if the timer has reached 1.3 seconds
    if (millis() - FrontdoorOpenStartTime >= customOpenDuration) {
      status += "FrontDoortoplimitSwitchstatus: false\n";
      break; // Exit the loop if the timer exceeds 1.3 seconds
    }
  }
  digitalWrite(motor1RelayPin1, LOW);
  digitalWrite(motor1RelayPin2, LOW);
  
  FrontbayDoorStatus = 1;
  status += "FrontbayDoorStatus:" + String(FrontbayDoorStatus);
  return status;
}

String BayFrontDoorClose(unsigned long customCloseDuration =customCloseDuration) {
  unsigned long FrontdoorCloseStartTime = 0;
  digitalWrite(motor1RelayPin1, LOW);
  digitalWrite(motor1RelayPin2, HIGH);
  FrontdoorCloseStartTime = millis(); // Record the start time
  String status = "FrontdoorCloseStartTime" + String(FrontdoorCloseStartTime);
 while (true) {
    // Check if the limit switch is pressed
    if (digitalRead(limitSwitch2) == LOW) {
      break; // Exit the loop if the limit switch is pressed
    }
    if(digitalRead(curtainSensor)==0){
      status += "CurtainSensor: True\n";
      delay(safetyCurtainDelay);
      BayFrontDoorOpen();
  }
    // Check if the timer has reached 1.3 seconds
    if (millis() - FrontdoorCloseStartTime >= customCloseDuration) {
      status += "FrontDoorbottomlimitSwitchstatus: false\n";
      break; // Exit the loop if the timer exceeds 1.3 seconds
    }
  }
  digitalWrite(motor1RelayPin1, LOW);
  digitalWrite(motor1RelayPin2, LOW);
  
  FrontbayDoorStatus = 0;
  status += "FrontbayDoorStatus:" + String(FrontbayDoorStatus);
  return status;
}

String BayRearDoorOpen(unsigned long customOpenDuration=customOpenDuration) {
  unsigned long ReardoorOpenStartTime = 0;
  digitalWrite(motor2RelayPin1, HIGH);
  digitalWrite(motor2RelayPin2, LOW);
  
  ReardoorOpenStartTime = millis(); // Record the start time
  String status = "ReardoorOpenStartTime" + String(ReardoorOpenStartTime);
 while (true) {
    // Check if the limit switch is pressed
    if (digitalRead(limitSwitch3) == LOW) {
      break; // Exit the loop if the limit switch is pressed
    }
    
    // Check if the timer has reached 1.3 seconds
    if (millis() - ReardoorOpenStartTime >= customOpenDuration) {
      status += "RearDoortoplimitSwitchstatus: false ";
      break; // Exit the loop if the timer exceeds 1.3 seconds
    }
  }
  digitalWrite(motor2RelayPin1, LOW);
  digitalWrite(motor2RelayPin2, LOW);
  RearbayDoorStatus = 1;
  status += "RearbayDoorStatus:" + String(RearbayDoorStatus);
  return status;
}

String BayRearDoorClose(unsigned long customCloseDuration=customCloseDuration) {
  unsigned long ReardoorCloseStartTime = 0;
  digitalWrite(motor2RelayPin1, LOW);
  digitalWrite(motor2RelayPin2, HIGH);
  
  ReardoorCloseStartTime = millis(); // Record the start time
  String status = "ReardoorCloseStartTime" + String(ReardoorCloseStartTime);
 while (true) {
    // Check if the limit switch is pressed
    if (digitalRead(limitSwitch4) == LOW) {
      break; // Exit the loop if the limit switch is pressed
    }
     // Check if the timer has reached 1.3 seconds
    if (millis() - ReardoorCloseStartTime >= customCloseDuration) {
      status += "RearDoorbottomlimitSwitchstatus: false ";
      break; // Exit the loop if the timer exceeds 1.3 seconds
    }
  }
  digitalWrite(motor2RelayPin1, LOW);
  digitalWrite(motor2RelayPin2, LOW);
  RearbayDoorStatus = 0;
  status += "RearbayDoorStatus:" + String(RearbayDoorStatus);
  return status;
}  
void triggerSensor(int sensorIndex) {
  digitalWrite(RelayPins[sensorIndex], LOW);
  delay(100);
  digitalWrite(TriggerPins[sensorIndex], LOW);
  delayMicroseconds(2);
  digitalWrite(TriggerPins[sensorIndex], HIGH);
  delayMicroseconds(10);
  digitalWrite(TriggerPins[sensorIndex], LOW);
  delayMicroseconds(2);
}

// ------------------------------------------------------------Height Measurment---------------------------------------------------
float calculateDistance(int sensorIndex) {
  float Time = pulseIn(EchoPins[sensorIndex], HIGH);
  return Time * 340 / 20000;
}
String measureHeight() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    triggerSensor(i);
    float distance = calculateDistance(i);
    sensor_val[i] = distance * 10;
    Serial.print("D" + String(i) + "=" + sensor_val[i]);
    digitalWrite(RelayPins[i], HIGH);
  }

  minValue = sensor_val[0];
  maxValue = sensor_val[0];

  for (int j = 0; j < NUM_SENSORS; j++) {
    minValue = min(minValue, sensor_val[j]);
    maxValue = max(maxValue, sensor_val[j]);
  }
  
  String status = "minValue" + String(minValue)+ "\n";
  status += "Empty_Tray height" + String(maxValue)+ "\n";
  float Object_Height = maxValue - minValue;
  status += "Height" + String(Object_Height)+ "\n";
//  delay(heightendDelay);
  heightloopendtime = millis();
  status += "heightloopendtime:" + String(heightloopendtime);
  return status;
}

// ------------------------------------------------------------Weight Measurment---------------------------------------------------
String measureWeight() {
  scale.set_scale(calibration_factor);
  float weight = scale.get_units(5);
  String status = "weight" + String(weight);
  weightloopendtime = millis();
  status += "weightloopendtime:" + String(weightloopendtime);
  return status;
}
int sendCurtainSensorValue() {
  curtainSensorStatus = digitalRead(curtainSensor);
  if(curtainSensorStatus==0){
    return(0);
  }
  else{
   return(1);
  }
}

// ------------------------------------------------------------Limit Switch Status---------------------------------------------------
String checkLimitSwitchStatus() {
    limitSwitch1status = digitalRead(limitSwitch1);
    String status = "FrontDoortoplimitSwitchstatus: " + String(limitSwitch1status) + "\n";
    limitSwitch2status = digitalRead(limitSwitch2);
    status += "FrontDoorbottomlimitSwitchstatus: " + String(limitSwitch2status) + "\n";
    limitSwitch3status = digitalRead(limitSwitch3);  
    status += "RearDoortoplimitSwitchstatus: " + String(limitSwitch3status) + "\n";
    limitSwitch4status = digitalRead(limitSwitch4);
    status += "RearDoorbottomlimitSwitchstatus: " + String(limitSwitch4status);
    return status;
}

// ------------------------------------------------------------Edge Detection Status---------------------------------------------------
String Edgedetect() {
  if (!edgeDetectionDone) { // Check if edge detection hasn't been done yet
    rightEdgeStatus = digitalRead(rightEdgedetect);
    rearEdgeStatus = digitalRead(rearEdgedetect);
    leftEdgeStatus = digitalRead(leftEdgedetect);


    // Create a formatted string with all edge detector statuses
    String status = String(rightEdgeStatus) + "," + String(rearEdgeStatus) + "," + String(leftEdgeStatus);


    // Send the formatted string via serial communication
    return status;
  }
  return "";
}

// ------------------------------------------------------------All Status---------------------------------------------------
String sendAllStatus() {
  String status = "";
  if (FrontbayDoorStatus == 1) {
    status += "FrontbayDoorStatus: Open\n";
  } else if (FrontbayDoorStatus == 0) {
    status += "FrontbayDoorStatus: Closed\n";
  }
  if (RearbayDoorStatus == 1) {
    status += "RearbayDoorStatus: Open\n";
  } else if (RearbayDoorStatus == 0) {
    status += "RearbayDoorStatus: Closed\n";
  }
  status += "CurtainSensor: " + String(sendCurtainSensorValue()) + "\n";
  status += "EdgeDetection: " + String(Edgedetect()) + "\n";
  status += "Weight: " + String(measureWeight()) + " \n";
  status += "Height: " + String(measureHeight()) + " ";
  return status;
}

// ------------------------------------------------------------ Loop Code---------------------------------------------------
void loop() {
  WiFiClient client = wifiServer.available();

  if (client) {
    while (client.connected()) {
      if (client.available() > 0) {
        String request = client.readStringUntil('\r');
        client.flush(); // Clear the client buffer
        String strValue;
        int intValue; // Variable to store the string
        parseStringAndInteger(request, strValue, intValue);
        if (strValue.length() > 0) {
            if (strValue.equals("code_BayFrontDoorOpen")) {
              customOpenDuration = intValue;
              String BayFrontDoorOpenstatus = BayFrontDoorOpen(customOpenDuration);
              client.println("BayFrontDoorOpenstatus\n" + String(BayFrontDoorOpenstatus));
          } else if (strValue.equals("code_BayFrontDoorClose")) {
              customCloseDuration = intValue;
              String BayFrontDoorClosestatus = BayFrontDoorClose(customCloseDuration);
              client.println("BayFrontDoorClosestatus\n" + String(BayFrontDoorClosestatus));
          } else if (strValue.equals("code_BayRearDoorOpen")) {
              customOpenDuration = intValue;
              String BayRearDoorOpenstatus = BayRearDoorOpen(customOpenDuration);
              client.println("BayRearDoorOpenstatus\n" + String(BayRearDoorOpenstatus));
          } else if (strValue.equals("code_BayRearDoorClose")) {
              customCloseDuration = intValue;
              String BayRearDoorClosestatus = BayRearDoorClose(customCloseDuration);
              client.println("BayRearDoorClosestatus\n" + String(BayRearDoorClosestatus));}
        } else if (request.indexOf("GetHeight") != -1) {
          String height = measureHeight();
          client.println("Height: " + String(height) + " cm");
        } else if (request.indexOf("GetWeight") != -1) {
          String weight = measureWeight();
          client.println("Weight: " + String(weight) + " kg");
        } else if (request.indexOf("GetCurtainSensor") != -1) {
          int Curtain = sendCurtainSensorValue();
          client.println("CurtainSensor: " + String(Curtain));
        } else if (request.indexOf("LimitSwitchStatus") != -1) {
          String LimitSwitch = checkLimitSwitchStatus();
          client.println("LimitSwitchStatus: " + String(LimitSwitch));
        } else if (request.indexOf("Edgedetect") != -1) {
          String Edgesensor = Edgedetect();
          client.println("Edgedetect: " + String(Edgesensor));
        } else if (request.indexOf("GetAllStatus") != -1) {
          String completestatus = sendAllStatus();
          client.println("COMPLETE STATUS\n" + String(completestatus));
        } else if (request.indexOf("code_BayFrontDoorOpen") != -1) {
            String BayFrontDoorOpenstatus = BayFrontDoorOpen(customOpenDuration);
            client.println("BayFrontDoorOpenstatus\n" + String(BayFrontDoorOpenstatus));
        } else if (request.indexOf("code_BayFrontDoorClose") != -1) {
            String BayFrontDoorClosestatus = BayFrontDoorClose(customCloseDuration);
            client.println("BayFrontDoorClosestatus\n" + String(BayFrontDoorClosestatus));
        } else if (request.indexOf("code_BayRearDoorOpen") != -1) {
            String BayRearDoorOpenstatus = BayRearDoorOpen(customOpenDuration);
            client.println("BayRearDoorOpenstatus\n" + String(BayRearDoorOpenstatus));
        } else if (request.indexOf("code_BayRearDoorClose") != -1) {
            String BayRearDoorClosestatus = BayRearDoorClose(customCloseDuration);
            client.println("BayRearDoorClosestatus\n" + String(BayRearDoorClosestatus)); 
        } else {
            client.println("Unknown command");
        }
        request = "";
        delay(100); // Give some time for the client to receive the response
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}

