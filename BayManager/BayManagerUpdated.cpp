#include <WiFi.h>
#include "HX711.h"

const int NUM_SENSORS = 16;
const int SENSOR_PINS[NUM_SENSORS] = {4, 7, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49};
const int TRIGGER_PINS[NUM_SENSORS] = {2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47};
const int ECHO_PINS[NUM_SENSORS] = {3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48};

HX711 scale;
float calibrationFactor = 20.775;
const char* ssid = "QPR";
const char* password = "ABCDEFGH";

const int DOOR_OPEN_DURATION = 1500;
const int DOOR_CLOSE_DURATION = 1300;
const int SAFETY_CURTAIN_DELAY = 100;

int doorStatus[2] = {0, 0}; // 0 for closed, 1 for open
int curtainSensorStatus = 1;
int limitSwitchStatus[4] = {1, 1, 1, 1};
bool edgeDetectionDone = false;

WiFiServer wifiServer(80);

// -------------------------Setup-------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  scale.begin(50, 51);
  scale.set_scale();
  scale.tare();
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], OUTPUT);
    digitalWrite(SENSOR_PINS[i], HIGH);
    pinMode(TRIGGER_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
  }
  
  int relayPins[4] = {A5, A6, A7, 0};
  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }

  int limitSwitchPins[4] = {A0, A1, A2, A3};
  for (int i = 0; i < 4; i++) {
    pinMode(limitSwitchPins[i], INPUT_PULLUP);
  }

  int edgeDetectPins[3] = {1, 52, 53};
  for (int i = 0; i < 3; i++) {
    pinMode(edgeDetectPins[i], INPUT_PULLUP);
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  wifiServer.begin();
}

// -------------------------Command Handler-------------------------------------------------------------------------

void handleCommand(String command, WiFiClient client) {
  String response;
  if (command.indexOf("GetHeight") != -1) {
    response = "Height: " + measureHeight() + " cm";
  } else if (command.indexOf("GetWeight") != -1) {
    response = "Weight: " + measureWeight() + " kg";
  } else if (command.indexOf("GetCurtainSensor") != -1) {
    response = "CurtainSensor: " + String(sendCurtainSensorValue());
  } else if (command.indexOf("LimitSwitchStatus") != -1) {
    response = "LimitSwitchStatus: " + checkLimitSwitchStatus();
  } else if (command.indexOf("Edgedetect") != -1) {
    response = "Edgedetect: " + Edgedetect();
  } else if (command.indexOf("GetAllStatus") != -1) {
    response = "COMPLETE STATUS\n" + sendAllStatus();
  } else {
    String strValue;
    int intValue;
    parseStringAndInteger(command, strValue, intValue);
    if (strValue.length() > 0) {
      if (strValue.equals("code_BayFrontDoorOpen")) {
        response = BayDoorOpen(intValue);
      } else if (strValue.equals("code_BayFrontDoorClose")) {
        response = BayDoorClose(intValue);
      } else if (strValue.equals("code_BayRearDoorOpen")) {
        response = BayDoorOpen(intValue);
      } else if (strValue.equals("code_BayRearDoorClose")) {
        response = BayDoorClose(intValue);
      }
    } else {
      response = "Unknown command";
    }
  }
  client.println(response);
}

// -------------------------Measure Height------------------------------------------------------------------------
String measureHeight() {
  // Implement height measurement logic
  // Calculate and return height measurement
}

// -------------------------Measure Weight------------------------------------------------------------------------
String measureWeight() {
  scale.set_scale(calibrationFactor);
  float weight = scale.get_units(5);
  String status = "weight" + String(weight);
  return status;
}

int sendCurtainSensorValue() {
  curtainSensorStatus = digitalRead(curtainSensor);
  if(curtainSensorStatus == 0){
    return 0;
  }
  else{
    return 1;
  }
}

// -------------------------Status Handler------------------------------------------------------------------------

String checkLimitSwitchStatus() {
  for (int i = 0; i < 4; i++) {
    limitSwitchStatus[i] = digitalRead(A0 + i);
  }
  String status = "FrontDoortoplimitSwitchstatus: " + String(limitSwitchStatus[0]) + "\n";
  status += "FrontDoorbottomlimitSwitchstatus: " + String(limitSwitchStatus[1]) + "\n";
  status += "RearDoortoplimitSwitchstatus: " + String(limitSwitchStatus[2]) + "\n";
  status += "RearDoorbottomlimitSwitchstatus: " + String(limitSwitchStatus[3]);
  return status;
}

String Edgedetect() {
  if (!edgeDetectionDone) {
    // Implement edge detection logic
    // Create a formatted string with all edge detector statuses
    String status = String(digitalRead(1)) + "," + String(digitalRead(52)) + "," + String(digitalRead(53));
    return status;
  }
  return "";
}

String sendAllStatus() {
  // Implement logic to send all status
  String status = "";
  if (doorStatus[0] == 1) {
    status += "FrontbayDoorStatus: Open\n";
  } else if (doorStatus[0] == 0) {
    status += "FrontbayDoorStatus: Closed\n";
  }
  if (doorStatus[1] == 1) {
    status += "RearbayDoorStatus: Open\n";
  } else if (doorStatus[1] == 0) {
    status += "RearbayDoorStatus: Closed\n";
  }
  status += "CurtainSensor: " + String(curtainSensorStatus) + "\n";
  status += "EdgeDetection: " + Edgedetect() + "\n";
  status += "Weight: " + measureWeight() + " \n";
  status += "Height: " + measureHeight() + " ";
  return status;
}

// -------------------------Bay Door Handle------------------------------------------------------------------------
String BayDoorOpen(int customDuration) {
  unsigned long doorOpenStartTime = 0;
  digitalWrite(A5, HIGH);
  digitalWrite(A6, LOW);
  
  doorOpenStartTime = millis(); // Record the start time
  String status = "DoorOpenStartTime" + String(doorOpenStartTime);
  while (true) {
    // Check if the limit switch is pressed
    if (digitalRead(A0) == LOW) {
      break; // Exit the loop if the limit switch is pressed
    }
    
    // Check if the timer has reached the custom duration
    if (millis() - doorOpenStartTime >= customDuration) {
      status += "DoorTopLimitSwitchStatus: false\n";
      break; // Exit the loop if the timer exceeds the custom duration
    }
  }
  digitalWrite(A5, LOW);
  digitalWrite(A6, LOW);
  
  doorStatus[0] = 1;
  status += "DoorStatus: Open";
  return status;
}

String BayDoorClose(int customDuration) {
  unsigned long doorCloseStartTime = 0;
  digitalWrite(A5, LOW);
  digitalWrite(A6, HIGH);
  doorCloseStartTime = millis(); // Record the start time
  String status = "DoorCloseStartTime" + String(doorCloseStartTime);
  while (true) {
    // Check if the limit switch is pressed
    if (digitalRead(A1) == LOW) {
      break; // Exit the loop if the limit switch is pressed
    }
    if (digitalRead(curtainSensor) == 0) {
      status += "CurtainSensor: True\n";
      delay(SAFETY_CURTAIN_DELAY);
      BayDoorOpen(DOOR_OPEN_DURATION);
    }
    // Check if the timer has reached the custom duration
    if (millis() - doorCloseStartTime >= customDuration) {
      status += "DoorBottomLimitSwitchStatus: false\n";
      break; // Exit the loop if the timer exceeds the custom duration
    }
  }
  digitalWrite(A5, LOW);
  digitalWrite(A6, LOW);
  
  doorStatus[0] = 0;
  status += "DoorStatus: Closed";
  return status;
}

void parseStringAndInteger(String command, String& str, int& value) {
  int separatorIndex = command.indexOf(':');
  if (separatorIndex != -1) {
    str = command.substring(0, separatorIndex);
    value = command.substring(separatorIndex + 1).toInt();
  } else {
    str = "";
    value = 0;
  }
}

void loop() {
  WiFiClient client = wifiServer.available();
  if (client) {
    while (client.connected()) {
      if (client.available() > 0) {
        String request = client.readStringUntil('\r');
        client.flush();
        handleCommand(request, client);
        delay(100);
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}