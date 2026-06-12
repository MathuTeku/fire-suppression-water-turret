#include <SoftwareSerial.h>

SoftwareSerial sim900(9, 10); // RX, TX

String phoneNumber = "+639182719703";
String smsMessage = "Test SMS from Jetson via Arduino SIM900A";

const int MAX_RETRIES = 3;

const int RELAY_PIN = 7;

//#define X_DIR_PIN 2
//#define X_STEP_PIN 3

//#define Y_DIR_PIN 4
//#define Y_STEP_PIN 5

// Most relay modules are active LOW.
// If your relay works opposite, change this to false.
const bool RELAY_ACTIVE_LOW = true;


void moveMotor(int dirPin, int stepPin, bool dir, int steps) {

  digitalWrite(dirPin, dir);

  for(int i=0;i<steps;i++) {

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(200);

    digitalWrite(stepPin, LOW);
    delayMicroseconds(200);
  }
}

void relayOn() {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN, LOW);
  } else {
    digitalWrite(RELAY_PIN, HIGH);
  }

  Serial.println("RELAY_ON");
}

void relayOff() {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }

  Serial.println("RELAY_OFF");
}

String readResponse(unsigned long timeout) {
  String response = "";
  unsigned long start = millis();

  while (millis() - start < timeout) {

    delay(1);

    while (sim900.available()) {
      char c = sim900.read();
      response += c;

      // Do not print every SIM900 character.
      // This keeps Jetson serial communication cleaner and faster.
    }
  }

  return response;
}

bool sendAT(String command, String expected, unsigned long timeout) {
  sim900.println(command);
  String response = readResponse(timeout);
  return response.indexOf(expected) != -1;
}

bool waitForPrompt(unsigned long timeout) {
  String response = "";
  unsigned long start = millis();

  while (millis() - start < timeout) {
    while (sim900.available()) {
      char c = sim900.read();
      response += c;

      if (response.indexOf(">") != -1) {
        return true;
      }
    }
  }

  return false;
}

bool checkNetwork() {
  sim900.println("AT+CREG?");
  String response = readResponse(3000);

  if (response.indexOf("0,1") != -1 || response.indexOf("0,5") != -1) {
    Serial.println("NETWORK_OK");
    return true;
  }

  Serial.println("NETWORK_FAIL");
  return false;
}

bool checkSignal() {
  sim900.println("AT+CSQ");
  String response = readResponse(3000);

  int colonIndex = response.indexOf(":");
  int commaIndex = response.indexOf(",");

  if (colonIndex != -1 && commaIndex != -1) {
    int signalLevel = response.substring(colonIndex + 2, commaIndex).toInt();

    Serial.print("SIGNAL:");
    Serial.println(signalLevel);

    if (signalLevel >= 10 && signalLevel <= 31) {
      return true;
    }
  }

  Serial.println("SIGNAL_FAIL");
  return false;
}

bool sendSMS(String number, String message) {
  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    Serial.print("SMS_ATTEMPT:");
    Serial.println(attempt);

    if (!checkNetwork()) {
      delay(1000);
      continue;
    }

    if (!checkSignal()) {
      delay(1000);
      continue;
    }

    if (!sendAT("AT+CMGF=1", "OK", 3000)) {
      Serial.println("SMS_MODE_FAIL");
      continue;
    }

    sim900.print("AT+CMGS=\"");
    sim900.print(number);
    sim900.println("\"");

    if (!waitForPrompt(5000)) {
      Serial.println("NO_PROMPT");
      sim900.write(27); // ESC cancel
      delay(1000);
      continue;
    }

    sim900.print(message);
    delay(300);
    sim900.write(26); // CTRL + Z

    String response = readResponse(5000);

    if (response.indexOf("+CMGS") != -1 && response.indexOf("OK") != -1) {
      Serial.println("SMS_SUCCESS");
      return true;
    }

    Serial.println("SMS_FAIL_RETRYING");
    delay(2000);
  }

  Serial.println("SMS_FAILED");
  return false;
}

bool makeCall(String number) {
  if (!checkNetwork()) return false;
  if (!checkSignal()) return false;

  sim900.print("ATD");
  sim900.print(number);
  sim900.println(";");

  readResponse(3000);

  Serial.println("CALL_STARTED");
  return true;
}

void hangUp() {
  sim900.println("ATH");
  readResponse(1000);
  Serial.println("CALL_ENDED");
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);
  sim900.begin(9600);

  pinMode(RELAY_PIN, OUTPUT);
  relayOff();

  delay(3000);

  Serial.println("ARDUINO_SIM900_RELAY_READY");

  sendAT("AT", "OK", 3000);
  sendAT("AT+CMGF=1", "OK", 3000);

  //pinMode(X_DIR_PIN, OUTPUT);
  //pinMode(X_STEP_PIN, OUTPUT);

  //pinMode(Y_DIR_PIN, OUTPUT);
  //pinMode(Y_STEP_PIN, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "SMS") {
      sendSMS(phoneNumber, smsMessage);
    }

    else if (command == "CALL") {
      makeCall(phoneNumber);
    }

    else if (command == "HANG") {
      hangUp();
    }

    else if (command == "RELAY_ON") {
      relayOn();
    }

    else if (command == "RELAY_OFF") {
      relayOff();
    }

    else if (command.startsWith("NUMBER:")) {
      phoneNumber = command.substring(7);
      phoneNumber.trim();

      Serial.print("NUMBER_SET:");
      Serial.println(phoneNumber);
    }

    else if (command.startsWith("MESSAGE:")) {
      smsMessage = command.substring(8);
      smsMessage.trim();

      Serial.println("MESSAGE_SET");
    }

    else if (command == "STATUS") {
      Serial.println("ARDUINO_OK");
      checkSignal();
      checkNetwork();
    }

  /*  else if(command.startsWith("X_LEFT:")) {

      int steps = command.substring(7).toInt();

      moveMotor(X_DIR_PIN, X_STEP_PIN, LOW, steps);
    }

    else if(command.startsWith("X_RIGHT:")) {

      int steps = command.substring(8).toInt();

      moveMotor(X_DIR_PIN, X_STEP_PIN, HIGH, steps);
    }
  */
  
    else {
      Serial.println("UNKNOWN_COMMAND");
    }
  }


}
