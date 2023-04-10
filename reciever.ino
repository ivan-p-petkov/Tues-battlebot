#include <esp_now.h>
#include <WiFi.h>

//Right motor 
int rightMotorPin1=13;
int rightMotorPin2=12;
//Left motor
int leftMotorPin1=14;
int leftMotorPin2=27;

const int RELAY_PIN = 2;
const int RELAY_PIN2 = 4;

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
unsigned long lastRecvTime = 0;
  
struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte switchPressed;
};
PacketData receiverData;


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue + "  " + receiverData.switchPressed;
  Serial.println(inputData);

  if (receiverData.switchPressed == true)
  {
    digitalWrite(RELAY_PIN, HIGH);
    delay(1000);
  }
  
  digitalWrite(RELAY_PIN, LOW);

  throttleAndSteeringMovements();
  
  lastRecvTime = millis();   
}

void throttleAndSteeringMovements()
{
  int throttle = map(receiverData.yAxisValue, 254, 0, -255, 255);
  int steering = map(receiverData.xAxisValue, 0, 254, -255, 255);  

  // Throttle movements
  if (throttle > 100) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else if (throttle < -100) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  // Steering movements
  if (steering > 100) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else if (steering < -100) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
}


 


void setUpPinModes()
{
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);

}


void setup() 
{
  setUpPinModes();

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() 
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(RELAY_PIN2, LOW);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(leftMotorPin2, LOW);
    
  }
}
