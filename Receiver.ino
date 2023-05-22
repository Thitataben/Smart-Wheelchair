#include <WiFi.h>
#include <PubSubClient.h>
const int enbA = 2;
const int enbB = 4;
const int IN1 = 26;    //Right Motor  (-)
const int IN2 = 27;    //Right Motor (+)
const int IN3 = 14;    //Left Motor  (+)
const int IN4 = 12;    //Right Motor (-)

int RightSpd = 130;
int LeftSpd = 130;

int incomingByte = 0;

/* WiFi */
const char* ssid = "....";        // WiFi name
const char* password = "....";    // WiFi password

/* NETPIE */
const char* mqtt_client = "....";     // Client id
const char* mqtt_username = "....";   // Token
const char* mqtt_password = "....";   // Secret

WiFiClient espClient;
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
PubSubClient client(espClient);
char msg[100];

String byteToString(byte* payload, unsigned int length_payload){ char buffer_payload[length_payload+1] = {0};
  memcpy(buffer_payload, (char*)payload, length_payload);
  return String(buffer_payload);
}

String charStarToString(char* payload){ // can not use this in serailization or deserailization because global/local variable problem
  String buffer=payload;
  return buffer;
}

String constCharStarToString(const char* payload){ // can not use this in serailization or deserailization because global/local variable problem
  String buffer =payload;
  return buffer;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String ms=byteToString(payload, length);  //massage
  String t=charStarToString(topic);         //topic
  if(t=="@msg/gesture"){
    if (ms == "gesture1"){
      //forward
      analogWrite(enbA,  RightSpd);
      analogWrite(enbB, LeftSpd);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4,  LOW);
    }
    else if (ms == "gesture2"){
      //backward
      analogWrite(enbA, RightSpd);
      analogWrite(enbB, LeftSpd);
      digitalWrite(IN1,  LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4,  HIGH);
    }
    else if (ms == "gesture3"){
      //left
      analogWrite(enbA,  RightSpd);
      analogWrite(enbB, LeftSpd);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4,  HIGH);
    }
    else if (ms == "gesture4"){
      //right
      analogWrite(enbA,  RightSpd);
      analogWrite(enbB, LeftSpd);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4,  LOW);
    }
    else if (ms == "RndNoise"){
      //stop
      analogWrite(enbA, 0);
      analogWrite(enbB,  0);  
    }
    Serial.println(ms);
  }
}

void setup(){
  Serial.begin(115200);
  // setup WiFi
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  delay(100);
  Serial.println();
  Serial.println("WiFi connected");
  // setup netpie
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.connect(mqtt_client, mqtt_username, mqtt_password);
  client.subscribe("@msg/gesture"); //sub auth
  delay(100);

  pinMode(enbA, OUTPUT);
  pinMode(enbB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
            
}

void loop(){
  if(client.connected()) {
    client.loop();
  } 
  else {
    if(WiFi.status() == WL_CONNECTED) {
      client.disconnect();
      client.connect(mqtt_client, mqtt_username, mqtt_password);
      client.subscribe("@msg/gesture");
      //Serial.println("reconnected to Netpie again");
      delay(100);
    } 
    else {
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      //Serial.println("reconnected to WiFi again");
      delay(100);
    }
  }
}
