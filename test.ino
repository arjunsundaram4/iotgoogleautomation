#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <LM35.h>
#include "DHT.h"        // including the library of DHT11 temperature and humidity sensor
#define DHTTYPE DHT11
#define Relay1  2
#define Relay2  16
#define WLAN_SSID       "Arjun_2"             // Your SSID
#define WLAN_PASS       "arjunsundaram4"        // Your password

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "narenks"            // Replace it with your username
#define AIO_KEY         "691c6f36a5424cf1999c3e987d99efac"   // Replace with your Project Auth Key

/************ Global State (you don't need to change this!) ******************/
#define dht_dpin 0
DHT dht(dht_dpin, DHTTYPE); 
int outputpin= A0;
float vref = 3.3;
float resolution = vref/1023;
int sensor = 13;              // the pin that the sensor is atteched to
int state = LOW; 
int val = 0;                 // variable to store the sensor status (value)

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temp");
Adafruit_MQTT_Publish photocell2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish motion = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Motion");
// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe Light1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/g-assistant"); // FeedName

Adafruit_MQTT_Subscribe fan = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/fan");

void MQTT_connect();

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(sensor, INPUT);  
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());
 

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&Light1);
  mqtt.subscribe(&fan);
}

void loop() {
 
  MQTT_connect();
  
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &Light1) {
      Serial.print(F("GotL: "));
      Serial.println((char *)Light1.lastread);
      int Light1_State = atoi((char *)Light1.lastread);
      digitalWrite(Relay1, !(Light1_State));
      
    }
       if (subscription == &fan) {
      Serial.print(F("GotF: "));
      Serial.println((char *)fan.lastread);
      int fan_State = atoi((char *)fan.lastread);
      digitalWrite(Relay2, !(fan_State));
      
    }
  }
  


//  float temperature = analogRead(outputpin);
// temperature = (temperature*resolution);
// temperature = temperature*100 + 20;
//  if(temperature < 25.00){
//    digitalWrite(Relay2,1);
//    }else{
//      digitalWrite(Relay2,0);
//    }
float h = dht.readHumidity();
float t = dht.readTemperature();
Serial.println(t);
Serial.println(h);
 if(t > 27.00){
    digitalWrite(Relay2,HIGH);
    }else{
      digitalWrite(Relay2,LOW);
    } 
Serial.print(F("\nSending Temp val "));
  if (! photocell.publish(t)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  Serial.print(F("\nSending Humidity val "));
  if (! photocell2.publish(h)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  val = digitalRead(sensor);   // read sensor value
  if (! motion.publish(val)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  if (val == HIGH) {           // check if the sensor is HIGH
    digitalWrite(Relay1, HIGH);   // turn LED ON 
    
    if (state == LOW) {
      Serial.println("Motion detected!"); 
      state = HIGH;       // update variable state to HIGH
    }
  } 
  else {
      digitalWrite(Relay1, LOW); // turn LED OFF
      if (state == HIGH){
        Serial.println("Motion stopped!");
        state = LOW;       // update variable state to LOW
    }
    Serial.print(F("\nSending Motion Val "));
  }
  delay(500);

}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  
}
