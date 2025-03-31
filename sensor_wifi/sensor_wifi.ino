#include <WiFi.h>
#include "BluetoothSerial.h"
#include <Firebase_ESP_Client.h>  
#include <addons/TokenHelper.h>   
#include <addons/RTDBHelper.h>   
#include <DFRobot_DHT11.h>
DFRobot_DHT11 DHT;
#include <time.h>


// -----------------------
// Credentials
// -----------------------
#define WIFI_SSID       "aaaaaaaa"
#define WIFI_PASSWORD   "aaaaaaaa"
#define FIREBASE_HOST   "https://smart-food-inventory-tracker-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH   "inventory" // e.g. "xxxxxxxxxxxxx"


// utp-5 config
static const char* ntpServer        = "pool.ntp.org";
static const long  gmtOffset_sec    = -5*3600;     // Change as needed
static const int   daylightOffset_s = 3600;     // Change as needed

//Bluetooth adapter

BluetoothSerial SerialBT;

// dht11 sensor
#define DHT11_PIN 4
//mq135 sensor
#define MQ_PIN                 34    // ADC input pin on ESP32
#define RL_VALUE               1     // Load resistor in kilo-ohms
#define RO_CLEAN_AIR_FACTOR    3.6   // Sensor resistance in clean air factor
//Bluetooth Button
#define BT_Pin 23

// Calibration/reading settings
#define CALIBARAION_SAMPLE_TIMES    50
#define CALIBRATION_SAMPLE_INTERVAL 500
#define READ_SAMPLE_INTERVAL        50
#define READ_SAMPLE_TIMES           5

// Gas 
#define GAS_LPG    0
#define GAS_CO     1
#define GAS_SMOKE  2

// Gas curves
float LPGCurve[3]   = {2.3,  0.21, -0.47};
float COCurve[3]    = {2.3,  0.72, -0.34};
float SmokeCurve[3] = {2.3,  0.53, -0.44};

// Global baseline resistance
float Ro = 10.0;


// obj firebase
FirebaseData fbdo;       // For read/write
FirebaseAuth auth;       // Authentication
FirebaseConfig config;   // Firebase config

//functions
// -----------------------
float MQResistanceCalculation(int raw_adc);
float MQCalibration(int mq_pin);
float MQRead(int mq_pin);
int   MQGetGasPercentage(float rs_ro_ratio, int gas_id);
int   MQGetPercentage(float rs_ro_ratio, float *pcurve);

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.print("Making device discoverable...");
  makeESP32Discoverable();

  // 1) Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

 
  config.host = FIREBASE_HOST;  
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  //Configure time via NTP
  configTime(gmtOffset_sec, daylightOffset_s, ntpServer);
  
  // wait here until we get the time
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Waiting for NTP time sync...");
    delay(1000);
  }
  Serial.println("Time synchronized with NTP!");

  //Calibrate MQ in clean air
  Serial.println("Calibrating MQ sensor in clean air...");
  Ro = MQCalibration(MQ_PIN);
  Serial.println("Calibration done.");
  Serial.print("Ro = ");
  Serial.print(Ro);
  Serial.println(" kΩ");

  
}

void loop()
{
  
  // 1) Read MQ
  // -----------------------
  float mqResistance = MQRead(MQ_PIN); 
  float ratio        = mqResistance / Ro;
  int lpg_ppm        = MQGetGasPercentage(ratio, GAS_LPG);
  int co_ppm         = MQGetGasPercentage(ratio, GAS_CO);
  int smoke_ppm      = MQGetGasPercentage(ratio, GAS_SMOKE);

  if(lpg_ppm>1000){
    lpg_ppm=1000;
  }
  if(co_ppm>1000){
    co_ppm=1000;
  }
  if(smoke_ppm>1000){
    smoke_ppm=1000;
  }

  Serial.print("LPG: ");
  Serial.print(lpg_ppm);
  Serial.print(" ppm | CO: ");
  Serial.print(co_ppm);
  Serial.print(" ppm | SMOKE: ");
  Serial.print(smoke_ppm);
  Serial.println(" ppm");
  
  delay(300);
 
  // 2) Read DHT11
  // -----------------------
  DHT.read(DHT11_PIN);
  float temperature = DHT.temperature;
  float humidity    = DHT.humidity;

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C | Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  delay(300);
  // 3) Get the current time
  // -----------------------
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }

  // Format the time 
  char formattedTime[32];
  strftime(formattedTime, sizeof(formattedTime), "%Y-%m-%d %H:%M:%S", &timeinfo);

  
  // 4) Build JSON for Firebase
  // -----------------------
  FirebaseJson json;
  json.set("temperature", temperature);
  json.set("humidity",    humidity);
  json.set("lpg",         lpg_ppm);
  json.set("co",          co_ppm);
  json.set("smoke",       smoke_ppm);
  json.set("time",        formattedTime);

  // pushJSON => each reading gets a unique key
  String path = "inventory"; 
  if (Firebase.RTDB.pushJSON(&fbdo, path, &json)) {
    Serial.println("Data pushed to Firebase with a unique key!");
  } else {
    Serial.print("Error sending data to Firebase: ");
    Serial.println(fbdo.errorReason());
  }

  delay(10000);
}

// -----------------------
// MQ Sensor Functions
// -----------------------
float MQResistanceCalculation(int raw_adc)
{
  if (raw_adc == 0) raw_adc = 1;  // Avoid /0
  return ( (float)RL_VALUE * (1023.0 - raw_adc) / (float)raw_adc );
}

float MQCalibration(int mq_pin)
{
  float val = 0;
  for (int i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val /= (float)CALIBARAION_SAMPLE_TIMES;
  val /= RO_CLEAN_AIR_FACTOR; 
  return val;
}

float MQRead(int mq_pin)
{
  float rs = 0;
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs /= (float)READ_SAMPLE_TIMES;
  return rs;
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  switch (gas_id) {
    case GAS_LPG:   return MQGetPercentage(rs_ro_ratio, LPGCurve);
    case GAS_CO:    return MQGetPercentage(rs_ro_ratio, COCurve);
    case GAS_SMOKE: return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }
  return 0;
}

int MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  float val = ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0];
  return (int)pow(10, val);
}

void makeESP32Discoverable()
{
  bool success = SerialBT.begin("ESPMOHA");

if (success) {
  Serial.println("✅ Bluetooth successfully initialized and discoverable as ESPMOHA");
} else {
  Serial.println("❌ Bluetooth initialization FAILED");
}
}