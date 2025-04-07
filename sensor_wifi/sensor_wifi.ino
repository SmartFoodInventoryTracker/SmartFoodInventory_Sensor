#include <WiFi.h>
#include "BluetoothSerial.h"
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <DFRobot_DHT11.h>
DFRobot_DHT11 DHT;
#include <Preferences.h>
#include <time.h>

// -----------------------
// Credentials
// -----------------------
#define WIFI_SSID "aaaaaaaa"
#define WIFI_PASSWORD "aaaaaaaa"
#define FIREBASE_HOST "https://smart-food-inventory-tracker-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "inventory"  // e.g. "xxxxxxxxxxxxx"


// utp-5 config
static const char *ntpServer = "pool.ntp.org";
static const long gmtOffset_sec = -5 * 3600;  // Change as needed
static const int daylightOffset_s = 3600;     // Change as needed

// dht11 sensor
#define DHT11_PIN 4
//mq135 sensor
#define MQ_PIN 34                // ADC input pin on ESP32
#define RL_VALUE 20              // Load resistor in kilo-ohms
#define RO_CLEAN_AIR_FACTOR 3.6  // Sensor resistance in clean air factor


// Calibration/reading settings
#define CALIBARAION_SAMPLE_TIMES 10
#define CALIBRATION_SAMPLE_INTERVAL 500
#define READ_SAMPLE_INTERVAL 50
#define READ_SAMPLE_TIMES 5

// Gas
#define GAS_LPG 0
#define GAS_CO 1
#define GAS_SMOKE 2
float LPGCurve[3] = { 2.3, 0.21, -0.47 };  //two points are taken from the curve.
                                           //with these two points, a line is formed which is "approximately equivalent"
                                           //to the original curve.
                                           //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float COCurve[3] = { 2.3, 0.72, -0.34 };   //two points are taken from the curve.
                                           //with these two points, a line is formed which is "approximately equivalent"
                                           //to the original curve.
                                           //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float SmokeCurve[3] = { 2.3, 0.53, -0.44 };
// Gas curves
//float LPGCurve[3]   = {2.3,  0.21, -0.47};
//float COCurve[3]    = {1,0.36,-0.35};
//float SmokeCurve[3] = {1,  0.41, -0.40};            // NH3

// Global baseline resistance
float Ro = 10.0;


// obj firebase
FirebaseData fbdo;      // For read/write
FirebaseAuth auth;      // Authentication
FirebaseConfig config;  // Firebase config

//functions
// -----------------------
float MQResistanceCalculation(int raw_adc);
float MQCalibration(int mq_pin);
float MQRead(int mq_pin);
int MQGetGasPercentage(float rs_ro_ratio, int gas_id);
int MQGetPercentage(float rs_ro_ratio, float *pcurve);



///////////////////////////// Bluetooth/WIFI ///////////////////////////////////
//Credentials
String SSID;
String PW;
String userId;

// Bluetooth Adapter
BluetoothSerial SerialBT;

//Preferences to store ssid and password
Preferences prefs;

void setup() {
  Serial.begin(115200);
  delay(100);

  //SetUp the nonVolatile memory
  setUpPrefs();
  //Initialize and make Bluetooth discoverable
  makeESP32Discoverable();
  //Retrieve data from bluetooth and set up ssid and pw
  storeCredentials();

  // 1) Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(SSID);
  WiFi.begin(SSID, PW);
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

void loop() {
  // 1) Read MQ
  // -----------------------
  float mqResistance = MQRead(MQ_PIN);
  float ratio = mqResistance / Ro;
  int lpg_ppm = (8.99 * MQGetGasPercentage(ratio, GAS_LPG));
  int co_ppm = (9.99 * MQGetGasPercentage(ratio, GAS_CO));
  int smoke_ppm = (7.99 * MQGetGasPercentage(ratio, GAS_SMOKE));

  //if(lpg_ppm>1000){
  //  lpg_ppm=1000;
  // }
  // if(co_ppm>1000)
  // co_ppm=1000;
  //}
  //if(smoke_ppm>1000){
  //  smoke_ppm=1000;
  // }

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
  float humidity = DHT.humidity;

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
  int temperature_condition;
  int humidity_condition;
  int lpg_ppm_condition;
  int co_ppm_condition;
  int smoke_ppm_condition;


  if (2 <= temperature && temperature < 4) {
    temperature_condition = 1;  // 1 represents the best temperature condition
  } else if (4 <= temperature && temperature < 6) {
    temperature_condition = 3;
  } else if (6 <= temperature && temperature < 8) {
    temperature_condition = 5;
  } else if (8 <= temperature && temperature < 10) {
    temperature_condition = 7;
  } else if (10 <= temperature && temperature < 12) {
    temperature_condition = 9;
  } else if (12 <= temperature && temperature < 14) {
    temperature_condition = 10;
  }

  if (40 <= humidity && humidity < 50) {
    humidity_condition = 1;  // 1 represents the best humidity condition
  } else if (50 <= humidity && humidity < 60) {
    humidity_condition = 3;
  } else if (60 <= humidity && humidity < 70) {
    humidity_condition = 5;
  } else if (70 <= humidity && humidity < 80) {
    humidity_condition = 7;
  } else if (80 <= humidity && humidity < 90) {
    humidity_condition = 9;
  } else if (90 <= humidity && humidity < 100) {
    humidity_condition = 10;
  }

  if (0 <= lpg_ppm && lpg_ppm < 100) {
    lpg_ppm_condition = 1;  // 1 represents the best LPG ppm condition
  } else if (100 <= lpg_ppm && lpg_ppm < 200) {
    lpg_ppm_condition = 3;
  } else if (200 <= lpg_ppm && lpg_ppm < 300) {
    lpg_ppm_condition = 5;
  } else if (300 <= lpg_ppm && lpg_ppm < 400) {
    lpg_ppm_condition = 7;
  } else if (400 <= lpg_ppm && lpg_ppm < 500) {
    lpg_ppm_condition = 9;
  } else if (500 <= lpg_ppm) {
    lpg_ppm_condition = 10;
  }

  if (0 <= co_ppm && co_ppm < 100) {
    co_ppm_condition = 1;  // 1 represents the best CO ppm condition
  } else if (100 <= co_ppm && co_ppm < 200) {
    co_ppm_condition = 3;
  } else if (200 <= co_ppm && co_ppm < 300) {
    co_ppm_condition = 5;
  } else if (300 <= co_ppm && co_ppm < 400) {
    co_ppm_condition = 7;
  } else if (400 <= co_ppm && co_ppm < 500) {
    co_ppm_condition = 9;
  } else if (500 <= co_ppm) {
    co_ppm_condition = 10;
  }

  if (0 <= smoke_ppm && smoke_ppm < 100) {
    smoke_ppm_condition = 1;  // 1 represents the best smoke ppm condition
  } else if (100 <= smoke_ppm && smoke_ppm < 200) {
    smoke_ppm_condition = 3;
  } else if (200 <= smoke_ppm && smoke_ppm < 300) {
    smoke_ppm_condition = 5;
  } else if (300 <= smoke_ppm && smoke_ppm < 400) {
    smoke_ppm_condition = 7;
  } else if (400 <= smoke_ppm && smoke_ppm < 500) {
    smoke_ppm_condition = 9;
  } else if (500 <= smoke_ppm) {
    smoke_ppm_condition = 10;
  }
  float overall_value = (temperature_condition + humidity_condition + co_ppm_condition + smoke_ppm_condition) / 4.0;
  int overall_condition;
  if (overall_value >= 1 && overall_value < 2) {
    overall_condition = 1;
  } else if (overall_value >= 2 && overall_value < 3) {
    overall_condition = 2;
  } else if (overall_value >= 3 && overall_value < 4) {
    overall_condition = 3;
  } else if (overall_value >= 4 && overall_value < 5) {
    overall_condition = 4;
  } else if (overall_value >= 5 && overall_value < 6) {
    overall_condition = 5;
  } else if (overall_value >= 6 && overall_value < 7) {
    overall_condition = 6;
  } else if (overall_value >= 7 && overall_value < 8) {
    overall_condition = 7;
  } else if (overall_value >= 8 && overall_value < 9) {
    overall_condition = 8;
  } else if (overall_value >= 9 && overall_value < 10) {
    overall_condition = 9;
  } else {
    overall_condition = 10;
  }
  // 4) Build JSON for Firebase
  // -----------------------
  FirebaseJson json;
  json.set("temperature", temperature);
  json.set("temperature condition", temperature_condition);
  json.set("humidity", humidity);
  json.set("humidity condition", humidity_condition);
  json.set("lpg", lpg_ppm);
  json.set("lpg condition", lpg_ppm_condition);
  json.set("co", co_ppm);
  json.set("co condition", co_ppm_condition);
  json.set("smoke", smoke_ppm);
  json.set("smoke condition", smoke_ppm_condition);
  json.set("time", formattedTime);
  json.set("overall condition", overall_condition);

  // pushJSON => each reading gets a unique key
  String path = "users/"+userId+"/fridge conditions";

  if (Firebase.RTDB.pushJSON(&fbdo, path, &json)) {
    Serial.println("Data pushed to Firebase with a unique key!");
  } else {
    Serial.print("Error sending data to Firebase: ");
    Serial.println(fbdo.errorReason());
  }

  delay(1000);
}

// -----------------------
// MQ Sensor Functions
// -----------------------
float MQResistanceCalculation(int raw_adc) {
  if (raw_adc == 0) raw_adc = 1;  // Avoid /0
  return ((float)RL_VALUE * (4095.0 - raw_adc) / (float)raw_adc);
}

float MQCalibration(int mq_pin) {
  float val = 0;
  for (int i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val /= (float)CALIBARAION_SAMPLE_TIMES;
  val /= RO_CLEAN_AIR_FACTOR;
  if (val <= 0) {
    Serial.print("FAILED CALLIBRATION: CALIBRATING AGAIN:( ");
    Serial.println(val);
    MQCalibration(mq_pin);
  }
  return val;
}

float MQRead(int mq_pin) {
  float rs = 0;
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs /= (float)READ_SAMPLE_TIMES;
  return rs;
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  switch (gas_id) {
    case GAS_LPG: return MQGetPercentage(rs_ro_ratio, LPGCurve);
    case GAS_CO: return MQGetPercentage(rs_ro_ratio, COCurve);
    case GAS_SMOKE: return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }
  return 0;
}

int MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  float val = ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0];
  return (int)pow(10, val);
}

void makeESP32Discoverable()
{
  bool success = SerialBT.begin("ESPMOHA");

  if (success) {
    Serial.println(" Bluetooth successfully initialized and discoverable as ESPMOHA");
  } else {
    Serial.println(" Bluetooth initialization FAILED");
  }
  connectedBT();
}


void connectedBT(){
  Serial.println("Waiting for a device to connect via BT");
  while (!SerialBT.hasClient()) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected succesfully in bluetooth");
}

String receiveData(){
  Serial.println("Waiting for credentials to be sent:");
  while (!SerialBT.available()) {
    delay(500);
    Serial.print(".");
  }
  String received = SerialBT.readStringUntil('\n');
  Serial.println("Received via Bluetooth");
  
  return received;
}

void storeCredentials()
{
  if(!(prefs.isKey("ssid") && prefs.isKey("pw")))
  {
   String result = receiveData();  

    int firstComma = result.indexOf(','); 
    int secondComma = result.indexOf(',', firstComma + 1);  

    // Extract the three parts using substring
    String id = result.substring(0, firstComma);
    String pw = result.substring(firstComma + 1, secondComma);
    userId = result.substring(secondComma + 1);  // until end

    prefs.putString("ssid",id);
    prefs.putString("pw",pw);
  }
  SSID=prefs.getString("ssid","Error");
  PW=prefs.getString("pw", "Error");
  prefs.end();
}

void setUpPrefs()
{
  prefs.begin("Wifi",false);
  prefs.clear();
}