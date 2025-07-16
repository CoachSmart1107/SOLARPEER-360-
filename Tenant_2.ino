#include <WhatabotAPIClient.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ========== WHATABOT SETTINGS ==========
#define WHATABOT_API_KEY "d5f52194-61ac-49c0-a24f"
#define WHATABOT_CHAT_ID "2348088950364"
#define WHATABOT_PLATFORM "whatsapp"

WiFiManager wifiManager;
WhatabotAPIClient whatabotClient(WHATABOT_API_KEY, WHATABOT_CHAT_ID, WHATABOT_PLATFORM);

#define AP_SSID "WhatabotAPI"
#define AP_PASS "whatabotapi"

// ========== IR SENSOR PINS ==========
const int irSensorPin1 = D5; // GPIO14
const int irSensorPin2 = D6; // GPIO12
const int irSensorPin3 = D7; // GPIO13

bool sent1 = false;
bool sent2 = false;
bool sent3 = false;

// ========== LCD ==========
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Change to 0x27 if needed

// ========== VOLTAGE SENSING ==========
int ANALOG_IN_PIN = A0;
float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 30000.0; // in ohms
float R2 = 7500.0;  // in ohms
float ref_voltage = 3.1;
int adc_value = 0;

// ========== CURRENT SENSING ==========
const int CURRENT_SENSOR_PIN = A0;  // Shared with voltage sensor
const float MAX_CURRENT = 2.0;      // Max current for ACS712-2A

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);

  // LCD init
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("SOLAR PEER 360");
  lcd.setCursor(3, 1);
  lcd.print("TENANT TWO");
  delay(3000);
  lcd.clear();

  // IR sensor setup
  pinMode(irSensorPin1, INPUT);
  pinMode(irSensorPin2, INPUT);
  pinMode(irSensorPin3, INPUT);

  // WiFi and WhatsApp bot
  wifiManager.autoConnect(AP_SSID, AP_PASS);
  whatabotClient.begin();
  whatabotClient.onMessageReceived(onMessageReceived);
  whatabotClient.onServerResponseReceived(onServerResponseReceived);
}

// ========== MAIN LOOP ==========
void loop() {
  whatabotClient.loop();

  // ====== IR Sensor Detection ======
  checkIRSensor(irSensorPin1, sent1, 1);
  checkIRSensor(irSensorPin2, sent2, 2);
  checkIRSensor(irSensorPin3, sent3, 3);

  // ====== Voltage & Current Reading ======
  adc_value = analogRead(ANALOG_IN_PIN);
  adc_voltage = (adc_value * ref_voltage) / 1023.0;
  in_voltage = adc_voltage / (R2 / (R1 + R2));

  float rawCurrent = analogRead(CURRENT_SENSOR_PIN);
  float current = (rawCurrent / 1023.0) * MAX_CURRENT;

  // ====== LCD Display ======
  lcd.setCursor(3, 0);
  lcd.print("TENANT TWO");
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(in_voltage, 2);
  lcd.print("V I:");
  lcd.print(current, 2);
  lcd.print("A");

  delay(1000); // Delay for stability
}

// ========== IR SENSOR FUNCTION ==========
void checkIRSensor(int pin, bool &flag, int sensorNumber) {
  int value = digitalRead(pin);
  if (value == LOW && !flag) {
    flag = true;
    String msg = "⚠️ Obstacle detected by IR Sensor " + String(sensorNumber) + " at TENANT 2.";
    Serial.println(msg);
    whatabotClient.sendMessageWS(msg);
  } else if (value == HIGH && flag) {
    flag = false;  // Reset flag when object is cleared
  }
}

// ========== CALLBACKS ==========
void onServerResponseReceived(String message) {
  Serial.println("Server: " + message);
}

void onMessageReceived(String message) {
  Serial.println("User: " + message);
  whatabotClient.sendMessageWS("Pong: " + message);
}
