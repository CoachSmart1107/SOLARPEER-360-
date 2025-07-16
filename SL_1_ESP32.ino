#define BLYNK_TEMPLATE_ID "TMPL2eoo_pNaq"
#define BLYNK_TEMPLATE_NAME "SOLAR PEER 360"
#define BLYNK_AUTH_TOKEN "68WgNT8MS9Avms0BgSD0Sm4n4j39SR3S"

#define BLYNK_PRINT Serial

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;

// WiFi credentials
char ssid[] = "projects";
char pass[] = "projects";

BlynkTimer timer;

// Pin Definitions
#define LDR1 34
#define LDR2 35
#define ANALOG_IN_PIN 32
#define CURRENT_SENSOR_PIN 33
#define SERVO_PIN 13
#define RELAY1_PIN 26
#define RELAY2_PIN 27
#define ERROR_MARGIN 10

const float R1 = 30000.0;
const float R2 = 7500.0;
const float REF_VOLTAGE = 3.3;
const int ADC_RESOLUTION = 4095;
const float ACS712_SENSITIVITY = 0.185;
const float ACS712_ZERO = REF_VOLTAGE / 2;

LiquidCrystal_I2C lcd(0x27, 16, 4);
Servo servo;

int Spoint = 90;
unsigned long lastEnergyCalc = 0;
float totalEnergy = 0.0; // in Watt-hours (Wh)

void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);

  lcd.init();
  lcd.backlight();

  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  pinMode(ANALOG_IN_PIN, INPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  digitalWrite(RELAY1_PIN, LOW);  // OFF
  digitalWrite(RELAY2_PIN, LOW);  // OFF

  servo.attach(SERVO_PIN);
  servo.write(Spoint);

  lcd.setCursor(5, 0);
  lcd.print("WELCOME");
  lcd.setCursor(7, 1);
  lcd.print("TO");
  lcd.setCursor(17, 0);
  lcd.print("SOLAR PEER 360");
  lcd.setCursor(1, 3);
  lcd.print("SYSTEM");
  delay(4000);
  lcd.clear();
}

// Relay control from Blynk switches
BLYNK_WRITE(V2) {
  int value = param.asInt();
  digitalWrite(RELAY1_PIN, value);
}

BLYNK_WRITE(V3) {
  int value = param.asInt();
  digitalWrite(RELAY2_PIN, value);
}

void loop() {
  Blynk.run();

  // Voltage Reading
  int adc_value = analogRead(ANALOG_IN_PIN);
  float adc_voltage = (adc_value * REF_VOLTAGE) / ADC_RESOLUTION;
  float in_voltage = adc_voltage / (R2 / (R1 + R2));

  // Current Reading
  int raw_current = analogRead(CURRENT_SENSOR_PIN);
  float sensor_voltage = (raw_current * REF_VOLTAGE) / ADC_RESOLUTION;
  float current = (sensor_voltage - ACS712_ZERO) / ACS712_SENSITIVITY;

  // LDRs
  int ldr1 = analogRead(LDR1);  // East
  int ldr2 = analogRead(LDR2);  // West
  int diff = abs(ldr1 - ldr2);

  if (diff > ERROR_MARGIN) {
    if (ldr1 > ldr2) {
      Spoint--;
    } else {
      Spoint++;
    }
    Spoint = constrain(Spoint, 0, 180);
    servo.write(Spoint);
  }

  // Energy Calculation (W = V * I, E += W * dt)
  unsigned long now = millis();
  float power = in_voltage * current;  // Watts
  float deltaTime = (now - lastEnergyCalc) / 3600000.0;  // convert ms to hours
  totalEnergy += power * deltaTime;
  lastEnergyCalc = now;

  // LCD Display
  lcd.setCursor(1, 0);
  lcd.print("SOLAR PEER 360");

  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(in_voltage, 2);
  lcd.print("V ");

  lcd.print("I:");
  lcd.print(current, 2);
  lcd.print("A");

  lcd.setCursor(0, 2);
  lcd.print("Angle:");
  lcd.print(Spoint);
  lcd.print((char)223);
  lcd.print(" ");

  lcd.setCursor(0, 3);
  lcd.print("Energy:");
  lcd.print(totalEnergy, 2);
  lcd.print("Wh ");

  // Send data to Blynk
  Blynk.virtualWrite(V5, ldr1);            // East LDR
  Blynk.virtualWrite(V6, ldr2);            // West LDR
  Blynk.virtualWrite(V0, in_voltage);      // Voltage
  Blynk.virtualWrite(V1, current);         // Current
  Blynk.virtualWrite(V4, totalEnergy);     // Energy

  // Serial Debug
  Serial.print("V: "); Serial.print(in_voltage);
  Serial.print(" V | I: "); Serial.print(current);
  Serial.print(" A | E: "); Serial.print(totalEnergy); Serial.println(" Wh");
  Serial.print("LDR1: "); Serial.print(ldr1);
  Serial.print(" | LDR2: "); Serial.println(ldr2);
  Serial.println("------");

  delay(1000);  // Adjust as needed
}