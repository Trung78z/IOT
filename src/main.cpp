#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include "WiFi.h"
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>

#define BLYNK_TEMPLATE_ID "TMPL6ffw92mzV"
#define BLYNK_TEMPLATE_NAME "esp32"
#define BLYNK_AUTH_TOKEN "SgQV1JUd1g7iwbsOBkMEIawcJygAXAiO"

#define outpled 14
#define inpdht11 12
#define inplm35 13
#define ONE_WIRE_BUS 15
#define outpledesp 2
#define touch 4

#define ssid "MSI"
#define pass "11112222"
WidgetLED LED(V2);
DHT my_sensor(inpdht11, DHT11);
OneWire oneWire(ONE_WIRE_BUS);

BLYNK_WRITE(V3)
{
  int button = param.asInt();
  if (button == 1)
  {
    digitalWrite(outpledesp, HIGH);
    digitalWrite(outpled, LOW);
    LED.on();
  }
  else
  {
    digitalWrite(outpledesp, LOW);
    digitalWrite(outpled, HIGH);
    LED.off();
  }
}
BLYNK_WRITE(V3);
void connectwifi();
float getTemperature();
void inp4();

void IRAM_ATTR isr()
{
  digitalWrite(outpled, HIGH);
  digitalWrite(outpledesp, LOW);
  Blynk.disconnect();
  Blynk.connect();
}

void setup()
{
  pinMode(outpledesp, OUTPUT);
  pinMode(outpled, OUTPUT);
  digitalWrite(outpled, LOW);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  attachInterrupt(touch, isr, CHANGE);
  Serial.begin(115200);
  my_sensor.begin();
  connectwifi();
}

void loop()
{
  // inp4();
  Blynk.run();
  float a = my_sensor.readHumidity();
  float b = my_sensor.readTemperature();
  float c = my_sensor.readTemperature(1);
  // float temperature = getTemperature();
  Blynk.virtualWrite(V1, a);
  Blynk.virtualWrite(V0, b);
  // Serial.print("Temperature: ");
  // Serial.print(temperature);
  Serial.print("°C\n");
  Serial.print("Nhiet do C: ");
  Serial.print(b);
  Serial.print("*C\tNhiet do F: ");
  Serial.print(c);
  Serial.print("*F\tDo am: ");
  Serial.print(a);
  Serial.println("");
  delay(500);
}

void inp4()
{
  if (touch)
  {

    digitalWrite(outpled, LOW);
    digitalWrite(outpledesp, HIGH);
    LED.on();
  }
  else
  {
    {
      digitalWrite(outpled, HIGH);
      digitalWrite(outpledesp, LOW);
      LED.off();
    }
  }
}
float getTemperature()
{
  byte i;
  byte data[12];
  byte addr[8];

  if (!oneWire.search(addr))
  {
    // No more sensors on the bus
    oneWire.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    // CRC is not valid
    return -1000;
  }

  if (addr[0] != 0x28)
  {
    // Not a DS18B20 sensor
    return -1000;
  }

  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0x44); // Start temperature conversion

  delay(7500); // Wait for conversion to complete

  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0xBE); // Read scratchpad

  for (i = 0; i < 9; i++)
  {
    data[i] = oneWire.read();
  }

  int16_t rawTemperature = (data[1] << 8) | data[0];
  byte resolution = ((data[4] & 0x60) >> 5) + 9;
  float temperature = (float)rawTemperature / 16.0;

  return temperature;
}
void connectwifi()
{

  Serial.print("Connecting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("..");
    delay(100);
  }
  Serial.print("Connected! ");
  Serial.println(WiFi.localIP());
  Serial.println("----------------");
}
