/**
 * pe32me162led // Count LED pulses from ISKRA ME-162, export to MQTT.
 *
 * Components:
 * - ISKRA ME-162 electronic meter with pulse LED
 * - ESP8266MOD+Wifi
 * - Grove system (Arduino building block) analog Light Sensor
 * - attach 3VC<->VCC, GND<->GND, A0<->SIG (ignore NC/not connected)
 *
 * Building:
 * - Arduino IDE setup: add
 *   https://arduino.esp8266.com/stable/package_esp8266com_index.json
 *   to "Additional Boards Managers URL".
 * - Install "ESP8266" boards.
 * - Select "Generic ESP8266 Module" board.
 * - Install "ArduinoMqttClient" module.
 *
 * Configuration:
 * - Set Wifi SSID+password in config.h
 * - Set MQTT broker details in config.h
 * - Set up broker and read pulse values from there; by taking the pulse
 *   delta, converting to Watt and storing it. (Take the Wh delta,
 *   divide by T delta, and multiply by 3600.)
 */
#include <ArduinoMqttClient.h>
#include <ESP8266WiFi.h>

/* In config.h, you should have:
const char wifi_ssid[] = "<ssid>";
const char wifi_password[] = "<password>";
const char mqtt_broker[] = "192.168.1.2";
const int  mqtt_port = 1883;
const char mqtt_topic[] = "some/topic";
*/
#include "config.h"

#define VERSION "v0"
//#define DEBUG

/* We use the guid to store something unique to identify the device by.
 * For now, we'll populate it with the ESP8266 Wifi MAC address. */
char guid[24]; // "EUI48:11:22:33:44:55:66"

/* For the analog pulse detection, using the Arduino Grove (plug & play)
 * Light Sensor, we measure the following values:
 *
 * 0..1023 | 0..100 | resistance K | (resistance from ((1023-V)*10/V))
 * --------+--------+--------------+----------------------------------
 *       3 |   <1.0 |         3400 | Dark (lowest value)
 *     200 |   20.0 |           41 | Ambient light
 *     400 |   40.0 |           16 | Laptop display light
 *     654 |   64.0 |            6 | Bright light (highest value)
 *
 * We use the threshold_value of 3.0 (real), resistance 320K, to detect
 * a weak red LED light. (The real value 5.0 detected false negatives.)
 */
const float threshold_value = 3.0;

// Buffer counter until N milliseconds have passed.
const int publish_buffer_time = 60 * 1000; // 60s

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

int was_on;
int pulse_count;
int last_pulse_count;

unsigned long last_publish; // millis()

void ensure_wifi();
void ensure_mqtt();
void pulse();

void setup()
{
  strncpy(guid, "undefined_guid", sizeof(guid) - 1);

  // Setup (should we skip this if !DEBUG?)
  Serial.begin(115200);
  delay(500);

  // Setup GUID
  strncpy(guid, "EUI48:", 6);
  strncpy(guid + 6, WiFi.macAddress().c_str(), sizeof(guid) - (6 + 1));

  // Welcome message
  Serial.print("Booted pe32me162led " VERSION " guid ");
  Serial.println(guid);

  // Initial connect
  ensure_wifi();
  ensure_mqtt();
}

void loop()
{
  // Read from the A0 (analog) port on the ESP8266 A0 op de ESP8266 voor
  // analog values.
  float measured_value = analogRead(A0) * 100.0 / 1024.0; // 0..100
  int is_on = (measured_value > threshold_value);

  // Super simple check: if the value fired, send a pulse. But don't
  // send any additional pulses until:
  // - after a wait of 100ms
  // - and that the value has subsided
  if (is_on && !was_on) {
#ifdef DEBUG
    Serial.print("was OFF, now ON; measurement: ");
    Serial.println(measured_value);
#endif
    pulse();

    // Sleep a while (*1), so we never detect flapping during a pulse.
    // We've calculated that there should never be more than 5 pulses
    // per second (*2).
    // We could Kalman filter the values, and check those values
    // instead, but then we couldn't sleep instead. Sleeping is easier
    // and better for the environment.
    // (*1)
    // - The human visual system can process 10 to 12 images per second.
    // - Assume the blinking led pulse is at most 100ms long, but
    //   probably a lot less.
    // (*2)
    // - 75Amp * 230V = 17250W
    // - One pulse every Wh, so 17250W / 3600 = 4.8Wh
    // - So at most 1/4.8 Wh/s = max. 5 pulses
    delay(100);

    // Measure value again. It should be OFF now, after the delay().
    measured_value = analogRead(A0) * 100.0 / 1024.0; // 0..100
    was_on = (measured_value > threshold_value);
#ifdef DEBUG
    if (was_on) {
      Serial.print("was ON anyway, unexpected; measurement: ");
      Serial.println(measured_value);
    }
#endif
  } else if (!is_on && was_on) {
#ifdef DEBUG
    Serial.println("was ON, now OFF, also unexpected");
#endif
    was_on = 0;
  }

  delay(1);
}

/**
 * Record a pulse, and send updated counter to MQTT when we've exceeded
 * the buffer time.
 */
void pulse() {
  ++pulse_count;
  if (pulse_count > 0xffffff) {
    // Reset after 24bits; it will happen at device reset, or when we
    // wrap here. By doing it sooner, we'll ensure that the recipient
    // can cope.
    pulse_count = 0;
  }

#ifdef DEBUG
  Serial.print(pulse_count);
  Serial.println(" Wh PULSE");
#endif

  unsigned long now = millis();
  if (last_publish > now) {
    last_publish = 0;
  }
  unsigned long tdelta = (now - last_publish);

  if (tdelta > publish_buffer_time) {
    // Calculate watt
    int pulse_delta = (pulse_count - last_pulse_count);

    // Sanity check. If pulse_delta is negative, or if there are more
    // pulses than 50/s (which is still 10x too much), then don't publish.
    if (pulse_delta > 0 && pulse_delta < (50 * publish_buffer_time)) {
      ensure_wifi();
      ensure_mqtt();

      // Record how much time we needed for wifi/mqtt reconnect.
      // Might be useful at some point.
      unsigned long conn_time = millis() - now;

      // (Wh * 3600) == Ws; (Tms / 1000) == Ts; W == Ws / T
      float watt = (pulse_delta * 3600.0) / (tdelta / 1000.0);

      Serial.print("pushing: count ");
      Serial.print(pulse_count);
      Serial.print(", watt ");
      Serial.println(watt);

      // Use simple application/x-www-form-urlencoded format.
      mqttClient.beginMessage(mqtt_topic);
      mqttClient.print("device_id=");
      mqttClient.print(guid);
      mqttClient.print("&watt=");
      mqttClient.print(watt);
      mqttClient.print("&watt_hour_pulses=");
      mqttClient.print(pulse_count);
      mqttClient.print("&conn_time=");
      mqttClient.print(conn_time); // how much time we lost on (re)connecting
      mqttClient.endMessage();
    }
    last_pulse_count = pulse_count;
    last_publish = now;
  }
}

/**
 * Check that Wifi is up, or connect when not connected.
 */
void ensure_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifi_ssid, wifi_password);
    for (int i = 30; i >= 0; --i) {
      if (WiFi.status() == WL_CONNECTED) {
        break;
      }
      delay(1000);
      ++pulse_count; // assume we're missing a pulse
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Wifi UP on \"");
      Serial.print(wifi_ssid);
      Serial.print("\", Local IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.print("Wifi NOT UP on \"");
      Serial.print(wifi_ssid);
      Serial.println("\".");
    }
  }
}

/**
 * Check that the MQTT connection is up or connect if it isn't.
 */
void ensure_mqtt() {
  mqttClient.poll();
  if (!mqttClient.connected()) {
    if (mqttClient.connect(mqtt_broker, mqtt_port)) {
      Serial.print("MQTT connected: ");
      Serial.println(mqtt_broker);
    } else {
      Serial.print("MQTT connection to ");
      Serial.print(mqtt_broker);
      Serial.print(" failed! Error code = ");
      Serial.println(mqttClient.connectError());
    }
  }
}

// vim: set ts=8 sw=2 sts=2 et ai:
