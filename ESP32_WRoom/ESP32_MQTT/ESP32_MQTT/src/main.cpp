#include <Arduino.h>

#include <WiFi.h>             // Allows ESP32 to establish connections with Wi-Fi networks
#include <PubSubClient.h>     // Enables ESP32 to connect to an MQTT broker
//<---- ------------------------------------------------------------------ ---->
const char *ssid = "RedmiSbzrgn12";
const char *password = "sbzrgn024";

const char *mqtt_broker = "broker.emqx.io";
const char *topic = "emqx/esp32";
const char *mqtt_username = "mrl";
const char *mqtt_password = "mrl";
const int mqtt_port = 1883;
//<---- ------------------------------------------------------------------ ---->
WiFiClient espClient;
PubSubClient client(espClient);
//<---- ------------------------------------------------------------------ ---->
void callback(char *topic, byte *payload, unsigned int length);
//<---- ------------------------------------------------------------------ ---->

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("<---- Connecting to WiFi ---->");
  }
  Serial.println("<---- Connected to the Wi-Fi network ---->");

  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  
  while (!client.connected())
  {
    String client_id = "esp32-client-";

    client_id += String(WiFi.macAddress());

    Serial.printf("<---- The client %s connects to the public MQTT broker ---->\n", client_id.c_str());

    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("<---- Public EMQX MQTT broker connected ---->");
    }
    else
    {
      Serial.print("<---- failed with state ---->");
      Serial.print(client.state());
      delay(2000);
    }
  }
  client.publish(topic, "<---- Hi, I'm ESP32 ---->");
  client.subscribe(topic);
}
//<---- ------------------------------------------------------------------ ---->
void loop()
{
  client.loop();
}
//<---- ------------------------------------------------------------------ ---->
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("<---- Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");

  for (int i = 0; i < length; i++)
  {
    Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("<---- ----------------------- ---->");
}