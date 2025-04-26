#include <WiFi.h>

const char* ssidDev = "RobotTest";
const char* pwdDev = "123mrl456";

WiFiServer wserver(123); // port 123
WiFiClient wsclient;

void setup()
{
  Serial.begin(115200);
  Serial.println("Setting up ESP32 as Access Point...");

  // Create WiFi Access Point
  WiFi.softAP(ssidDev, pwdDev);
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  // Start the server
  wserver.begin();
  Serial.println("Server started, waiting for clients...");
}

void loop() 
{
  // Check if a new client has connected
  wsclient = wserver.available();
  if (wsclient) 
  {
    Serial.println("New client connected");
    String currentLine = "";

    while (wsclient.connected()) 
    {
      if (wsclient.available()) 
      {
        char c = wsclient.read();
        Serial.print(c);

        if (c == '\n') 
        {
          Serial.println("Command received: " + currentLine);
          wsclient.println("Hello from ESP32!" + currentLine);
          currentLine = "";
        } 
        else 
        {
          currentLine += c;
        }
      }
    }

    // Client disconnected
    wsclient.stop();
    Serial.println("Client disconnected.");
  }
}
