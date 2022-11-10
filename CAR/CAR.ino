#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#include <L298N.h>

const char *ssid = "debarba-robot";
const char *password = "qwert12345";

ESP8266WebServer server(80);

int L1 = 5;
int L2 = 4;
int R1 = 0;
int R2 = 2;

int MOVEMENT_DELAY = 100;
int MOVEMENT_LEFT_RIGHT_DELAY = 60;

void handleHttpRoot()
{
    Serial.println("Client connected");
    server.send(200, "text/html", "You are connected!");
}

void right()
{
    Serial.println("right");
    server.send(200, "text/html", "right");

    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);

    delay(MOVEMENT_LEFT_RIGHT_DELAY);

    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
}

void left()
{
    Serial.println("left");
    server.send(200, "text/html", "left");

    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);

    delay(MOVEMENT_LEFT_RIGHT_DELAY);

    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
}

void front()
{
    Serial.println("front");
    server.send(200, "text/html", "front");

    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);

    delay(MOVEMENT_DELAY);

    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
}

void back()
{
    Serial.println("back");
    server.send(200, "text/html", "back");

    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);

    delay(MOVEMENT_DELAY);

    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
}

void setup()
{
    delay(1000);

    Serial.begin(115200);
    Serial.println();

    Serial.println("Configuring access point...");

    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.println("Ip: " + myIP.toString());

    server.on("/", handleHttpRoot);
    server.on("/left", left);
    server.on("/right", right);
    server.on("/front", front);
    server.on("/back", back);

    server.begin();
    Serial.println("HTTP server started");

    pinMode(R1, OUTPUT);
    pinMode(R2, OUTPUT);
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
}

void loop()
{
    server.handleClient();
}