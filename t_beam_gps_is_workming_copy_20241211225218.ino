#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <PulseSensorPlayground.h>
#include <ThingSpeak.h>
#include <ESP_LM35.h>
#include <WiFi.h>  // Wi-Fi library for ThingSpeak communication

// the website with the data is send to https://thingspeak.mathworks.com/channels/2777173

// Hardware objects
HardwareSerial GPSSerial(1);  // Use Serial1 for GPS (GPIO 34 RX, GPIO 12 TX for T-Beam)

// GPS and sensor objects
TinyGPSPlus gps;
PulseSensorPlayground pulseSensor;
ESP_LM35 tempSensor(32);  // LM35 connected to pin 32 (analog input)

// ThingSpeak setup
WiFiClient client;
unsigned long channel = 2777173;  // Replace with your ThingSpeak channel number
const char* API = "HJGCL9RBYNV0BHUF";  // Replace with your ThingSpeak API key

// Wi-Fi credentials
const char* ssid = "MDX welcomes you";  // Replace with your Wi-Fi SSID
const char* password = "MdxL0vesyou";  // Replace with your Wi-Fi password

// Timer variables
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 30000;  // Update every 30 seconds

// Manual input flags
bool manualInput = false;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize GPS (using Serial1 for T-Beam GPS module)
  GPSSerial.begin(9600, SERIAL_8N1, 34, 12);  // RX = GPIO 34, TX = GPIO 12
  Serial.println("GPS initialized.");

  // Initialize pulse sensor 
  pulseSensor.analogInput(35);  
  pulseSensor.begin();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to Wi-Fi");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

void loop() {
  // Ask the user for manual input if needed
  if (manualInput) {
    float latitude, longitude, temperature;
    int heartRate;

    // Manual input for Latitude
    Serial.println("Enter Latitude:");
    while (Serial.available() == 0);  // Wait for input
    latitude = Serial.parseFloat();   // Read input

    // Manual input for Longitude
    Serial.println("Enter Longitude:");
    while (Serial.available() == 0);  // Wait for input
    longitude = Serial.parseFloat();  // Read input

    // Manual input for Heart Rate
    Serial.println("Enter Heart Rate:");
    while (Serial.available() == 0);  // Wait for input
    heartRate = Serial.parseInt();    // Read input

    // Manual input for Temperature
    Serial.println("Enter Temperature (Celsius):");
    while (Serial.available() == 0);  // Wait for input
    temperature = Serial.parseFloat();  // Read input

    // Send the manual data to ThingSpeak
    sendDataToThingSpeak(latitude, longitude, heartRate, temperature);
    manualInput = false;  // Reset the flag after sending data
  } else {
    if (millis() - lastUpdate > updateInterval) {
      // GPS data initialization
      float latitude = 0.0, longitude = 0.0;

      // Read GPS data and process it
      while (GPSSerial.available()) {
        gps.encode(GPSSerial.read());  // Process each byte of GPS data
      }

      // Wait for valid GPS data (up to 120 seconds for better GPS fix)
      unsigned long gpsStartMillis = millis();
      while (!gps.location.isValid() && (millis() - gpsStartMillis < 120000)) {  // Wait for 120 seconds (extended)
        gps.encode(GPSSerial.read());
        delay(100);  // Slight delay to slow down the reading process
        Serial.println("Waiting for GPS fix...");
      }

      // Use fallback coordinates if GPS fix is not valid
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Serial.printf("GPS Fix: Lat: %.6f, Lon: %.6f\n", latitude, longitude);
      } else {
        // Fallback to your defined coordinates if GPS fix is not acquired
        latitude = 25.103583;  // Starting Latitude (your defined location)
        longitude = 55.164381; // Starting Longitude (your defined location)
        Serial.println("GPS fix not acquired. Using fallback coordinates.");
        Serial.printf("Fallback Coordinates: Lat: %.6f, Lon: %.6f\n", latitude, longitude);
      }

      // Sensor data
      int heartRate = pulseSensor.getBeatsPerMinute();  // Use getBeatsPerMinute() for heart rate
      if (heartRate > 0) {
        Serial.printf("Heart Rate: %d BPM\n", heartRate);
      } else {
        Serial.println("No heartbeat detected.");
        heartRate = 0; // Set heartbeat to 0 if none detected
      }

      // LM35 Sensor Debugging
      float temperature = tempSensor.tempC();  // Read temperature from LM35
      if (temperature < 0 || temperature > 50) {  // Realistic temperature range for LM35
        Serial.println("Invalid temperature reading.");
        temperature = 0.0;  // Reset invalid reading
      } else {
        Serial.printf("Temperature: %.2f°C\n", temperature);
      }

      // Send data to ThingSpeak if we have valid GPS data (or fallback coordinates)
      sendDataToThingSpeak(latitude, longitude, heartRate, temperature);

      // Update the timer
      lastUpdate = millis();
    }

    while (GPSSerial.available()) {
      char c = GPSSerial.read();
      Serial.print(c);  // Print raw GPS data to see if any data is coming in.
      gps.encode(c);
    }
  }
}

// Function to send data to ThingSpeak
void sendDataToThingSpeak(float latitude, float longitude, int heartRate, float temperature) {
  ThingSpeak.setField(1, latitude);  // Field 1 - Latitude
  ThingSpeak.setField(2, longitude); // Field 2 - Longitude
  ThingSpeak.setField(4, heartRate); // Field 4 - Heart Rate
  ThingSpeak.setField(5, temperature); // Field 5 - Temperature

  // Send data to ThingSpeak
  int statusCode = ThingSpeak.writeFields(channel, API);
  if (statusCode == 200) {
    Serial.println("Data sent to ThingSpeak successfully.");
  } else {
    Serial.printf("Failed to send data to ThingSpeak. Status code: %d\n", statusCode);
  }
}
