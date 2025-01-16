#include <TinyGPS++.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>

// GPS, LCD, and GSM setup
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Use UART1 (GPIO 16 for RX, 17 for TX)
LiquidCrystal_I2C lcd(0x3F, 20, 4); // 20x4 LCD with I2C address 0x3F
HardwareSerial gsmSerial(0); // Use UART0 (default TX=1, RX=3 for GSM)

// Variables for distance calculation
double prevLat = 0.0, prevLng = 0.0;
double totalHighwayDistance = 0.0;
double currentHighwayDistance = 0.0;
bool isInHighway = false;  // Flag to track if the vehicle is on the highway

// Define new highway perimeter coordinates (rectangle-shaped road)
double highwayCoords[4][2] = {
    {13.150107, 77.569923}, // Top right corner
    {13.150080, 77.569923}, // Bottom right corner
    {13.150092, 77.570686}, // Bottom left corner
    {13.150070, 77.570688}  // Top left corner
};

// Timer for toggling display
unsigned long lastToggleTime = 0;
bool showLat = true;

// Fee calculation constant
const float RATE_PER_10M = 50.0; // Rs. 50 per 10 meters

// SMS flags to ensure messages are sent only once per transition
bool smsSentOnEntry = false;
bool smsSentOnExit = false;

void setup() {
  Serial.begin(115200);  // For debugging
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // GPS UART1
  gsmSerial.begin(9600, SERIAL_8N1, 1, 3);  // GSM UART0

  lcd.begin();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("GPS Initializing...");
  Serial.println("Starting GPS Debugging...");
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
    Serial.print(c);  // Print raw NMEA sentences
  }

  // Call enhanced debug info
  printDebugInfo();

  // Check if location is valid
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();

    // Calculate distance if previous coordinates exist
    if (prevLat != 0.0 && prevLng != 0.0) {
      double distance = TinyGPSPlus::distanceBetween(lat, lng, prevLat, prevLng);
      totalHighwayDistance += distance;

      // Check if the vehicle is in the highway perimeter
      bool wasInHighway = isInHighway;
      isInHighway = isInsidePerimeter(lat, lng);

      // If the vehicle has entered the highway and was not already there, send SMS
      if (isInHighway && !wasInHighway && !smsSentOnEntry) {
        currentHighwayDistance = 0.0;  // Reset current highway distance
        sendSMS("You are now on the highway. Toll calculation starts.");
        smsSentOnEntry = true;  // Ensure SMS is only sent once on entry
        smsSentOnExit = false;  // Reset exit SMS flag
      }

      // If the vehicle is on the highway, accumulate the highway distance
      if (isInHighway) {
        currentHighwayDistance += distance;
      }

      // If the vehicle has exited the highway, send SMS with the total fare
      if (!isInHighway && wasInHighway && !smsSentOnExit) {
        float fare = (currentHighwayDistance / 10) * RATE_PER_10M;
        sendSMS("You are now off the highway. Total fare: Rs " + String(fare, 2));
        smsSentOnExit = true;  // Ensure SMS is only sent once on exit
        smsSentOnEntry = false;  // Reset entry SMS flag
      }
    }

    prevLat = lat;
    prevLng = lng;

    // Toggle display every 3 seconds
    if (millis() - lastToggleTime > 3000) {
      showLat = !showLat;
      lastToggleTime = millis();
    }

    // Display on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(isInHighway ? "Status: Highway" : "Status: Normal");

    lcd.setCursor(0, 1);
    lcd.print("Total Dist: ");
    lcd.print(totalHighwayDistance, 2);
    lcd.print(" m");

    lcd.setCursor(0, 2);
    lcd.print("Highway Dist: ");
    lcd.print(currentHighwayDistance, 2);
    lcd.print(" m");

    lcd.setCursor(0, 3);
    lcd.print("Rate: Rs ");
    lcd.print(currentHighwayDistance / 10 * RATE_PER_10M);  // Rate calculation: Rs 50 per 10 meters
  } else {
    // Show satellite count when location is invalid
    lcd.setCursor(0, 0);
    lcd.print("GPS Initializing...");
    lcd.setCursor(0, 1);
    lcd.print("Sats: ");
    lcd.print(gps.satellites.value());
  }

  delay(1000);
}

// Function to check if the current location is inside the defined highway perimeter
bool isInsidePerimeter(double lat, double lng) {
  // Check if the point is within the rectangle formed by the four corners
  bool isInLatRange = (lat >= highwayCoords[2][0] && lat <= highwayCoords[0][0]);
  bool isInLngRange = (lng >= highwayCoords[0][1] && lng <= highwayCoords[2][1]);

  return isInLatRange && isInLngRange;
}

// Function to send SMS using the GSM module
void sendSMS(String message) {
  gsmSerial.println("AT+CMGF=1"); // Set SMS mode
  delay(100);
  gsmSerial.println("AT+CMGS=\"+917204358788\""); // Send to this phone number
  delay(100);
  gsmSerial.print(message); // The message to send
  gsmSerial.write(26); // End SMS with CTRL+Z
  delay(1000);
}

// Function to print detailed GPS debug information
void printDebugInfo() {
  Serial.println("\n--- GPS Debug Info ---");

  // Raw NMEA data check
  Serial.print("NMEA Raw Data: ");
  Serial.println(gpsSerial.available() ? "Data Received" : "No Data");

  // Satellite count
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());

  // HDOP
  Serial.print("HDOP: ");
  Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : 0);

  // Latitude and Longitude
  if (gps.location.isValid()) {
    Serial.print("Lat: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Lng: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Location not valid.");
  }

  // Check for valid NMEA sentences
  Serial.println("---------------------");
}
