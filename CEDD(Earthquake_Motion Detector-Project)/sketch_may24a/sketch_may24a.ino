#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SD.h>
#include <SPI.h>
#include <TMRpcm.h>

#define ADXL345_ADDRESS (0x53)
#define SD_ChipSelectPin 4
#define SPEAKER_PIN 9

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
TMRpcm tmrpcm;

int motionCounter = 0;
unsigned long lastMotionTime = 0;
unsigned long lastMotionDetectedTime = 0;
const unsigned long motionTimeout = 10000; // 10 seconds
const unsigned long referenceDuration = 5000; // 5 seconds

bool isStable = false;
float referencePosition = 0.0;
float highestValue = -INFINITY; // Initialize to negative infinity
float lowestValue = INFINITY; // Initialize to positive infinity
unsigned long referenceStartTime = 0;

void setup(void) {
  Serial.begin(9600);

  // Initialize the accelerometer
  if (!accel.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);

  // Initialize the SD card
  if (!SD.begin(SD_ChipSelectPin)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  // Set up the speaker pin for TMRpcm
  tmrpcm.speakerPin = SPEAKER_PIN;

  // Optional: Set the initial volume
  tmrpcm.setVolume(5);
}

void loop(void) {
  sensors_event_t event;
  accel.getEvent(&event);

  // Calculate the magnitude of the acceleration vector
  float acceleration = sqrt(event.acceleration.x * event.acceleration.x +
                            event.acceleration.y * event.acceleration.y +
                            event.acceleration.z * event.acceleration.z);

  Serial.print("Acceleration: ");
  Serial.println(acceleration, 2);

  unsigned long currentTime = millis();

  // Check for reference position setting
  if (!isStable && currentTime - referenceStartTime < referenceDuration) {
    // Update highest and lowest values
    if (acceleration > highestValue) {
      highestValue = acceleration;
    }
    if (acceleration < lowestValue) {
      lowestValue = acceleration;
    }
  } else if (!isStable) {
    // Reference position setting complete
    isStable = true;
    referencePosition = (highestValue + lowestValue) / 2.0; // Set reference position as the average of highest and lowest
    Serial.print("Reference Position: ");
    Serial.println(referencePosition);
  }

  // Once stable, perform earthquake detection
  if (isStable) {
    // Check if 10 seconds have passed since the last motion detection

    // Check if motion detected
    if ((acceleration > highestValue + 0.02 && acceleration < highestValue + 0.5) || 
        (acceleration < lowestValue - 0.5 && acceleration > lowestValue - 0.1)) {
       motionCounter++;
       Serial.println("Motion detected!");

      if (motionCounter = 2) {
        Serial.println("Motion detected 3 times in a row!");
        // Play the earthquake audio file
        tmrpcm.play((char*)"modt.wav");
        // Reset the counter after detecting an earthquake
        motionCounter = 0;
        // Delay to avoid continuous triggering
        delay(1500); // This delay can be adjusted or removed based on your requirements
      }
    } else if (acceleration > highestValue + 0.20 || acceleration < lowestValue - 0.2) {
      Serial.println("Earthquake detected!");
      // Play the earthquake audio file
      tmrpcm.play((char*)"erdt.wav");
      // Reset the counter if an earthquake is detected
      motionCounter = 0;
      // Delay to avoid continuous triggering
      delay(1500); // This delay can be adjusted or removed based on your requirements
    }
  }

  delay(50); // Adjust delay as needed
}
