#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver PCA = Adafruit_PWMServoDriver(0x40);

// PCA9685 channels for the 6 motors
const int servoChannels[6] = {0, 1, 2, 3, 4, 5};

// Typical servo pulse range
const int SERVO_MIN = 150;   // ~0 degrees
const int SERVO_MAX = 600;   // ~180 degrees

// Convert angle to PCA9685 pulse
int angleToPulse(int angle)
{
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setup()
{
  Serial.begin(115200);

  // Start I2C
  Wire.begin();

  // Start PCA9685
  PCA.begin();

  // PCA9685 frequency for servos
  PCA.setPWMFreq(50);

  delay(500);

  // Start all 6 servos at 0 degrees
  for (int i = 0; i < 6; i++)
  {
    PCA.setPWM(servoChannels[i], 0, angleToPulse(0));
  }

  delay(1000);

  // Gradually move all servos from 0 to 90 degrees
  for (int angle = 0; angle <= 90; angle++)
  {
    for (int i = 0; i < 6; i++)
    {
      PCA.setPWM(
        servoChannels[i],
        0,
        angleToPulse(angle)
      );
    }

    delay(20);  // Increase this for slower movement
  }

  Serial.println("All 6 servos reached 90 degrees.");
}

void loop()
{
  // Nothing else happens
}