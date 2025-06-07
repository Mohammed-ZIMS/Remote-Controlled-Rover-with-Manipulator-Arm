// Motor control pins for Motor A (left side)
const int motorA_IN1 = 7;
const int motorA_IN2 = 8;
// Motor control pins for Motor B (right side)
const int motorB_IN3 = 12;
const int motorB_IN4 = 13;

// Define button indices for clarity
const int FORWARD_BUTTON = 0;
const int BACKWARD_BUTTON = 1;
const int LEFT_BUTTON = 2;
const int RIGHT_BUTTON = 3;
const int GRIPPER_BUTTON = 4; // 5th button

// Define potentiometer indices
const int POT1 = 6; // Index after the 6 button states
const int POT2 = 7;
const int POT3 = 8;
const int POT4 = 9;
const int POT5 = 10;

// Include the library for the 16-channel PWM servo driver (PCA9685)
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channel assignments
const int SERVO1_CHANNEL = 0;
const int SERVO2_CHANNEL = 1;
const int SERVO3_CHANNEL = 2;
const int SERVO4_CHANNEL = 3;
const int SERVO5_CHANNEL = 4;
const int GRIPPER_SERVO_CHANNEL = 5; // 6th servo

// Servo pulse width limits (adjust based on your servos)
const int SERVO_MIN_PULSE = 500;  // Minimum pulse width in microseconds
const int SERVO_MAX_PULSE = 2500; // Maximum pulse width in microseconds

// Gripper servo degrees
const int GRIPPER_CLOSED_DEGREE = 0;
const int GRIPPER_OPEN_DEGREE = 180; // Changed to 180
bool gripperOpen = false;

// Function to map analog value (0-1023) to a degree (0-180)
int mapPotTo180(int analogValue)
{
    return map(analogValue, 0, 1023, 0, 180);
}

// Function to convert degree to servo pulse width (microseconds)
int degreeToPulse(int degree)
{
    return map(degree, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void setup()
{
    Serial.begin(9600);
    pinMode(motorA_IN1, OUTPUT);
    pinMode(motorA_IN2, OUTPUT);
    pinMode(motorB_IN3, OUTPUT);
    pinMode(motorB_IN4, OUTPUT);

    pwm.begin();
    pwm.setPWMFreq(60); // Standard servo frequency

    // Initialize gripper to the closed position
    int closedPulse = degreeToPulse(GRIPPER_CLOSED_DEGREE);
    pwm.writeMicroseconds(GRIPPER_SERVO_CHANNEL, closedPulse);
}

void moveForward(int speed)
{
    digitalWrite(motorA_IN1, HIGH);
    digitalWrite(motorA_IN2, LOW);
    digitalWrite(motorB_IN3, HIGH);
    digitalWrite(motorB_IN4, LOW);
    // You might want to incorporate speed control here using PWM on enable pins.
}

void moveBackward(int speed)
{
    digitalWrite(motorA_IN1, LOW);
    digitalWrite(motorA_IN2, HIGH);
    digitalWrite(motorB_IN3, LOW);
    digitalWrite(motorB_IN4, HIGH);
    // Implement speed control if needed.
}

void turnLeft(int speed)
{
    digitalWrite(motorA_IN1, LOW);
    digitalWrite(motorA_IN2, HIGH);
    digitalWrite(motorB_IN3, HIGH);
    digitalWrite(motorB_IN4, LOW);
    // Implement speed control if needed (differential steering).
}

void turnRight(int speed)
{
    digitalWrite(motorA_IN1, HIGH);
    digitalWrite(motorA_IN2, LOW);
    digitalWrite(motorB_IN3, LOW);
    digitalWrite(motorB_IN4, HIGH);
    // Implement speed control if needed (differential steering)
