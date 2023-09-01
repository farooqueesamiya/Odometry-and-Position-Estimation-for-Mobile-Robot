#include <math.h>  // Include the math library for mathematical functions

#define PI 3.1415926535897932384626433832795  // Define the constant value of PI

// Robot control parameters
int N = 20;  // Not clear what this variable is for
int contadorTicks = 500;  // Number of ticks for encoder counting
int tam = 10;  // Size of the data vector for filtering
int k = 10;  // Time interval for odometry calculations

// PID control constants
const int targetPosition = 0;  // Target position
const double kp = 0.1;  // Proportional gain
const double ki = 0.01;  // Integral gain
const double kd = 0.5;  // Derivative gain

// Variables for encoder counting and control signals
int currentposition = 0;  // Current position
int previousposition = 0;  // Previous position
int totalerror = 0;  // Accumulated error
int previouserror = 0;  // Previous error
int controlsignal = 0;  // Control signal

// Variables for motor control
int currentPosition = 0;  // Current position
int previousPosition = 0;  // Previous position
int totalError = 0;  // Accumulated error
int previousError = 0;  // Previous error
int controlSignal = 0;  // Control signal
volatile unsigned actualsampling = 0;  // Current time in milliseconds
volatile unsigned presampling = 0;  // Previous time in milliseconds
volatile unsigned deltasampling = 0;  // Time interval between loop iterations

// Variables for odometry and robot position
float error = 0;  // Error variables
float Kp = 0.9;  // Control gain
int PWMr = 0;  // PWM for right motor
int PWMl = 0;  // PWM for left motor
int PWMmax = 150;  // Maximum PWM value
int PWMmin = -150;  // Minimum PWM value

// Motor control pins
#define in1 9
#define in2 8
#define in3 7
#define in4 6

// Variables for robot position tracking
float totaldistance = 0;  // Total distance traveled from the center
float x = 0;  // x distance
float y = 0;  // y distance
float phi = 0;  // Angular position

// Desired robot position
float Xd = -1000;
float Yd = -1000;
float Phid = atan2(Yd - y, Xd - x);  // Desired angle

// Robot physical parameters
float diameter = 6.8;  // Diameter of the wheels
float longitude = 19.4;  // Distance between the wheels
float V = 0;  // Linear velocity
float W = 0;  // Angular velocity

// Variables for right wheel encoder
volatile unsigned actualsamplingInterruptR = 0;
volatile unsigned presamplingInterruptR = 0;
double deltasamplingInterruptR = 0;
int encoderR = 3;  // Pin for right wheel encoder
int motorR = 5;  // Pin for right motor
double frecuencyR = 0;  // Frequency of encoder interrupts
double Wr = 0;  // Angular velocity of right wheel
double Vr = 0;  // Linear velocity of right wheel
int CR = 0;  // Counter for right encoder ticks
float vectorR[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Data vector for filtering
float Rdistance = 0;  // Distance traveled by right wheel
int Rtick = 0;  // Right wheel tick count
int Rtickpre = 0;  // Previous right wheel tick count
int deltaRtick = 0;  // Change in right wheel tick count

// Variables for left wheel encoder
volatile unsigned actualsamplingInterruptL = 0;
volatile unsigned presamplingInterruptL = 0;
double deltasamplingInterruptL = 0;
int encoderL = 2;  // Pin for left wheel encoder
int motorL = 10;  // Pin for left motor
double frecuencyL = 0;  // Frequency of encoder interrupts
double Wl = 0;  // Angular velocity of left wheel
double Vl = 0;  // Linear velocity of left wheel
int CL = 0;  // Counter for left encoder ticks
float vectorL[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Data vector for filtering
float Ldistance = 0;  // Distance traveled by left wheel
int Ltick = 0;  // Left wheel tick count
int Ltickpre = 0;  // Previous left wheel tick count
int deltaLtick = 0;  // Change in left wheel tick count

void setup() {
  // Setup code runs once at the beginning
  attachInterrupt(digitalPinToInterrupt(encoderR), REncoder, FALLING);  // Attach ISR for right wheel encoder
  attachInterrupt(digitalPinToInterrupt(encoderL), LEncoder, FALLING);  // Attach ISR for left wheel encoder
  Serial.begin(9600);  // Initialize serial communication
  pinMode(in1, OUTPUT);  // Set motor control pins as outputs
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(motorL, OUTPUT);
  pinMode(motorR, OUTPUT);
  digitalWrite(in1, 0);  // Set initial motor control states
  digitalWrite(in2, 1);
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
  delay(500);  // Delay for stability
}

void REncoder() {
  // Right wheel encoder interrupt service routine
  Rtick++;  // Increment right wheel tick count
  CR++;
  if (CR == contadorTicks) {
    // Filtering and frequency calculation
    float media = 0;
    for (int i = 0; i < tam - 1; i++) {
      vectorR[i] = vectorR[i + 1];
    }
    vectorR[tam - 1] = deltasamplingInterruptR;

    for (int i = 0; i < tam; i++) {
      media = vectorR[i] + media;
    }
    media = media / tam;
    deltasamplingInterruptR = media;
    frecuencyR = (1000) / deltasamplingInterruptR;
    presamplingInterruptR = actualsamplingInterruptR;
    CR = 0;
  }
}

void LEncoder() {
  // Left wheel encoder interrupt service routine
  Ltick++;  // Increment left wheel tick count
  CL++;
  if (CL == contadorTicks) {
    // Filtering and frequency calculation
    float media = 0;
        for (int i = 0; i < tam - 1; i++) {
      vectorL[i] = vectorL[i + 1];
    }
    vectorL[tam - 1] = deltasamplingInterruptL;

    for (int i = 0; i < tam; i++) {
      media = vectorL[i] + media;
    }
    media = media / tam;
    deltasamplingInterruptL = media;
    frecuencyL = (1000) / deltasamplingInterruptL;
    presamplingInterruptL = actualsamplingInterruptL;
    CL = 0;
  }
}

void loop() {
  actualsampling = millis();  // Get the current time in milliseconds
  actualsamplingInterruptR = millis();
  actualsamplingInterruptL = millis();
  deltasampling = (double)actualsampling - presampling;  // Calculate the time interval

  if (deltasampling >= k) {
    // If enough time has passed, perform odometry calculations
    odometry();
    analogWrite(motorR, 1000);  // Set PWM for right motor
    analogWrite(motorL, 1000);  // Set PWM for left motor

    // Output the robot's position to the serial monitor
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
  }
}

void odometry() {
  // Perform odometry calculations to update the robot's position and orientation
  deltaRtick = Rtick - Rtickpre;  // Calculate change in right wheel ticks
  Rdistance = PI * diameter * (deltaRtick / (double)20);  // Calculate distance traveled by the right wheel

  deltaLtick = Ltick - Ltickpre;  // Calculate change in left wheel ticks
  Ldistance = PI * diameter * (deltaLtick / (double)20);  // Calculate distance traveled by the left wheel

  totaldistance = (Rdistance + Ldistance) / 2;  // Calculate total distance traveled

  x = x + totaldistance * cos(phi);  // Update x position
  y = y + totaldistance * sin(phi);  // Update y position

  phi = phi + ((Rdistance - Ldistance) / longitude);  // Update angular position
  phi = atan2(sin(phi), cos(phi));  // Normalize the angle

  Rtickpre = Rtick;  // Update previous right wheel tick count
  Ltickpre = Ltick;  // Update previous left wheel tick count
}

