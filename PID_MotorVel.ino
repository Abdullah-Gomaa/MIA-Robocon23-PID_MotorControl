#include <ros.h>
#include <std_msgs/Float32.h>
/*****************************************************************************************************************************************/
//_____________________________________________________Pre-build Configrations_____________________________________________________________//

#define encoderPinA    3
#define encoderPinB    2
#define PWM_Pin        5
#define DIR_Pin        6

#define Topic_Name  "SetVel"

#define encoderRes     512  /* Counts/rev */
#define maxRPM         200

// PID Tuning based in Ziegler and Nicholas Method (Z-N)        // (C-C)
#define Kp          0.8                                         // 0.3522
#define KI          0.638                                       // 0.2317
#define Kd          0.2508                                      // 0.0798
//________________________________________________________________________________________________________________________________________//

/*****************************************************************************************************************************************/
// Functions Prototypes
void CalculateMotorVelocity();
void PID_Routine();
void Control_Motor();
/*****************************************************************************************************************************************/
// Variables Declarations
float Setpoint_Velocity = 0;
int32_t count = 0;
float motorVelocity = 0;
float motorVelocityPrev = 0;
float VelocityFiltered = 0;

float error = 0;
float error_Int = 0;
float error_Der = 0;
float error_prev = 0;

float current_time = 0;
float prev_time = 0;
float deltaT = 0;

uint16_t PID_output = 0;
uint8_t  PWM_value = 0;
int8_t   DIR_value = 0;
/*****************************************************************************************************************************************/
// _______________________________________________________________ ROS PART_______________________________________________________________//

// Implement subscriber callback function
void callback(const std_msgs::Float32 &msg)
{
  // Set recieved msg in the global variable to be accessed through the program
  Setpoint_Velocity = msg.data;
}

// Create a node handler
  ros :: NodeHandle n;
  
// Instantiate Subsciber to topic name with msg type float32
  ros::Subscriber<std_msgs::Float32> sub(Topic_Name, &callback );

//  HardwareSerial Serial3(PB11, PB10);  // For STM
/*****************************************************************************************************************************************/
//_________________________________________________________ Setup Function ________________________________________________________________//

void setup() {
// Init the ros node
  n.initNode();
// Subscribe to the topic being published
  n.subscribe(sub);

// Setting pins directions
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
// Set interrupts
  attachInterrupt(digitalPinToInterrupt(encoderPinA) , &encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB) , &encoderB, CHANGE);

// Set Motor Pins
  pinMode(DIR_Pin, OUTPUT); 
  pinMode(PWM_Pin, OUTPUT); 

/*  Serial3.begin(115200);                  
  (node.getHardware())-> setPort(&Serial3);            // For STM
  (node.getHardware())-> setBaud(115200);
*/
}
//_________________________________________________________ Loop Function ________________________________________________________________//

void loop() {
// Spin Once for callback
  n.spinOnce();
// Call PID Routine
  PID_Routine();  
}
/*****************************************************************************************************************************************/
//_________________________________________________________ Helping Functions _____________________________________________________________//

void CalculateMotorVelocity()
{
  // Calculate motor current velocity in rpm
  motorVelocity = count / encoderRes * 60.0;

  // Low-pass filter (25 Hz cutoff) To remove High Frequency noise
  VelocityFiltered = 0.854*VelocityFiltered + 0.0728*motorVelocity + 0.0728 * motorVelocityPrev;
  motorVelocityPrev = motorVelocity;
}
void Control_Motor()
{
/* Using Cytron driver */

// Set PWM Value using PID output   
  PWM_value = (uint8_t)fabs(PID_output);
 
// Set Rotation Direction
  if(PID_output > 0)
      DIR_value = LOW;
  else if (PID_output < 0)
      DIR_value = HIGH; 
  
  digitalWrite(DIR_Pin, DIR_value);
  analogWrite(PWM_Pin, PWM_value);
}
/*****************************************************************************************************************************************/
//_________________________________________________________ PID Implmentation _____________________________________________________________//

void PID_Routine()
{
  CalculateMotorVelocity();
  
  current_time = millis();
  // deltaT in minutes to be suitable with rpm velocity error calculations
  deltaT = (current_time - prev_time)/1.0e3*60.0;

// Calculate errors
  error = Setpoint_Velocity - VelocityFiltered;
  // To overcome integral windup  
  if(PID_output< maxRPM)
    error_Int = error_Int + (error * deltaT);
  error_Der = (error - error_prev) / deltaT ; 
  
// perform PID Calculations
  PID_output = (Kp*error) + (KI*error_Int) + (Kd*error_Der);  

  Control_Motor();       
       
  error_prev = error;
  prev_time = current_time;
}
/*****************************************************************************************************************************************/
//_________________________________________________________ Encoder ISR ___________________________________________________________________//
// ISR for encoder PinA
void encoderA()
{
  if( (digitalRead(encoderPinA) != (digitalRead(encoderPinB) )))
    count++;  // CW
  else
    count--;  // CCW
}
// ISR for encoder PinB
void encoderB()
{
  if( (digitalRead(encoderPinA) != (digitalRead(encoderPinB) )))
    count--;  // CCW
  else
    count++; // CW
}
/*****************************************************************************************************************************************/
