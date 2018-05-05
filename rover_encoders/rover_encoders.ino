/* **************************************************************************************
 *  NOVA ROVER TEAM - URC2018
 *  This code connects four rotary encoders to interrupt service routines
 *  to calculate their angular velocities, and a voltage sensor to test
 *  the voltage level the battery. 
 *  
 *  The motor + encoder being used:
 *  https://www.servocity.com/118-rpm-hd-premium-planetary-gear-motor-w-encoder
 *  
 *  Author: Andrew Stuart
 ****************************************************************************************/
#define USE_USBCON     // for Pro Micro
#include <ros.h>       // ROS Arduino library
#include <rover/RPM.h> // ROS msg for encoders
#include <rover/Voltages.h> // ROS msg for encoders
#include <std_msgs/Float32.h> //Float type for ROS msg for battery voltage sensor

/************************************************************************************
* VARIABLE/MACRO DECLARATIONS
 ************************************************************************************/
#define LOOP_HERTZ 50     // Frequency of angular velocity calculation (in Hertz)
#define VOLT_LOOP_HERTZ 1 // Frequency of voltage publishing, must be smaller than LOOP_HERTZ
#define GEAR_RATIO 71  // Provided by servo data sheet
#define MOTOR_CYCLES_PER_REV 12  // Provided by servo data sheet

// Pins connected to Channel A of Encoders 1-4
// Pins 0 and 1 are used for serial comms to the raspi
#define ENC0_CHA 0   //FR
#define ENC1_CHA 1   //BR
#define ENC2_CHA 2   //BL
#define ENC3_CHA 3   //FL

// Encoder counters to keep track of number of pulses (set to long for safety)
volatile long encCnts[4] = {0, 0, 0, 0};

// Loop rate
const float dt = 1.0/((float) LOOP_HERTZ);  // Time step (sec)

int voltLoopCnt = 0; // Counter for voltage message publishing
const int voltLoopMax = LOOP_HERTZ/VOLT_LOOP_HERTZ;
const float calibVolt = 0.02174848; // Calibrate raw voltage to actual

const unsigned int ppr = GEAR_RATIO * MOTOR_CYCLES_PER_REV * 2; // Pulses per revolution (pulses/rev)

// Declare required ROS variables
ros::NodeHandle  nh;
rover::RPM       msg; // Predefine ROS msg
ros::Publisher   encoders("/encoders", &msg);

rover::Voltages volt_msg; //ROS msg for voltage readings
ros::Publisher   voltage("/voltage", &volt_msg);

/************************************************************************************
* SETUP FUNCTION
* This function runs once to setup pins, hardware, etc...
 ************************************************************************************/
void setup() 
{
  nh.initNode();          // Initialise ROS node 
  nh.advertise(encoders); // Advertise publisher "encoders"
  nh.advertise(voltage); // Advertise publisher "encoders"
  
  //Serial.begin(9600);   // Enable serial w/ 9600 baud rate
  InitPullUpResistors();  // Enable pull-up resistors on encoder channel pins
  InitInterrupts();       // Enable interrupts on Channel A of Encoders 1-4
}

/************************************************************************************
* SOFTWARE LOOP
* This function will continuously loop forever. It is used to periodically calculate  
* the angular velocity of the wheels
 ************************************************************************************/

void ReadVoltage()
{
  float f1, f2, f3, f4;

  f1 = analogRead (A9)*calibVolt; //read the values from voltage sensors.
  volt_msg.v1 = f1;
  
  f2 = analogRead (A8)*calibVolt; 
  volt_msg.v2 = f2;
  
  f3 = analogRead (A7)*calibVolt; 
  volt_msg.v3 = f3;
  
  f4 = analogRead (A10)*calibVolt; 
  volt_msg.v4 = f4;
}

void loop() 
{    
  voltLoopCnt++;
  if (voltLoopCnt >= voltLoopMax) // If such time has passed for the voltage loop to occur
  {
    ReadVoltage();
    voltage.publish( &volt_msg ); // Publish the voltage reading messages
    voltLoopCnt = 0;
  }
  
  msg.rpm_fl = round((LOOP_HERTZ*60*encCnts[0])/ppr); // Front-left RPM
  msg.rpm_fr = round((LOOP_HERTZ*60*encCnts[1])/ppr); // Front-right RPM
  msg.rpm_bl = round((LOOP_HERTZ*60*encCnts[2])/ppr); // Back-left RPM
  msg.rpm_br = round((LOOP_HERTZ*60*encCnts[3])/ppr); // Back-right RPM

  encCnts[0] = 0;
  encCnts[1] = 0;
  encCnts[2] = 0;
  encCnts[3] = 0;
  
  encoders.publish( &msg ); // Publish the RPM messages
  nh.spinOnce();            // Finalise ROS messages
  
  delay(1000*dt);
}

/************************************************************************************
* INTERRUPT SERVICE ROUTINES
* ISRs for each corresponding encoder.
 ************************************************************************************/
void enc0() {
  encCnts[0]++;
}
void enc1() {
  encCnts[1]++;
}
void enc2() {
  encCnts[2]++;
}
void enc3() {
  encCnts[3]++;
}

/************************************************************************************
* MISC. FUNCTIONS
 ************************************************************************************/
void InitPullUpResistors() {
  // Setup pull-up resistors for interrupt pins
  pinMode(ENC0_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC3_CHA, INPUT_PULLUP);
}

void InitInterrupts() {
  // Encoders are set to changing value, so that Channel A is read on both rising/falling edges.
  attachInterrupt(digitalPinToInterrupt(ENC0_CHA), enc0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), enc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHA), enc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_CHA), enc3, CHANGE);
}
