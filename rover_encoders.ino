/* **************************************************************************************
 *  NOVA ROVER TEAM - URC2018
 *  This code connects four rotary encoders to interrupt service routines
 *  to calculate their angular velocities.
 *  
 *  The motor + encoder being used:
 *  https://www.servocity.com/118-rpm-hd-premium-planetary-gear-motor-w-encoder
 *  
 *  Last modified by - Andrew Stuart (4/10/2017)
 ****************************************************************************************/

/************************************************************************************
* VARIABLE/MACRO DECLARATIONS
 ************************************************************************************/
#define LOOP_HERTZ 10  // Frequency of angular velocity calculation (in Hertz)
#define GEAR_RATIO 71  // Provided by servo data sheet
#define MOTOR_CYCLES_PER_REV 12  // Provided by servo data sheet

//!!!!
//todo: cycles per rev of motor may be 6 due to half resolution with encoders; needs verification
//!!!!

// Pins connected to Channel As of Encoders 1-4
// Pins 0 and 1 are used for serial comms to the raspi
#define ENC0_CHA 2 	//FR
#define ENC1_CHA 21 //BR
#define ENC2_CHA 10 //BL
#define ENC3_CHA 9	//FL

// Encoder counters to keep track of number of pulses (set to long for safety)
volatile long encCnts[4] = {0, 0, 0, 0};

// Loop rate
const float dt = 1.0/((float) LOOP_HERTZ);  // Time step (sec)

const unsigned int ppr = GEAR_RATIO * MOTOR_CYCLES_PER_REV * 2; // Pulses per revolution (pulses/rev)


/************************************************************************************
* SETUP FUNCTION
* This function runs once to setup pins, hardware, etc...
 ************************************************************************************/
void setup() 
{
  Serial.begin(9600); // Enable serial w/ 9600 baud rate
  InitPullUpResistors(); // Enable pull-up resistors on encoder channel pins
  InitInterrupts(); // Enable interrupts on Channel A of Encoders 1-4
}

/************************************************************************************
* SOFTWARE LOOP
* This functison will continuously loop forever. It is used to periodically calculate  
* the angular velocity of the wheels
 ************************************************************************************/
void loop() 
{  
  for (int i=0; i<4; i++) {
    Serial.print("Encoder " + String(i) + ": " + String(round((LOOP_HERTZ*60*encCnts[i])/ppr)) + " RPM\n"); // 
    encCnts[i] = 0; 
  }
  Serial.print("\n");
  //todo: check if timer interrupt is more accurate than delay function
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
