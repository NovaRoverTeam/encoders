/* **************************************************************************************
 *  NOVA ROVER TEAM - URC2018
 *  This code connects four rotary encoders to interrupt service routines
 *  to calculate their angular velocities.
 *  
 *  The motor + encoder being used:
 *  https://www.servocity.com/118-rpm-hd-premium-planetary-gear-motor-w-encoder
 *  
 *  Last modified by - Andrew Stuart (2/10/2017)
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
#define ENC0_CHA 1
#define ENC1_CHA 0
#define ENC2_CHA 2
#define ENC3_CHA 3

// Pins connected to Channel Bs of Encoders 1-4
// todo: choose channel B pins for encoders
#define ENC0_CHB    // insert later
#define ENC1_CHB    // insert later
#define ENC2_CHB    // insert later
#define ENC3_CHB    // insert later

// Encoder counters to keep track of number of pulses (set to long for safety)
volatile long encCnts[4] = {0, 0, 0, 0};

// Loop rate
const float dt = 1.0/((float) LOOP_HERTZ);  // Time step

const unsigned int ppr = GEAR_RATIO * MOTOR_CYCLES_PER_REV; // Pulses per revolution
const float coeff = 2*PI/(dt*ppr); // Converts pulse count to ang vel

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
* This function will continuously loop forever. It is used to periodically calculate  
* the angular velocity of the wheels
 ************************************************************************************/
void loop() 
{  
  // Initialise angular velocities
  float angVels[4] = {0, 0, 0, 0};

  // Calculate angular velocities and reset encoder counters
  // todo: review calculations
  for (int i=0; i<4; i++) {
    angVels[i] = coeff*encCnts[i];
    Serial.print("Encoder " + String(i) + " has RPM: " + String(60*angVels[i]/(2*PI)) + "\n"); 
    encCnts[i] = 0; 
  }
  Serial.print("\n");
  //todo: check if timer interrupt is more accurate that delay function
  delay(1000*dt);
}

/************************************************************************************
* INTERRUPT SERVICE ROUTINES
* ISRs for each corresponding encoder. CW movement is 10->01->10, so if the encoder
* channel values are different, the wheel must be moving clockwise. Otherwise, if the
* values are the same (00->11->00), the wheel must be moving in an anti-clockwise 
* direction, so the count is decremented.
 ************************************************************************************/
void enc0() {
  if(digitalRead(ENC0_CHA)!=digitalRead(ENC0_CHB)) {
    encCnts[0]++;
  }
  else {
    encCnts[0]--;
  }
}
void enc1() {
  if(digitalRead(ENC1_CHA)!=digitalRead(ENC1_CHB)) {
    encCnts[1]++;
  }
  else {
    encCnts[1]--;
  }
}
void enc2() {
   if(digitalRead(ENC2_CHA)!=digitalRead(ENC2_CHB)) {
    encCnts[2]++;
  }
  else {
    encCnts[2]--;
  }
}
void enc3() {
  if(digitalRead(ENC3_CHA)!=digitalRead(ENC3_CHB)) {
    encCnts[3]++;
  }
  else {
    encCnts[3]--;
  }
}

/************************************************************************************
* MISC. FUNCTIONS
 ************************************************************************************/
void InitPullUpResistors() {
  pinMode(ENC0_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC3_CHA, INPUT_PULLUP);
  pinMode(ENC0_CHB, INPUT_PULLUP);
  pinMode(ENC1_CHB, INPUT_PULLUP);
  pinMode(ENC2_CHB, INPUT_PULLUP);
  pinMode(ENC3_CHB, INPUT_PULLUP);
}

void InitInterrupts() {
  // Encoders are set to changing value, so that Channel A is read on both rising/falling edges.
  attachInterrupt(digitalPinToInterrupt(ENC0_CHA), enc0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), enc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHA), enc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_CHA), enc3, CHANGE);
}
