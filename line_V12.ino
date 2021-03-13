/*
 * Pin Descriptions
  A0: Center sensor
  A1: Right Sensor
  A2: Left Sensor
  D2: Right Wheel Encoder
  D3: Left Wheel Encoder
  D7: Left Wheel Enable
  D8: Right Wheel Enable
  D9: Right PWM 
  D10: Right PWM
  D11: Right LED
  D12: Middle LED
  D13: Left LED
*/



void setup() {

  ////////////////////////////////////////////////////////////
  // ADC config with ADLAR = 0 starting on pin A0
  // ADC config in single run mode.
  // ADCSRA = 0xEB;
  ADCSRA = 0xCB;
  ADCSRB = 0x00;
  ADMUX = 0x40;

  // Enable external interrupts.
  EICRA |= 0x0F;
  EIMSK |= 0x03;

  //Initialize adc to remove first start delay
  ADCSRA |= 0x40;

  //Timer counter 0 configuration for wheel speed. IN FAST PWM
  TCCR0A |= 0xA1;
  TCCR0B |= 0x01;

  // Configure timer counter 1 for use with PWM for the right wheel, as it turns faster than the left.
  // The job of this PWM implementation will be to slow down the left wheel to approximately match
  // the speed of the right wheel.
  // Configured in 8 bit mode, counting to 0x00FF as we cannot count to OCR1A in 8-bit mode.
  TCCR1A |= 0xA1;
  TCCR1B |= 0x01;

  // These registers are both set to zero here so that they need not be
  // assigned a value of zero anywhere else except the reverse function
  // when the time comes to implement it.
  // OCRXB = 0 is forward moving directionality.
  OCR0B = 0;
  OCR1B = 0;

  // Enable Both Motors
  PORTB |= 0x01;
  PORTD |= 0x80;

  // Set up output pins
  DDRD |= 0xE0;
  DDRB |= 0x3F;

  // Pull up write for wheel encoders.
  PORTD |= 0x0C;

  //Serial.begin(9600);





}
// ISR variables for grabbing ADC conversions and assigning them to sensor values
volatile unsigned int adcResult = 0;
volatile unsigned char digital = 0; // Value that gets set to 1 or 0 based on ADC value
volatile unsigned int middle = 0;
volatile unsigned int left = 0;
volatile unsigned int right = 0;
volatile unsigned char dir = 0; // dir is the last know direction that the car was turning

//analog values for line sensors
volatile unsigned int aMiddle = 0;
volatile unsigned int aRight = 0;
volatile unsigned int aLeft = 0;

//PID VARIABLES
int errArray[3];
int linePos[3];
int error;

// volatile unsigned int prop = 0;

//Wheel encoder Variables
volatile unsigned int rTicks = 0; // Keeps track of the right wheel ticks
volatile unsigned int lTicks = 0;// Keeps track of the left wheel ticks
volatile int ticks;

// Stop moving before 3-point
volatile char kill = 0;

bool halfDone()
{
  if (((rTicks + lTicks) / 2) > 160000)// should be 1600 but its higher to test halfdone false
  {
    return true;
  }
  else
  {
    return false;
  }
}


void loop() {


  // Start by getting our updated sensor values.
  updateSensor();


  // Fast mode check.
  // If halfDone returns true, we go to fast mode.
  if (halfDone() == true)
  {
    if ((left == 0) && (middle == 1) && (right == 0)) {
      forward();
      updateSensor();
    }
    if ((left == 1) && (middle == 1) && (right == 0)) {
      error = 0.75;
      turnRight(error);
      updateSensor();
    }
    if ((left == 1) && (middle == 0) && (right == 0)) {
      error = 0.50;
      turnRight(error);
      updateSensor();
    }
    if ((left == 0) && (middle == 1) && (right == 1)) {
      error = 0.75;
      turnLeft(error);
      updateSensor();
    }
    if ((left == 0) && (middle == 0) && (right == 1)) {
      error = 0.50;
      turnLeft(error);
      updateSensor();
    }
    if ((left == 0) && (middle == 0) && (right == 0)) {

      if (dir == 1) {
        error = 0.40;
        turnLeft(error);
        updateSensor();
      }
      if (dir == 2) {
        error = 0.40;
        turnRight(error);
        updateSensor();
      }

      if ((dir == 3)) {
        //updateSensor();
        //int ticks = ((rTicks + lTicks) / 2);
        //if (ticks > (ticks +10)){ // if we have seen white and dir = 3 for 10 ticks
        dontMove();
        //pointTurn();
        updateSensor();
        //}
      }
    }
  }
  
  // Otherwise, we're in slow mode.
  if (halfDone() == false)
  {
    if ((left == 0) && (middle == 1) && (right == 0)) {
      forward();
      updateSensor();
    }
    if ((left == 1) && (middle == 1) && (right == 0)) { //Want tank turns here
      error = 0.50;//original: 0.75
      rTank(error);
      updateSensor();
    }
    if ((left == 1) && (middle == 0) && (right == 0)) {
      error = 0.25;//original: 0.50
      turnRight(error);
      updateSensor();
    }
    if ((left == 0) && (middle == 1) && (right == 1)) { //Want tank turns here
      error = 0.50;//original: 0.75
      lTank(error);
      updateSensor();
    }
    if ((left == 0) && (middle == 0) && (right == 1)) {
      error = 0.25;//original: 0.50
      turnLeft(error);
      updateSensor();
    }
    if ((left == 0) && (middle == 0) && (right == 0)) {
      if (dir == 1) {
        error = 0.30;//original: 0.40
        rTank(error);
        updateSensor();
      }
      if (dir == 2) {
        error = 0.30;//original: 0.40
        lTank(error);
        updateSensor();
      }
      if ((dir == 3 )) {
        updateSensor();
        if (ticks > 400) { // if we have seen white and dir = 3 for 10 ticks
          dontMove();
          //pointTurn();
          updateSensor();
        }
      }
    }
  }


}//End of Loop


// Get Sensor value functions //
void updateSensor() {
  // Update all sensor values
  getMiddle();
  getLeft();
  getRight();
  ticks = ((rTicks + lTicks) / 2);
}

// All update sensor values are identical in functionality. 
// Get Middle IR sensor value
void getMiddle() {

  // Look at the middle sensor.
  ADMUX = 0x40;

  // Write a 1 to the ADCSRA register to start the next ADC conversion for the middle sensor
  ADCSRA |= 0x40;

  // Delay to allow for ADC converison to finish.
  _delay_us(90);


  if (ADSC == 6)
  {
    // Set our middle value equal to the digital value returned from the ISR
    aMiddle = adcResult;
    errArray[1] = aMiddle;

    middle = digital;
    linePos[1] = middle;
  }
  // If middle sees black tape, turn on our LED.
  if (middle == 1) {
    PORTB |= 0x10;
  }
  // Otherwise, turn it off.
  else {
    PORTB &= 0x2F;
  }
}

// Get left IR sensor value
void getLeft() {
  ADMUX = 0x42;
  ADCSRA |= 0x40;
  _delay_us(90);
  if (ADSC == 6)
  {
    aLeft = adcResult;
    errArray[0] = aLeft;

    left = digital;
    linePos[0] = left;
  }
  if (left == 1) {
    PORTB |= 0x20;
  }
  else {
    PORTB &= 0x1F;
  }

}
// Get right IR sensor value
void getRight() {
  ADMUX = 0x41;
  ADCSRA |= 0x40;
  _delay_us(90);
  if (ADSC == 6)
  {
    aRight = adcResult;
    errArray[2] = aRight;

    right = digital;
    linePos[2] = right;
    //Serial.print("Right Sensor: \t"); Serial.print(right); Serial.print('\t');
  }
  if (right == 1) {
    PORTB |= 0x08;
  }
  else {
    PORTB &= 0x37;
  }
  _delay_us(300);
}

//////  Standard Movement Control   //////
void forward()
{
  dir = 0;

  // Set both registers so that we move forward. max 255

  OCR0A = 230;
  OCR0B = 0;

  OCR1A = 230;
  OCR1B = 0;

  dir = 3; // Last Know direction forward
}

void dontMove() {

  OCR0A = 0;
  OCR0B = 0;

  OCR1A = 0;
  OCR1B = 0;
  if (kill==1){
    PORTD &= 0x7F;
    PORTB &= 0xFE;
  }

}

void turnLeft(int error)
{
  // Modulate the speed of the left wheel to be slower than that of the right wheel
  // To achieve a forward facing turn left.

  // Reset direction just to be safe.
  dir = 0;

  // Set registers for a rolling left turn.
  OCR0A = 230;
  OCR0B = 0;

  OCR1A = 180 * error;
  OCR1B = 0;

  // 1 means left, in terms of last known diretion.
  dir = 1;

}

void turnRight(int error)
{
  // Modulate the speed of the right wheel to be slower than that of the left wheel
  // To achieve a forward facing turn right.
  dir = 0;

  OCR0A = 180 * error;
  OCR0B = 0;

  OCR1A = 230;
  OCR1B = 0;

  dir = 2;
}
//////    Reverse Movement Control    //////
void reverse() {

  OCR0A = 0;
  OCR0B = 210;

  OCR1A = 0;
  OCR1B = 210;

}
void rReverse() {

  OCR0A = 0;
  OCR0B = 0;

  OCR1A = 0;
  OCR1B = 210;

}
void lReverse() {

  OCR0A = 0;
  OCR0B = 210;

  OCR1A = 0;
  OCR1B = 0;

}
//////////////////////TANK CONTROL////////////////////////////
void rTank(int error) {
  OCR0A = 255;//180
  OCR0B = 0;

  OCR1A = 0;
  OCR1B = 255;//210

}
void lTank(int error) {


  OCR0A = 0;
  OCR0B = 255;//210

  OCR1A = 255;//180
  OCR1B = 0 ;

}

///  Precision movement control    ///

void rticksMove(int ticks)
{
  int newTicks;
  newTicks = rTicks + ticks;

  while (newTicks > rTicks) {
    //Left
    OCR0A = 0;
    OCR0B = 0;

    //Right
    OCR1A = 230;
    OCR1B = 0 ;
  }
  dontMove();


}
void lticksMove(int ticks)
{
  int newTicks;
  newTicks = lTicks + ticks;

  while (newTicks > lTicks) {
    //Left
    OCR0A = 230;
    OCR0B = 0;

    //Right
    OCR1A = 0;
    OCR1B = 0 ;
  }
  dontMove();
}

// Reverse precision movement control

void lticksReverse(int ticks) {
  int newTicks;
  newTicks = lTicks + ticks;

  while (newTicks > lTicks) {
    lReverse();
  }
  dontMove();
}

void tForward(int ticks) {
  int rnewTicks = ticks + rTicks;
  int lnewTicks = ticks + lTicks;

  while ((rnewTicks > rTicks) || (lnewTicks > lTicks)) {
    forward();
  }
}
void tReverse(int ticks) {
  int rnewTicks = ticks + rTicks;
  int lnewTicks = ticks + lTicks;

  while ((rnewTicks > rTicks) || (lnewTicks > lTicks)) {
    reverse();
  }
}

void pointTurn() { // 3 point turn logic
  _delay_ms(1000);
  tReverse(50);
  dontMove();
  rticksMove(183);// 183 save
  lticksReverse(230);//230 save
  dir = 0;
  updateSensor();
}

ISR(ADC_vect)
{
  // Grab ADC result
  unsigned char lsb = ADCL;
  unsigned char msb = ADCH;
  adcResult = lsb + (msb << 8);
  if (adcResult > 600) {
    digital = 1;
  }
  else {
    digital = 0;
  }
}

ISR(INT0_vect)
{
  rTicks += 1;
}

ISR(INT1_vect)
{
  lTicks += 1;
}
