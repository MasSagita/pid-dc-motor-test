/*
 * PID DC Motor
 * 
 * motor dc with encoder
 * 
 * Mas Sagita 2019
 */

const byte pinEncoderA = 2;
const byte pinEncoderB = 3;

const byte pinPwm = 6;
const byte pinDirA = 7;
const byte pinDirB = 8;

boolean startMotor = false;

int encoder = 0;
int motorDirection = 0;
int sv = 0;

int hallMagnet = 97;

double pv = 0;
double setRpm = 0;

double error = 0;
double prevError = 0;
double sumError = 0;

double pidOut = 0;

double kp = 0;
double ki = 0;
double kd = 0;

int timer1Counter;
int i = 0;

int counterData = 0;
int batasAtas = 600;
int batasBawah = 0;

void setup() {
  // put your setup code here, to run once:

  pinMode(pinEncoderA, INPUT_PULLUP);
  pinMode(pinEncoderB, INPUT_PULLUP);

  pinMode(pinDirA, OUTPUT);
  pinMode(pinDirB, OUTPUT);
  pinMode(pinPwm, OUTPUT);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(pinEncoderA), encoderA, RISING);

  //timer intterupt setup
  noInterrupts(); //disable all interupts
  TCCR1A = 0;
  TCCR1B = 0;

  //preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  timer1Counter = 59286;

  TCNT1 = timer1Counter;  //preload timer
  TCCR1B |= (1 << CS12);  //256 prescaler
  TIMSK1 |= (1 << TOIE1); //enable timer overflow interrupt
  interrupts();           //enable all interrupts

  analogWrite(pinPwm, 0);
  digitalWrite(pinDirA, 0);
  digitalWrite(pinDirB, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(pinDirA, 1);
  digitalWrite(pinDirB, 0);

  startMotor = 1;

  setRpm = 250;
  kp = 0.7;
  ki = 0.0;
  kd = 0.008;

  Serial.print(batasBawah);
  Serial.print("\t");
  Serial.print(batasAtas);
  Serial.print("\t");
  Serial.print(setRpm);
  Serial.print("\t");
  Serial.println(pv);
  Serial.print("\t");
  Serial.println(encoder);
}

void encoderA() {
  encoder += 1;                               //increasing encoder at new pulse
  motorDirection = digitalRead(pinEncoderB);  //read direction of motor
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Counter;
  pv = 60 * (((float)encoder / hallMagnet) / 0.1);
  encoder = 0;

  if (startMotor) {
    error = setRpm - pv;
    pidOut = (error * kp) + (sumError * ki) + ((error - prevError) * kd);
    prevError = error;
    sumError += error;

    if (sumError > 4000) sumError = 4000;
    if (sumError < -4000) sumError = -4000;
  }

  error = 0;
  prevError = 0;
  sumError = 0;

  if (pidOut < 255 & pidOut > 0) analogWrite(pinPwm, pidOut);
  else {
    if (pidOut > 255) analogWrite(pinPwm, 255);
    else analogWrite(pinPwm, 0);
  }
}

