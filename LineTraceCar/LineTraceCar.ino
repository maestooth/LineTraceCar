#include <TaskScheduler.h>
#include <IRremote.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Callback methods prototypes
// High level decision
void determineDirection();
// Sensor Read
void reaadSensor();
// Motor output
void motorOutput();
void irSensor();

// ---------------------------------------------------------------------------
//Tasks
// ---------------------------------------------------------------------------
Scheduler runner;
Task tDetermineDirection(300, TASK_FOREVER, &determineDirection);
Task tReaadSensor(50, TASK_FOREVER, &reaadSensor);
Task tMotorOutput(250, TASK_FOREVER, &motorOutput);
Task tIrSensor(100, TASK_FOREVER, &irSensor);
// task flag
boolean isMove = false;
boolean isInitialized = false;

// ---------------------------------------------------------------------------
// Servo
// ---------------------------------------------------------------------------
Servo leftServo;
#define leftServoPin 10
Servo rightServo;
#define rightServoPin 9

// forward fast
#define LF_FAST 100
#define RF_FAST 80
// forward slow
#define LF_SLOW 95
#define RF_SLOW 84
// stop
#define SERVOSTOP 90
// back fast
#define LB_FAST 86
#define RB_FAST 80
// back slow
#define LB_SLOW 85
#define RB_SLOW 100

// Flags
int BIT_FLAG_LEFT = 0b0001;
int BIT_FLAG_CENTER = 0b0010;
int BIT_FLAG_RIGHT = 0b0100;

int BIT_FLAG_STRAIGHT = BIT_FLAG_CENTER;
int BIT_FLAG_LEFT_1 = BIT_FLAG_LEFT;
int BIT_FLAG_RIGHT_1 = BIT_FLAG_RIGHT;
int BIT_FLAG_LEFT_2 = BIT_FLAG_LEFT | BIT_FLAG_CENTER;
int BIT_FLAG_RIGHT_2 = BIT_FLAG_RIGHT | BIT_FLAG_CENTER;

// ---------------------------------------------------------------------------
// Line Sensor
// ---------------------------------------------------------------------------
const int lineLeft = 14;
const int lineCenter = 15;
const int lineRight = 16;

#define LINE_THRESHOLD 300
// value array
long valuesLeft[5];
long valuesCenter[5];
long valuesRight[5];
int arrayIndex = 0;
// average
long avarageLeft = 0;
long avarageCenter = 0;
long avarageRight = 0;
// flag
long fServo = 0;
long fServoPrev = 0;
boolean flagStart = false;

// ---------------------------------------------------------------------------
// IRSensor
// ---------------------------------------------------------------------------
/*
*  Default is Arduino pin D8.
*  You can change this to another available Arduino Pin.
*  Your IR receiver should be connected to the pin defined here
*/
int RECV_PIN = 8;
#define CODE_UP 0x810
IRrecv irrecv(RECV_PIN);
decode_results results;
// ---------------------------------------------------------------------------
// States
// ---------------------------------------------------------------------------
int servoStateLeft = SERVOSTOP;
int servoStateRight = SERVOSTOP;

int cnt = 0;

void irSensor() {
  if (irrecv.decode(&results)) {
    switch (results.value) {
      case CODE_UP:
        if (isMove) {
          tReaadSensor.disable();
          runner.deleteTask(tReaadSensor);
          tMotorOutput.disable();
          runner.deleteTask(tMotorOutput);
          tDetermineDirection.disable();
          runner.deleteTask(tDetermineDirection);
          servoStop();
          arrayIndex = 0;
          isInitialized = false;
          isMove = false;
        } else {
          runner.addTask(tReaadSensor);
          tReaadSensor.enable();
          runner.addTask(tMotorOutput);
          tMotorOutput.enable();
          runner.addTask(tDetermineDirection);
          tDetermineDirection.enable();
          isMove = true;
        }
        break;
    }
    irrecv.resume(); // Receive the next value
  }
}

void determineDirection() {
  if (!isInitialized) {
    Serial.println("arrayIndex <= 4");
    return;
  }
  // 状態を初期化する
  arrayIndex = 0;
  // 平均を出す
  for (int i = 0; i < 5; i++) {
    avarageLeft = avarageLeft + valuesLeft[arrayIndex];
    avarageCenter = avarageCenter + valuesCenter[arrayIndex];
    avarageRight = avarageRight + valuesRight[arrayIndex];
    if (i == 4) {
      avarageLeft = avarageLeft / 5;
      avarageCenter = avarageCenter / 5;
      avarageRight = avarageRight / 5;
    }
  }
  Serial.print("(L,C,R) = (");
  Serial.print(avarageLeft);
  Serial.print(", ");
  Serial.print(avarageCenter);
  Serial.print(", ");
  Serial.print(avarageRight);
  Serial.println(")");

  // 状態を保持
  if (avarageCenter >= LINE_THRESHOLD) {
    fServo = BIT_FLAG_CENTER;
  }
  if (avarageLeft >= LINE_THRESHOLD) {
    fServo = fServo | BIT_FLAG_LEFT;
  }
  if (avarageRight >= LINE_THRESHOLD) {
    fServo = fServo | BIT_FLAG_RIGHT;
  }
  Serial.println(fServo, BIN);
  // 進行方向を決める
  if (fServo == BIT_FLAG_LEFT) {
    servoStateLeft = LF_SLOW;
    servoStateRight = RB_SLOW;
  } else if (fServo == BIT_FLAG_CENTER || fServo == BIT_FLAG_RIGHT) {
    servoStateLeft = LF_FAST;
    servoStateRight = SERVOSTOP;
  } else {
    servoStateLeft = LF_SLOW;
    servoStateRight = RF_FAST;
  }
  //  if (fServo == BIT_FLAG_LEFT_2) {
  //    servoStateLeft = LB_SLOW;
  //    servoStateRight = RF_SLOW;
  //  } else if (fServo == BIT_FLAG_RIGHT_2) {
  //    servoStateLeft = LF_SLOW;
  //    servoStateRight = RB_SLOW;
  //  } else  if (fServo == BIT_FLAG_LEFT_1) {
  //    servoStateLeft = LB_SLOW;
  //    servoStateRight = RF_FAST;
  //  } else if (fServo == BIT_FLAG_RIGHT_1) {
  //    servoStateLeft = LF_FAST;
  //    servoStateRight = RB_SLOW;
  //  } else if (fServo == BIT_FLAG_STRAIGHT) {
  //    servoStateLeft = LF_SLOW;
  //    servoStateRight = RF_SLOW;
  //  } else {
  //    servoStateLeft = LF_SLOW;
  //    servoStateRight = RF_SLOW;
  //  }
  fServo = 0;
}

void reaadSensor() {
  if (arrayIndex > 4) {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);              // wait for a second
    isInitialized = true;
    Serial.println("isInitialized = true");
    return;
  }
  valuesLeft[arrayIndex] = analogRead(lineLeft);
  valuesCenter[arrayIndex] = analogRead(lineCenter);
  valuesRight[arrayIndex] = analogRead(lineRight);
  arrayIndex++;
}

void motorOutput() {
  moveServo(servoStateLeft, servoStateRight);
}

void moveServo(int left, int right) {
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  leftServo.write(left);
  rightServo.write(right);
}

void servoStop() {
  Serial.println("servoStop");
  rightServo.write(SERVOSTOP);
  rightServo.detach();
  leftServo.write(SERVOSTOP);
  leftServo.detach();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  irrecv.enableIRIn(); // Start the receiver

  runner.init();
  runner.addTask(tIrSensor);
  tIrSensor.enable();
}

void loop() {
  // put your main code here, to run repeatedly:
  runner.execute();
}
