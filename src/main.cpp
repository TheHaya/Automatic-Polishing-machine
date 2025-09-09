#include <Arduino.h>
#include <elapsedMillis.h>
#include <Rotary.h>
#include <LedController.hpp>

const int BUT1 = 17, BUT2 = 18;
const int LEDR = 42, LEDB = 41, LEDG = 40;
const int ROT_DT1 = 14, ROT_CLK1 = 13;
const int ROT_DT2 = 12, ROT_CLK2 = 11;
const int MOT_DIR = 1, MOT_STEP = 2;
const int MS1_PIN = 38, MS2_PIN = 37, MS3_PIN = 36;
const int SEG_DIN = 21, SEG_CLK = 16, SEG_LOAD = 39;
const int RELAY = 46;
volatile uint8_t valLeft_A = 0, valRight_A = 8;
volatile uint8_t valLeft_B = 0, valRight_B = 4;
volatile int currentSpeed = 0;
volatile int currentRots = 0;
volatile int enc_counter1 = 0;
volatile int enc_counter2 = 0;
//const byte SEG[] = {2, 42, 41, 40, 39, 38, 37, 36};
//const byte DIGIT_SEG[] = {16, 15, 18, 17}; 
//const uint8_t MAP_SEG_ROT[] = {0b11000000, 0b11111001, 0b10100100, 0b10110000, 0b10011001,
//                              0b10010010, 0b10000010, 0b11111000, 0b10000000, 0b10010000};

elapsedMillis relay_millis;
elapsedMillis but1Millis, but2Millis;
unsigned long buttonTimer = 200;
const int stepsRotate = 200;
const int microStep = 16;
bool but1Up, but2Up = HIGH;

bool but1Press, but2Press;
bool doStep = false;
bool doConst = false;
bool isBusy = false;
volatile int segCounter = valLeft_A * 10 + valRight_A; 
volatile int segSpdCounter = valLeft_B * 10 + valRight_B;
int segDelay = segSpdCounter * segCounter;
int motDelay; 
hw_timer_t* timer = nullptr;

Rotary enc1 = Rotary(ROT_DT1,ROT_CLK1);
Rotary enc2 = Rotary(ROT_DT2,ROT_CLK2);
LedController lc(SEG_DIN, SEG_CLK, SEG_LOAD, 1);

/*void IRAM_ATTR onTimer() {
  static uint8_t currentNum = 0;
  digitalWrite(DIGIT_SEG[currentNum], LOW);
  currentNum = (currentNum + 1) % 4;
  uint8_t pattern;
  switch (currentNum) {
    case 0: pattern = MAP_SEG_ROT[valRight_A];
            break;
    case 1: pattern = MAP_SEG_ROT[valLeft_A];
            break;
    case 2: pattern = MAP_SEG_ROT[valRight_B];
            break;
    case 3: pattern = MAP_SEG_ROT[valLeft_B];
            break;
  } 
  for(int i = 0; i < 8; ++i){
    digitalWrite(SEG[i], (pattern >> i) & 0b00000001);
  }
  digitalWrite(DIGIT_SEG[currentNum], HIGH);
}*/

void IRAM_ATTR enc1ISR(){
  unsigned char result1 = enc1.process();
  if(!isBusy){
    if (result1 == DIR_CW) {
      enc_counter1++;
      segCounter += 1;
    } else if (result1 == DIR_CCW) {
      enc_counter1--;
      segCounter -= 1;
    }
    segCounter  = constrain(segCounter, 1, 15);
    valLeft_A = segCounter / 10;
    valRight_A = segCounter % 10;
  }
 
  /*static int reqFlancs = 0;
  static bool lastEncState = HIGH;
  bool encState = digitalRead(ROT_CLK1);
  bool encFlank = digitalRead(ROT_DT1);
  if(encState != lastEncState && encState == LOW){
    reqFlancs += 1;
    if(reqFlancs == 4){
      segCounter += (encFlank ? +1 : -1);
      segCounter  = constrain(segCounter, 1, 15);
      valLeft_A = segCounter / 10;
      valRight_A = segCounter % 10;
      reqFlancs = 0;
    }
  }
  lastEncState = encState;*/
}

void IRAM_ATTR enc2ISR(){
  unsigned char result2 = enc2.process();
  if(!isBusy){
    if (result2 == DIR_CW) {
      enc_counter2++;
      segSpdCounter += 1;
    } else if (result2 == DIR_CCW) {
      enc_counter2--;
      segSpdCounter -= 1;
    }
    segSpdCounter  = constrain(segSpdCounter, 1, 15);
    valLeft_B = segSpdCounter / 10;
    valRight_B = segSpdCounter % 10;
  }
  /*static bool lastEncState = HIGH;
  bool encState = digitalRead(ROT_CLK2);
  bool encFlank = digitalRead(ROT_DT2);
  if(encState != lastEncState && encState == LOW){
    segSpdCounter += (encFlank ? +1 : -1);
    segSpdCounter  = constrain(segSpdCounter, 1, 25);
    valLeft_B = segSpdCounter / 10;
    valRight_B = segSpdCounter % 10;
  }
  lastEncState = encState; */
}

void setup() {
  Serial.begin(115200);
  pinMode(BUT1, INPUT_PULLUP);
  pinMode(BUT2, INPUT_PULLUP);
  pinMode(ROT_CLK1, INPUT_PULLUP);
  pinMode(ROT_DT1, INPUT_PULLUP);
  pinMode(ROT_CLK2, INPUT_PULLUP);
  pinMode(ROT_DT2, INPUT_PULLUP);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(MOT_STEP, OUTPUT);
  pinMode(MOT_DIR, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);

  lc.setScanLimit(0, 3);
  lc.activateAllSegments();
  lc.setIntensity(7);
  lc.clearMatrix();
  /*for(byte p : SEG){
    pinMode(p, OUTPUT);
    digitalWrite(p, HIGH);
  }
  for(byte p : DIGIT_SEG){
  pinMode(p, OUTPUT);
  digitalWrite(p, HIGH);
  }*/
  digitalWrite(LEDG, HIGH);
  digitalWrite(MOT_DIR, HIGH);
  digitalWrite(RELAY, HIGH);
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, HIGH);
  attachInterrupt(ROT_CLK1, enc1ISR, CHANGE);
  attachInterrupt(ROT_DT1, enc1ISR, CHANGE);
  attachInterrupt(ROT_CLK2, enc2ISR, CHANGE);
  attachInterrupt(ROT_DT2, enc2ISR, CHANGE);
/*
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 50000, true);
  timerAlarmEnable(timer); */
}

void writeDigits(){
  lc.setDigit(0, 0, valLeft_A, false);
  lc.setDigit(0, 1, valRight_A, false);
  lc.setDigit(0, 2, valLeft_B, false);
  lc.setDigit(0, 3, valRight_B, false);
}

void userRotate(){
  long totalSteps = segCounter  * stepsRotate * microStep;
  bool lastBut1 = HIGH;
  elapsedMillis cancelBut = 0;
  isBusy = true;
  motDelay = 6 / (segSpdCounter * 0.0004) / microStep;
  for(long i = 0; i < totalSteps; i++){
    digitalWrite(MOT_STEP, HIGH);
    delayMicroseconds(motDelay);
    digitalWrite(MOT_STEP, LOW);
    delayMicroseconds(motDelay);
    bool currentBut1 = digitalRead(BUT1);
    if(lastBut1 == HIGH && currentBut1 == LOW && cancelBut > buttonTimer) {
      break;
    }
    lastBut1 = currentBut1;
  }
  digitalWrite(LEDG, !digitalRead(LEDG));
  digitalWrite(LEDR, !digitalRead(LEDR));
  isBusy = false;
}

void infRotate(){
  bool lastBut2 = HIGH;
  elapsedMillis infButton = 0;
  isBusy = true;
  motDelay = 6 / (segSpdCounter * 0.0004) / microStep;
  while(true){
    digitalWrite(MOT_STEP, HIGH);
    delayMicroseconds(motDelay);
    digitalWrite(MOT_STEP, LOW);
    delayMicroseconds(motDelay);
    bool currentBut2 = digitalRead(BUT2);
    if(lastBut2 == HIGH && currentBut2 == LOW && infButton > buttonTimer) {
      digitalWrite(LEDG, !digitalRead(LEDG));
      digitalWrite(LEDB, !digitalRead(LEDB));
      break;
    }
    lastBut2 = currentBut2;
  }
  isBusy = false;
}

void loop() {
  if(digitalRead(BUT1) == LOW){
    printf("button pressed");
  }
  if(!isBusy){
    but1Press = digitalRead(BUT1);
    if(but1Up == HIGH && but1Press == LOW && but1Millis > buttonTimer) {
      //relay_millis = 0;
      digitalWrite(LEDR, !digitalRead(LEDR));
      digitalWrite(LEDG, !digitalRead(LEDG));
      digitalWrite(RELAY, LOW);
      //while(relay_millis <= 1000);
      userRotate();
      but1Millis = 0;
      digitalWrite(RELAY, HIGH);
    }
  but1Up = but1Press;
  }
  
  if(!isBusy){
    but2Press = digitalRead(BUT2);
    if(but2Up == HIGH && but2Press == LOW && but2Millis > buttonTimer) {
      //relay_millis = 0;
      digitalWrite(LEDG, !digitalRead(LEDG));
      digitalWrite(LEDB, !digitalRead(LEDB));
      digitalWrite(RELAY, LOW);
      //while(relay_millis <= 1000);
      infRotate();
      but2Millis = 0;
      digitalWrite(RELAY, HIGH);
    }
    but2Up = but2Press;
  }

  writeDigits();
} 