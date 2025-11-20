//#include "TCS34725.h"
//TCS34725 tcs;

int in11 = 4;
int in21 = 5;
int in31 = 6;
int in41 = 7;
int in12 = 8;
int in22 = 9;
int in32 = 10;
int in42 = 11;
unsigned long Time = 0;
unsigned long Timeout = 1100;

void setup() {
  pinMode(in11, OUTPUT);
  pinMode(in21, OUTPUT);
  pinMode(in31, OUTPUT);
  pinMode(in41, OUTPUT);
  pinMode(in12, OUTPUT);
  pinMode(in22, OUTPUT);
  pinMode(in32, OUTPUT);
  pinMode(in42, OUTPUT);
  Serial.begin(9600);
  /*
  Wire.begin();
  if (!tcs.attach(Wire))
        Serial.println("ERROR: TCS34725 NOT FOUND !!!");
      
  tcs.integrationTime(33); // ms
  tcs.gain(TCS34725::Gain::X01);*/
}


void loop() {
  /*
  if (tcs.available()) // if current measurement has done
    {
        TCS34725::Color color = tcs.color();
        Serial.println(tcs.lux());
        delay(100);
    }*/

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Time = millis();
    if (cmd == "0"){
      mv_stop();
    } else if (cmd == "2") {
      mv_rot_l();
    } else if (cmd == "1") {
      mv_front();
    } else if (cmd == "3") {
      mv_rot_r();
    }
  }

  if (millis() - Time > Timeout) {
    mv_stop();
  }
}

void mv_front(){
  digitalWrite(in11, 0);
  digitalWrite(in21, 1);
  digitalWrite(in31, 0);
  digitalWrite(in41, 1);
  digitalWrite(in12, 0);
  digitalWrite(in22, 1);
  digitalWrite(in32, 0);
  digitalWrite(in42, 1);
}

void mv_stop(){
  digitalWrite(in11, 0);
  digitalWrite(in21, 0);
  digitalWrite(in31, 0);
  digitalWrite(in41, 0);
  digitalWrite(in12, 0);
  digitalWrite(in22, 0);
  digitalWrite(in32, 0);
  digitalWrite(in42, 0);
}

void mv_rot_l(){
  digitalWrite(in11, 1);
  digitalWrite(in21, 0);
  digitalWrite(in31, 0);
  digitalWrite(in41, 1);
  digitalWrite(in12, 0);
  digitalWrite(in22, 1);
  digitalWrite(in32, 1);
  digitalWrite(in42, 0);
}

void mv_rot_r(){
  digitalWrite(in11, 0);
  digitalWrite(in21, 1);
  digitalWrite(in31, 1);
  digitalWrite(in41, 0);
  digitalWrite(in12, 1);
  digitalWrite(in22, 0);
  digitalWrite(in32, 0);
  digitalWrite(in42, 1);
}
