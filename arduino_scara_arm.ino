#include <Servo.h>

Servo servoBaza;
Servo servoLokiec;

int obecnyBaza = 30;
int obecnyLokiec = 160; 
int predkosc = 40; 

const int pinMagnesIN1 = 4;
const int pinMagnesIN2 = 5;

void setup() {
  servoBaza.attach(9);
  servoLokiec.attach(10);
  servoBaza.write(obecnyBaza);
  servoLokiec.write(obecnyLokiec);

  
  pinMode(pinMagnesIN1, OUTPUT);
  pinMode(pinMagnesIN2, OUTPUT);
  pusc(); 
  
  Serial.begin(9600);
}

void chwyc() {
  digitalWrite(pinMagnesIN1, HIGH);
  digitalWrite(pinMagnesIN2, LOW);
}

void pusc() {
  digitalWrite(pinMagnesIN1, LOW);
  digitalWrite(pinMagnesIN2, HIGH);
  delay(50); 
  digitalWrite(pinMagnesIN1, LOW);
  digitalWrite(pinMagnesIN2, LOW);
}

void plynnyRuch(int celBaza, int celLokiec) {
  while (obecnyBaza != celBaza || obecnyLokiec != celLokiec) {
    if (obecnyBaza < celBaza) obecnyBaza++;
    else if (obecnyBaza > celBaza) obecnyBaza--;
    if (obecnyLokiec < celLokiec) obecnyLokiec++;
    else if (obecnyLokiec > celLokiec) obecnyLokiec--;
    servoBaza.write(obecnyBaza);
    servoLokiec.write(obecnyLokiec);
    delay(predkosc);
  }
}

void sekwencyjnyRuch(int celBaza, int celLokiec) {
  while (obecnyBaza != celBaza) {
    if (obecnyBaza < celBaza) obecnyBaza++;
    else if (obecnyBaza > celBaza) obecnyBaza--;
    servoBaza.write(obecnyBaza);
    delay(predkosc);
  }
  while (obecnyLokiec != celLokiec) {
    if (obecnyLokiec < celLokiec) obecnyLokiec++;
    else if (obecnyLokiec > celLokiec) obecnyLokiec--;
    servoLokiec.write(obecnyLokiec);
    delay(predkosc);
  }
}

void wykonajCyklPickAndPlace(int celBaza, int celLokiec, int typDetalu) {
  int tempZrzutBaza;
  int tempZrzutLokiec;

  if (typDetalu == 0) { //0 to Śruba
    tempZrzutBaza = 109;
    tempZrzutLokiec = 160;
  } else {              
    tempZrzutBaza = 130; 
    tempZrzutLokiec = 160;
  }

  plynnyRuch(celBaza, celLokiec);
  delay(200); 
  chwyc();
  delay(500); 
  
  sekwencyjnyRuch(tempZrzutBaza, tempZrzutLokiec); // Zrzut zależny od obiektu
  delay(200);
  pusc();
  delay(500);
  
  sekwencyjnyRuch(30, 160);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); 
    data.trim();
    
    if (data == "P") {
      sekwencyjnyRuch(30, 160);
      return;
    }

    int firstComma = data.indexOf(',');
    int secondComma = data.lastIndexOf(',');
    
    if (firstComma > 0 && secondComma > firstComma) {
      int docelowyBaza = data.substring(0, firstComma).toInt();
      int docelowyLokiec = data.substring(firstComma + 1, secondComma).toInt();
      int typDetalu = data.substring(secondComma + 1).toInt();
      
      docelowyBaza = constrain(docelowyBaza, 10, 150);
      docelowyLokiec = constrain(docelowyLokiec, 10, 170);
      
      wykonajCyklPickAndPlace(docelowyBaza, docelowyLokiec, typDetalu);
    }
  }
}