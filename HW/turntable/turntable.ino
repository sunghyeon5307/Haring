#include <AccelStepper.h>
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet2.h>
#include <EthernetUdp2.h>
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 0, 103);
unsigned int localPort = 30000; 
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
EthernetUDP Udp;
// 스텝 모터 핀 설정
#define STEP_PIN 2
#define DIR_PIN 5
#define Z_STEP_PIN 7
#define Y_STEP_PIN 5
#define X_STEP_PIN 3
#define Z_DIR_PIN 6
#define Y_DIR_PIN 4
#define X_DIR_PIN 2
AccelStepper stepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);

unsigned long t = 0;
const int stepsPerDegree = 8;

void setup() {
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  Serial.begin(115200); // 시리얼 통신 시작
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
}

void loop() {
  /*if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // G코드 읽기
    processCommand(command);
  }*/
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      String command = packetBuffer;
  }

  /*if(millis() - main_t > 100){
    main_t = millis();
    Udp.beginPacket("192.168.0.100", 60000);
    Udp.write(data.c_str());
    Udp.endPacket();
  }*/
}

// G코드 처리
void processCommand(String command) {
  if (command.startsWith("G68 R")) {
    int angle = command.substring(5).toInt();
    rotateMotor(angle);
  }
  if (command.startsWith("G67 R")) {
    int angle = command.substring(5).toInt();
    rotateMotor2(angle);
  }
}

// 모터 회전 함수
void rotateMotor(int degrees) {
  if (degrees < 0) {
    Serial.println("Invalid angle");
    return;
  }
  
  long steps = degrees * stepsPerDegree;
  stepper.moveTo(steps);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  Serial.println("1");
}
void rotateMotor2(int degrees) {
  if (degrees < 0) {
    Serial.println("Invalid angle");
    return;
  }
  
  long steps = degrees * stepsPerDegree;
  stepper2.moveTo(steps);
  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }
  Serial.println("1");
}