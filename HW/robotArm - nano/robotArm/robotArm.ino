//#include "pinout.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "fanControl.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet2.h>
#include <EthernetUdp2.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 0, 102);

unsigned int localPort = 30000; 
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
EthernetUDP Udp;
//char  ReplyBuffer[] = "acknowledged"; 
//#define LED_PIN 38
//내가 만든 보드에서는 X가 Z에 해당하고 Z가 X에 해당한다
#define Z_STEP_PIN 7
#define Y_STEP_PIN 5
#define X_STEP_PIN 3
#define Z_DIR_PIN 6
#define Y_DIR_PIN 4
#define X_DIR_PIN 2
#define Z_ENABLE_PIN 8
#define Y_ENABLE_PIN A1
#define X_ENABLE_PIN A2
#define PUMP_PIN A0 //밸브랑 펌프랑 같은신호
//#define VALVE_PIN A1
#define LAMP_G A3
#define LAMP_Y A4
#define LAMP_R A5
//#define LAMP_B A7 //사용불가
#define EMG_STOP 9

//#include <Stepper.h>
#include <Servo.h>

Servo pump; 
Servo valve;
bool gripper_state = false;
unsigned long main_t = 0;

//Stepper stepper(2400, STEPPER_GRIPPER_PIN_0, STEPPER_GRIPPER_PIN_1, STEPPER_GRIPPER_PIN_2, STEPPER_GRIPPER_PIN_3);
RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
//RampsStepper stepperExtruder(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN);
//FanControl fan(FAN_PIN);
RobotGeometry geometry;
Interpolation interpolator;
Queue<Cmd> queue(15);
Command command;

float h_offset = 0;
float l_offset = 0;

bool old_emg_stop = HIGH;

void setup() {
  Serial.begin(9600);  

  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  pinMode(LAMP_G,OUTPUT);
  pinMode(LAMP_Y,OUTPUT);
  pinMode(LAMP_R,OUTPUT);
  //pinMode(LAMP_B,OUTPUT);
  pinMode(EMG_STOP,INPUT_PULLUP);
  
  //reduction of steppers..
  stepperHigher.setReductionRatio(32.0 / 9.0, 200 * 16);  //big gear: 32, small gear: 9, steps per rev: 200, microsteps: 16
  stepperLower.setReductionRatio( 32.0 / 9.0, 200 * 16);
  stepperRotate.setReductionRatio(32.0 / 9.0, 200 * 16);
  //stepperExtruder.setReductionRatio(32.0 / 9.0, 200 * 16);
  
  //start positions..
  stepperHigher.setPositionRad(0);  //90°
  stepperLower.setPositionRad(0);          // 0°
  stepperRotate.setPositionRad(0);         // 0°
  //stepperExtruder.setPositionRad(0);
  
  //enable and init..
  setStepperEnable(false);
  //interpolator.setC  yurrentPos(0,19.5,134,0);
  interpolator.setInterpolation(0,19.5,134,0, 0,19.5,134,0);
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  //geometry.set(0,19.5,134);
  //Serial.println("start");

  h_offset = geometry.getHighRad();
  l_offset = geometry.getLowRad();
  //Serial.print(h_offset);
  //Serial.print(", ");
  //Serial.print(l_offset);
  //Serial.println();
}

void setStepperEnable(bool enable) {
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable); 
  //stepperExtruder.enable(enable); 
  //fan.enable(enable);
}

void loop () {
  /*
  if(Serial.available()){
    char c = Serial.read();
    if(c == '0'){
      pump.write(0);
      pump.detach();
      valve.write(0);
      valve.detach();
      gripper_state = false;
    }else if(c == '1'){
      pump.attach(PUMP_PIN);
      pump.write(180);
      valve.attach(VALVE_PIN);
      valve.write(180);
      gripper_state = true;
    }
  }
  */
  
  //수신한게 있으면~
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    String gcode = packetBuffer;
    command.processMessage(gcode);
    queue.push(command.getCmd());
  }
  /*
  if(digitalRead(btn1)==LOW){
    pump.attach(PUMP_PIN);
    pump.write(180);
    valve.attach(VALVE_PIN);
    valve.write(180);
    gripper_state = true;
  }
  if(digitalRead(btn2)==LOW){
    pump.write(0);
    pump.detach();
    valve.write(0);
    valve.detach();
    gripper_state = false;
  }
  */
  if(millis() - main_t > 100){
    main_t = millis();
    /*
    Serial.print(stepperRotate.getPosition()); //기어비 반영
    Serial.print(", ");
    Serial.print(stepperLower.getPosition()); //기어비 반영
    Serial.print(", ");
    Serial.print(stepperHigher.getPosition()); //기어비 반영
    */
    
    String data = String(geometry.getXmm()) + "," +
                String(geometry.getYmm()) + "," +
                String(geometry.getZmm()) + "," +
                String(geometry.getRotRad()*(180/PI)) + "," +
                String((geometry.getLowRad()-l_offset)*(180/PI)) + "," +
                String((geometry.getHighRad()-h_offset)*(180/PI)) + "," +
                String(interpolator.isFinished()) + "," +
                gripper_state;

    Udp.beginPacket("192.168.0.100", 60001);
    Udp.write(data.c_str());
    Udp.endPacket();

  }

  //update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad (geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  //stepperExtruder.stepToPositionRad(interpolator.getEPosmm());
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update(); 
  //fan.update();
  
  if (!queue.isFull()) {
    if (command.handleGcode()) {
      queue.push(command.getCmd());
      printOk();
    }
  }
  if ((!queue.isEmpty()) && interpolator.isFinished()) {
    executeCommand(queue.pop());
  }
}

void cmdMove(Cmd (&cmd)) {
  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
}
void cmdDwell(Cmd (&cmd)) {
  delay(int(cmd.valueT * 1000));
}
void cmdGripperOn(Cmd (&cmd)) {
  pump.attach(PUMP_PIN);
  pump.write(180);
  //valve.attach(VALVE_PIN);
  //valve.write(180);
  gripper_state = true;
  /*
  stepper.setSpeed(5);
  stepper.step(int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
  */
}
void cmdGripperOff(Cmd (&cmd)) {
  pump.write(0);
  pump.detach();
  //valve.write(0);
  //valve.detach();
  gripper_state = false;
  /*
  stepper.setSpeed(5);
  stepper.step(-int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
  */
}
void cmdStepperOn() {
  setStepperEnable(true);
}
void cmdStepperOff() {
  setStepperEnable(false);
} 
void cmdFanOn() {
  //fan.enable(true);
}
void cmdFanOff() {
  //fan.enable(false);
}

void handleAsErr(Cmd (&cmd)) {
  printComment("Unknown Cmd " + String(cmd.id) + String(cmd.num) + " (queued)"); 
  printFault();
}

void executeCommand(Cmd cmd) {
  if (cmd.id == -1) {
    String msg = "parsing Error";
    printComment(msg);
    handleAsErr(cmd);
    return;
  }
  
  if (cmd.valueX == NAN) {
    cmd.valueX = interpolator.getXPosmm();
  }
  if (cmd.valueY == NAN) {
    cmd.valueY = interpolator.getYPosmm();
  }
  if (cmd.valueZ == NAN) {
    cmd.valueZ = interpolator.getZPosmm();
  }
  if (cmd.valueE == NAN) {
    cmd.valueE = interpolator.getEPosmm();
  }
  
   //decide what to do
  if (cmd.id == 'G') {
    switch (cmd.num) {
      case 0: cmdMove(cmd); break;
      case 1: cmdMove(cmd); break;
      case 4: cmdDwell(cmd); break;
      //case 21: break; //set to mm
      //case 90: cmdToAbsolute(); break;
      //case 91: cmdToRelative(); break;
      //case 92: cmdSetPosition(cmd); break;
      default: handleAsErr(cmd); 
    }
  } else if (cmd.id == 'M') {
    switch (cmd.num) {
      //case 0: cmdEmergencyStop(); break;
      case 3: cmdGripperOn(cmd); break;
      case 5: cmdGripperOff(cmd); break;
      case 17: cmdStepperOn(); break;
      case 18: cmdStepperOff(); break;
      case 106: cmdFanOn(); break;
      case 107: cmdFanOff(); break;
      default: handleAsErr(cmd); 
    }
  } else {
    handleAsErr(cmd); 
  }
}