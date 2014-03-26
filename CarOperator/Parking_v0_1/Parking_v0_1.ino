#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <util.h>
#include <SPI.h>
#include <Servo.h>
#include <TimedAction.h>
int pinSpeed=7;
int pinLenkung=4;
boolean run;
Servo servoLenkung,servoSpeed; //Die Objekte zur Steuerung der Servos

void setup(){
  servoSpeed.attach(pinSpeed);
  servoLenkung.attach(pinLenkung); 
   
  run =true;
}

void loop(){
  if(run==true){
     servoLenkung.write(90);
     delay(1000);
     servoSpeed.write(78);
     delay(100);
     servoSpeed.write(30);
     delay(3000);
     servoLenkung.write(110);
     delay(3000);
     servoSpeed.write(68);
     delay(1000);
     run=false;
  }
}
