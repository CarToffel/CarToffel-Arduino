#include <Average.h>
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

int pinTriggerFront = 30;  //Der Trigger-Pin des Distanzsensors GELB
int pinEchoFront = 31;     //Der Echo-Pin des Distanzsensors BLAU 

int pinTriggerBack = 32;  //Der Trigger-Pin des Distanzsensors GELB
int pinEchoBack = 33;  //Der Echo-Pin des Distanzsensors BLAU

int pinTriggerRight = 34; //Der Trigger-Pin des Distanzsensors GELB
int pinEchoRight=35;  //Der Echo-Pin des Distanzsensors BLAU

int pinTriggerLeft = 36;  //Der Trigger-Pin des Distanzsensors GELB
int pinEchoLeft = 37;  //Der Echo-Pin des Distanzsensors BLAU

int pinLenkung = 4;   //Der Pin für die Lenkung
int pinSpeed = 7;     //Die PWM-Pins der Servos

int currentSpeed = 80; //Die derzeitige Geschwindigkeit
int currentAngle = 90;  //Der Aktuelle Winkel des Lenkservos 90=neutral >90 = rechts <90=links; Maximalwerte 50 und 130

Servo servoLenkung,servoSpeed; //Die Objekte zur Steuerung der Servos
            
int packetSize =0;             //Die Größe des angekommenen Packets
long distanceFront[5] = {0,0,0,0,0};            //Die Ditanz des Abstandssensor
long distanceBack[5] = {0,0,0,0,0};  
long distanceRight[5] = {0,0,0,0,0};  
long distanceLeft[5] = {0,0,0,0,0};  

long averageFront =0;
long averageBack =0;
long averageRight=0;
long averageLeft=0;



char currentOrder;            //Der derzeitige Befehl

TimedAction sensortimer = TimedAction(100,readSensor);      //Der Timer für die Sensorabrufe
TimedAction distancetimer = TimedAction(1000,sendDistance);  //Der Timer für das Senden der Distanz
TimedAction statustimer = TimedAction(1000,printStatus);     //Der Timer für die Ausgabe des Status
  
byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // Die MAC-Adresse des Arduino
IPAddress ip(10,0,0,99);                // IP des Arduino
IPAddress remote(10,0,0,101);           // IP des Client (Smartphone)
IPAddress gateway(10,0,0,1);            // Gateway
IPAddress subnet(255,255,255,0);        // Subnetadresse

int remoteport=8888;                      //Der Port unseres Clienten(Smartphone)
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //Der Buffer in dem unsere empfangenen Plakete gelagert werden
EthernetUDP Udp;                            //Das Objekt fÃ¼r unsere UDP-Verbindung

void setup(){
  servoLenkung.attach(pinLenkung);        //WeiÃŸt dem Lenkservo den Pin 2 zu
  servoSpeed.attach(pinSpeed);            //WeiÃŸt dem Speedervo den Pin 3 zu
  servoSpeed.write(currentSpeed);
  servoLenkung.write(currentAngle);
  
  initializeNetwork();
  initializeSensors();
}

void loop(){
  receiveOrder(); //liefert den Befehl vom Ethernetshield
  processOrder(currentOrder,servoLenkung, servoSpeed); //verarbeitet den erhaltenen Befehl
  avoidCollision();
  sensortimer.check();      //checkt jede Sekunde die Sensordaten
  distancetimer.check();    //schickt jede Sekunde die derzeitige Distanz zum Clienten
  statustimer.check();      //schickt jede Sekunde den derzeitigen Status an die Console
  delay(10);
}

void initializeSensors(){
  pinMode(pinEchoFront, INPUT);                //Setzt den PinMode für den Echo-Pin
  pinMode(pinTriggerFront, OUTPUT);            //Setzt den PinMode für den Trigger-Pin

  pinMode(pinEchoBack, INPUT);                
  pinMode(pinTriggerBack, OUTPUT);
  
  pinMode(pinEchoRight,INPUT);               
  pinMode(pinTriggerRight, OUTPUT);
  
  pinMode(pinEchoLeft, INPUT);                
  pinMode(pinTriggerLeft, OUTPUT);
}

void initializeNetwork(){
  Ethernet.begin(mac,ip,gateway,subnet);  //Ethernet wird initialisiert
  Udp.begin(8888);                        //Startet den UDP-Server auf Port 8888

  Serial.begin(9600);
  Serial.println("Server-IP: ");
  Serial.println(Ethernet.localIP());     // Ausgabe der eigenen IP
  Serial.println("Waiting 3 Seconds for Car to boot");  //Startprozess des Autos abwarten
  delay(1000);
  Serial.println("1");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("3");
}

void receiveOrder(){ // Diese Methode dient dazu Befehle aus dem WLAN-Netz auszulesen
  packetSize = Udp.parsePacket();  // Schaut nach ob ein Paket gekommen ist
  if(packetSize){                      // Ist  ein Paket gekommen, wird dieses verarbeitet 
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);  //liest das Paket in den Buffer ein
    currentOrder = packetBuffer[0];  //holt sich den ersten Wert aus dem Buffer (unser Befehl)
    remote =  Udp.remoteIP();    //Holt sich die IP des Clienten
    remoteport = Udp.remotePort();   //Holt sich den Port des Clienten
  }
  else {
    currentOrder = 'x';
  }
}

void processOrder(char order,Servo lenkung, Servo speeds){ //Diese Methode dient dazu den erhaltenen Wert aus receiveOrder zu verarbeiten
  currentAngle = lenkung.read(); //liest den derzeitigen Wert des Lenkservos aus 
  switch (currentOrder){
    case 's': //s ... stop
      speeds.write(75);
      break;
    case 'f': //f...forward
      if(currentSpeed < 86){ //Maximalwerte varrieren je nach verwendetem Akku: genutzt wird bei diesen Werten ein 7-Zellen Akku mit 3300mAh
      currentSpeed=currentSpeed+1;  //ErhÃ¶ht die derzeitige Geschwindigkeit
         if(currentSpeed <= 79 && currentSpeed >= 70){
           currentSpeed=80;
         }   
         speeds.write(currentSpeed);   //Schreibt den Wert in den Servo     
      }
      break;
    case 'b': //b...backward
      if(currentSpeed > 65){
        currentSpeed=currentSpeed-1;  //Senkt die derzeitige Geschwindigkeit 
        if(currentSpeed >= 70 && currentSpeed <= 79){
          currentSpeed = 70;
        }
        
        speeds.write(currentSpeed);   //siehe case 'f'
      }  
      break;
    case 'l': //l ...left 
      if(currentAngle-5 >= 65 && currentAngle-5 <= 115){
        currentAngle = currentAngle - 5; //Ã„ndert den derzeitigen Lenkwinkel         
      }
      else{   // ist der Wert des Servos ausserhalb des Wertebereichs, wird er hier entsprechend korrigiert
        if(currentAngle>115){
            currentAngle=115;
        }
            
        if(currentAngle<65){
           currentAngle=65;
        }       
      }
      lenkung.write(currentAngle);
      break;
    case 'r': //r ... right
      if(currentAngle+5 >= 65 && currentAngle+5 <= 115){
        currentAngle = currentAngle + 5;  //Siehe case 'l'         
      }
      else{   // ist der Wert des Servos ausserhalb des Wertebereichs, wird er hier entsprechend korrigiert
        if(currentAngle>115){
            currentAngle=115;
        }       
        if(currentAngle<65){
           currentAngle=65;
        }       
      }
      lenkung.write(currentAngle);  //Schreibt den Wert in den Servo
      break;
  } 
} 
void printStatus(){     //Prints the status of the car
    Serial.println();
    Serial.print("Geschwindigkeit: ");
    Serial.print(currentSpeed);
    Serial.print(" Lenkwinkel: ");
    Serial.print(currentAngle);
    Serial.print(" Abstand vorne: ");
    Serial.print(averageFront); 
    Serial.print(" Abstand hinten: ");
    Serial.print(averageBack); 
    Serial.print(" Abstand rechts: ");
    Serial.print(averageRight); 
    Serial.print(" Abstand links: ");
    Serial.print(averageLeft); 
}

void sendDistance(){      //Leitet die derzeitige Distanz an den Clienten weiter 
    remote =  Udp.remoteIP();    //Holt sich die IP des Clienten
    remoteport = Udp.remotePort();   //Holt sich den Port des Clienten
    Udp.beginPacket(remote,6666);  //Startet ein UDP-Packet
    Udp.print(averageFront);
    Udp.print(",");
    Udp.print(averageBack);
    Udp.print(",");
    Udp.print(averageLeft);
    Udp.print(",");
    Udp.print(averageRight);
    Udp.print(",");
    Udp.print(currentSpeed);
    Udp.endPacket();                 // Beendet das UDP Paket 
}

void updateArray(long array[], long value){
  for(int x=4;x>=1;x--){
    array[x]=array[x-1];
  }
  array[0]=value;
}

void readSensor(){
  int pinTrigger=0;
  int pinEcho=0;
  long distance=0;
  long duration;
  
  for(int x=0;x<4;x++){
    switch(x){
      case 0:
        pinTrigger = pinTriggerFront;
        pinEcho=pinEchoFront;
        break;
      case 1:
        pinTrigger = pinTriggerBack;
        pinEcho=pinEchoBack;
        break;
      case 2:
        pinTrigger = pinTriggerRight;
        pinEcho=pinEchoRight;
        break;
      case 3:
        pinTrigger = pinTriggerLeft;
        pinEcho=pinEchoLeft;
        break;
    }
    digitalWrite(pinTrigger, LOW); 
    delayMicroseconds(2);
    digitalWrite(pinTrigger,HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrigger,LOW);
  
    duration = pulseIn(pinEcho, HIGH);  //Wartet auf einen Pulse vom Ultraschallsensor
    distance = (duration/2) /29;  
    if(distance > 300){
      distance = 300;
      if(distance < 4){
        distance =4;
      }
    }
    if(x==0){
      updateArray(distanceFront,distance);
      averageFront= mean(distanceFront,5);
    }else if(x==1){
      updateArray(distanceBack,distance);
      averageBack= mean(distanceBack,5);
    }else if(x==2){
      updateArray(distanceRight,distance);
      averageRight= mean(distanceRight,5);
    }else if(x==3){
      updateArray(distanceLeft,distance);
      averageLeft= mean(distanceLeft,5);
    } 
  }
}

void avoidCollision(){
  //if((currentSpeed <= 65 && distanceBack <= 100)||(currentSpeed <= 70 && distanceBack <= 50)){
  if(averageBack <= 75){  
    currentSpeed= 75;
    servoSpeed.write(75);
  }
  //if((currentSpeed >= 80 &&  distanceFront <= 50) || (currentSpeed >= 85 && distanceFront <= 100)){
  if(averageFront < 75){  
    currentSpeed = 75;
    servoSpeed.write(75);
  }  
}

void park(){
  servoLenkung.write(115); //einschlagen nach rechts
  while(averageBack > 40){  //solange man nicht ansteht, zurückschieben
    servoSpeed.write(52);  //zurückschieben
    delay(5);   
  }
  servoSpeed.write(60);    //stop
  servoLenkung.write(65);  //links einschlagen
  while(averageBack > 20){
    servoSpeed.write(52);  //weiter zurück
    delay(5);
  }
  servoSpeed.write(60);    //stop
  servoLenkung.write(93);} //leicht rechts einschlagen
  while(averageFront > 20){
    servoSpeed.write(72);
    delay(5);
  }
  servoSpeed.write(60);
  servoLenkung.write(90);
}







