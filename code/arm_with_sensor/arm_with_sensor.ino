#include <Servo.h>
#include <SharpIR.h>

Servo base;  
Servo arm;

#define ir A0
#define model 1080

SharpIR SharpIR(ir, model);

int pos = 0; 

void setup() {
  
  arm.write(10);
 
  Serial.begin(9600);
}

void loop() {
  delay(300);   

  unsigned long pepe1=millis();  // takes the time before the loop on the library begins

  int dis=SharpIR.distance();  // this returns the distance to the object you're measuring


  Serial.print("distance: ");  // returns it to the serial monitor
  Serial.println(dis);
  
  unsigned long pepe2=millis()-pepe1;  // the following gives you the time taken to get the measurement
  Serial.println(pepe2);

   if(dis < 40 ){
      Serial.println("PICKING OBJECT");
      base.attach(5); 
    arm.attach(3); 

  base.write(80);
  
 
  
  for (pos = 80; pos <= 180; pos ++) { 
   base.write(pos);              
    delay(20);            
 }
    arm.write(90);
    delay(500);
     arm.write(10);
     
 for (pos = 180; pos >= 80; pos --) { 
    base.write(pos);              
    delay(20);  
    
 }
  
   base.detach();
      
      
}
}
