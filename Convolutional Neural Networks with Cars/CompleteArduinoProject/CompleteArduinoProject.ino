#include <IRremote.h> // Infrared Remote Library
#include <Servo.h>  //Servo Motot library
Servo myservo;      // create servo object to control servo

int Echo = A4; // Pin receives echo pulse
int Trig = A5; // Pin sends echo pulse



//IR REMOTE CODES
#define LEFT    16720605  // Specific Code For Left Button
#define RIGHT   16761405 // Specific Code for Right Button

#define RECV_PIN  12

//Line Tracking IO define
#define LineTracker_Right !digitalRead(10)
#define LineTracker_Middle !digitalRead(4)
#define LineTracker_Left !digitalRead(2)



#define ENB 5   // Left  wheel speed
#define IN1 7   // Left  wheel forward
#define IN2 8   // Left  wheel reverse
#define IN3 9   // Right wheel reverse
#define IN4 11  // Right wheel forward
#define ENA 6   // Right wheel speed

// initial speed of car >=0 to <=255
#define carSpeed 200 
 
int rightDistance = 0, leftDistance = 0, middleDistance = 0;

IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long val;
unsigned long preMillis;

int button = 0; // MODE SWITCHER

//CODE FOR CARS MOVEMENT
void forward()
{
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go forward!");
}

void back()
{
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go back!");
}

void left()
{
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("go left!");
}

void right()
{
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go right!");
}

void stop()
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("STOP!");
}

//Ultrasonic distance measurement Sub function
int getDistance() {
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    return (int)pulseIn(Echo, HIGH) / 60;
   // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
}


void setup() 
{ 
  //Serial Port begin
  Serial.begin(9600);
  //Initialize Servo Motor
  myservo.attach(3);
  //Define inputs and outputs
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();
  irrecv.enableIRIn();
    
}
void loop() 
{
  // CODE TO SWITCH BETWEEN MODES
if (irrecv.decode(&results))
  { 
    preMillis = millis();
    val = results.value;
    Serial.println(val);
    irrecv.resume();   
  }

   switch(val)
    {
      case LEFT: button = 1; break;
      case RIGHT: button = 2; break;
    }

// MODE 1 LINE DETECTION ALGORITHM
  if (button == 1)
{
  if(LineTracker_Middle)
  {
    forward();
  }
  else if(LineTracker_Right) 
  {
    right();
    while(LineTracker_Right);
  }
  else if(LineTracker_Left) 
  {
    left();
    while(LineTracker_Left);
  }

 }

// MODE 2 OBSTACLE DETECTION ALGORITHM
 if (button == 2)
 {

myservo.write(90);  //setservo position according to scaled value
    delay(500);
    middleDistance = getDistance();

    if(middleDistance <= 30) {
      stop();
      delay(500);
      myservo.write(10);
      delay(1000);
      rightDistance = getDistance();

      delay(500);
      myservo.write(90);
      delay(1000);
      myservo.write(180);
      delay(1000);
      leftDistance = getDistance();

      delay(500);
      myservo.write(90);
      delay(1000);
      if(rightDistance > leftDistance) {
        right();
        delay(360);
      }
      else if(rightDistance < leftDistance) {
        left();
        delay(360);
      }
      else if((rightDistance <= 30) || (leftDistance <= 30)) {
        back();
        delay(180);
      }
      else {
        forward();
      }
    }
    else {
        forward();
    }

  
 }

}


 
 
