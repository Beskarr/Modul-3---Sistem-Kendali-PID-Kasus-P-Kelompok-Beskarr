//Code Untuk Posisi

//motor directory
#define CW  0
#define CCW 1

//define button
#define pushsub 6
#define pushplus 5
 
//motor control pin
#define motorDirPin 7
#define motorPWMPin 9
#define enablePin 8
 
//encoder pin
#define encoderPinA 2
#define encoderPinB 4
 
//encoder var
int encoderPos = 0;

float Kp          = 10;
int   targetPos;
int   error;
int   control;
int   velocity;
 
//external interrupt encoder
void doEncoderA()
{
  digitalRead(encoderPinB)?encoderPos--:encoderPos++;
}
 
void setup()
{
  //setup interrupt
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA,RISING);

    //setup motor driver
    pinMode(motorDirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);
  
    pinMode(pushplus, INPUT);
    pinMode(pushsub, INPUT);
  

    Serial.begin(9600);
}

 
void loop()
{ 
  //Kondisi untuk menaikan atau menurunkan KP menggunakan push button
    if(digitalRead(pushplus)==LOW){
      Kp = Kp + 1;
      while(digitalRead(pushplus)==LOW){}
    }
    if(digitalRead(pushsub)==LOW){
      Kp = Kp - 1;
      while(digitalRead(pushsub)==LOW){}
    }

    targetPos = analogRead(A5)/10; //potentiometer sebagai penentu targetpos
    error   = targetPos - encoderPos; 
    control = Kp * error;

    velocity = min(max(control, -255), 255); 

    if(velocity >= 0)
    {
        digitalWrite(motorDirPin, CW); //output 
        analogWrite(motorPWMPin, velocity); //output duty 
    }
    else
    {
        digitalWrite(motorDirPin, CCW);
        analogWrite(motorPWMPin, 255+velocity);
    }
    Serial.println(encoderPos);
}
