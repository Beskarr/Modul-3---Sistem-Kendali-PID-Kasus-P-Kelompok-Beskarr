//Kode untuk kecepatan

//deklarasi pin sensor
int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;
int sensor6 = A5;
int dataSensor[6];

int pushp = 13;
int pushm = 12;

//deklarasi pin enable
int leftEN = 4;
int rightEN = 2;

//deklarasi pin motor kiri
int leftMotor1 = 5;
int leftMotor2 = 6; //default selalu 0

//deklarasi pin motor kanan
int rightMotor1 = 3;
int rightMotor2 = 11; //default selalu 0

//
int rightMotorSpeed;
int leftMotorSpeed;

//deklarasi variabel untuk menyimpan nilai error
int lastError = 0;
int error = 0;

//PID
int moveControl;
int motorSpeed = 150;
int kp = 5;
int ki = 0;
int kd = 0;
int sensorBit;



void setup()
{
  //inisialisasi pin sensor
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);
  
  //inisialisasi pin enable
  pinMode(leftEN, OUTPUT);
  pinMode(rightEN, OUTPUT);
  
  //inisialisasi pin motor kiri
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  
  //inisialisasi pin motor kanan
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  
  //inisialisasi pin button
  pinMode(pushp, INPUT);
  pinMode(pushm, INPUT);
  Serial.begin(9600);
}
void readSensor(){
  dataSensor[0] = analogRead(sensor1);
  dataSensor[1] = analogRead(sensor2);
  dataSensor[2] = analogRead(sensor3);
  dataSensor[3] = analogRead(sensor4);
  dataSensor[4] = analogRead(sensor5);
  dataSensor[5] = analogRead(sensor6);
  for(int i = 0; i <=5; i++){
    if (dataSensor[i] > 35){
      dataSensor[i] = 1;
    }
    else {
      dataSensor[i] = 0;
    }
  }
  
  sensorBit = 0;
  for(int i = 0; i <=5; i++){
    sensorBit += dataSensor[i] * (1 << i);
  }

}

void loop()
{
  if(digitalRead(pushp)==HIGH){
    kp = kp + 1;
    while(digitalRead(pushp)==HIGH){}
  }
  if(digitalRead(pushm)==HIGH){
    kp = kp - 1;
    while(digitalRead(pushm)==HIGH){}
  }
  
  while (digitalRead(pushp)==LOW && digitalRead(pushm)==LOW){
   
    readSensor();

  //menentukan pembacaan sensor
    switch(sensorBit){
      case 62: error = -5; break;
      case 60: error = -4; break;
      case 61: error = -3; break;
      case 57: error = -2; break;
      case 59: error = -1; break;
      case 51: error = 0; break;
      case 55: error = 1; break;
      case 39: error = 2; break;
      case 47: error = 3; break;
      case 15: error = 4; break;
      case 31: error = 5; break;
    }
       

    moveControl = setMotor(error, lastError,kp,ki,kd);
    //rumus motor kanan motor kiri
    rightMotorSpeed = motorSpeed - moveControl; 
    leftMotorSpeed = motorSpeed + moveControl; 
    
    digitalWrite(rightEN, HIGH);
    analogWrite(rightMotor1, rightMotorSpeed);
    analogWrite(rightMotor2, 0);
      
    digitalWrite(leftEN, HIGH);
    analogWrite(leftMotor1,  leftMotorSpeed);
    analogWrite(leftMotor2, 0);
    Serial.print(rightMotorSpeed);
    Serial.print("\n");
    lastError = error; 
  }
}
    
int setMotor(int error, int lastError, int kp, int ki, int kd){
    int rate_d = error - lastError; 
    int rate_i = error + lastError; 
    int moveControl = (kp * error) + (kd * rate_d) + (ki * rate_i); //(-25) + 0 + 0
    
    return moveControl;
}
