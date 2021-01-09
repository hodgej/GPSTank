#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
//float pitch = 0;
float roll = 0;
//float yaw = 0;

int side1motor1pin1 = 3;
int side1motor1pin2 = 2; //pwm
int side1motor2pin1 = 5;
int side1motor2pin2 = 4; //pwm
int side2motor1pin1 = 6;
int side2motor1pin2 = 7; //pwm
int side2motor2pin1 = 8;
int side2motor2pin2 = 9; //pwm

int toggle{1};

float angle{90.0};


const float ROTATION_SENSITIVITY{5.0}; //the lower the more accurate and harder to achieve.

void setup()
{
  
  Serial.begin(115200);
  
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
      delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3); 

  pinMode(side1motor1pin1, OUTPUT);
  pinMode(side1motor1pin2, OUTPUT);
  pinMode(side1motor1pin1, OUTPUT);
  pinMode(side1motor2pin2, OUTPUT);
  pinMode(side2motor1pin1, OUTPUT);
  pinMode(side2motor1pin2, OUTPUT);
  pinMode(side2motor2pin2, OUTPUT);
  pinMode(side2motor2pin2, OUTPUT);

  delay(10000);
}


void loop()
{
  getCurrentHeading();
  //if (toggle == 1) {
      if ( (angle == roll || (clampAngle(roll-angle)) < ROTATION_SENSITIVITY) == false){
    faceAngle(90);
//  
    }
    else if(toggle==0){
      toggle=1;
      Serial.println("CORRECT");
      side1(0);
      side2(0);
      delay(1000);
      
    }
    else{
      side1(0);
      Serial.print("FINISHED WITH ERROR: ");
      Serial.println( abs(angle-roll));
      side2(0);
    }
    

  
  delay((timeStep * 1000) - (millis() - timer)); //global timestep with latency elimination

}

float getCurrentHeading(){
  
    timer = millis();

      Vector norm = mpu.readNormalizeGyro();


    //pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    if (roll < 0){roll += 360;}
    if (roll > 360){ roll=0; }
    //yaw = yaw + norm.ZAxis * timeStep;
    return roll;
}


int faceAngle(float angle){

  //later on, add ability to check to see error value for a given sensitivity; use angle to determine estemated finishing coordonates, and calculate the distance between the est. coordonates and desired ones.
  if (angle == roll || (clampAngle(roll-angle)) < ROTATION_SENSITIVITY){
    side1(0);
    Serial.println("ALLDONE");
    side2(0);
    toggle += 1;
    //Serial.println(toggle);
    return 1; //fail: unrequired
  }
  else{
    Serial.println(clampAngle(angle-roll));
   //left or right?
   if(clampAngle(angle-roll)  > 180){ //right
    Serial.println("RIGHT");
    side2(0);
    side1(1);
   }
   else{
    Serial.println("LEFT");
    side1(0);
    side2(1);
   }

   
  //while( !(angle == roll || abs(clampAngle(roll-angle < ROTATION_SENSITIVITY)))) {getCurrentHeading();} //do nothing until equals roll
  return 0;
}
  }



float clampAngle(float roll){
    if (roll < 0){roll += 360;}
    if (roll > 360){ roll=0; }

    return roll;
}

void side1(int operation){
  switch(operation){
    case 0:
      digitalWrite(side1motor1pin2, LOW);
      digitalWrite(side1motor2pin1, LOW);
      break;
    case 1:
        digitalWrite(side1motor1pin2, HIGH);
      digitalWrite(side1motor2pin1, HIGH);
      break;
  }
}

void side2(int operation){
  switch(operation){
    case 0:
            digitalWrite(side2motor1pin2, LOW);
      digitalWrite(side2motor2pin1, LOW);
      break;
    case 1:
            digitalWrite(side2motor1pin2, HIGH);
      digitalWrite(side2motor2pin1, HIGH);
      break;
  }
}
