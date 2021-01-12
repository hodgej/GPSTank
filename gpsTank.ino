#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// Forward Declarations //
float clampAngle(float);
void updateHeading();
void updatePosition();
void side1(int);
void side2(int);
void fixTrajectory(float);

// Timers //
unsigned long timer = 0;
float timeStep = 0.01;

// Control Knowledge //
float roll = 0;
double pos[1][2];
const double waypoint[15][2] = 
{{36.437549,-94.201316},
{36.437549,-94.201316},
{36.437549,-94.201316},
{36.437583,-94.201263},
{36.437583,-94.201263},
{36.437583,-94.201263},
{36.437583,-94.201164},
{36.437583,-94.201164},
{36.437583,-94.201164},
{36.437450,-94.201164},
{36.437450,-94.201164},
{36.437450,-94.201164},
{36.437564,-94.201316},
{36.437564,-94.201316},
{36.437564,-94.201316}};


// Hardware Init. //
int side1motor1pin1 = 3;
int side1motor1pin2 = 2; //pwm
int side1motor2pin1 = 5;
int side1motor2pin2 = 4; //pwm
int side2motor1pin1 = 6;
int side2motor1pin2 = 7; //pwm
int side2motor2pin1 = 8;
int side2motor2pin2 = 9; //pwm

TinyGPSPlus gps;
MPU6050 mpu;
SoftwareSerial ss(10, 11);

// META Information //

//TODO: ADD WAYPOINT VARIABLE
int currentWaypoint{0};        // Index of currently used waypoint
float angle{0};                   // The angle which it strives to reach

// Independent Variables //
const float ROTATION_SENSITIVITY{5.0}; //the lower it is, the more accurate it is, albeit harder to achieve.
const long WAYPOINT_SENSITIVITY{5.0};


void setup(){
  Serial.begin(115200);
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
      Serial.println("MPU6050 NOT FOUND! Check wiring!");
      delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3); 

  // Initialize Motors
  pinMode(side1motor1pin1, OUTPUT);
  pinMode(side1motor1pin2, OUTPUT);
  pinMode(side1motor1pin1, OUTPUT);
  pinMode(side1motor2pin2, OUTPUT);
  pinMode(side2motor1pin1, OUTPUT);
  pinMode(side2motor1pin2, OUTPUT);
  pinMode(side2motor2pin2, OUTPUT);
  pinMode(side2motor2pin2, OUTPUT);
  // Delay for startup (to prevent sudden chaos!)
  delay(10000);
}


void loop(){
  updateHeading(); // DEBUG: Make sure these are actually updating
  updatePosition();
  
  delay((timeStep * 1000) - (millis() - timer)); //global timestep with latency elimination
}

void navigate(){
   createAngle(pos[0][0], pos[0][1], waypoint[currentWaypoint][0], waypoint[currentWaypoint][1]); //new angle based on new position TODO: check for major discrepencies to ensure nothings wrong
   
   if ( (angle == roll || (clampAngle(roll-angle)) < ROTATION_SENSITIVITY) == false){
      fixTrajectory(angle);
   } else if( findHypotenuse(pos[0][0], pos[0][1], waypoint[currentWaypoint][0], waypoint[currentWaypoint][1] ) > WAYPOINT_SENSITIVITY){
      side1(1);
      side2(1);
   } else{
      currentWaypoint++;
   }
}

  
void updatePosition(){
  while (ss.available() > 0)
    if (gps.encode(ss.read())){
      pos[0][0] = gps.location.lat(); // DEBUG: TRY SWITCHING THEM AROUND
      pos[0][1] = gps.location.lng();
    }
      
}

void updateHeading(){
    timer = millis();
    
    Vector norm = mpu.readNormalizeGyro();
    roll = roll + norm.XAxis * timeStep;
    roll = clampAngle(roll);
}


float createAngle(double x1, double y1, double x2, double y2){ //x1/y1 is currnt pos
    double t1 = x2-x1;
    double t2 = y2-y1;
    double t3 = sqrt(t1*t1+t2*t2);
    double sinOfT1T3 = t1/t3;
    float toAngle = ( pow(sinOfT1T3, -1) * 180 ) / 3.14159 ;
    return toAngle;
}

float findHypotenuse(double x1, double y1, double x2, double y2){ //x1/y1 is currnt pos
    double t1 = x2-x1;
    double t2 = y2-y1;
    double t3 = sqrt(t1*t1+t2*t2);
    return t3;
}

void fixTrajectory(float angle){
  //later on, add ability to check to see error value for a given sensitivity; use angle to determine estemated finishing coordonates, and calculate the distance between the est. coordonates and desired ones.
    if(clampAngle(angle-roll)  > 180){ //right
      side2(0);
      side1(1);
    }
    else{                              //left
      side1(0);
      side2(1);
    }
  
}

  
  float clampAngle(float val){
      if (val < 0)   {val += 360;}
      if (val > 360) { val = 0  ;}
      return val;
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
