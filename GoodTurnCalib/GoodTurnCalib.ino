

//Import relevant libraries.
//#include <DueTimer.h>
#include <Encoder.h>
#include <Servo.h>
#include "Shield2AMotor.h"
#include <math.h>
#include <Wire.h>
#include <VL53L0X.h>


//---------------------------------------------------------------------setup and variables------------------------------------------------------------------------------------------------------
//Assigns right and left encoder pins.
Shield2AMotor motor(SIGNED_MAGNITUDE);
Encoder myEncoderR(2, 13);
Encoder myEncoderL(3, 12);

VL53L0X sensor;
Servo GripperServo;
Servo RotationServo;
bool ReverseDirection;
bool PivotRight;
bool PivotLeft;
float a;
float newX ;
float newY ;

signed int motor1Speed, motor2Speed, motorSpeed; //What is this for?
double differenceX = 21;
double differenceY =21;
int DropOffX = 0; //Calibrate Drop-off X location with camera
int DropOffY = 0; //Calibrate Drop-off Y location with camera
int HomeLocationX = 0; //Might not need
int HomeLocationY = 0;
boolean Block1Found = false;
boolean Block2Found = false;

//Servo rotation initial angles and block coordinate variables
int RotationAngle = 93;
int GripperAngle = 90;
float BlockX1 =0;
float BlockX2;
float BlockY1;
float BlockY2;
boolean IsHome = false;
boolean Ran = false;
//Intiates and declares all global variables.
int DropOffLocationX = 300; //Calibrate Drop Off location
int DropOffLocationY = 300;
double angleDiff;
volatile long startPositionR;
volatile long startPositionL;
volatile float DistanceTot;
//Motor Speed variables
volatile float MotorVoltageL = 0;
volatile float MotorVoltageR = 0;
volatile float Distancetot = 0;
volatile double MotorSpeedL;
volatile double MotorSpeedR;
volatile float DesiredSpeedR;
volatile float DesiredSpeedL;
volatile float EncoderTicksPerCm = 174.7708;  //Number of encoder ticks required to travel 1 cm
boolean check = false;
boolean Localization = false;
//Starting variables
char Start; // start & start1 used for remote start of robot
String Start1;
int i = 0; //used for creating PWM Speed chart
boolean Print = false;
int Speed = 10;
//PID variables
volatile float  PreviousErrorL, PreviousErrorR, ErrorSumL, ErrorSumR, LeftAdjustmentPID, RightAdjustmentPID;
volatile float ChangeInTime = 50000; // time between interrupts
volatile float KPL = 0;    //0.6      .99   settle as fast as possible
volatile float KDL = 0;    //0.3       
volatile float KIL = 0;    // 0.01       integral takes too long to settle
volatile float KPR = 0;    //.8      .99
volatile float KDR = 0;      //0.1
volatile float KIR = 0;     //0.01

void setup() {
 // Timer1.attachInterrupt(TimerInterrupt);  //setup the timer Interrupt
//  Timer1.start(ChangeInTime); //calls the interrupt every 50 ms
  Serial.begin(9600);
  GripperServo.attach(9); 
  RotationServo.attach(10);
  delay(20);
  RotationServo.write(RotationAngle);
  GripperServo.write(GripperAngle);
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
}


//---------------------------------------------------------------------------------------------------Main Loop-----------------------------------------------------------------------------------------------------------
  void loop() 
  { //Too alter turns go down to turn functions and change multiplier.
   /* delay(3000);
    DesiredSpeedR = Speed;
    DesiredSpeedL = Speed;
    SetSpeed();
    delay(500);
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
    SetSpeed();
    delay(500);*/
    delay(3000);
    a=.1744444;// 10 degree turn
    angledRightTurnSmall();
    delay(500);
     angledRightTurnSmall();
    delay(500);
     angledRightTurnSmall();
    delay(500);
       angledRightTurnSmall();
    delay(500);
       angledRightTurnSmall();
    delay(500);
         angledRightTurnSmall();
    delay(500);
        angledRightTurnSmall();
    delay(500);
      angledRightTurnSmall();
    delay(1000);
    a=.1744444;// 30 degree turn
   angledRightTurnSmall();
    delay(2000);
        a = 0.523333;
    angledLeftTurn();
    delay(1000);
    angledRightTurn();
    delay(1000);
        a = 0.785;
    angledLeftTurn();
    delay(1000);
    angledRightTurn();
    delay(1000);
    a = 1.57;
    angledLeftTurn();
    delay(1000);
    angledRightTurn();
    delay(1000);
    a = 2.71;
    angledLeftTurn();
    delay(1000);
    angledRightTurn();
    delay(1000);
    a = 3.14;
    angledLeftTurn();
    delay(1000);
    angledRightTurn();
    delay(1000);
    /*
    angledLeftTurn();
     delay(1000);
    angledLeftTurn();
     delay(1000);
    angledLeftTurn();*/
   /* a = 1.57;   //90 degree turn
    delay(1000);
    angledRightTurn();  //test right
    delay(1000);
    angledLeftTurn();   //test left
    delay(2500);
    a = 3.14;  // 180 degree turn
    angledRightTurn();  
    delay(1000);
    angledLeftTurn();
    delay(2500);
 //   a = 4.71;  //270 degree turn
  //  angledRightTurn();
   // delay(500);
  //  angledLeftTurn();
  //  delay(1000);
*/

/*    DesiredSpeedL = 10;
    DesiredSpeedR = 10;
    delay(3000);
    */
  }
void SetSpeed() // -------------------Timer Interrupt-------------------------------------------------------------------------------------------------------------------------------------------------
{
        

    //Gyro? or should this be checked only during a turn
    //Receive coordinates from python
    
      //----------------Calculate Wheel Speed---------------------
          //Reads an intial value from both the right and left encoder.
      /*  if(Localization == true){
             newX = Serial.parseFloat();
             newY = Serial.parseFloat();
          }
          */
     
       //Checks speed for both the right and left motor, tracks total distance
       
        //Reads the new values from both the right and left encoder.
        volatile long newPositionR = myEncoderR.read();
        volatile long newPositionL = myEncoderL.read();
    
          //Calculates the speed of both wheels using the change in encoder position (in cm). 
         volatile float SpeedR = abs((newPositionR - startPositionR)/ EncoderTicksPerCm) / (0.05);
         volatile float SpeedL = abs((newPositionL - startPositionL)/ EncoderTicksPerCm) / (0.05);
  
          //Calculates the average distance traveked and adds it to the total count.
          volatile float DistanceR = abs(newPositionR - startPositionR)/ EncoderTicksPerCm;
          volatile float DistanceL = abs(newPositionL - startPositionL)/ EncoderTicksPerCm;
          volatile float DistanceAvg = (DistanceR+DistanceL)/2;
          DistanceTot += DistanceAvg;    

           //newX = Serial.parseFloat();
           //newY = Serial.parseFloat();
          //Print Left & Right Speeds for corresponding PWM values ----- used to make speed algorithm
       /*   Serial.print("L:");
          Serial.print(i);  
          Serial.print(",");   
          Serial.print(SpeedL);

          Serial.print("R:");
          Serial.print(i);  
          Serial.print(",");   
          Serial.println(SpeedR);
          */

          startPositionR = newPositionR;
          startPositionL = newPositionL;

          
  //-----------------------------PID Speed Control-------------------------------------------------------------------------------------------------------------------------------------------
  volatile float ErrorR,ErrorL, ChangeInErrorR,ChangeInErrorL;
  


    //PID for Left Motor
          //Calculate Errors
          ErrorL = DesiredSpeedL - SpeedL;
          ChangeInErrorL = ErrorL - PreviousErrorL;
          PreviousErrorL = ErrorL;
          ErrorSumL += ErrorL;
        
          //Calculate amount of correction needed
          LeftAdjustmentPID = KPL * ErrorL +  KIL* ErrorSumL * 0.05 + ChangeInErrorL * KDL/0.05;
      
          //Calculate the amount of PWM needed to reach the desired speed, add PID for correction
          MotorVoltageL = FindSpeedL(DesiredSpeedL) + LeftAdjustmentPID;
      
    //PID for Right Motor
          ErrorR = DesiredSpeedR - SpeedR;
          ChangeInErrorR = ErrorR - PreviousErrorR;
          PreviousErrorR = ErrorR;
          ErrorSumR += ErrorR;
          
          //Calculate amount of correction needed
          RightAdjustmentPID = KPR * ErrorR +  KIR* ErrorSumR * 0.05 + ChangeInErrorR * KDR/0.05;
      
          //Calculate the amount of PWM needed to reach the desired speed, add PID for correction
           MotorVoltageR = FindSpeedR(DesiredSpeedR) + RightAdjustmentPID;
             if(ReverseDirection == true){
               MotorVoltageR = -MotorVoltageR;
               MotorVoltageL = -MotorVoltageL;
              }
              if(PivotRight == true){
               MotorVoltageR = MotorVoltageR;
               MotorVoltageL = -MotorVoltageL;
              }

          if(PivotLeft == true){
               MotorVoltageR = -MotorVoltageR;
               MotorVoltageL = MotorVoltageL;
              }
   //Update Motor voltages
   //Serial.println(LeftAdjustmentPID);
   //Serial.println(MotorVoltageL);
   motor.control(-MotorVoltageR , MotorVoltageL);      // Right Motor is physically reveresed
        

      
}


//----------------------------------------------------------------------------------------------------Functions---------------------------------------------------------------------------------------------------
 // Functions to use the correct PWM for a desired speed
 
 //Checks the PWM : Speed table, assigns the correct PWM for a desired speed
 float FindSpeedR(float DS) 
{
      if (DS > 0)                 // positive speed
      {
        return -(DS*DS*DS*DS*0.00001) + DS*DS*DS*0.0021 - DS*DS* 0.0947 + DS* 2.1185 + 7.8296;//Equation from Excel based on Speed : PWM chart
      }
      else if (DS ==0)  {         // No speed
      return 0;}
    
      else if (DS < 0)  {         // negative speeds
      return   - (-(DS*DS*DS*DS*0.00001) + DS*DS*DS*0.0021 - DS*DS* 0.0947 + DS* 2.1185 + 7.8296); //Equation from Excel based on Speed : PWM chart)
}     }

//Same Speed finder, but for left motor (has different equation)
float FindSpeedL(float DS)    
{
      if (DS > 0)                 // positive speed
      {
        return  -(DS*DS*DS*DS*0.00006) + (DS*DS*DS*0.0065) - (DS*DS*0.2205) + (DS* 3.3727) + 10.309; //Equation from Excel based on Speed : PWM chart
      }
      else if (DS ==0) {          // No speed
      return 0;}
    
      else if (DS < 0)   {        // negative speeds
      return  - (-(DS*DS*DS*DS*0.00006) + (DS*DS*DS*0.0065) - (DS*DS*0.2205) + (DS* 3.3727) + 10.309);
      }
}

 void angledRightTurn()
  {
    int j = 375*(a);    //Calculates the appropriate delay which will result in the correct turning angle being achieved based on testing and calibration.
    PivotRight = true;
    //Starts motors at the calibrated values for the length of time calculated. Motor speed briefly lowered before motors are turned off.
    DesiredSpeedR = 10;
    DesiredSpeedL = 10;
    SetSpeed();
    delay(j);
    PivotRight = false;
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
     SetSpeed();
    delay(1500);
    
  }

  //Turns robot to the left based on the calculated angleDiff.
  void angledLeftTurn()
  {
    int k = 375*(a);    //Calculates the appropriate delay which will result in the correct turning angle being achieved based on testing and calibration.
    PivotLeft = true;
    //Starts motors at the calibrated values for the length of time calculated. Motor speed briefly lowered before motors are turned off.
    DesiredSpeedR = 10;
    DesiredSpeedL = 10;
     SetSpeed();
    delay(k);
    PivotLeft = false;
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
     SetSpeed();
    delay(3000);
   
  }
  
   void angledRightTurnSmall()
     {
    int k = 730*(a);    //Calculates the appropriate delay which will result in the correct turning angle being achieved based on testing and calibration.
    PivotRight = true;
    //Starts motors at the calibrated values for the length of time calculated. Motor speed briefly lowered before motors are turned off.
    DesiredSpeedR = 10;
    DesiredSpeedL = 10;
     SetSpeed();
    delay(k);
    PivotRight = false;
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
     SetSpeed();
    delay(3000);
   
  }

   void angledLeftTurnSmall()
     {
    int k = 730*(a);    //Calculates the appropriate delay which will result in the correct turning angle being achieved based on testing and calibration.
    PivotLeft = true;
    //Starts motors at the calibrated values for the length of time calculated. Motor speed briefly lowered before motors are turned off.
    DesiredSpeedR = 10;
    DesiredSpeedL = 10;
     SetSpeed();
    delay(k);
    PivotLeft = false;
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
     SetSpeed();
    delay(3000);
   
  }
  
