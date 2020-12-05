//Import relevant libraries.
//#include <DueTimer.h>
#include <Encoder.h>
#include <Servo.h>
#include "Shield2AMotor.h"
#include <math.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//---------------------------------------------------------------------setup and variables------------------------------------------------------------------------------------------------------
//Assigns right and left encoder pins.
Shield2AMotor motor(SIGNED_MAGNITUDE);
Encoder myEncoderR(2, 13);
Encoder myEncoderL(3, 12);

VL53L0X sensor;
Servo GripperServo;
Servo RotationServo;
bool PivotRight;
bool PivotLeft;
int input_counter = 0;
boolean stopFlag = 0;
boolean grabFlag = 0;
boolean grabReset = 0;
double grabTimer;
double straightTime =0;
boolean straightFlag;
boolean firstFlag;
boolean ReverseDirection = false;
float Speed = 10;
float currentAngle;
double desiredAngle;
float newX ;
float newY ;
float startX ;
float startY ;

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
int RotationAngle = 95;
int GripperAngle = 90;
float BlockX1 =0;
float BlockX2;
float BlockY1;
float BlockY2;
float blueBlockX3;
float blueBlockY3;
float blueBlockX4;
float blueBlockY4;
boolean IsHome = false;
boolean Ran = false;
//Intiates and declares all global variables.
int DropOffLocationX = 530; //550//Calibrate Drop Off location
int DropOffLocationY = 390; //420 when lined up
int blueDropOffLocationX = 0; //550//Calibrate Drop Off location
int blueDropOffLocationY = 390; //420 when lined up
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
boolean PrintBlue = false;

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
 // Timer1.start(ChangeInTime); //calls the interrupt every 50 ms
  Serial.begin(9600);
  GripperServo.attach(10); 
  RotationServo.attach(9);
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
  { 
      //Wait for Block coordinates before starting code
      while(BlockX1 == 0){
        BlockX1 = Serial.parseFloat();
      }
      
      //assign the block coordinates ONCE
      if (Print == false){
        Print = true;
        BlockY1 = Serial.parseFloat();
        BlockX2 = Serial.parseFloat();
        BlockY2 = Serial.parseFloat();

        //-----------Print block coordinates for testing---------------------
        /*
         Serial.print("Green Blocks: ");
        Serial.println(BlockX1); //For testing purposes
        Serial.print(" Y1=");
        Serial.print(BlockY1);
        Serial.print(" X2=");
        Serial.print(BlockX2);
        Serial.print(" Y2=");
        Serial.print(BlockY2);    
        */
      }
      
      while(blueBlockX3 == 0){
        blueBlockX3 = Serial.parseFloat();
      }
      
      if (PrintBlue == false){
        PrintBlue = true;
        blueBlockY3 = Serial.parseFloat();
        blueBlockX4 = Serial.parseFloat();
        blueBlockY4 = Serial.parseFloat();
        
        /*
        Serial.println("Blue Blocks: ");
        Serial.print(blueBlockX3); //For testing purposes
        Serial.print(" Y1=");
        Serial.print(blueBlockY3);
        Serial.print(" X2=");
        Serial.print(blueBlockX4);
        Serial.print(" Y2=");
        Serial.println(blueBlockY4);
        */
      }
      
       
       //Wait for "s" from python script (press space bar)
       // Start = Serial.read();
        //Serial.print(Start);
        Start1 += Start;
       if (stopFlag == 0)
       {
        Start1 = "s";
        stopFlag = 1;
       }       
      //delay(1000);
      if (Start1 == "s")    //Enters if statement once an "s" is received.
      { 
        int startTime = millis();   //Assigns start time to a variable.  
        //Intializes variables for the starting x and y coordinates, reads the data being received, and converts them from a single string to two integer values.
        //float startX = Serial.parseFloat();
        //float startY = Serial.parseFloat();
        startX = 0;
        startY = 0;
        DesiredSpeedR = Speed;
        DesiredSpeedL = Speed;
        SetSpeed();
        delay(500);  
        DesiredSpeedR = 0;
        DesiredSpeedL = 0;
        SetSpeed();

        //-------------------------------------------------------------------------------------------------Start Block Search----------------------------------------------------------------------------------------------------------------------
          while (i<9)
        {
          //might move these to multitask, unsure yet
          
         while(Serial.available() > 3){
          newX = Serial.parseFloat();
          newY = Serial.parseFloat();
          }
          newX = Serial.parseFloat();
          newY = Serial.parseFloat();

          /*
           Serial.print("X: ");
           Serial.print(newX);
           Serial.print(" / Y: ");
           Serial.println(newY);
           */
               
          DesiredSpeedR = 0;
          DesiredSpeedL = 0;
          SetSpeed();
           /*
          newX = Serial.parseFloat();
          newY = Serial.parseFloat();    
          //Takes x amount of time to parse (this varies straight time)
          newX = Serial.parseFloat();
          newY = Serial.parseFloat();
          DesiredSpeedR = 0;
          DesiredSpeedL = 0;
           SetSpeed();
          newX = Serial.parseFloat();
          newY = Serial.parseFloat();
          newX = Serial.parseFloat();
          newY = Serial.parseFloat();
          */
          

          int check = sensor.readRangeContinuousMillimeters();
          Serial.println(check);
          if ((check < 50) && grabFlag == 0 && i<8)  //is there block close by?
          {
           // if(grabReset == 0)    //perform pick up and drop off
            //{
              //perform pick up and drop off
              DesiredSpeedR = Speed;
              DesiredSpeedL = Speed;
              SetSpeed();
              delay(500);  //Hudson Edit remove if faulty
              DesiredSpeedR=0;
              DesiredSpeedL=0;
              SetSpeed();
              GripperClose();
              Serial.println("Got-Block");
              grabFlag = 1;
              i++;
              //grabReset = 1;
              //Start1 == "";
           // }
         /*   else
            {  
              grabFlag = 1;
              grabReset = 0;
              i++; //increment i
              //Start1 = "s";
            }
            */
          }
          else
          {
            double differenceX = startX - newX;
            double differenceY = startY - newY;
            double currentAngle = atan2(differenceY, differenceX);
            if (i == 0)
            {
               desiredAngle = atan2(newY-BlockY1, newX-BlockX1);
            }
            if (i == 1)
            {
               desiredAngle = atan2(newY-DropOffLocationY, newX-DropOffLocationX);
             /*  Serial.print("X: ");
               Serial.print(newX);
               Serial.print(" / Y: ");
               Serial.println(newY);
               */
                if ((newX  > (DropOffLocationX-75)) && (newY > (DropOffLocationY-75)))//(((newX-DropOffLocationX)>30 || (newX-DropOffLocationX)>-30) && ((newY-DropOffLocationY)>30 || (newY - DropOffLocationY)<-30))
                {
                  Serial.println("open");
                  GripperOpen();
                  grabFlag = 0;
                  i++;
                  Serial.println("Block has been dropped off");
                }
            }
            if (i == 2)
            {/*
               Serial.print("X: ");
              Serial.print(newX);
              Serial.print(" / Y: ");
               Serial.println(newY);
               */
               currentAngle=atan2((startY - newY),(startX - newX));
               desiredAngle = atan2(newY-BlockY2, newX-BlockX2);
               Serial.println("go to block 2");
            }
            if (i == 3)
            {
               desiredAngle = atan2(newY-DropOffLocationY, newX-DropOffLocationX);
               Serial.println("Going to drop-off");
                if ((newX  > (DropOffLocationX-75)) && (newY > (DropOffLocationY-75)))//(((newX-DropOffLocationX)>30 || (newX-DropOffLocationX)>-30) && ((newY-DropOffLocationY)>30 || (newY - DropOffLocationY)<-30))
                {
                  Serial.println("open");
                  GripperOpen();
                  grabFlag = 0;
                  i++;
                }
            }
           /* if (i == 4)
            {
               desiredAngle = atan2(newY-startY-50, newX);
               Serial.println("go home x");
               if (newX < 25)
               {
                //angleDiff = -1.57;
                //angledRightTurn();
                i++;
               }
            }*/
            if (i == 4) //Pick Up Blue Block One
            {
               desiredAngle = atan2(newY-blueBlockY3, newX-blueBlockX3);
               Serial.println("go to blue block 1");
            }
            if (i == 5) //DropOff Block One (Blue)
            {
               desiredAngle = atan2(newY-blueDropOffLocationY, newX-blueDropOffLocationX);
               Serial.println("Going to drop-off");
                if ((newX  > (blueDropOffLocationX-75)) && (newY > (blueDropOffLocationY-75)))//(((newX-blueDropOffLocationX)>30 || (newX-blueDropOffLocationX)>-30) && ((newY-blueDropOffLocationY)>30 || (newY - blueDropOffLocationY)<-30))
                {
                  Serial.println("open");
                  GripperOpen();
                  grabFlag = 0;
                  i++;
                }
            }
             if (i == 6) //Pick Up Blue Block Two
            {
               desiredAngle = atan2(newY-blueBlockY4, newX-blueBlockX4);
               Serial.println("go to blue block 2");
            }
            if (i == 7) //DropOff Block One (Blue)
            {
               desiredAngle = atan2(newY-blueDropOffLocationY, newX-blueDropOffLocationX);
               Serial.println("Going to drop-off");
                if ((newX  > (blueDropOffLocationX-75)) && (newY > (blueDropOffLocationY-75)))//(((newX-blueDropOffLocationX)>30 || (newX-blueDropOffLocationX)>-30) && ((newY-blueDropOffLocationY)>30 || (newY - blueDropOffLocationY)<-30))
                {
                  Serial.println("open");
                  GripperOpen();
                  grabFlag = 0;
                  i++;
                }
            }
           /* if (i == 4)
            {
               desiredAngle = atan2(newY-startY-50, newX);
               Serial.println("go home x");
               if (newX < 25)
               {
                //angleDiff = -1.57;
                //angledRightTurn();
                i++;
               }
            }*/
            if (i == 8)
            {
               desiredAngle = atan2(newY, newX);
               Serial.println("go home y");
               if (newX < 20 && newX > -20 && newY < 20 && newY > -20)
               {
                i++;
                Start1 = "";
               }
            }
            if (Start1 == "s")
            {
              angleDiff = desiredAngle - currentAngle; //Calculates the difference between the desired angle of the robot and the current angle.
             /* Serial.print("x:");
              Serial.println(newX);
              Serial.print("y:");
              Serial.println(newY);
              Serial.print("angle:");
              Serial.println(angleDiff);
              */
              
  
              //angle decision tree
              if(angleDiff < -0.035)    //Checks if the angle different is less than the given radian value. Value can be adjusted to create a smaller or larger threshold. Default motion in this case is to turn right.
              {
                angleDiff = -angleDiff;   //Flips the sign of the angle so that the input to the respective funtions will be postitive.
              
                if(angleDiff >= PI)   //Checks if the angle is greater that Pi radians. To make motion more efficient the robot should turn tleft if it is true.
                {
                  angleDiff = 2*PI - angleDiff;   //Finds the equivalent angle to turn in the left direction.
                  //Serial.println("Turning Left");
                  angledLeftTurn();   //Turns left by the appropriate amount to align the robot to the desired angle.
                }
          
                else    //Otherwise turn right by the difference between the desired and current angles.
                {
                  // Serial.println("Turning Right");
                  angledRightTurn();    //Turns right by the appropriate amount to align the robot to the desired angle.
                }
              }
          
              else if(angleDiff > 0.035)    //Checks if the angle difference is greater than the given radian value. Value can be adjusted to create a smaller or larger threshold. Default motion in this case is to turn left.
              {
                if(angleDiff >= PI)   //Checks if the angle is greater that Pi radians. To make motion more efficient the robot should turn right if it is true.
                {
                  angleDiff = 2*PI - angleDiff;//Finds the equivalent angle to turn in the right direction.
                  //Serial.println("Turning Right");
                  angledRightTurn();    //Turns right by the appropriate amount to align the robot to the desired angle.
                }
          
                else    //Otherwise turn right by the difference between the desired and current angles.
                {
                //  Serial.println("Turning Left");
                  angledLeftTurn();   //Turns left by the appropriate amount to align the robot to the desired angle.
                }
              }
          
              else    //If the angle difference falls within the threshold the robot will continue forward.
              {
                 // Serial.println("Straight"); //Moves straight forward.
              }
              //reset values
              angleDiff = 0;    //Resets angleDiff to 0.
              startX = newX;    //Sets starting x for the next loop to the old new x.
              startY = newY;    //Sets starting y for the next loop to the old new y.
              DesiredSpeedR = Speed;
              DesiredSpeedL = Speed;
              SetSpeed();
              delay(500);
              DesiredSpeedR=0;
              DesiredSpeedL=0;
              SetSpeed();
            }            
          }
        }
   }
      
    DesiredSpeedR=0;
    DesiredSpeedL=0;
    SetSpeed();
  }
void SetSpeed() // -------------------Timer Interrupt-------------------------------------------------------------------------------------------------------------------------------------------------
{
      //----------------Calculate Wheel Speed---------------------
          //Reads an intial value from both the right and left encoder.
       //Checks speed for both the right and left motor, tracks total distance
      // checkDistance = sensor.readRangeContinuousMillimeters();
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
       motor.control(-MotorVoltageR , MotorVoltageL );      // Right Motor is physically reveresed normally +9 right and -5 left       when dead ish  +9   , -7          
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
      return  -(-(DS*DS*DS*DS*0.00006) + (DS*DS*DS*0.0065) - (DS*DS*0.2205) + (DS* 3.3727) + 10.309);
      }
}
void angledRightTurn()
  {
    int j = 460*(angleDiff);    //Calculates the appropriate delay which will result in the correct turning angle being achieved based on testing and calibration. 500 half life
    PivotRight = true;
    //Starts motors at the calibrated values for the length of time calculated. Motor speed briefly lowered before motors are turned off.
    DesiredSpeedR = 10;  //15
    DesiredSpeedL = 10;     //0
     SetSpeed();
    delay(j);
    PivotRight = false;
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
     SetSpeed();
    delay(500);
    
  }

  //Turns robot to the left based on the calculated angleDiff.
   void angledLeftTurn()
  {
    int k = 510*(angleDiff);    //Calculates the appropriate delay which will result in the correct turning angle being achieved based on testing and calibration. 300 half life
    PivotLeft = true;
    //Starts motors at the calibrated values for the length of time calculated. Motor speed briefly lowered before motors are turned off.
    DesiredSpeedR = 10;   //0
    DesiredSpeedL = 10;   //10
     SetSpeed();
    delay(k);
    PivotLeft = false;
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
     SetSpeed();
    delay(500);
  }
  
  void GripperClose(){
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
    SetSpeed();
    GripperServo.write(50); //55
    delay(100);
    for(RotationAngle = 93; RotationAngle > 70; RotationAngle--)  
    {                                  
    RotationServo.write(RotationAngle);               
    delay(20);                   
    }                               
    RotationServo.write(82);               
  }

  void GripperOpen(){
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
    SetSpeed();
    
    //Reverse
    ReverseDirection = true;
    RotationServo.write(95);
    delay(100);
    GripperServo.write(90); 
    DesiredSpeedR = Speed;
    DesiredSpeedL = Speed;
    SetSpeed();
    delay(1000);
    ReverseDirection = false;
    DesiredSpeedR = 0;
    DesiredSpeedL = 0;
    SetSpeed();

    //180
    Serial.println("stopped reverse");
    angleDiff = 2.7;
    angledRightTurn();
    
    while(Serial.available() > 3){
    startX = Serial.parseFloat();
    startY = Serial.parseFloat();
    }
    startX = Serial.parseFloat();
    startY = Serial.parseFloat();

    //straight
    DesiredSpeedR = Speed;
    DesiredSpeedL = Speed;
    SetSpeed();
    delay(500);
   
    while(Serial.available() > 3){ // read serial as long at least 1 bit of data in the serial
    newX = Serial.parseFloat();
    newY = Serial.parseFloat();
    }
    newX = Serial.parseFloat();
    newY = Serial.parseFloat();
    //currentAngle=atan2((startY - newY),(startX - newX));
  }
