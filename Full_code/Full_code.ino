#include <Wire.h> //Included Wire library 

// bluetooth variable starts--------------------------------------------------


 int counter =0;
 char c;
 String dataIn;
 int8_t indexOfA,  indexOfB,  indexOfC,  indexOfD, indexOfE,  indexOfF,  indexOfG,  indexOfH ,indexOfI;

 String    data1,  data2,  data3,  data4, data5,  data6,  data7,  data8, data9;

 float newKp_bluetooth=0;
 float newKd_bluetooth=0;
 float newKi_bluetooth=0;

 bool F,B,L,R,S,startagain;

// bluetooth variable ends--------------------------------------------------


// potentiometer variable starts=============================================

float pot1=0;
float pot2=0;
float pot3=0;
float newkp_pot=0;
float newkd_pot=0;
float newki_pot=0;


// potentiometer variable ends=============================================


// servo declaration starts---------------------------------

  #include <Servo.h>
  Servo myservo1;  // create servo1 object to control a servo
  Servo myservo2;  // create servo2 object to control a servo
  unsigned long counter2=0;
// servo declaration ends---------------------------------

//mpu variable starts-------------------------------------

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float pitchTarget;
float pitch_Error;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

//mpu variable ends--------------------------------------

//kalman filter variable starts--------------------------

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

//kalman filter variable ends--------------------------

//pid variable starts---------------------------------- 

float Kp=112;
float Kd=316;
float Ki=1.80;

float Kp_old=Kp;
float Kd_old=Kd;
float Ki_old=Ki;

//pid variable ends------------------------------------
float rotating_speed=100;

unsigned long milliOld;
unsigned long milliNew; 
unsigned long dt;


//Declaring global variables for mpu

float rollTarget=-4.70;
float rollActual;
float rollError=0;
float rollErrorOld;
float rollErrorChange;
float rollErrorSlope=0;
float rollErrorArea=0;
float rollBalance=0;
float rollBalance_left=0;
float rollBalance_right=0;


//Defining variable for motor
int New_speed_left;
int New_speed_right;
int speed_motor;
int ISR_counter_left=0;
int ISR_counter_right=0;
int Old_speed_left=0;
int Old_speed_right=0;


void setup() {
Serial.begin(57600);
// servo setup starts---------------------------------
   
   

   myservo1.attach(10);                 // attaches the servo on pin 10 to the servo object
   myservo2.attach(11);                 // attaches the servo on pin 11 to the servo object

   myservo1.write(5);                  // sets the servo position according to the scaled value
   delay(20);
   myservo2.write(20);                  // sets the servo position according to the scaled value
   delay(20);                           // waits for the servo to get there
   delay(2000);

// servo setup ends---------------------------------


  Wire.setClock(400000);  //Setting the clock frequency 400khz for I2C communication
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();


//Gyro calibration starts ----------------------------------------------------------------------------

 for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {  //rinning for loop 2000 times for calibration purpose
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;  
//gyro calibration ends---------------------------------------------------------------------------------


angle();  //Called angle function


pinMode(A0, INPUT);             //Setting the A0 as input for using potentiometer as input for setting Kp
pinMode(A1, INPUT);             //Setting the A1 as input for using potentiometer as input for setting Kd
pinMode(A2, INPUT);             //Setting the A2 as input for using potentiometer as input for setting Ki

digitalWrite(5,HIGH);           // Setting direction pin of left stepper high for rotating it clockwise
digitalWrite(6,HIGH);           // Setting direction pin of right stepper high for rotating it clockwise
DDRD =B01101100;                //Setting PORTD of arduino to input and output from D2 to D7 by Direction register


//timer2 starts================================================================


   TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
   TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
   TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
   TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
   TCCR2B |= (1 << CS20);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
   OCR2A = 99;                                                               //The compare register is set to 200us
   TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode


//timer2 ends==================================================================

  milliNew=micros();                                                         //stored current time in milli new for comparing it later

}

void loop() {
  
  //potentiometer  loop starts----------------------------------------------------
Serial.print("kp= ");
   Serial.print(Kp);
   Serial.print("kd= ");
   Serial.print(Kd);
   Serial.print("ki= ");
   Serial.print(Ki);
   Serial.print("rolltarget= ");
   Serial.println(rollTarget);
   
  pot1=analogRead(A0);            //Storing analog value of potentiometer at A0 in pot1 variable
  pot2=analogRead(A1);            //Storing analog value of potentiometer at A1 in pot2 variable
  pot3=analogRead(A2);            //Storing analog value of potentiometer at A2 in pot3 variable



  newkp_pot=map(pot1,0,1023,0,400);  //mapping the value of pot1 to 0-400 and storing it in variable newkp_pot
  newkd_pot=map(pot2,0,1023,0,400);  //mapping the value of pot2 to 0-400 and storing it in variable newkd_pot
  newki_pot=map(pot3,0,1023,0,1500); //mapping the value of pot3 to 0-1500 and storing it in variable newki_pot

  
  if(newkp_pot>10) Kp=newkp_pot;   //putting the condition that the value of newkp_pot will only stored in Kp when it is greater than 10
  if(newkd_pot>10) Kd=newkd_pot;   //putting the condition that the value of newkd_pot will only stored in Kd when it is greater than 10
  if(newki_pot>200) Ki=newki_pot/100;   //putting the condition that the value of newki_pot will only stored in Ki when it is greater than 200


  //potentiometer loop ends-----------------------------------



  //servo loop starts-----------------------------------

 counter2++;                           //increasing the counter2 by one when the servo code starts
 if(counter2>100)                      //executing the condition only when the main loop has already ran 100 times
 {
 myservo1.write(40);                  // sets the servo position to 40 degree
   
 myservo2.write(45);                  // sets the servo position 45 degree

 
 }

if(counter2>200)                      // it gets activated ehen counter2 get higher than 200 i.e the main loop ran 200 times
{

 myservo1.detach();                   //disconnect the servo1 and stop sending the signal to servo1
  myservo2.detach();                  //disconnect the servo1 and stop sending the signal to servo1
}

//servo loop ends----------------------------------------

//bluetooth loop begins=======================================================================



   while(Serial.available()>0)      //The loop activates when the serial buffer has data greater than 1 of arduino
   {
   c=Serial.read();                 //store the buffer data in string "c"
   if(c=='\n') {break;}             // when line change i.e "\n" is detected in string "c" then get out of while loop
   else        {dataIn+=c;}         
   }
   if(c=='\n')
   {
   Parse_the_Data();                //calling the function to storing the data in different variable from data1 to data 9

   F=data1.toInt();                 //Converting the string stored in data1 to integer and storing it in F for moving forward
   S=data2.toInt();                 //Converting the string stored in data2 to integer and storing it in S for stopping robot
   B=data3.toInt();                 //Converting the string stored in data3 to integer and storing it in B for moving robot backward
   
   R=data4.toInt();                 //Converting the string stored in data4 to integer and storing it in R  for turnibg robot rightward
   L=data5.toInt();                 //Converting the string stored in data5 to integer and storing it in L  for turning robot leftward
   newKp_bluetooth=data6.toFloat(); //Converting the string stored in data6 to integer and storing it in newKp_bluetooth  for setting Kp from app
   newKd_bluetooth=data7.toFloat(); //Converting the string stored in data7 to integer and storing it in newKd_bluetooth  for setting Kp from app
   newKi_bluetooth=data8.toFloat(); //Converting the string stored in data8 to integer and storing it in newKi_bluetooth  for setting Kp from app
   startagain=data9.toInt();        //Converting the string stored in data9 to integer and storing it in startagain for resetting arduino when reset button is pressed in app

   c=0;                             // deleting the data stored in "c" for storing new data from serial buffer memory of arduino
   dataIn="";                       // making dataIn blank
   }
  
   if(newKp_bluetooth>10) Kp=newKp_bluetooth;  //putting the condition that the value of newKp_bluetooth will only stored in Kp when it is greater than 10
   if(newKd_bluetooth>10) Kd=newKd_bluetooth;  //putting the condition that the value of newKd_bluetooth will only stored in Kd when it is greater than 10
   if(newKi_bluetooth>1.1) Ki=newKi_bluetooth;   //putting the condition that the value of newKi_bluetooth will only stored in Ki when it is greater than 2

//bluetooth loop ends==========================================================================




 angle();                          //Calling angle() function
  
 rollActual=KalmanAngleRoll;        //storing the current roll angle in rollActual
 
                                                 
 milliOld=milliNew;                 //storing the old time in milliNew
 milliNew=micros();                //storing the current time in milliNew for comparing it with milliold                                                                                              
 dt=milliNew-milliOld;             //Calculating the time difference bw milliNew and milliOld                                                                                         



  rollErrorOld=rollError;
  rollError=rollTarget-rollActual;                                                //error in roll angle wrt reference angle
  rollErrorChange=rollError-rollErrorOld;                                         //difference bw previous and current error
  rollErrorSlope=rollErrorChange/dt;                                              //Slope of error vs time
  rollErrorArea=rollErrorArea+rollError*dt;                                       //adding the previoud roll area and current area



  if(rollErrorArea > 200)rollErrorArea = 200;                                     //limiting the integral value for quicker response
  else if(rollErrorArea < -200)rollErrorArea = -200;

  rollBalance=Kp*rollError+Kd*rollErrorSlope+Ki*rollErrorArea;                    //Calculating the PID Balance point in rollBalance

   if(rollBalance < 10 && rollBalance > -10)rollBalance = 0;                      //consider the robot balanced when rollBalance is from -10to10 and setting rollBalance to 0.

  if(rollBalance > 400)rollBalance = 400;                                        //limit the PID-controller to the maximum controller output i.e 400
  else if(rollBalance < -400)rollBalance = -400;


  if(AngleRoll > 55 || AngleRoll < -55){                                           //stop the robot if it tilt greater than 55 degree
    rollBalance = 0;                                                             //Set the PID controller output to 0 so the motors stop moving
   } 

   
//for controlling bot via app------------------------------------------------------------------------------
 
rollBalance_left=rollBalance;                                                    // storing the value of rollBalance in rollBalance_left variable separately to control left motor separately.
rollBalance_right=rollBalance;                                                   // storing the value of rollBalance in rollBalance_right variable separately to control right motor separately.
 
 if(R==1 || L==1)                                                             //checking the command through bluetooth i.e. left or right
{
 counter++;
// Serial.print("  counter =" );
//  Serial.print(counter);
  if(R==1){                                                                   //rotating the bot in right direction
//     Serial.print("  Right Triggered " );
    rollBalance_left += rotating_speed;                                       //Increase the left motor speed
    rollBalance_right -= rotating_speed;                                      //Decrease the right motor speed
  }
  if(L==1){                                                                   //rotating the bot in left direction         
//    Serial.print("  Left Triggered " );       
    rollBalance_left -= rotating_speed;                                       //Decrease the left motor speed
    rollBalance_right += rotating_speed;                                      //Increase the right motor speed
  }

  
  
 if(counter>20)                                                               //turning the motor in left or right direction for 20 loops when left or right command is recieved through bluetooth
 {
  counter=0;
    R=0;
    L=0;
    F=0;
    B=0;
 }}
 if(F==1){                                                                   //checking for forward command through bluetooth
   rollTarget+=+0.3;                                                         //increasing the balance point of bot by 0.3 degree to move the bot in forward direction 
   F=0;                                                                      //setting again f=0 to avoid the continuous increment of 0.3 degree in set point in every loop
  }
 if(B==1){                                                                   //checking for backward command through bluetooth
    rollTarget+=-0.3;                                                        //decreasing the balance point of bot by 0.3 degree to move the bot in backward direction 
    B=0;                                                                     //setting again b=0 to avoid the continuous decrement of 0.3 degree in set point in every loop
    }   

 if(S==1){                                                                   //checking for stop command through bluetooth 
    rollTarget=-5.2;                                                         //setting the balance point of bot again to normal position i.e.-5.2 degree 
    R=0;                                                                     
    L=0;                                                                     //setting all other commands to zero to stop the bot and balance it at the current position
    F=0;
    B=0;
  }
  if(startagain==1){                  //resetting the arduino when reset buttomn is pressed in app and startagain becomes 1
 pinMode(12, OUTPUT);                //enable pin 12 as output
digitalWrite(12, HIGH);              // setting the D12 as high for resetting the arduino
  }
 
 
 
 
 
// for controlling bot via app-----------------------------------------------------------------------------------------------
 
 
 
 //Motor pulse calculations

//To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
//for right motor---
if(rollBalance_right > 0)rollBalance_right = 404 - (1/(rollBalance_right + 9)) * 5500;           //changing the linar behaviour of rollbalance_right variable into logarithmic behaviour which is required to stepper motor
  else if(rollBalance_right < 0)rollBalance_right = -404 - (1/(rollBalance_right - 9)) * 5500;
  if(rollBalance_right > 0)New_speed_right = 394 - rollBalance_right;                            //Calculating the needed pulse time for the  stepper motor controllers
  else if(rollBalance_right < 0)New_speed_right = -394 - rollBalance_right;
  else New_speed_right = 0;


//for left motor----
  if(rollBalance_left > 0)rollBalance_left = 404 - (1/(rollBalance_left + 9)) * 5500;             //changing the linar behaviour of rollbalance_left variable into logarithmic behaviour which is required to stepper motor
  else if(rollBalance_left < 0)rollBalance_left = -404 - (1/(rollBalance_left - 9)) * 5500;
//Calculating the needed pulse time for the  stepper motor                                        //Calculating the needed pulse time for the  stepper motor controllers
  if(rollBalance_left > 0)New_speed_left = 394 - rollBalance_left;
  else if(rollBalance_left < 0)New_speed_left = -394 - rollBalance_left;
  else New_speed_left = 0;

}

//kalman filter function starts--------------------------------------------------------
//KalmanState is the predicted angle at each state , KalmanUncertainty is the uncertainty in predicted angle at each state 
//KalmanInput is the rotation rate is degree/secound measured by gyroscope , KalmanMeasurement is the angle measured by accelerometer trigonometry
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {                 
  KalmanState=KalmanState+0.004*KalmanInput;                                           //predicted angle(KalmanState) is calculated by adding previous predicted angle and angle change between previous state and current state             
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;                         //uncertainity in predicted angle(KalmanUncertainty) is calculated by adding uncertainity in previous state and iteration length
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);                //kalmangain is calulated here by ratio of predicted angle and accelerometer angle
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);                //new predicted angle is calculated using kalmangain which is much accurate than previous one
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;                                //calculating uncertainity in new predicted angle using kalaman gain
  Kalman1DOutput[0]=KalmanState;                                                       //storing value of predicted angle(KalmanState) in Kalman1DOutput matrix at 0th index
  Kalman1DOutput[1]=KalmanUncertainty;                                                 //storing value of uncertainity in predicted angle(KalmanUncertainty) in Kalman1DOutput matrix at 1st index
}

//kalman filter function ends-----------------------------------------------------------

//gyro_signal function stats------------------------------------------------------------
//this fuction is used to calculate the angle using accelerometer trigonomentry

void gyro_signals(void) {
  Wire.beginTransmission(0x68);                                           //mpu communication starts
  Wire.write(0x1A);                                                       //switch on low pass filter
  Wire.write(0x05);
  Wire.endTransmission();                                                 //mpu communication ends
  Wire.beginTransmission(0x68);                                           //mpu communication statrs
  Wire.write(0x1C);                                                       //configure the accelerometer output
  Wire.write(0x10);
  Wire.endTransmission();                                                 //mpu coomunication ends
  Wire.beginTransmission(0x68);                                           //mpu communication starts
  Wire.write(0x3B);                                                       //first resister to store the accelerometer value
  Wire.endTransmission();                                                 //mpu communication ends
  Wire.requestFrom(0x68,6);                                               //requesting for 6 bytes to store all values of accelerometer
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();                       //reading and storing accelerometer x-direction value 
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();                       //reading and storing accelerometer y-direction value
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();                       //reading and storing accelerometer z-direction value
  Wire.beginTransmission(0x68);                                           //mpu coomunication starts
  Wire.write(0x1B);                                                       //configure the gyroscope output 
  Wire.write(0x8);
  Wire.endTransmission();                                                 //mpu coomunication ends
  Wire.beginTransmission(0x68);                                           //mpu coomunication starts
  Wire.write(0x43);                                                       //first resister to store the gyrosocpe value
  Wire.endTransmission();                                                 //mpu coomunication ends
  Wire.requestFrom(0x68,6);                                               //requesting for 6 bytes to store all values of gyroscope
  int16_t GyroX=Wire.read()<<8 | Wire.read();                             //reading and storing gyrosocpe x-direction value
  int16_t GyroY=Wire.read()<<8 | Wire.read();                             //reading and storing gyroscope y-direction value
  int16_t GyroZ=Wire.read()<<8 | Wire.read();                             //reading and storing gyrosocpe z-direction value
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;                                            //converting the gyrosocpe measurments to physical values
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096-0.11;                                          //converting the accelerometer measurments to physical values
  AccZ=(float)AccZLSB/4096+0.1;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);           // calculating roll angle through accelerometer trigonomentry
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);         // calculating pitch angle through accelerometer trigonomentry
}

//gyro_signal function ends------------------------------------------------------------

//angle function starts----------------------------------------------------------------
//this function gives angle obtained after applying kalman filter
 
void angle()
{
gyro_signals();
  RateRoll-=RateCalibrationRoll;                                                                          //making RateRoll zero at normal position
  RatePitch-=RateCalibrationPitch;                                                                        //making ratePitch zero at normal position
  RateYaw-=RateCalibrationYaw;                                                                            //making RatePitch zero at normal position
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);                            //applying kalman filter to roll angle
  KalmanAngleRoll=Kalman1DOutput[0];                                                                      //storing kalman filtered roll angle
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];                                                           //storing error in kalman filtered roll angle
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);                        //applying kalman filter to pitch angle
  KalmanAnglePitch=Kalman1DOutput[0];                                                                     //storing kalman filtered pitch angle
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];                                                          //storing kalman filtered pitch angle
  // Serial.print("Roll_Angle ");
  // Serial.print(KalmanAngleRoll);

  while (micros() - milliNew < 4000);                                                                     //delay 4000 microsecound
  milliNew=micros();                    
}

//angle function ends----------------------------------------------------------

//bluetooth function starts ===============================================================================================

 void Parse_the_Data()
 {
 indexOfA = dataIn.indexOf("A");                                                //storing the index of 'A' in bluetooth data
 indexOfB = dataIn.indexOf("B");                                                //storing the index of 'B' in bluetooth data
 indexOfC = dataIn.indexOf("C");                                                //storing the index of 'C' in bluetooth data
 indexOfD = dataIn.indexOf("D");                                                //storing the index of 'D' in bluetooth data
 indexOfE = dataIn.indexOf("E");                                                //storing the index of 'E' in bluetooth data
 indexOfF = dataIn.indexOf("F");                                                //storing the index of 'F' in bluetooth data
 indexOfG = dataIn.indexOf("G");                                                //storing the index of 'G' in bluetooth data
 indexOfH = dataIn.indexOf("H");                                                //storing the index of 'H' in bluetooth data
 indexOfI = dataIn.indexOf("I");                                                //storing the index of 'I' in bluetooth data
 data1 = dataIn.substring(0,indexOfA);                                          //storing the substring before 'A' in bluetooth data
 data2 = dataIn.substring(indexOfA+1,indexOfB);                                 //storing substring between 'A' and 'B' in bluetooth data
 data3 = dataIn.substring(indexOfB+1,indexOfC);                                 //storing substring between 'B' and 'C' in bluetooth data
 data4 = dataIn.substring(indexOfC+1,indexOfD);                                 //storing substring between 'C' and 'D' in bluetooth data
 data5 = dataIn.substring(indexOfD+1,indexOfE);                                 //storing substring between 'D' and 'E' in bluetooth data
 data6 = dataIn.substring(indexOfE+1,indexOfF);                                 //storing substring between 'E' and 'F' in bluetooth data
 data7 = dataIn.substring(indexOfF+1,indexOfG);                                 //storing substring between 'F' and 'G' in bluetooth data
 data8 = dataIn.substring(indexOfG+1,indexOfH);                                 //storing substring between 'G' and 'H' in bluetooth data
 data9 = dataIn.substring(indexOfH+1,indexOfI);                                 //storing substring between 'H' and 'I' in bluetooth data
 }


//bluetooth function ends ===============================================================================================


 ISR(TIMER2_COMPA_vect)
{

//left motor------------------------------------------------------------------------
  ISR_counter_left ++;                                      
  if(ISR_counter_left > Old_speed_left)                                                       //for  changing direction of motor
  {             
    ISR_counter_left = 0;                                       
    Old_speed_left = New_speed_left;                   
    if(Old_speed_left < 0){                                     
      PORTD |= B01000000;                                                  
      Old_speed_left *= -1; 
                                     
    }
    else 
    {
      PORTD &= B10111111;
    }
  }
  else if(ISR_counter_left == 1) PORTD |= B00001000;                                          //giving high signal to pin 3
           
  else if(ISR_counter_left == 2)PORTD &= B11110111;                                           //giving low signal to pin 3



 //left motor--------------------------------------------------------------------------


 //right motor--------------------------------------------------------------------------

//
ISR_counter_right ++;                                      
  if(ISR_counter_right > Old_speed_right)                                                       //for  changing direction of motor
  {             
    ISR_counter_right = 0;                                       
    Old_speed_right = New_speed_right;                   
    if(Old_speed_right < 0){                                     
      PORTD |= B00100000;                                                  
      Old_speed_right *= -1; 
                                     
    }
    else 
    {
      PORTD &= B11011111;
    }
  }
  else if(ISR_counter_right == 1) PORTD |= B00000100;                                            //giving high signal to pin 2
           
  else if(ISR_counter_right == 2)PORTD &= B11111011;                                             //giving low signal to pin 2

// right motor--------------------------------------------------------------------------
}
