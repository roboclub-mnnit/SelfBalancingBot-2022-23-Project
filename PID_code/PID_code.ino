float Kp=0;
float Ki=0;
float Kd=0;

unsigned long milliOld;
unsigned long milliNew; 
unsigned long dt;

float pitchTarget=0;
float pitchActual;
float pitchError=0;
float pitchErrorOld;
float pitchErrorChange;
float pitchErrorSlope=0;
float pitchErrorArea=0;
float pitchBalance=;


float rollTarget=0;
float rollActual;
float rollError=0;
float rollErrorOld;
float rollErrorChange;
float rollErrorSlope=0;
float rollErrorArea=0;
float rollBalance=;






void setup() {

  
//Add millis in last of setup 
milliNew=millis();
}

void loop() {
 rollActual=;
 pitchActual=;
 milliOld=milliNew;
 milliNew=millis();
 dt=milliNew-milliOld;


  rollErrorOld=rollError;
  rollError=rollTarget-rollActual;  //error in roll angle wrt reference angle
  rollErrorChange=rollError-rollErrorOld; //difference bw previous and current error
  rollErrorSlope=rollErrorChange/dt;  //Slope of error vs time
  rollErrorArea=rollErrorarea+rollError*dt; //adding the previoud roll area and current area

  pitchErrorOld=pitchError;
  pitchError=pitchTarget-pitchActual;  //error in pitch angle wrt reference angle
  pitchErrorChange=pitchError-pitchErrorOld; //difference bw previous and current error
  pitchErrorSlope=pitchErrorChange/dt;  //Slope of error vs time
  pitchErrorArea=pitchErrorarea+pitchError*dt; //adding the previoud pitch area and current area

  rollBalance=rollBalance+Kp*rollError+Kd*rollErrorSlope+KirollErrorArea

  pitchBalance=pitchBalance+Kp*pitchError+Kd*pitchErrorSlope+Ki*pitchErrorArea
}
