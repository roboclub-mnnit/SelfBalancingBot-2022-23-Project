int a=1;
unsigned long milliNew; 
unsigned long milliOld;

void setup(){
  Serial.begin(57600);
  DDRD =B01101100;
//PinMode(2,OUTPUT);
//PinMode(5,OUTPUT);
//PORTD =B00110000;

  cli();
  TCCR1A=0;
  TCCR1B=0;
  TCCR1B |=B00000010;
  TIMSK1 |=B00000010;
  OCR1A=900;
  sei();
  
  // milliNew=millis();
}
void loop()
{
  //  milliOld=millis();

}
ISR(TIMER1_COMPA_vect)
{
  TCNT1=0;
  if(a==1){
  PORTD =B01101100;
  a--;
  Serial.println(a);
  }
  else
  {
  PORTD =B01100000;
  a++;
  Serial.println(a);
  }
  
}
