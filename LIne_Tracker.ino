int i=0, j=0, k=0, m=0, a=0, b=0;

int motor11=26, motor12=27, motor21=31, motor22=30, pwm1=6, pwm2=7;
int speed1=200, speed2=200; //right    left
int counter = 0;
int seconds=500;
int return_time;
int Setpoint = 100;

float right_flag, right, center, left, left_flag, right_flag_up, left_flag_up, center_up, right_up, left_up;
float right_flag_avg, right_avg, center_avg, left_avg, left_flag_avg, right_flag_up_avg, left_flag_up_avg, center_up_avg, right_up_avg, left_up_avg;
float right_flag_avg1=0, right_avg1=0, center_avg1=0, left_avg1=0, left_flag_avg1=0, right_flag_up_avg1=0, left_flag_up_avg1=0, center_up_avg1=0, right_up_avg1=0, left_up_avg1=0;
float right_flag_avg2=0, right_avg2=0, center_avg2=0, left_avg2=0, left_flag_avg2=0, right_flag_up_avg2=0, left_flag_up_avg2=0, center_up_avg2=0, right_up_avg2=0, left_up_avg2=0;

int multiplier=1;
char path[1000];

double Input, Output;
double kp=2, ki=3, kd=1;
unsigned long lastTime;
double ITerm, lastInput;
int SampleTime = 400; //1 sec
double outMin=0, outMax=250;
bool inAuto = false;
 
#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;

void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(9600); 
  delay(5000); 
}


void loop()
{  
  if(k==0)
  {
    Serial.println("calibrate");  
    calibrate();
    k=1;
  }
  if(m==0)
  {
    Serial.println("Tracking_Line");  
    Tracking_Line();
    m=1;
  }
}

void Tracking_Line()
{
  while(1)
  {
    left_flag=analogRead(A0);
    left=analogRead(A1);    
    center=analogRead(A2);
    right_flag=analogRead(A3);
    right=analogRead(A4);
     
    right_flag_up=analogRead(A5);
    right_up=analogRead(A6);
    center_up=analogRead(A7);
    left_up=analogRead(A8);    
    left_flag_up=analogRead(A9);

  if(left_flag_up<left_flag_up_avg)
  {
    a=1;
    Serial.println("                      a==1");
    Serial.println("                      b==");
    Serial.print(b);
    Serial.println(" ");
  }
  if(right_flag_up<right_flag_up_avg)
  {
    b=1;
    Serial.println("                      b==1");
    Serial.println("                      a==");
    Serial.print(a);
    Serial.println(" ");
  }
  
  if((right_flag<right_flag_avg)||(left_flag<left_flag_avg)||((right_flag<right_flag_avg)&&(left_flag<left_flag_avg)))
  {
      if((right_flag<right_flag_avg)&&(left_flag>left_flag_avg))
      {
              if((a==0)&&(b==1))
              {
                  Serial.println("c0a1");
                  while(right_flag<right_flag_avg)
                  {
                      right_flag=analogRead(A3);
                      straight();
                      Serial.println("S");
                  }
                  path[j]='S';Serial.println(j);Serial.println("path==S");delay(300);
                  j=j+1;
                  a=0;
                  b=0;
              }
              else if((a==1)&&(b==1))
              {
                  Serial.println("c0b1");
                  while(left_flag_up>left_flag_up_avg)
                  {
                       left_flag_up=analogRead(A9);
                       straight();
                       Serial.println("S");
                  }
                  path[j]='S';Serial.println(j);Serial.println("path==S");delay(300);
                  j=j+1;
                  a=0;
                  b=0;
            }
      

   }
   else if((left_flag<left_flag_avg)&&(right_flag>right_flag_avg))
   {   
       if((a==1)&&(b==0))
       {
          Serial.println("c2b1");
          while(left_flag_up>left_flag_up_avg)
          {
              Serial.println("c2b1a");
              left_flag_up=analogRead(A9);
              turn_left();
              Serial.println("L");
          }
          while(center_up>center_up_avg)
          {
              Serial.println("c2b1b");
              center_up=analogRead(A7);
              turn_left();
              Serial.println(center_up);
              Serial.println(center_up);
              Serial.println("L");
          }
          path[j]='L';Serial.println(j);Serial.println("path==L");delay(300);
          j=j+1;
          a=0;
          b=0;
       }
       else if((a==1)&&(b==1))
       {
          Serial.println("c0b1");
          while(left_flag_up>left_flag_up_avg)
          {
             left_flag_up=analogRead(A9);
             straight();
             Serial.println("S");
          }
          path[j]='S';Serial.println(j);Serial.println("path==S");delay(300);
          j=j+1;
          a=0;
          b=0;
      }
   }
  else if((right_flag<right_flag_avg)&&(left_flag<left_flag_avg))
  {
       if((a==1)&&(b==1))
       {
          Serial.println("c0b1");
          while(left_flag_up>left_flag_up_avg)
          {
             left_flag_up=analogRead(A9);
             straight();
             Serial.println("S");
          }
          path[j]='S';Serial.println(j);Serial.println("path==L");delay(300);
          j=j+1;
          a=0;
          b=0;
       }
      else if((right_flag_up<right_flag_up_avg)&&(left_flag_up<left_flag_up_avg)&&(right<right_avg)&&(left<left_avg)&&(center_up>center_up_avg))
      {
        Serial.println("c5a");
        stop_bot();
      }  
    }
  }
  if((right<right_avg)&&(center<center_avg))
  {
    Input=97;
    Serial.println("Plus");delay(300);
  }
  if((right<right_avg)&&(center>center_avg))
  {
    Input=95;
    Serial.println("plus++");delay(300);
  }
  if((right>right_avg)&&(center<center_avg)&&(left>left_avg))
  {
     Input=100;
     Serial.println("zero");delay(300);
     
  }  
  if((left<left_avg)&&(center<center_avg))
  {
    Input=103;
    Serial.println("minus");delay(300);
  }
  if((left<left_avg)&&(center>center_avg))
  {
    Input=105;
    Serial.println("minus--");delay(300);

  }
  Compute();Serial.println(Output);delay(300);
  if(Output>0)
  {
    speed1=speed1-(Output*multiplier);
    speed2=speed2;  
    digitalWrite(motor11, HIGH);
    digitalWrite(motor12, LOW);
    analogWrite(pwm1, speed1);
    
    digitalWrite(motor21, HIGH);
    digitalWrite(motor22, LOW);
    analogWrite(pwm2, speed2); 
  }
  else if(Output<0)
  {
    speed1=speed1;
    speed2=speed2+(Output*multiplier);
    
    digitalWrite(motor11, HIGH);
    digitalWrite(motor12, LOW);
    analogWrite(pwm1, speed1);
    
    digitalWrite(motor21, HIGH);
    digitalWrite(motor22, LOW);
    analogWrite(pwm2, speed2); 
  }
  }
}


void turn_right()
{
  digitalWrite(motor11, LOW);
  digitalWrite(motor12, HIGH);
  analogWrite(pwm1, 250);
  
  digitalWrite(motor21, HIGH);
  digitalWrite(motor22, LOW);
  analogWrite(pwm2, 250);
  
}

void turn_left()
{  
  digitalWrite(motor11, HIGH);
  digitalWrite(motor12, LOW);
  analogWrite(pwm1, 250);
 
  digitalWrite(motor21, LOW);
  digitalWrite(motor22, HIGH);
  analogWrite(pwm2, 250);
}
void straight()
{
  digitalWrite(motor11, HIGH);
  digitalWrite(motor12, LOW);
  analogWrite(pwm1, 250);
  
  digitalWrite(motor21, HIGH);
  digitalWrite(motor22, LOW);
  analogWrite(pwm2, 250);
}
void stop_bot()
{
  digitalWrite(motor11, HIGH);
  digitalWrite(motor12, LOW);
  analogWrite(pwm1, 0);
  
  digitalWrite(motor21, HIGH);
  digitalWrite(motor22, LOW);
  analogWrite(pwm2, 0);
}

void calibrate()
{
for(i=0; i<5; i++)
  {  
    digitalWrite(13, HIGH);   
    delay(1000);             
  
  }
for(i=0; i<100;i++)
  {

    left_flag=analogRead(A0);
    left=analogRead(A1);    
    center=analogRead(A2);
    right_flag=analogRead(A3);
    right=analogRead(A4);
     
    right_flag_up=analogRead(A5);
    right_up=analogRead(A6);
    center_up=analogRead(A7);
    left_up=analogRead(A8);    
    left_flag_up=analogRead(A9);
 


    left_flag_avg1=left_flag_avg1+left_flag;
    left_avg1=left_avg1+left;   
    center_avg1=center_avg1+center; 
    right_flag_avg1=right_flag_avg1+right_flag;
    right_avg1=right_avg1+right;
    
    right_flag_up_avg1=right_flag_up_avg1+right_flag_up;
    left_flag_up_avg1=left_flag_up_avg1+left_flag_up;
    center_up_avg1=center_up_avg1+center_up;
    right_up_avg1=right_up_avg1+right_up;
    left_up_avg1=left_up_avg1+left_up;
    
  }
   
    left_flag_avg1=left_flag_avg1/100;
    left_avg1=left_avg1/100;   
    center_avg1=center_avg1/100; 
    right_flag_avg1=right_flag_avg1/100;
    right_avg1=right_avg1/100;
    
    right_flag_up_avg1=right_flag_up_avg1/100;
    left_flag_up_avg1=left_flag_up_avg1/100;
    center_up_avg1=center_up_avg1/100;
    right_up_avg1=right_up_avg1/100;
    left_up_avg1=left_up_avg1/100;

digitalWrite(13, LOW);  
delay(10000); 

for(i=0; i<5;i++)
  {  
    digitalWrite(13, HIGH);   
    delay(1000);             

  }
    
  for(i=0; i<100;i++)
  {
    left_flag=analogRead(A0);
    left=analogRead(A1);    
    center=analogRead(A2);
    right_flag=analogRead(A3);
    right=analogRead(A4);
     
    right_flag_up=analogRead(A5);
    right_up=analogRead(A6);
    center_up=analogRead(A7);
    left_up=analogRead(A8);    
    left_flag_up=analogRead(A9);
 


    left_flag_avg2=left_flag_avg2+left_flag;
    left_avg2=left_avg2+left;   
    center_avg2=center_avg2+center; 
    right_flag_avg2=right_flag_avg2+right_flag;
    right_avg2=right_avg2+right;
    
    right_flag_up_avg2=right_flag_up_avg2+right_flag_up;
    left_flag_up_avg2=left_flag_up_avg2+left_flag_up;
    center_up_avg2=center_up_avg2+center_up;
    right_up_avg2=right_up_avg2+right_up;
    left_up_avg2=left_up_avg2+left_up;
  }
    
    left_flag_avg2=left_flag_avg2/100;
    left_avg2=left_avg2/100;   
    center_avg2=center_avg2/100; 
    right_flag_avg2=right_flag_avg2/100;
    right_avg2=right_avg2/100;
    
    right_flag_up_avg2=right_flag_up_avg2/100;
    left_flag_up_avg2=left_flag_up_avg2/100;
    center_up_avg2=center_up_avg2/100;
    right_up_avg2=right_up_avg2/100;
    left_up_avg2=left_up_avg2/100;

    
    left_flag_avg=(left_flag_avg2+left_flag_avg1)/2;
    left_avg=(left_avg2+left_avg1)/2;
    center_avg=(center_avg2+center_avg1)/2;
    right_flag_avg=(right_flag_avg2+right_flag_avg1)/2;
    right_avg=(right_avg2+right_avg1)/2;

    
    right_flag_up_avg=(right_flag_up_avg2+right_flag_up_avg1)/2;
    right_up_avg=(right_up_avg2+right_up_avg1)/2;
    center_up_avg=(center_up_avg2+center_up_avg1)/2;
    left_up_avg=(left_up_avg2+left_up_avg1)/2;
    left_flag_up_avg=(left_flag_up_avg2+left_flag_up_avg1)/2;
      Serial.print(left_flag_up_avg);
  Serial.print("  ");
  Serial.print(left_up_avg);
  Serial.print("  ");
  Serial.print(center_up_avg);
  Serial.print("  ");
  Serial.print(right_up_avg);
  Serial.print("  ");
  Serial.print(right_flag_up_avg);  
  Serial.println("  ");
  
  
  Serial.print(left_flag_avg);
  Serial.print("  ");
  Serial.print(left_avg);
  Serial.print("  ");
  Serial.print(center_avg);
  Serial.print("  ");
  Serial.print(right_avg);
  Serial.print("  ");
  Serial.print(right_flag_avg);
  Serial.print("  ");
  Serial.println("  "); 
  Serial.println("  "); 
    digitalWrite(13, LOW); 
    delay(5000); 
    Serial.println("Done");    
    delay(300);
}

void Compute()
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      if(dInput>0)
      {
        Output = kp * error + ITerm- kd * dInput;
      }
      else if(dInput<0)
      {
        Output = kp * error + ITerm-(- (kd * dInput));
      }
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
