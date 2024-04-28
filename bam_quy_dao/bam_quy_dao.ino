define 
define m1a 2;
define m1b 3;
const byte pwm[]={9,10,11,12};

double xd,yd,thetad;
double pos[3]={x=0,y=0,theta=0};


void setup() {
  // put your setup code here, to run once:
  pinMode();
  
}

void loop() {
  // put your main code here, to run repeatedly:
t=millis();
quydao(t);
}

double quydao(t){//tinh toan quy dao chua xong
  freq = 2*pi/30;
  xd= 1.1 + 0.7*sin(freq*t);
  yd= 0.9 + 0.7*sin(2*freq*t);
  thetad= 0; 
  w_spd[]={motor_speed(1),motor_speed(2),motor_speed(3),motor_speed(4)};
  pos[]= {}
}
double pid(){// chua co bo dieu khien pid

}
void motor_control(double speed, char ena,char in1,char in2){
  speed=constraint(speed,0,255);
  analogwrite(ena,speed);
  if (speed>0){
    digitalwrite(in1,1);
    digitalwrite(in2,0);
  }
  else if (speed>0){
    digitalwrite(in1,1);
    digitalwrite(in2,0);
  else {
    digitalwrite(in1,0);
    digitalwrite(in2,0);
  }
}
double calc_spd(int count){//tinh toan toc do cua 1 dong co
  double theta= count/(13*19.2);
  double speed_now= theta*t*60/1000;//m/s
  return speed_now;
}

void countpulse1a(){
  count1a++;
}
void countpulse1b(){
  count1b++;
}
void countpulse2a(){
  count2a++;
}
void countpulse2b(){
  count2b++;
}
void countpulse3a(){
  count3a++;
}
void countpulse3b(){
  count3b++;
}
void countpulse4a(){
  count4a++;
}
void countpulse4b(){
  count4b++;
}
