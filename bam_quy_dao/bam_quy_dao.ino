define

double xd,yd,thetad;
double pos[3]={x=0,y=0,theta=0};


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

double quydao(t){
  freq = 2*pi/30;
  xd= 1.1 + 0.7*sin(freq*t);
  yd= 0.9 + 0.7*sin(2*freq*t);
  thetad= 0; 
  w_sp[]={motor_speed(1),motor_speed(2),motor_speed(3),motor_speed(4)};
  pos[]= {}
}
double pid(){

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
void countpulse1a(){
  count++;
}
double motor_speed(){
  double theta= count/(13*19.2);
  double speed_now= theta*t*60/1000;//m/s
  return speed_now;
}