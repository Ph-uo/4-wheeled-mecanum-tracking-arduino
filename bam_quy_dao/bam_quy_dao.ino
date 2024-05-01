#include <MatrixMath.h>
//define - khai báo
#define m1a 2
#define m1b 3
const byte pwm[]={9,10,11,12};
//coordinate - tọa độ
double xd,yd,thetad;
double x=0,y=0,theta=0;
double pos[3]={x,y,theta};
//robot properties - thông số xe
double l=0.2, d=0.2;


//others - khác
unsigned long t,t_prv=0;

//matrix - ma trận-------------note: kiểm tra xem ngăn nhớ của mạch có thể xử lý được từng này dữ liệu double không
double fw_kinematic[3][4]={{1,1,1,1},{-1,1,-1,1},{-1/(l+d),-1/(l+d),1/(l+d),1/(l+d)}};//dong hoc thuan
double inv_kinematic[4][3]={{1,-1,-(l+d)},{1,1,-(l+d)},{1,-1,(l+d)},{1,1,(l+d)}};//dong hoc nguoc
double Rot[3][3]={{cos(theta),sin(theta),0},{-sin(theta),cos(theta),0},{0,0,1}};//ma trận quay

void setup() {
  Serial.begin(9600);
  for(int i=0;i<5;i++){pinMode(pwm[i],OUTPUT);}
  
    
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
  //1,2,3,4 là tượng trưng cho tốc độ dc1,dc2,..., thay bằng biến đếm xung của các động cơ tương ứng
  w_spd[1][4]={motor_speed(1),motor_speed(2),motor_speed(3),motor_speed(4)};
  double result[3][1]={};
  internal_mobile_spd= Matrix.Mutiply(w_spd,fw_kinematic,1,4,3,result)/4;
}
double pid(){// chua co bo dieu khien pid

}
void motor_control(double speed, char ena,char in1,char in2){
  speed=constrain(speed,0,255);//hàm giới hạn
  analogWrite(ena,speed);
  if (speed>0){
    digitalWrite(in1,1);
    digitalWrite(in2,0);
  }
  else if (speed>0){
    digitalWrite(in1,1);
    digitalWrite(in2,0);
  else {
    digitalWrite(in1,0);
    digitalWrite(in2,0);
  }
}
double motor_speed(unsigned long count){//ham tinh toan toc do cua 1 dong co; input =xung encoder, output= toc do (m/s) va chieu quay
  double theta= count/(13*19.2);
  double speed_now= theta*t*60/1000;//m/s
  if(){//xet chieu quay, chua co ham
    
  }
  return speed_now;
}

// thiet ke lai bo dem xung, chua khai bao bien
/*
void countpulse1a(){
  unsigned long count1a++;
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
*/