#include <MatrixMath.h>
//define - khai báo

const byte pwm[]={9,10,11,12};
const byte ina[]={3,4,5,6};
const byte inb[]={22,23,24,25};
const byte encdA []={13,14,15,16};
const byte encdB []={17,18,19,20};
//coordinate - tọa độ
double xd,yd,thetad;
double x=0,y=0,theta=0;
//robot properties(m) - thông số xe(m)
double l=0.2, d=0.2,r=0.15;

//others varaible  - các thông số khác
unsigned long t,t_prv=0;
float error,error_prev, freq = 2*3.14/30;
float integr,derivate;

//matrix - ma trận-------------note: kiểm tra xem ngăn nhớ của mạch có thể xử lý được từng này dữ liệu double không
double fw_kinematic[3][4]={{1,1,1,1},{-1,1,-1,1},{-1/(l+d),-1/(l+d),1/(l+d),1/(l+d)}};//dong hoc thuan
double inv_kinematic[4][3]={{1,-1,-(l+d)},{1,1,-(l+d)},{1,-1,(l+d)},{1,1,(l+d)}};//dong hoc nguoc
double Rot[3][3]={{cos(theta),sin(theta),0},{-sin(theta),cos(theta),0},{0,0,1}};//ma trận quay

void setup() {
  Serial.begin(9600);
  for(int i=0;i<5;i++){
    pinMode(pwm[i],OUTPUT);
    pinMode(ina[i],OUTPUT);
    pinMode(inb[i],OUTPUT);
    pinMode(encdA,inputpullup);
    pinMode(encdB,inputpullup);
    }
  attachinterupt(pintointerupt(encdA[0]),countpulse1(),RISING);

  attachinterupt(pintointerupt(encdA[1]),countpulse2(),RISING);

  attachinterupt(pintointerupt(encdA[2]),countpulse3(),RISING);

  attachinterupt(pintointerupt(encdA[3]),countpulse4(),RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

quydao();
}
double motor_speed(unsigned long count){//ham tinh toan toc do cua 1 dong co; input =xung encoder, output= toc do (m/s) va chieu quay
  double theta= count/(13*19.2);
  double speed_now= theta*t*60/1000;//m/s
  //if(){//xet chieu quay, chua co ham
    
  //}
  return speed_now;
}

void quydao(){//tinh toan quy dao chua xong
  t=millis();
  //quỹ đạo đặt
  xd= 1.1 + 0.7*sin(freq*t);
  yd= 0.9 + 0.7*sin(2*freq*t);
  thetad= 0; 
  //tính toán quỹ đạo thực
  //1,2,3,4 là tượng trưng cho tốc độ dc1,dc2,..., thay bằng biến đếm xung của các động cơ tương ứng
  double w_spd[4][1]={{motor_speed(1)},{motor_speed(2)},{motor_speed(3)},{motor_speed(4)}};
  double result[3][1]={};
  Matrix.Multiply(&w_spd[0][0],&fw_kinematic[0][0],4,1,3,&result[0][0]);
  // tính tốc độ thực của robot trong hệ tọa độ internal
  double internal_mobile_spd[3][1];
  for (int i = 0; i < 3; i++) {
      internal_mobile_spd[i][0] = result[i][0] / 4;
  }
  //tốc độ thực của robot trong hệ global
  double global_mobile_spd [3][1];
  Matrix.Multiply(&internal_mobile_spd[0][0],&Rot[0][0],3,1,3,&global_mobile_spd[0][0]);
  //quỹ đạo thực tế
  x=x+global_mobile_spd[0][0];
  y=y+global_mobile_spd[1][0];
  theta=theta+global_mobile_spd[2][0];
}
double pid(float setpoint, float input, float kp, float ki, float kd){//chưa xong
  double pid,dt;// chưa có dt
  error = setpoint-input;
  integr += (dt * (error + error_prev) / 2);
  derivate= (error - error_prev) / dt;
  pid = kp*error + ki*integr + kd*derivate ;
  error_prev = error;
  return pid;
}
void motor_control(double speed, char ena,char in1,char in2){
  speed=constrain(speed,0,255);//hàm giới hạn
  
  if (speed>0){
    digitalWrite(in1,1);
    digitalWrite(in2,0);
  }
  else if (speed>0){
    digitalWrite(in1,1);
    digitalWrite(in2,0);
  }
  else {
    digitalWrite(in1,0);
    digitalWrite(in2,0);
  }
  analogWrite(ena,abs(speed));
}
/*double motor_speed(unsigned long count){//ham tinh toan toc do cua 1 dong co; input =xung encoder, output= toc do (m/s) va chieu quay
  double theta= count/(13*19.2);
  double speed_now= theta*t*60/1000;//m/s
  if(){//xet chieu quay, chua co ham
    
  }
  return speed_now;
}*/

// thiet ke lai bo dem xung, chua khai bao bien
/*
void countpulse1a(){
  unsigned long count1a++;
}

void countpulse2a(){
  unsigned long count2a++;
}

void countpulse3a(){
  unsigned long count3a++;
}

void countpulse4a(){
  unsigned long count4a++;
}

*/
