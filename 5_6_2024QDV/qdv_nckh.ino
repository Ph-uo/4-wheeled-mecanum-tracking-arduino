#include <DueTimer.h>
#include <Encoder.h>
int motor1En1 = 8, motor1in1 = 24, motor1in2 = 25;
int motor2En2 = 9, motor2in1 = 23, motor2in2 = 22;
int motor3En3 = 12, motor3in1 = 26, motor3in2 = 27;
int motor4En4 = 11, motor4in1 = 29, motor4in2 = 28;

const float L = 1.0; 
const float R = 1.0;
const float vertices[5][2] = { {1, -1}, {L, R}, {-1, R}, {-1, -1}, {1, -1} }; // Đỉnh của hình chữ nhật

float kp_ex = 0.2, ki_ex = 0.003, kd_ex = -0.001;
float kp_ey = 0.2, ki_ey = 0.003, kd_ey = -0.001;
float kp_eth = 0.05, ki_eth = 0.001, kd_eth = -0.001;
float error_ex, error_prevex = 0, integ_ex, integ_prevex = 0;
float error_ey, error_prevey = 0, integ_ey, integ_prevey = 0;
float error_eth, error_preveth = 0, integ_eth, integ_preveth = 0;
float ex1 = 0, ey1 = 0,eth1=0;

float kp1=0.5, ki1=0.2, kd1=0.01;
// float kp2=0.45, ki2=0.2, kd2=0;
// float kp3=0.45, ki3=0.2, kd3=0;
// float kp4=0.45, ki4=0.2, kd4=0;
float theta_now1 = 0, theta_prev1 = 0;
float theta_now2 = 0, theta_prev2 = 0;
float theta_now3 = 0, theta_prev3 = 0;
float theta_now4 = 0, theta_prev4 = 0;

float xd_dot, yd_dot, xd = 1, yd = -1,thetad=0;
float heso = 0.15, r = 0.075;
float vx = 0, vy = 0, ex = 0, ey = 0,eth=0;
//float x = 0, y = 0;
float x = 1, y = -1;
float vxa=0,vya=0,vx1 = 0, vy1 = 0, w1 = 0;
float lx = 0.225, ly = 0.18, a = 0, pi = 3.14;
float Ui1 = 0, Ui2 = 0, Ui3 = 0, Ui4 = 0;
float x_dot = 0, y_dot = 0;
float t = 0;
float dt = 0.1;
float dt1 = 0.095;

int PWMval1 = 0, PWMval2 = 0, PWMval3 = 0, PWMval4 = 0;
float motorSpeed1 = 0, motorSpeed2 = 0, motorSpeed3 = 0, motorSpeed4 = 0;
float RPM_input1 = 0, RPM_input2 = 0, RPM_input3 = 0, RPM_input4 = 0;

float error_now1, error_prev1 = 0, integ_now1, integ_prev1 = 0;
float error_now2, error_prev2 = 0, integ_now2, integ_prev2 = 0;
float error_now3, error_prev3 = 0, integ_now3, integ_prev3 = 0;
float error_now4, error_prev4 = 0, integ_now4, integ_prev4 = 0;
float Vmin = 1, Vmax = 24;
float PID_DC1, PID_DC2, PID_DC3, PID_DC4;

float x_encoder = 0, y_encoder = 0;
float x_dot_encoder = 0, y_dot_encoder = 0;
float vx_encoder = 0, vy_encoder = 0, w_encoder = 0;
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount3 = 0;
volatile long encoderCount4 = 0;
long encoderCount_prev1 = 0;
long encoderCount_prev2 = 0;
long encoderCount_prev3 = 0;
long encoderCount_prev4 = 0;

float x_cam = 0,y_cam = 0,th_cam=0;
const byte pinEncoderA1 = 33, pinEncoderA2 = 32; // Chân đọc encoder B
Encoder myEncoder1(pinEncoderA1, pinEncoderA2);
const byte pinEncoderB1 = 30, pinEncoderB2 = 31; // Chân đọc encoder B
Encoder myEncoder2(pinEncoderB1, pinEncoderB2);
const byte pinEncoderC1 = 35, pinEncoderC2 = 34; // Chân đọc encoder B
Encoder myEncoder3(pinEncoderC1, pinEncoderC2);
const byte pinEncoderD1 = 36, pinEncoderD2 = 37; // Chân đọc encoder B
Encoder myEncoder4(pinEncoderD1, pinEncoderD2);

void my_HN1() {
  encoderCount1 = myEncoder1.read();
}
void my_HN2() {
  encoderCount2 = myEncoder2.read();
}
void my_HN3() {
  encoderCount3 = myEncoder3.read();
}
void my_HN4() {
  encoderCount4 = myEncoder4.read();
}

void quydao_HV() {
  if (x_cam == 0.00 && y_cam == 0.00) {
    t = 0;
    xd = 0;
    yd = 0;
    xd_dot = 0;
    yd_dot = 0;
  }
  int segment_index = floor((t * heso) / (2 * (L + R)));
  float t_segment = fmod(t * heso, 2 * (L + R));
  int start_index =segment_index % 4;
  int end_index = (segment_index + 1) % 4;

  float start_vertex_x = vertices[start_index][0];
  float start_vertex_y = vertices[start_index][1];
  float end_vertex_x = vertices[end_index][0];
  float end_vertex_y = vertices[end_index][1];

  xd = start_vertex_x + (end_vertex_x - start_vertex_x) * (t_segment / (2 * (L + R)));
  yd = start_vertex_y + (end_vertex_y - start_vertex_y) * (t_segment / (2 * (L + R)));
  thetad=1.57;// rad(qdv)
  //thetad=atan2(xd,yd);//rad(qd tron)

  ex = xd - x_cam;
  ey = yd - y_cam;
  eth = thetad-th_cam;

  error_ex = ex;
  integ_ex = integ_prevex + (dt * (error_ex + error_prevex) / 2);
  ex1 = kp_ex * error_ex + ki_ex * integ_ex + (kd_ex * ((error_ex - error_prevex) / dt));
  integ_prevex = integ_ex;
  error_prevex = error_ex;

  error_ey = ey;
  integ_ey = integ_prevey + (dt * (error_ey + error_prevey) / 2);
  ey1 = kp_ey * error_ey + ki_ey * integ_ey + (kd_ey * ((error_ey - error_prevey) / dt));
  integ_prevey = integ_ey;
  error_prevey = error_ey;

  error_eth = eth;
  integ_eth = integ_preveth + (dt * (error_eth + error_preveth) / 2);
  eth1 = kp_eth * error_eth + ki_eth * integ_eth + (kd_eth * ((error_eth - error_preveth) / dt));
  integ_preveth = integ_eth;
  error_preveth = error_eth;

  vxa = ex1;
  vya = ey1;
  w1 = eth1;

  vx1 = (cos(th_cam) * vxa) + (sin(th_cam)) * vya;
  vy1 = (-sin(th_cam) * vxa) + (cos(th_cam)) * vya;

  a = lx + ly;
  Ui1 = (1 / r) *( vx1 - vy1 + (-a * w1));
  Ui2 = (1 / r) *( vx1 + vy1 + ( a * w1));
  Ui3 = (1 / r) *( vx1 + vy1 + (-a * w1));
  Ui4 = (1 / r) *( vx1 - vy1 + ( a * w1));
  
  t += dt1;
}

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  pinMode(motor1En1, OUTPUT);
  pinMode(motor1in1, OUTPUT);
  pinMode(motor1in2, OUTPUT);
  pinMode(motor2En2, OUTPUT);
  pinMode(motor2in1, OUTPUT);
  pinMode(motor2in2, OUTPUT);
  pinMode(motor3En3, OUTPUT);
  pinMode(motor3in1, OUTPUT);
  pinMode(motor3in2, OUTPUT);
  pinMode(motor4En4, OUTPUT);
  pinMode(motor4in1, OUTPUT);
  pinMode(motor4in2, OUTPUT);

  Timer1.attachInterrupt(my_HN1);
  Timer1.start(1000);
  Timer2.attachInterrupt(my_HN2);
  Timer2.start(1000);
  Timer3.attachInterrupt(my_HN3);
  Timer3.start(1000);
  Timer4.attachInterrupt(my_HN4);
  Timer4.start(1000);

}

void loop() {
  Data_camera_esp();
  Serial.print(x_cam);
  Serial.print(",");
  Serial.print(y_cam);
  Serial.print(",");    
  Serial.print(th_cam);
  Serial.print(",");
  Serial.print(vy1);
  if (x_cam == 0.00 && y_cam == 0.00) {
    
    analogWrite(motor1En1, 0);
    analogWrite(motor2En2, 0);
    analogWrite(motor3En3, 0);
    analogWrite(motor4En4, 0);
    
    digitalWrite(motor1in1, 0);
    digitalWrite(motor1in2, 0);
    digitalWrite(motor2in1, 0);
    digitalWrite(motor2in2, 0);
    digitalWrite(motor3in1, 0);
    digitalWrite(motor3in2, 0);
    digitalWrite(motor4in1, 0);
    digitalWrite(motor4in2, 0);
    Serial.println("dung lai:");
    // Ngừng việc đọc encoder khi dừng lại
    Timer1.stop();
    Timer2.stop();
    Timer3.stop();
    Timer4.stop();
  } 
  else {
    // Nếu không, tiếp tục tính toán và điều khiển
    quydao_HV();
    PID1();
    PID2();
    PID3();
    PID4();
    Timer1.attachInterrupt(my_HN1);
    Timer1.start(1000);
    Timer2.attachInterrupt(my_HN2);
    Timer2.start(1000);
    Timer3.attachInterrupt(my_HN3);
    Timer3.start(1000);
    Timer4.attachInterrupt(my_HN4);
    Timer4.start(1000);

    //Serial.print();
    // Serial.print(xd);
    // Serial.print(",");
    // Serial.print(yd);
    // Serial.print(",");
    // Serial.print(x_cam);
    // Serial.print(",");
    // Serial.print(y_cam);
    // Serial.print(",");    
    // Serial.print(vx1);
    // Serial.print(",");
       Serial.println("dang chay");
  }
}

void PID1() {
  RPM_input1 = abs(((Ui1*r)/(pi*2*r))*60);
  //RPM_input1 = (((Ui1*r)/(pi*2*r))*60);
  theta_now1 = encoderCount1 - encoderCount_prev1;
  encoderCount_prev1 = encoderCount1;
  motorSpeed1 = (theta_now1) * 14 / (13 * 19.2 * (100 / 1000.0));
  if (Ui1 > 0) {
    error_now1 = RPM_input1 - motorSpeed1;
  } else if (Ui1 < 0) {
    error_now1 = RPM_input1 + motorSpeed1;
  }
  integ_now1 = integ_prev1 + (0.1 * (error_now1 + error_prev1) / 2);
  PID_DC1 = kp1*error_now1 + ki1*integ_now1 + (kd1*(error_now1 - error_prev1)/0.1);
  if (PID_DC1 > Vmax) {
    PID_DC1 = Vmax;
    integ_now1 = integ_prev1;
  }
  if (PID_DC1 < Vmin) {
    PID_DC1 = Vmin;
    integ_now1 = integ_prev1;
  }
  PWMval1 = int(180 * abs(PID_DC1) / Vmax);
  PWMval1=constrain(PWMval1,0,150);
  if (Ui1 > 0) {
    digitalWrite(motor1in1, HIGH);
    digitalWrite(motor1in2, LOW);
    analogWrite(motor1En1, PWMval1);    
  }
  if (Ui1 < 0) {
    digitalWrite(motor1in1, LOW);
    digitalWrite(motor1in2, HIGH);
    analogWrite(motor1En1, PWMval1);
  }
  theta_prev1 = theta_now1;
  integ_prev1 = integ_now1;
  error_prev1 = error_now1;
}
void PID2() {
  RPM_input2 = abs(((Ui2*r)/(pi*2*r))*60);
  //RPM_input2 = (((Ui2*r)/(pi*2*r))*60);
  theta_now2 = encoderCount2 - encoderCount_prev2;
  encoderCount_prev2 = encoderCount2;
  motorSpeed2 = (theta_now2) * 14 / (13 * 19.2 * (100 / 1000.0));
  if (Ui2 > 0) {
    error_now2 = RPM_input2 - motorSpeed2;
  } else if (Ui2 < 0) {
    error_now2 = RPM_input2 + motorSpeed2;
  }

  integ_now2 = integ_prev2 + (0.1 * (error_now2 + error_prev2) / 2);
  PID_DC2 = kp1*error_now2 + ki1*integ_now2 + (kd1*(error_now2 - error_prev2)/0.1);
  if (PID_DC2 > Vmax) {
    PID_DC2 = Vmax;
    integ_now2 = integ_prev2;
  }
  if (PID_DC2 < Vmin) {
    PID_DC2 = Vmin;
    integ_now2 = integ_prev2;
  }

  PWMval2 = int(180 * abs(PID_DC2) / Vmax);
  PWMval2=constrain(PWMval2,0,150);
  if (Ui2 > 0) {
    digitalWrite(motor2in1, HIGH);
    digitalWrite(motor2in2, LOW);
    analogWrite(motor2En2, PWMval2);    
  }
  if (Ui2 < 0) {
    digitalWrite(motor2in1, LOW);
    digitalWrite(motor2in2, HIGH);
    analogWrite(motor2En2, PWMval2);
  }
  theta_prev2 = theta_now2;
  integ_prev2 = integ_now2;
  error_prev2 = error_now2;
}
void PID3() {
  RPM_input3 = abs(((Ui3*r)/(pi*2*r))*60);
  //RPM_input3 = (((Ui3*r)/(pi*2*r))*60);
  theta_now3 = encoderCount3 - encoderCount_prev3;
  encoderCount_prev3 = encoderCount3;
  motorSpeed3 = (theta_now3) * 14 / (13 * 19.2 * (100 / 1000.0));
  if (Ui3 > 0) {
    error_now3 = RPM_input3 - motorSpeed3;
  } else if (Ui3 < 0) {
    error_now3 = RPM_input3 + motorSpeed3;
  }
  integ_now3 = integ_prev3 + (0.1 * (error_now3 + error_prev3) / 2);
  PID_DC3 = kp1*error_now3 + ki1*integ_now3 + (kd1*(error_now3 - error_prev3)/0.1);
  if (PID_DC3 > Vmax) {
    PID_DC3 = Vmax;
    integ_now3 = integ_prev3;
  }
  if (PID_DC3 < Vmin) {
    PID_DC3 = Vmin;
    integ_now3 = integ_prev3;
  }

  PWMval3 = int(180 * abs(PID_DC3) / Vmax);
  PWMval3=constrain(PWMval3,0,150);
  if (Ui3 > 0) {
    digitalWrite(motor3in1, HIGH);
    digitalWrite(motor3in2, LOW);
    analogWrite(motor3En3, PWMval3);    
  }
  if (Ui3 < 0) {
    digitalWrite(motor3in1, LOW);
    digitalWrite(motor3in2, HIGH);
    analogWrite(motor3En3, PWMval3);
  }
  theta_prev3 = theta_now3;
  integ_prev3 = integ_now3;
  error_prev3 = error_now3;
}
void PID4() {
  RPM_input4 = abs(((Ui4*r)/(pi*2*r))*60);
  //RPM_input4 = (((Ui4*r)/(pi*2*r))*60);
  theta_now4 = encoderCount4 - encoderCount_prev4;
  encoderCount_prev4 = encoderCount4;
  motorSpeed4 = (theta_now4) * 14 / (13 * 19.2 * (100 / 1000.0));
  if (Ui4 > 0) {
    error_now4 = RPM_input4 - motorSpeed4;
  } else if (Ui4 < 0) {
    error_now4 = RPM_input4 + motorSpeed4;
  }
  integ_now4 = integ_prev4 + (0.1 * (error_now4 + error_prev4) / 2);
  PID_DC4 = kp1*error_now4 + ki1*integ_now4 + (kd1*(error_now4 - error_prev4)/0.1);
  if (PID_DC4 > Vmax) {
    PID_DC4 = Vmax;
    integ_now4 = integ_prev4;
  }
  if (PID_DC4 < Vmin) {
    PID_DC4 = Vmin;
    integ_now4 = integ_prev4;
  }

  PWMval4 = int(180 * abs(PID_DC4) / Vmax);
  PWMval4=constrain(PWMval4,0,150);
  if (Ui4 > 0) {
    digitalWrite(motor4in1, HIGH);
    digitalWrite(motor4in2, LOW);
    analogWrite(motor4En4, PWMval4);    
  }
  if (Ui4 < 0) {
    digitalWrite(motor4in1, LOW);
    digitalWrite(motor4in2, HIGH);
    analogWrite(motor4En4, PWMval4);
  }
  theta_prev4 = theta_now4;
  integ_prev4 = integ_now4;
  error_prev4 = error_now4;
}

void Data_camera_esp() {
  if (Serial3.available() > 0) {
    // Đọc dữ liệu từ ESP32
    String inputString = Serial3.readStringUntil('\n');
    inputString.trim();
    Serial.println(inputString);
    // Tách giá trị x, y, th và w từ chuỗi nhận được
    int commaIndex1 = inputString.indexOf(',');
    if (commaIndex1 != -1) {
      String xString = inputString.substring(0, commaIndex1);
      int commaIndex2 = inputString.indexOf(',', commaIndex1 + 1);
      if (commaIndex2 != -1) {
        String yString = inputString.substring(commaIndex1 + 1, commaIndex2);
        String thString = inputString.substring(commaIndex2 + 1);
        // Chuyển đổi chuỗi thành số thực 
        x_cam = xString.toFloat();
        y_cam = yString.toFloat();
        th_cam = thString.toFloat();
      }
    }
  }

}

