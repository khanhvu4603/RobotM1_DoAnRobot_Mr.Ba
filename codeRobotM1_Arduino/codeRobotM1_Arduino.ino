// Động cơ DC áp dụng cho trục vít me (tức là trục Z của robot)

#include <TimerOne.h>
String inputCOM ;
typedef struct PID {
  float Kp;
  float Ki;
  float Kd;
  float Kb;
  float SetPoint;
  float Last_error;
  float Windup_guard;
  float PTerm;
  float ITerm;
  float DTerm;
  float WindupMax;
  float WindupMin;
  float FeedbackWindup;
  float Delta_error;
  float Output;
  float OutMax;
  float OutMin;
  float Sample_time;
} PID_TypeDef;
PID_TypeDef pid;
uint32_t i = 0;  // chống nhiễu cho cảm biến

  // Gán giá trị vào các biến
int  emty, nhanA1,nhanA2,nhanB1,nhanB2,nhanC1,nhanC2,nhanD1,nhanD2,nhanE1,nhanE2,nhanF1,nhanF2, tt_kinematic, stt = 0, OK1 =0, OK2 = 0, vl =0  ;
long  d1,d2,d3,d4,d5,d6;


// Hàm tính toán PID
float PID_compute(float Error) {

  pid.PTerm = pid.Kp * Error;

  pid.ITerm += ((Error + pid.Last_error) * pid.Ki * 0.5
                + pid.FeedbackWindup * pid.Kb)
               * (pid.Sample_time / 1000.0);
  if (pid.ITerm > pid.WindupMax)
    pid.ITerm = pid.WindupMax;
  else if (pid.ITerm < pid.WindupMin)
    pid.ITerm = pid.WindupMin;

  pid.Delta_error = Error - pid.Last_error;
  pid.DTerm = pid.Kd * pid.Delta_error / (pid.Sample_time / 1000.0);
  pid.Last_error = Error;

  pid.Output = pid.PTerm + pid.ITerm + pid.DTerm;
  if (pid.Output > pid.OutMax) {
    pid.FeedbackWindup = pid.Output - pid.OutMax;
    pid.Output = pid.OutMax;
  } else if (pid.Output < pid.OutMin) {
    pid.FeedbackWindup = pid.Output - pid.OutMin;
    pid.Output = pid.OutMin;
  } else {
    pid.FeedbackWindup = 0;
  }
  return pid.Output;
}

// Khai báo chân cho động cơ Encoder JGB37
const int ENA = 13;
const int IN1 = 6;
const int IN2 = 7;
const int encoderA = 2;
const int encoderB = 3;
volatile long encoderCount = 0;
bool motorDirection = true;

// Khai báo chân cho hai động cơ bước
const int stepPin2 = 5;  // 4
const int dirPin2 = 11;  //10
const int enPin2 = 9;    //8
const int stepPin1 = 4;  //5
const int dirPin1 = 10;  //11
const int enPin1 = 8;    //9
const int stepPerRevolution = 30000;
long newPulses = 0;

const int namcham1 = 51;
const int namcham2 = 50;



// Biến lưu tổng số xung động cơ cần quay
long totalPulses = 0;
// Khai báo biến lưu góc hiện tại của động cơ bước
int currentAngle1 = 0;  // Động cơ bước 1
int currentAngle2 = 0;  // Động cơ bước 2

unsigned long previousMillis = 0;  // Lưu thời gian trước đó
const long interval = 5000;        // Khoảng thời gian delay (1000ms = 1 giây)
int state = 100;                   // Biến trạng thái để theo dõi tiến trình


// Khai báo các biến toàn cục
String input;    // Chuỗi nhập từ Serial
int angle1 = 0;  // Góc quay động cơ bước 1
int angle2 = 0;  // Góc quay động cơ bước 2
int d = 0;
unsigned long startTime;
unsigned long startTimecc;
// Khai báo cảm biến home cho động cơ JGB37 và động cơ bước
const int homeSensorJGB37 = 12;  // Cảm biến home cho JGB37
const int homeSensorStep1 = 53;  // Cảm biến home cho động cơ bước 1
const int homeSensorStep2 = 52;  // Cảm biến home cho động cơ bước 2

// Khai báo cáo hàm di chuyển điểm
void setHome();
void movetoA();
void movetoB();
void movetoC();
void movetoD();
void movetoE();
void movetoEnd();
void readEncoder();
void hut();
void tha();
void driveMotor(float speed);
void ProcessStep(int nhan1, int nhan2);

// Hàm setup
void setup() {
  inputCOM.reserve(256);  // Dự trữ bộ nhớ cho chuỗi nhận được
  Serial.begin(9600);  // Khởi tạo giao tiếp Serial
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, CHANGE);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);

  digitalWrite(enPin1, LOW);
  digitalWrite(enPin2, LOW);
  pinMode(namcham1, OUTPUT);
  pinMode(namcham2, OUTPUT);

  pinMode(homeSensorJGB37, INPUT);
  pinMode(homeSensorStep1, INPUT_PULLUP);
  pinMode(homeSensorStep2, INPUT_PULLUP);
  pinMode(10, OUTPUT);  // Chân điều khiển nam châm điện

  startTime = millis();
  startTimecc = millis();
  pid.Kp = 6;
  pid.Ki = 10;
  pid.Kd = 0;
  pid.WindupMax = 100;
  pid.WindupMin = -100;
  pid.OutMax = 180;
  pid.OutMin = -180;
  pid.Sample_time = 5;

  Timer1.initialize(5000);  //don vi us
  Timer1.attachInterrupt(PID);
  
}



// Hàm phân tích chuỗi và lưu vào biến
void parseData(String data) {
  // Tách chuỗi dựa vào khoảng trắng
  int index = 0;
  String values[20]; // Dùng mảng để lưu các giá trị

  // Tách các giá trị từ chuỗi
  for (int i = 0; i < 20; i++) {
    int nextSpace = data.indexOf(' ', index); // Tìm khoảng trắng tiếp theo
    if (nextSpace == -1) {
      values[i] = data.substring(index); // Nếu không còn khoảng trắng, lấy phần còn lại
      break;
    }
    values[i] = data.substring(index, nextSpace); // Lấy giá trị giữa khoảng trắng
    index = nextSpace + 1; // Cập nhật vị trí tiếp theo
  }

  // Gán giá trị vào các biến
  emty = values[1].toDouble();
  d1 = values[2].toDouble();
  nhanA1 = values[3].toDouble();
  nhanA2 = values[4].toDouble();
  d2 = values[5].toDouble();
  nhanB1 = values[6].toDouble();
  nhanB2 = values[7].toDouble();
  d3 = values[8].toDouble();
  nhanC1 = values[9].toDouble();
  nhanC2 = values[10].toDouble();
  d4 = values[11].toDouble();
  nhanD1 = values[12].toDouble();
  nhanD2 = values[13].toDouble();
  d5 = values[14].toDouble();
  nhanE1 = values[15].toDouble();
  nhanE2 = values[16].toDouble();

  d6 = values[17].toDouble();
  nhanF1 = values[18].toDouble();
  nhanF2 = values[19].toDouble();
  tt_kinematic = 0;
  stt = 0;
  Serial.println("CU TAO DAI 30 CM ");
  
}



uint8_t mode = 0;
int space1;
int space2;
int space18;
float err = 0;
void loop() {
  if (Serial.available() > 0) {
    
    input = Serial.readString();
    inputCOM = input;
    input.trim();
    
    int indexIK = inputCOM.indexOf("IK ");
    if (indexIK != -1) {
      int endIndex = inputCOM.indexOf(" ", indexIK + 3);  // Tìm khoảng trắng sau giá trị q0
      String IKString = inputCOM.substring(indexIK + 3, endIndex);
      tt_kinematic = IKString.toInt(); 
    }
    if(tt_kinematic == 1)
    {
      parseData(inputCOM);
      state = 0;
    }
    
    if (input == "S") {
      Serial.println("Home Command Received");
      setHome();
      Serial.println("Home Completed, waiting for next command...");
      return;
    }

    if (input == "H") {
      hut();
      return;
    }

    if (input == "T") {
      tha();
      return;
    }

    space1 = input.indexOf(' ');
    space2 = input.indexOf(' ', space1 + 1);
    space18 = input.indexOf(' ', input.lastIndexOf(' ', space1));

    if (input.startsWith("A")) {
      int space1 = input.indexOf(' ');
      int space2 = input.indexOf(' ', space1 + 1);

      long d = input.substring(1, space1).toInt();  // Lấy phần số sau 'A'
      int nhan1 = input.substring(space1 + 1, space2).toInt();
      int nhan2 = -input.substring(space2 + 1).toInt();

      newPulses = -(((350 - d) * 16450) / 340);
      ProcessStep(nhan1, nhan2);
      Serial.println("Đã chạy xong 1 điểm");
      input = "";
      return;
    }
  }

  // IK 1 50 -30 -30 50 30 30 50 -15 -15 50 15 15 50 -45 -45 200 -90 0
  if (input != "S" && space18 != -1 && input.startsWith("B") && state == 100) {
    Serial.println("Processing Move Commands...");
    state = 0;
    tt_kinematic =0;
  }
  //Serial.println("state: " + String(state));
  switch (state) {
    case 0:  // Di chuyển đến A
      if(stt == 0) movetoA(150);
      else if(stt == 1) movetoB(150);
      else if(stt == 2) movetoC(150);
      else if(stt == 3) movetoD(150);
      else if(stt == 4) movetoE(150);
      // previousMillis = millis();  // Lưu thời gian khi bắt đầu
      state = 1;                  // Chuyển sang trạng thái tiếp theo
      hut();
      break;
    case 1:  // Trễ trước khi gọi hut() sau movetoA()
      if (abs(err) < 50 && OK1 == 1 && OK2 == 1) 
      {  
        if (vl == 0)
        {
          startTimecc = millis();
          vl = 1;
        }
        // previousMillis = millis();  // Cập nhật thời gian
        if (millis() - startTimecc > 1000)
        {
          state = 2;
          vl = 0;
        }
          // Chuyển sang trạng thái tiếp theo
      }
      break;
    case 2:  // Di chuyển đến End sau hut()
             // if (millis() - previousMillis >= interval) {
      if(stt == 0) movetoA(0);
      else if(stt == 1) movetoB(0);
      else if(stt == 2) movetoC(0);
      else if(stt == 3) movetoD(0);
      else if(stt == 4) movetoE(0);
      // previousMillis = millis();  // Cập nhật thời gian
      state = 3;  // Chuyển sang trạng thái tiếp theo
      // }
      break;
    case 3:  // Trễ trước khi gọi tha() sau movetoEnd()
             // if (millis() - previousMillis >= interval) {
      if (abs(err) < 50 && OK1 == 1 && OK2 == 1) {
        if (vl == 0)
        {
          startTimecc = millis();
          vl = 1;
        }
        // previousMillis = millis();  // Cập nhật thời gian
        if (millis() - startTimecc > 1000)
        {
          state = 4;
          vl = 0;
        }
      }
      break;
    case 4:  // Di chuyển đến B
             // if (millis() - previousMillis >= interval) {
             // if (abs(err) < 50) {
      if(stt == 0) movetoA(150);
      else if(stt == 1) movetoB(150);
      else if(stt == 2) movetoC(150);
      else if(stt == 3) movetoD(150);
      else if(stt == 4) movetoE(150);
      // previousMillis = millis();  // Cập nhật thời gian
      state = 5;  // Chuyển sang trạng thái tiếp theo
      // }
      break;
    case 5:  // Trễ trước khi gọi hut() sau movetoB()
             // if (millis() - previousMillis >= interval) {
        if (vl == 0)
        {
          startTimecc = millis();
          vl = 1;
        }
        // previousMillis = millis();  // Cập nhật thời gian
        if (millis() - startTimecc > 1000)
        {
          state = 6;
          vl = 0;
        }
      break;
    case 6:  // Di chuyển đến End sau hut()
             // if (millis() - previousMillis >= interval) {
             // if (abs(err) < 50) {
      movetoEnd();
      // previousMillis = millis();  // Cập nhật thời gian
      state = 7;  // Chuyển sang trạng thái tiếp theo
      // }
      break;
    case 7:  // Trễ trước khi gọi tha() sau movetoEnd()
             // if (millis() - previousMillis >= interval) {
      if (abs(err) < 50 && OK1 == 1 && OK2 == 1) {
        tha();
        // previousMillis = millis();  // Cập nhật thời gian
        if (vl == 0)
        {
          startTimecc = millis();
          vl = 1;
        }
        // previousMillis = millis();  // Cập nhật thời gian
        if (millis() - startTimecc > 1000)
        {
          if(stt >= 5) 
          {
            state = 100;
          }
          else {
            stt ++;
            state = 0;  // Chuyển sang trạng thái tiếp theo
          }
          vl = 0;
        }
      }
      break;
  }
}


// Hàm move to 5 points
void movetoA(long ad)  //123
{
  newPulses = -(((350 - (d1 + ad)) * 16450) / 340);
  ProcessStep(nhanA1, nhanA2);
  Serial.println("Đã chạy xong 1 điểm");
  // In kết quả xung và góc sau khi chạy ra
  Serial.println("Moving to A: X=" + String(nhanA1) + ", Y=" + String(nhanA2) + ", Z=" + String(newPulses));
}
void movetoB(long ad)  //456
{
  newPulses = -(((350 - (d2 + ad)) * 16450) / 340);
  ProcessStep(nhanB1, nhanB2);
  Serial.println("Đã chạy xong 1 điểm");

  // In kết quả xung và góc sau khi chạy ra
  Serial.println("Moving to B: X=" + String(nhanB1) + ", Y=" + String(nhanB2) + ", Z=" + String(newPulses));
}

void movetoC(long ad)  //789
{
  newPulses = -(((350 - (d3 + ad)) * 16450) / 340);
  ProcessStep(nhanC1, nhanC2);
  Serial.println("Đã chạy xong 1 điểm");
  // In kết quả xung và góc sau khi chạy ra
  Serial.println("Moving to C: X=" + String(nhanC1) + ", Y=" + String(nhanC2) + ", Z=" + String(newPulses));
}

void movetoD(long ad)  //10 11 12
{
  newPulses = -(((350 - (d4 + ad)) * 16450) / 340);
  ProcessStep(nhanD1, nhanD2);
  ProcessStep(nhanD1, nhanD2);
  Serial.println("Đã chạy xong 1 điểm");
  // In kết quả xung và góc sau khi chạy ra
  Serial.println("Moving to D: X=" + String(nhanD1) + ", Y=" + String(nhanD2) + ", Z=" + String(newPulses));
}
void movetoE(long ad)  //13 14 15
{
  newPulses = -(((350 - (d5 + ad)) * 16450) / 340);
  ProcessStep(nhanE1, nhanE2);
  ProcessStep(nhanE1, nhanE2);
  Serial.println("Đã chạy xong 1 điểm");
  // In kết quả xung và góc sau khi chạy ra
  Serial.println("Moving to E: X=" + String(nhanE1) + ", Y=" + String(nhanE2) + ", Z=" + String(newPulses));
}

void movetoEnd()  //16 17 18
{
  newPulses = -(((350 - d6) * 16450) / 340);
  ProcessStep(nhanF1, nhanF2);
  Serial.println("Đã chạy xong 1 điểm");

  // In kết quả xung và góc sau khi chạy ra
  Serial.println("Moving to END: X=" + String(nhanF1) + ", Y=" + String(nhanF2) + ", Z=" + String(newPulses));
}

// Hàm setHome cho động cơ JGB37
void setHomeJGB37() {
  while (digitalRead(homeSensorJGB37) == LOW) {  // Quay đến khi cảm biến phát hiện home
    // setMotor(230, true); // Quay động cơ với tốc độ chậm
    driveMotor(-230);
  }
  driveMotor(0);
  encoderCount = 0;  // Đặt giá trị xung tại home là 0
  Serial.println("JGB37 đã về home và đặt lại xung.");
}

//hàm setHomeStep1 và setHomeStep2 để dừng động cơ khi đụng cảm biến
void setHomeStep1() {
  while (digitalRead(homeSensorStep1) == HIGH) {  // Quay đến khi cảm biến phát hiện home
    digitalWrite(dirPin1, LOW);                   // Xác định hướng quay về home
    stepMotor(stepPin1);
    if (digitalRead(homeSensorStep1) == LOW) {  // Kiểm tra nếu đụng cảm biến
      break;                                    // Dừng vòng lặp
    }
  }
  currentAngle1 = 0;
  Serial.println("Động cơ bước 1 đã về home và đặt lại góc.");
}

void setHomeStep2() {
  while (digitalRead(homeSensorStep2) == HIGH) {  // Quay đến khi cảm biến phát hiện home
    digitalWrite(dirPin2, HIGH);                  // Xác định hướng quay về home
    stepMotor(stepPin2);
    if (digitalRead(homeSensorStep2) == LOW) {  // Kiểm tra nếu đụng cảm biến
      break;                                    // Dừng vòng lặp
    }
  }
  currentAngle2 = 0;  // Đặt góc hiện tại tại home là 0
  Serial.println("Động cơ bước 2 đã về home và đặt lại góc.");
}

// Hàm xử lý setHome cho tất cả các động cơ
void setHome() {
  Serial.println("Đang set home cho tất cả động cơ...");

  // Set home cho JGB37
  setHomeJGB37();
  Serial.print("Trạng thái encoderCount: ");
  Serial.println(encoderCount);

  // Set home cho động cơ bước 2
  setHomeStep2();
  Serial.print("Góc động cơ bước 2: ");
  Serial.println(currentAngle2);
  currentAngle2 = 0;
  ProcessStep(0, 122);
  currentAngle2 = 0;


  // Set home cho động cơ bước 1
  setHomeStep1();
  Serial.print("Góc động cơ bước 1: ");
  Serial.println(currentAngle1);
  currentAngle1 = 0;
  ProcessStep(-135, 0);
  currentAngle1 = 0;

  Serial.println("Tất cả động cơ đã về home.");
  encoderCount = 0;
  // ProcessStep(-135, 125);
  currentAngle1 = 0;  // Đặt góc hiện tại tại home là 0
  currentAngle2 = 0;  // Đặt góc hiện tại tại home là 0
}


// Đọc encoder
void readEncoder() {
  static int lastState = LOW;
  int currentState = digitalRead(encoderA);
  
  if (currentState != lastState) {
    if (digitalRead(encoderB) != currentState) {
      encoderCount--;
    } else {
      encoderCount++;
    }
    lastState = currentState;
  }
}

void driveMotor(float speed) {
  float pwm = abs(speed);
  if (speed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, pwm);
}

// Hàm điều khiển động cơ bước
void RunStep(int angle, int &currentAngle, int stepPin, int dirPin) {
  OK1 = 0;
  int targetAngle = currentAngle + angle;  // Tính góc đích
  int steps = (abs(targetAngle - currentAngle) <= 360) ? (stepPerRevolution * float(abs(targetAngle - currentAngle)) / 360) : 0;

  digitalWrite(dirPin, angle > 0 ? LOW : HIGH);  // Xác định hướng quay

  for (int i = 0; i < steps; i++) {
    stepMotor(stepPin);
  }
  OK1 = 1;
}

void RunStep2(int angle, int &currentAngle, int stepPin, int dirPin) {
  OK2 = 0;
  int targetAngle = currentAngle + angle;  // Tính góc đích
  int steps = (abs(targetAngle - currentAngle) <= 360) ? (stepPerRevolution * float(abs(targetAngle - currentAngle)) / 360) : 0;

  digitalWrite(dirPin, angle > 0 ? LOW : HIGH);  // Xác định hướng quay

  for (int i = 0; i < steps; i++) {
    stepMotor(stepPin);
  }
  OK2 = 1;
}

void stepMotor(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(200);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(200);
}

void ProcessStep(int nhan1, int nhan2) {
  angle1 = nhan1 - currentAngle1;
  angle2 = nhan2 - currentAngle2;
  currentAngle1 = nhan1;
  currentAngle2 = nhan2;

  RunStep(angle1, currentAngle1, stepPin1, dirPin1);
  delay(20);
  RunStep2(angle2, currentAngle2, stepPin2, dirPin2);
  delay(20);
}

// Điều khiển nam châm điện
void hut() {
  digitalWrite(namcham1, HIGH);  // Đặt chân D10 lên mức HIGH
  // digitalWrite(namcham2, LOW);
}

void tha() {
  digitalWrite(namcham1, LOW);  // Đặt chân D10 xuống mức LOW
  // digitalWrite(namcham2, LOW);
}

void PID()
{
    err = newPulses - encoderCount;
    //Serial.println("err " + String(err));
    float pulse = PID_compute(err);
   //Serial.print("newPulses " + String(newPulses)); Serial.print(" "); Serial.print("encoderCount " + String(encoderCount)); Serial.print(" "); Serial.println("Pulse " + String(-1 * pulse));
    driveMotor(-1 * pulse);
    startTime = millis();
}
