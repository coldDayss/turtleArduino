// 상보필터를 적용한 Roll과 Pitch, Yaw의 각도 구하기       

#define USE_ARDUINO_INTERRUPTS true    // 심박 센서 
#include <PulseSensorPlayground.h> //심박 센서

#include<Wire.h> // 블루투스
#include <SoftwareSerial.h>// 블루투스 

#define BT_RXD 8 //블루투스
#define BT_TXD 7 //블루투스 

//포트 구성//////////////////////////////////////////////
// 자이로 센서 SDA=A4, SCL=A5
SoftwareSerial BTSerial(BT_RXD, BT_TXD); // 블루투스 TXD=D7, RXD=D8
int trigPin = 2; //초음파 센서 D2
int echoPin = 3; //초음파 센서 D3
int piezo = 6; //피에조 선서 D6
const int PulseWire = 0;  //심박센서 A0 

//심박센서 코드 
int Threshold = 550; //심박센서 코드
PulseSensorPlayground pulseSensor; //심박센서 코드 

//자이로 센서 코드 
const int MPU_ADDR = 0x68;    // I2C통신을 위한 MPU6050의 주소
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;   // 가속도(Acceleration)와 자이로(Gyro)
double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFiX, angleFiY, angleFiZ;
const double RADIAN_TO_DEGREE = 180 / 3.14159;  
const double DEG_PER_SEC = 32767 / 250;    // 1초에 회전하는 각도
const double ALPHA = 1 / (1 + 0.04);
// GyX, GyY, GyZ 값의 범위 : -32768 ~ +32767 (16비트 정수범위)
unsigned long now = 0;   // 현재 시간 저장용 변수
unsigned long past = 0;  // 이전 시간 저장용 변수
double dt = 0;           // 한 사이클 동안 걸린 시간 변수 
double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;

//앱인벤터에서 가져온 값을 저장하는 변수
int Temp = 0;//앱인벤터에서 바로 받아온 값 
float Temp2 = 0;//float 자료형으로 변환시키기 위해 사용하는 변수 
float tdegree = -1000;//기울기 값을 저장하기 위한 변수
float tdistance = -1000;//거리 값을 저장하기 위한 변수 

//셋업 구성 //////////////////////////////////////////////////
void setup() {
  //자이로센서 코드 
  initSensor();
  BTSerial.begin(9600);//블루투스 코드
  Serial.begin(9600);
  caliSensor();   //  초기 센서 캘리브레이션 함수 호출
  past = millis(); // past에 현재 시간 저장  

  //초음파 센서 코드 
  pinMode(echoPin, INPUT);   // 초음파 echoPin 입력    
  pinMode(trigPin, OUTPUT);  // 초음파 trigPin 출력
  
  //피에조 센서 코드 
  pinMode(piezo, OUTPUT);

  /*
  tone(piezo, 523); // 5옥타브 도
  delay(1000);
  tone(piezo, 587); // 레
  delay(1000);
  tone(piezo, 659); // 미
  delay(1000);
  tone(piezo, 698); // 파
  delay(1000);  
  tone(piezo, 784); // 솔
  delay(1000);
  tone(piezo, 880); // 라
  delay(1000);
  tone(piezo, 988); // 시
  delay(1000);
  tone(piezo, 1046); // 6옥타브 도
  delay(1000);
  noTone(piezo);
  */

  //심박 센서 코드 
  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(Threshold);
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");
}}
//루프 구성/////////////////////////////////////////////////////
void loop() {

   
  


  //심박 센서 코드 
  int myBPM = pulseSensor.getBeatsPerMinute();
  /*
  if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened". 
   Serial.println("♥  A HeartBeat Happened ! "); // If test is "true", print a message "a heartbeat happened".
   Serial.print("BPM: ");                        // Print phrase "BPM: " 
   Serial.println(myBPM);}                       // Print the value inside of myBPM. 
  */
  
  
  //초음파 센서 코드 
  long duration, distance;
  digitalWrite(trigPin, HIGH);  // trigPin에서 초음파 발생(echoPin도 HIGH)        
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);    // echoPin 이 HIGH를 유지한 시간을 저장 한다.
  distance = ((float)(34000 * duration) / 1000000) / 2;

  /*
  Serial.print("\nDIstance:"); // 물체와 초음파 센서간 거리를 표시        
  Serial.print(distance);
  Serial.println("mm\n");
  */
  
  
  //블루투스 코드
  /*if (BTSerial.available()) {
    Serial.write(BTSerial.read());
  }
  if (Serial.available()) {
    BTSerial.println(Serial.read());
  }*/

 
  getData(); 
  getDT();

  angleAcX = atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2)));
  angleAcX *= RADIAN_TO_DEGREE;
  angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
  angleAcY *= RADIAN_TO_DEGREE;
  // 가속도 센서로는 Z축 회전각 계산 불가함.
  
  // 가속도 현재 값에서 초기평균값을 빼서 센서값에 대한 보정
  angleGyX += ((GyX - averGyX) / DEG_PER_SEC) * dt;  //각속도로 변환
  angleGyY += ((GyY - averGyY) / DEG_PER_SEC) * dt;
  angleGyZ += ((GyZ - averGyZ) / DEG_PER_SEC) * dt;

  // 상보필터 처리를 위한 임시각도 저장
  double angleTmpX = angleFiX + angleGyX * dt;
  double angleTmpY = angleFiY + angleGyY * dt;
  double angleTmpZ = angleFiZ + angleGyZ * dt;

  // (상보필터 값 처리) 임시 각도에 0.96가속도 센서로 얻어진 각도 0.04의 비중을 두어 현재 각도를 구함.
  angleFiX = ALPHA * angleTmpX + (1.0 - ALPHA) * angleAcX;
  angleFiY = ALPHA * angleTmpY + (1.0 - ALPHA) * angleAcY;
  angleFiZ = angleGyZ;    // Z축은 자이로 센서만을 이용하열 구함.  

  //시리얼 모니터 실행 코드 
  Serial.print(distance);//초음파 코드 
  Serial.print("\t");
  Serial.print(angleAcX);// 기울기 코드 
  Serial.print("\t");
  Serial.println(myBPM);//심박수 코드 

  //블루투스 전송 코드 
  BTSerial.print(distance);//초음파 코드 
  BTSerial.print("\t");
  BTSerial.println(angleAcX);// 기울기 코드 
  BTSerial.print("\t");
  BTSerial.println(myBPM);//심박수 코드

  /*
  //블루투스 수신코드 
  //입력된 값이 80보다 크다면 기울기 값으로 인식
  //입력된 값이 80보다 작다면 거리 값으로 인식 
  if(BTSerial.available()){  
    Temp = BTSerial.read();
    Temp2 = float(Temp);            
      if(Temp2>=80){ //기울기값 저장 
        tdegree = Temp2;
      }else{//거리값 저장
        tdistance = Temp2;
      }}
  */
  /*
  //경고음 발생 조건
  //실시간 기울기 값이 지정된 각도보다 30도이상 감소할 경우 피에조 알람을 울림 
  if(angleAcX < tdegree-30.0){
    tone(piezo, 784); // 솔
    delay(500);
    tone(piezo, 784); // 솔
    delay(500);
    tone(piezo, 880); // 라
    delay(500);
    tone(piezo, 880); // 라
    delay(500);
    tone(piezo, 784); // 솔
    delay(500);
    tone(piezo, 784); // 솔
    delay(500);
    tone(piezo, 659); // 미
    delay(500);
  }
  //실시간 거리 값이 저장된 거리보다 10CM이상 가까울 경우 피에조 알람을 울림
  if(distance < tdistance-10){
    tone(piezo, 659); // 미
    delay(1000);
    tone(piezo, 587); // 레
    delay(500);
    tone(piezo, 523); // 5옥타브 도
    delay(500);
    tone(piezo, 587); // 레
    delay(500);
    tone(piezo, 659); // 미
    delay(500);
    tone(piezo, 659); // 미
    delay(500);
    tone(piezo, 659); // 미
    delay(500);
  }
  */

}

void initSensor() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);   // I2C 통신용 어드레스(주소)
  Wire.write(0x6B);    // MPU6050과 통신을 시작하기 위해서는 0x6B번지에    
  Wire.write(0);
  Wire.endTransmission(true);
}

void getData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // AcX 레지스터 위치(주소)를 지칭합니다
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);  // AcX 주소 이후의 14byte의 데이터를 요청
  AcX = Wire.read() << 8 | Wire.read(); //두 개의 나뉘어진 바이트를 하나로 이어 붙여서 각 변수에 저장
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// loop 한 사이클동안 걸리는 시간을 알기위한 함수
void getDT() {
  now = millis();   
  dt = (now - past) / 1000.0;  
  past = now;
}

// 센서의 초기값을 10회 정도 평균값으로 구하여 저장하는 함수
void caliSensor() {
  double sumAcX = 0 , sumAcY = 0, sumAcZ = 0;
  double sumGyX = 0 , sumGyY = 0, sumGyZ = 0;
  getData();
  for (int i=0;i<10;i++) {
    getData();
    sumAcX+=AcX;  sumAcY+=AcY;  sumAcZ+=AcZ;
    sumGyX+=GyX;  sumGyY+=GyY;  sumGyZ+=GyZ;
    delay(50);
  }
  averAcX=sumAcX/10;  averAcY=sumAcY/10;  averAcZ=sumAcY/10;
  averGyX=sumGyX/10;  averGyY=sumGyY/10;  averGyZ=sumGyZ/10;
}
