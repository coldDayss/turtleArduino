# turtleArduino
거북목증후군을 예방하기 위한 아두이노 코드

<센서 구성 및 배치>
1. 초음파 센서[HC-SR04](5V // trig = D2 // echo = D3)
2. 블루투스[HC-06](5V // TXD = D8 // RXD= D7)
3. 심박수 센서[HW-827](5V // A0)
4. 자이로 센서[HW-123](5V // SDA = A4 // SCL = A5)

초음파 센서는 모니터밑에 위치하며 자이로 센서는 사용자의 몸에 부착(헤드폰에 부착하여 사용자의 목에 건 상태로 실험을 진행했음)

스마트폰과 아두이노를 블루투스로 연결하여 초음파, 자이로, 심박수 센서값을 실시간으로 불러올 수 있음

거리 초기화, 기울기 초기화 버튼을 이용하여 사용자마다 다른 환경, 체형에 맞게 기준을 적용시킬 수 있음

설정된 거리값+15CM보다 가까워지거나 설정된 기울기값+10°보다 가까워질 경우 스마트폰에서 문자나 음성으로 사용자에게 경고를 함

![Screenshot_20220611-125526](https://user-images.githubusercontent.com/80840462/173209123-78ffcf0d-d64f-4f0e-831f-e031ef48d712.png)
![Screenshot_20220611-125744](https://user-images.githubusercontent.com/80840462/173209112-0e78c2f2-ddf2-444d-86a1-a9ccd08bf971.png)
