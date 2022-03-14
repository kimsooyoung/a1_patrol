ros_control_go1에서는 사실상 두가지만 신경 쓰면 된다.

* HighCmd.msg 
* HighState.msg

이들이 이전과 바뀐 내용을 비교해보자. 

HighCmd => 사실상 BmsCmd 밖에 없음, 그런데 Bms는 명령 내릴게 딱히 없다.
led는 나중에 해보자. => 특정 상황에 대한 indicator가 될 수 있음.

```
BmsCmd bms

int8 off
int8[3] reserve
```

```
LED.msg

uint8 r
uint8 g
uint8 b
```

HighState => motorState, BmsState, rangeObstacle 추가됨
이것들의 publisher를 추가해보자.

motorState
```
uint8 mode           # motor current mode 
float32 q            # motor current position（rad）
float32 dq           # motor current speed（rad/s）
float32 ddq          # motor current speed（rad/s）
float32 tauEst       # current estimated output torque（N*m）
float32 q_raw        # motor current position（rad）
float32 dq_raw       # motor current speed（rad/s）
float32 ddq_raw      # motor current speed（rad/s）
int8 temperature     # motor temperature（slow conduction of temperature leads to lag）
uint32[2] reserve
```

BmsState bms
```
uint8 version_h
uint8 version_l
uint8 bms_status
uint8 SOC

int32 current
uint16 cycle
int8[2] BQ_NTC
int8[2] MCU_NTC
int16[10] cell_vol
```


```
float32[4] rangeObstacle
```