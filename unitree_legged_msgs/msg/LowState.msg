uint8[2] head
uint8 levelFlag
uint8 frameReserve

uint32[2] SN
uint32[2] version
uint16 bandWidth

IMU imu
MotorState[20] motorState
BmsState bms
int16[4] footForce              # force sensors     # Old version Aliengo is different
int16[4] footForceEst           # force sensors     # Old version Aliengo does not have
uint32 tick                     # reference real-time from motion controller (unit: us)
uint8[40] wirelessRemote        # wireless commands
uint32 reserve                  # Old version Aliengo does not have

uint32 crc
