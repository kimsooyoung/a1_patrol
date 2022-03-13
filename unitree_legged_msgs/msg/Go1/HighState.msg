uint8[2] head
uint8 levelFlag
uint8 frameReserve

uint32[2] SN
uint32[2] version
uint16 bandWidth
IMU imu
MotorState[20] motorState
BmsState bms

int16[4] footForce
int16[4] footForceEst

uint8 mode
float32 progress                    # new on Go1, reserve
uint8 gaitType                      # new on Go1, 0.idle  1.trot  2.trot running  3.climb stair
float32 footRaiseHeight             # (unit: m, default: 0.08m), foot up height while walking

float32[3] position                 # (unit: m), from own odometry in inertial frame, usually drift
float32 bodyHeight                  # (unit: m, default: 0.28m)
float32[3] velocity                 # (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
float32 yawSpeed                    # (unit: rad/s), rotateSpeed in body frame        

float32[4] rangeObstacle
Cartesian[4] footPosition2Body      # foot position relative to body
Cartesian[4] footSpeed2Body         # foot speed relative to body
uint8[40] wirelessRemote
uint32 reserve                      # Old version Aliengo does not have
uint32 crc