#ifndef MD_CAN_MOTOR_DRIVER_H_
#define MD_CAN_MOTOR_DRIVER_H_


#include "CAN_driver.h"
#include "variant.h"  //필요?

#define GEAR_RATIO 30
#define PID_VER 1        //CAN통신 주고받기 가능 여부 테스트 위한 것
#define PID_PNT_VEL_CMD 207
#define PID_MONITOR 196  //모터 1의 모니터데이터(위치정보:D4,5,6,7)
#define PID_MONITOR2 201 //모터 2의 모니터데이터



// 아래 define들은 터틀봇관련
// Limit values (XM430-W210-T and XM430-W350-T)
#define BURGER_DXL_LIMIT_MAX_VELOCITY            265     // MAX RPM is 61 when XL is powered 12.0V

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define LEFT                            0
#define RIGHT                           1

#define LINEAR                           0
#define ANGULAR                          1

#define VELOCITY_CONSTANT_VALUE         41.69988758  // V = r * w = r     *        (RPM             * 0.10472)
                                                     //           = r     * (0.229 * Goal_Velocity) * 0.10472
                                                     //
                                                     // Goal_Velocity = V / r * 41.69988757710309

#define DEBUG_SERIAL  SerialBT2



class DrokMdMotorDriver
{
 public:
  DrokMdMotorDriver();
  ~DrokMdMotorDriver();
  bool init(String turtlebot3);
  void close(void);
  bool setTorque(bool onoff);
  bool getTorque();
  bool readEncoder(int32_t &left_value, int32_t &right_value);
  bool writeVelocity(int16_t L_RPM, int16_t R_RPM);
  bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);

 private:
  //uint32_t baudrate_;
  //float  protocol_version_;
  //uint8_t left_wheel_id_;
  //uint8_t right_wheel_id_;
  bool torque_;

  uint8_t TQ_OFF[8]={5,0,0,0,0,0,0,0};  //자연정지 PID_TQ_OFF(5번) ,private??// PID 174 도 살펴보기

  uint16_t dynamixel_limit_max_velocity_;

};






#endif  //MD_CAN_MOTOR_DRIVER_H_
