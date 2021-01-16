#include "md_can_motor_driver.h"

//init(),close(), read encoder제외하고 완료


DrokMdMotorDriver::DrokMdMotorDriver()
//: //baudrate_(BAUDRATE),
  //protocol_version_(PROTOCOL_VERSION),
  //left_wheel_id_(DXL_LEFT_ID),
  //right_wheel_id_(DXL_RIGHT_ID)
{
  torque_ = false;
  //dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
}



bool DrokMdMotorDriver::init(String turtlebot3)
{

  uint8_t* test_CAN_comm = NULL;
  test_CAN_comm = CAN_read(PID_VER);      //PID 1을 읽어오는명령으로 CAN 통신을 테스트
  
  if(test_CAN_comm[1]>0){
      DEBUG_SERIAL.println("Success to CAN_comm");
  }
  else{
      DEBUG_SERIAL.println("Failed to CAN_comm");
      return false;
  }
  /*DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }

  // Enable Dynamixel Torque
  setTorque(true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  
  if (turtlebot3 == "Burger")
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
  else if (turtlebot3 == "Waffle or Waffle Pi")
    dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
  else
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;

  DEBUG_SERIAL.println("Success to init Motor Driver");*/
  return true;
}


DrokMdMotorDriver::~DrokMdMotorDriver()
{
  close();
}

/***************************************************
 * 모터토크 on/off함수
 * 다이나믹셀은 토크on/off가 있지만 MD로봇에서 토크on이 의미없음
 ***************************************************/
bool DrokMdMotorDriver::setTorque(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  torque_ = onoff;

  if(!onoff){
    CAN_write(TQ_OFF);   //TQ_OFF : 모터를 자연정지시킴
  }
  //else : MD로봇은 토크 on/off가 따로 없음(터틀봇코드 형식에 맞추기 위한 함수)

  return true;
}

bool DrokMdMotorDriver::getTorque()//(터틀봇코드 형식에 맞추기 위한 함수)
{
  return torque_;
}


void DrokMdMotorDriver::close(void)  //토크 off 외에 특별한 기능 없음
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  //portHandler_->closePort();
  //DEBUG_SERIAL.end();
}


/*********************************
 * 엔코더값을읽음(bradcasting되는데이터를읽기만할건지, 직접 읽어오기 할건지는 미정
 *********************************/
bool DrokMdMotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  /*int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result*/

  int32_t r_enc;
  int32_t l_enc;
  uint8_t* read_arr = NULL;
  
  read_arr = CAN_read(PID_MONITOR);   //오른쪽모터 = 1번모터일경우
  r_enc = Byte2Int32(read_arr[4],read_arr[5],read_arr[6],read_arr[7]);
  
  read_arr = CAN_read(PID_MONITOR2);  //왼쪽모터 = 2번모터일경우
  l_enc = Byte2Int32(read_arr[4],read_arr[5],read_arr[6],read_arr[7]);

  /*
  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  uint8_t* r_data=read(PID_MONITOR);
  uint8_t* l_data=read(PID_MONITOR2);
  
  int32_t right_counting = Byte2Int32(r_data[4], r_data[5], r_data[6], r_data[7]);
  int32_t left_counting  = Byte2Int32(l_data[4], l_data[5], l_data[6], l_data[7]);

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  left_value  = groupSyncReadEncoder_->getData(left_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_value = groupSyncReadEncoder_->getData(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  groupSyncReadEncoder_->clearParam();*/
  return true;
}

/************************************************************
 * 입력한 RPM에 따라 모터를 구동하는 함수
 ***********************************************************/
bool DrokMdMotorDriver::writeVelocity(int16_t L_RPM, int16_t R_RPM)
{
  
  uint8_t RPM_vel_arr[8]={PID_PNT_VEL_CMD,1,0,0,1,0,0,0};  //(MD_gogo 배열과 같은 역할,rpm값 배열)

  //오른쪽 모터 : 1번모터(D2,D3) ,왼쪽모터 2번 모터(D5,D6) 
  //int8_t D1 =1;         //2ch 제어기는 D1,D4가 0이 아니면 두 채널 모두 구동(??)
  
  int8_t D2=R_RPM & 0xff;        //Low data
  int8_t D3=R_RPM>>8 & 0xff;     //high data  
  
  //int8_t D4=1;       
  int8_t D5=L_RPM & 0xff;        //Low data
  int8_t D6=L_RPM>>8 & 0xff;     //high data 
  //int8_t D7;
  
  RPM_vel_arr[2]=D2;
  RPM_vel_arr[3]=D3;
  RPM_vel_arr[5]=D5;
  RPM_vel_arr[6]=D6;

  CAN_write(RPM_vel_arr); 

  return true;
}

/*****************************************************************************
 * value(cmd_vel)값을 받아서 RPM값으로 변환하고 writeVelocity 함수를 실행
 * 실제 사용하는 모터구동함수
 ****************************************************************************/
bool DrokMdMotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
  bool dxl_comm_result = false;

  float lin_vel = value[LINEAR];  
  float ang_vel = value[ANGULAR]; 

  int16_t R_wheel_RPM=0,L_wheel_RPM=0;

  R_wheel_RPM = (int16_t)(GEAR_RATIO*30.0*((2*lin_vel) + (wheel_separation*ang_vel))/(2*wheel_radius*3.141593));
  L_wheel_RPM = -1*(int16_t)(GEAR_RATIO*30.0*((2*lin_vel) - (wheel_separation*ang_vel))/(2*wheel_radius*3.141593));

  dxl_comm_result = writeVelocity(L_wheel_RPM, R_wheel_RPM);
  
  if (dxl_comm_result == false)
    return false;

  return true;
}
/*
//=====================================================================================
//=====================================================================================
//=====================================================================================

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  left_wheel_id_(DXL_LEFT_ID),
  right_wheel_id_(DXL_RIGHT_ID)
{
  torque_ = false;
  dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
}

bool Turtlebot3MotorDriver::init(String turtlebot3)
{
  DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }

  // Enable Dynamixel Torque
  setTorque(true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  
  if (turtlebot3 == "Burger")
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
  else if (turtlebot3 == "Waffle or Waffle Pi")
    dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
  else
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;

  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

bool Turtlebot3MotorDriver::setTorque(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  torque_ = onoff;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_LEFT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_RIGHT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  return true;
}

bool Turtlebot3MotorDriver::getTorque()
{
  return torque_;
}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
  DEBUG_SERIAL.end();
}

bool Turtlebot3MotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  left_value  = groupSyncReadEncoder_->getData(left_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_value = groupSyncReadEncoder_->getData(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  groupSyncReadEncoder_->clearParam();
  return true;
}

bool Turtlebot3MotorDriver::writeVelocity(int64_t left_value, int64_t right_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t left_data_byte[4] = {0, };
  uint8_t right_data_byte[4] = {0, };


  left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_value));
  left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_value));
  left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_value));
  left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_wheel_id_, (uint8_t*)&left_data_byte);
  if (dxl_addparam_result != true)
    return false;

  right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_value));
  right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_value));
  right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_value));
  right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_wheel_id_, (uint8_t*)&right_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}

bool Turtlebot3MotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity_cmd[2];

  float lin_vel = value[LEFT];
  float ang_vel = value[RIGHT];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

  dxl_comm_result = writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}*/
