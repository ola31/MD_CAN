
#include <ros.h>
#include <RTOS.h>
#include <time.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32.h>


#include <CAN.h>
#include<stdio.h>                
#include<stdlib.h> 
 
     
#define USE_USBCON
#define SEND_INTERVAL_MS 1000

#define distance 0.4       //바퀴 사이 간격
#define wheel_radius 0.08  //바퀴 반지름
#define gear_ratio 1       //원래 기어비 : 30

/********************************************************************
*터틀봇코드관련 include (turtlebot3_burger.h)
********************************************************************/
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
/********************************************************************
*터틀봇코드관련 define (turtlebot3_burger.h)
********************************************************************/
#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.160           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

/********************************************************************
*터틀봇코드관련 define (turtlebot3_core_config.h)
********************************************************************/
#define FIRMWARE_VER "1.2.3"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree


/*******************************************************************************
 * 터틀봇코드 관련 함수정의(turtlebot3_core_config.h)
 ******************************************************************************/

// Function prototypes

void publishImuMsg(void);
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

void waitForSerialLink(bool isConnected);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
//ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Turtlebot3
turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("firmware_version", &version_info_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Command velocity of Turtlebot3 using RC100 remote controller
geometry_msgs::Twist cmd_vel_rc100_msg;
ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Turtlebot3
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;





 
/*******************************************************************************
 * 기존 함수,변수선언(CAN통신 등 관련)
 ******************************************************************************/

static int16_t R_RPM=0,L_RPM=0;  //  -32768<={ RPM }<=32767  (RPM범위)(32767보다 크면 음수화, -32768보다 작으면 오버플로우)

static bool same_vel=false;            //이전RPM==현재 RPM 이면 true



void VelCb(const geometry_msgs::Twist& cmd_msg);          //콜백함수
void Switch_command(const std_msgs::UInt8& cmd_mode_msg);   //콜백함수 
void mdcanCallback(const std_msgs::UInt8MultiArray& msg); //콜백함수


void CAN_Until_8(uint8_t* Arr);                         //CAN통신 날리는 함수
void cmd_vel2RPM(float lin_vel, float ang_vel);         //선,각속도->를 RPM값으로.


ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", VelCb );             //선,각속도 sub
ros::Subscriber<std_msgs::UInt8> sub2("cmd_mode", Switch_command );       //배열번호값 sub
ros::Subscriber<std_msgs::UInt8MultiArray> sub3("mdcan", mdcanCallback);  //직접입력한 CAN data값 sub


std_msgs::Int32 msg1;  //추가
std_msgs::Int32 msg2;  //추가
ros::Publisher pub1("r_rpm", &msg1); //추가
ros::Publisher pub2("l_rpm", &msg2); //추가


std_msgs::UInt8MultiArray re_msg;
ros::Publisher pub("recived", &re_msg);  //CAN리턴값 pub







//==================================================================================
//====================    모터 제어기 CAN통신 명령어 모음   =============================


uint8_t MD_gogo[8]={207,1,0,0,1,0,0,0};  //듀얼채널 모터 2개 구동(PID207번) ,속도제어배열
uint8_t MD_ros[8]={0,0,0,0,0,0,0,0};      //ROS serial 통신으로 직접 모든 명령값을 지정

uint8_t REQ_PID_DATA[8]={4,0,0,0,0,0,0,0};  //데이터 요청 PID_REQ_PID_DATA(4번)
uint8_t TQ_OFF[8]={5,0,0,0,0,0,0,0};  //자연정지 PID_TQ_OFF(5번) 
uint8_t BRAKE[8]={6,0,0,0,0,0,0,0};   //전기적 브레이크 PID_BRAKE(6번) 


uint8_t nothing[1]={0};    // nothing일때는 CAN통신 보내지않도록 되어있음


//==================================================================================
uint8_t CAN_recieved[8]={0,0,0,0,0,0,0,0};


uint8_t*goArr=nothing;    // 명령어배열을 저장하는 포인터 (goArr의 초기 값 = nothing 배열주소)











/*********************************************************************
*  ROS TOPIC CALLBACK 함수 (3개)
*********************************************************************/


void VelCb(const geometry_msgs::Twist& cmd_msg)
{   

   //cmd_vel값을 받아, MD_gogo배열에 속도(rpm)값 저장
   
   float linearX = cmd_msg.linear.x;
   float angularZ = cmd_msg.angular.z;

   cmd_vel2RPM(linearX, angularZ);

}


void Switch_command(const std_msgs::UInt8& cmd_mode_msg)
{   
  
  //정수값(1~4)를 받아, 각 경우에 해당하는 명령배열을 goArr에 저장
   
  uint8_t Mode = cmd_mode_msg.data;
  switch(Mode)
  {

    case 0 : goArr=nothing; break;
    case 1 : goArr=MD_gogo; break;
    case 2 : goArr=MD_ros; break;
    //case 3 : goArr=Vel_CTRL; break;
    //case 4 : goArr=Vel_GOGO; break;
    //case 7 : goArr=re_set; break;
    case 7 : goArr=TQ_OFF; break;
    default : goArr=nothing;
  }
  
}



void mdcanCallback(const std_msgs::UInt8MultiArray& msg)  
{
    //ros serial로 받은 CAN통신값(PID, data0~7)을 
    //MD_ros배열에 넣어주는 콜백함수
    
    MD_ros[0]=msg.data[0];
    MD_ros[1]=msg.data[1];
    MD_ros[2]=msg.data[2];
    MD_ros[3]=msg.data[3];
    MD_ros[4]=msg.data[4];
    MD_ros[5]=msg.data[5];
    MD_ros[6]=msg.data[6];
    MD_ros[7]=msg.data[7];
    
}









//CAN 통신 관련 변수 

uint32_t t_time, id, i;
can_message_t tx_msg, rx_msg;
/*
 *  typedef struct 
 *  {
 *    uint32_t id      : Identifier of received message
 *    uint32_t length  : Length of received message data
 *    uint8_t  data[8] : Data of received message
 *    uint8_t  format  : Type of ID
 *  } can_message_t;
 * 
 * BAUDRATE :
 *   CAN_BAUD_125K
 *   CAN_BAUD_250K
 *   CAN_BAUD_500K
 *   CAN_BAUD_1000K
 * 
 * FORMAT :
 *   CAN_STD_FORMAT
 *   CAN_EXT_FORMAT
*/



// 쓰레드관련
osThreadId thread_id_CAN;
osThreadId thread_id_ROS;







/*************************************************************
*        <THREAD_NAME_CAN 스레드에서 돌아가는 함수>
*      <500ms 마다 goArr 에 저장된 배열을 CAN통신으로 날려보냄.>
*************************************************************/

static void Thread_CAN(void const *argument)  
{
  (void) argument;
  
  tx_msg.id = 0xB7B801;            //183,184,01
  tx_msg.format = CAN_EXT_FORMAT;  //엠디로봇은 CAN Extended mode only
  static uint8_t pre_D2=2,pre_D3=2,pre_D5=2,pre_D6=2;

  for(;;)  //무한 반복  
  {
      if(goArr==MD_gogo)
      {
        if(goArr[2]==pre_D2 && goArr[3]==pre_D3 && goArr[5]==pre_D5 && goArr[6]==pre_D6)
        {
          same_vel==true;
        }
        else
        {
          same_vel=false;
          pre_D2=goArr[2];pre_D3=goArr[3];pre_D5=goArr[5];pre_D6=goArr[6];
        }
      }
    
      if((same_vel==false)&&(goArr!=nothing))
      {
          CAN_Until_8(goArr);
      }
      delay(30);
    
  }
  
}




/************************************************************
*     <THREAD_NAME_ROS 스레드에서 돌아가는 함수> 
*       ROS spinonce가 스레드에서 계속 돌아가게 함 
************************************************************/

static void Thread_ROS(void const *argument)
{
  (void) argument;

  for(;;)
  {
    //터틀봇코드
    uint32_t t = millis();
    updateTime();
    updateVariable(nh.connected());
    updateTFPrefix(nh.connected()); 

    //원래코드(CAN 회신데이터 publish)
    if(CanBus.readMessage(&rx_msg)){
        
        for(i=0;i<8;i++)
        {
          CAN_recieved[i]=rx_msg.data[i];
        }     
    
        re_msg.data[0]=CAN_recieved[0];
        
        re_msg.data = CAN_recieved;
        re_msg.data_length = 8;
    
        pub.publish(&re_msg);
    }
    
    msg1.data=(int)(R_RPM); //추가
    msg2.data=(int)(L_RPM); //추가
    pub1.publish(&msg1); //추가
    pub2.publish(&msg2);

    //터틀봇코드
    
    
    
    nh.spinOnce();
    delay(5);   //can통신을 보내주는 delay보다 짧아야 한다. 
  }
}








/***************************************************************
*  SET UP  
***************************************************************/

void setup() 
{

  nh.initNode();

  nh.advertise(pub1); //추가
  nh.advertise(pub2); //추가

  nh.advertise(pub);//추가(received Pub)
  
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);

  //nh.advertise(pub);//추가

  nh.advertise(sensor_state_pub);  
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);

  tf_broadcaster.init(nh);

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();


  
  Serial.begin(115200);
  
  if (CanBus.begin(CAN_BAUD_500K, CAN_EXT_FORMAT) == false)           
  {Serial.println("CAN open fail!!");}
 
  else
  {
    id = 184;      // 사용자 제어기 (엠디로봇 : 184), (로보큐브 : 0x123; )  
    CanBus.configFilter(id, 0, CAN_EXT_FORMAT);
  }


  // define thread
  osThreadDef(THREAD_NAME_CAN, Thread_CAN, osPriorityNormal, 0, 1024);
  osThreadDef(THREAD_NAME_ROS, Thread_ROS, osPriorityNormal, 0, 1024);
 
  // create thread
  thread_id_CAN = osThreadCreate(osThread(THREAD_NAME_CAN), NULL);
  thread_id_ROS = osThreadCreate(osThread(THREAD_NAME_ROS), NULL);


  // start kernel
  osKernelStart();
  
}





void loop()   //loop함수는 사용하지 않음
{
  
}






/******************************************************************
*      배열인자를 CAN으로 보내주는 함수  
******************************************************************/

void CAN_Until_8(uint8_t* Arr)  //MD로봇은 세미콜론 없이 항상 8개씩 보내줌. 
{

    //MD로봇은 세미콜론 없이 항상 8개씩 보내줌. 
    
    int i;

    for(i=0;i<8;i++)
    {
      tx_msg.data[i]=Arr[i];
    }
    tx_msg.length = 8;
    CanBus.writeMessage(&tx_msg);
    
}



/******************************************************************************
*   전진 선속도, 회전 각속도를 받아서 
*   좌,우 바퀴의 회전속도(rpm)을 계산하고 속도 배열(MD_gogo)에 넣어주는 함수
******************************************************************************/


void cmd_vel2RPM(float linear_vel, float angular_vel)
{
  
  //오른쪽 모터 : 1번모터(D2,D3) ,왼쪽모터 2번 모터(D5,D6)
  
  R_RPM = (int16_t)(gear_ratio*30.0*((2*linear_vel) + (distance*angular_vel))/(2*wheel_radius*3.141593));
  L_RPM = -1*(int16_t)(gear_ratio*30.0*((2*linear_vel) - (distance*angular_vel))/(2*wheel_radius*3.141593));


  //int8_t D1 =1;         //2ch 제어기는 D1,D4가 0이 아니면 두 채널 모두 구동(??)
  
  int8_t D2=R_RPM & 0xff;      //Low data
  int8_t D3=R_RPM>>8 & 0xff;     //high data  
  
  //int8_t D4=1;       
  int8_t D5=L_RPM & 0xff;    
  int8_t D6=L_RPM>>8 & 0xff;
  //int8_t D7;

  MD_gogo[2]=D2;
  MD_gogo[3]=D3;
  MD_gogo[5]=D5;
  MD_gogo[6]=D6;
  
}









/*********************************************************************************/

/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{ 
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);  
}


/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}


/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;  

  battery_state_pub.publish(&battery_state_msg);
}



/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}




/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}



/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}



/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}




/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      sensors.initIMU();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}


/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}


/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}




/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}
