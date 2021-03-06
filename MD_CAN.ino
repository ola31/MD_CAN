
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


std_msgs::Int32 msg1;  //추가
std_msgs::Int32 msg2;  //추가
ros::Publisher pub1("r_rpm", &msg1); //추가
ros::Publisher pub2("l_rpm", &msg2); //추가


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
osThreadId thread_id_COMMAND;







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
      delay(500);
    
  }
  
}




/************************************************************
*     <THREAD_NAME_COMMAND 스레드에서 돌아가는 함수> 
*       ROS spinonce 가 스레드에서 계속 돌아가게 함 
************************************************************/

static void Thread_Command(void const *argument)
{
  (void) argument;

  for(;;)
  { 
    if(CanBus.readMessage(&rx_msg))
    {
        for(i=0;i<8;i++){CAN_recieved[i]=rx_msg.data[i];}     
    
        re_msg.data[0]=CAN_recieved[0];
        
        
        re_msg.data = CAN_recieved;
        re_msg.data_length = 8;
    
        pub.publish(&re_msg);
    }
    
    msg1.data=(int)(R_RPM); //추가
    msg2.data=(int)(L_RPM); //추가
    pub1.publish(&msg1); //추가
    pub2.publish(&msg2);
    
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
  osThreadDef(THREAD_NAME_COMMAND,  Thread_Command,  osPriorityNormal, 0, 1024);
 
  // create thread
  thread_id_CAN = osThreadCreate(osThread(THREAD_NAME_CAN), NULL);
  thread_id_COMMAND = osThreadCreate(osThread(THREAD_NAME_COMMAND), NULL);


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

}
