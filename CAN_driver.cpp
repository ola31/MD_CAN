#include "CAN_driver.h"

/***********************************/
/*.h파일에서 선언하면 중복정의로 에러발생*/
uint32_t id;
can_message_t tx_msg, rx_msg;
bool interupt_on;
uint8_t CAN_recieve_arr[8];
uint8_t CAN_read_arr[8];
uint8_t R_PID_g;  //R_PID를 저장하는 전역변수 선언
/***********************************/


/***************************************
 * CAN통신 시작(초기화)
 **************************************/
void CAN_initialize(void)
{
  if (CanBus.begin(CAN_BAUD_500K, CAN_EXT_FORMAT) == false)           
  {/*Serial.println("CAN open fail!!");*/}
 
  else{
    id = 184;      // 사용자 제어기 (엠디로봇 : 184), (로보큐브 : 0x123; )  
    CanBus.configFilter(id, 0, CAN_EXT_FORMAT);
  }
}

/****************************************
 * Arr배열을 CAN통신으로 보내주는 함수
 ***************************************/
void CAN_write(uint8_t* Arr) 
{

    //MD로봇은 세미콜론 없이 항상 8개씩 보내줌. 
    
    tx_msg.id = 0xB7B801;            //183,184,01
    tx_msg.format = CAN_EXT_FORMAT;  //엠디로봇은 CAN Extended mode only
    
    int i;
    for(i=0;i<8;i++)
    {
      tx_msg.data[i]=Arr[i];
    }
    tx_msg.length = 8;
    CanBus.writeMessage(&tx_msg);
    
}

/*************************************************
 * MD450T제어기에서 보내는 CAN데이터를 수신하는 함수
 ************************************************/
void CAN_recieve(void) 
{

    int i=0;
    if(CanBus.readMessage(&rx_msg)){
    
      for(i=0;i<8;i++)
      {
        CAN_recieve_arr[i]=rx_msg.data[i];
      }     
    
      CanBus.detachRxInterrupt();  //리턴메시지를 수신하면 인터럽트를 종료한다.
      interupt_on=false;
    }
    
}

/**********************************************
 * R_PID값을 읽어오는 함수
 *********************************************/

uint8_t* CAN_read(uint8_t R_PID)
{
  //R_PID : 값을읽어오고자하는 MD_450T의 PID(파라미터 id)

  interupt_on=false;
  R_PID_g=R_PID;                                        //전역변수 R_PID_g에 R_PID를 저장
  //uint8_t CAN_recieved[8]={0,0,0,0,0,0,0,0};
  uint8_t arr_[8]={PID_REQ_PID_DATA,R_PID,0,0,0,0,0,0};
  
  CanBus.attachRxInterrupt(canRxHandlerTemplate);       //리턴메시지를 수신할 때까지 계속 검사한다.
  interupt_on=true;
  CAN_write(arr_);

  if(!interupt_on){
    
    return CAN_read_arr;
  }
  
}


void canRxHandlerTemplate(can_message_t *arg)
{
  int i=0;
  if(CanBus.readMessage(&rx_msg)){
    if(rx_msg.data[0]=R_PID_g){
    
        for(i=0;i<8;i++)
        {
          CAN_read_arr[i]=rx_msg.data[i];
        }     
    
        CanBus.detachRxInterrupt();  //리턴메시지를 수신하면 인터럽트를 종료한다.
        interupt_on=false;

    }
  }
        
}
/**********************************************************
 * 2바이트정수를 1바이트의 Low_byte, High_byte정수로 나눠주는 함수
 **********************************************************/
data Int2LHByte(int16_t nIn)
{
  data d1;
  d1.low = nIn & 0xff;
  d1.high = nIn>>8 & 0xff;
  return d1;
}

/*******************************************************
 * 1바이트 low,high 데이터를 받아 원 데이터인 2바이트 정수를 만듬
 ******************************************************/
int16_t LHByte2Int16(uint8_t low, uint8_t high)
{
  return (low | (int16_t)high<<8);
}

/***********************************************************
 * 1바이트 D4,D5,D6,D7 데이터를 받아 원 데이터인 4바이트 정수를 만듬
 ***********************************************************/
int32_t Byte2Int32(uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  return ((int32_t)d4 | (int32_t)d5<<8 | (int32_t)d6<<16 | (int32_t)d7<<32);
}
