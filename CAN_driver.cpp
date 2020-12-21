#include "CAN_driver.h"


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

/**********************************************
 * R_PID값을 읽어오는 함수
 *********************************************/

uint8_t* CAN_read(uint8_t R_PID)
{
  //R_PID : 값을읽어오고자하는 MD_450T의 PID(파라미터 id)

  interupt_on=false;
   
  //uint8_t CAN_recieved[8]={0,0,0,0,0,0,0,0};
  uint8_t arr_[8]={PID_REQ_PID_DATA,R_PID,0,0,0,0,0,0};
  
  CanBus.attachRxInterrupt(canRxHandlerTemplate); //리턴메시지를 수신할 때까지 계속 검사한다.
  interupt_on=true;
  CAN_write(arr_);

  if(!interupt_on){
    
    return CAN_recieved;
  }
  
}


void canRxHandlerTemplate(can_message_t *arg)
{
  int i=0;
  if(CanBus.readMessage(&rx_msg)){
    
      for(i=0;i<8;i++)
      {
        CAN_recieved[i]=rx_msg.data[i];
      }     
    
      CanBus.detachRxInterrupt();  //리턴메시지를 수신하면 인터럽트를 종료한다.
      interupt_on=false;
  }
        
}
