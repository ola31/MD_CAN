#ifndef CAN_DRIVER_H_
#define CAN_DRIVER_H_

#include <CAN.h>

uint32_t id;
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

#define PID_REQ_PID_DATA 4

void CAN_initialize(void);
void CAN_write(uint8_t* Arr);
uint8_t* CAN_read(uint8_t R_PID);
void canRxHandlerTemplate(can_message_t *arg);
uint8_t CAN_recieved[8]={0,0,0,0,0,0,0,0};
bool interupt_on;





#endif  //CAN_DRIVER_H_
