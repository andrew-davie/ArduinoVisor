////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lite weight version of Nextion library.                                                                                    //
// Uses a fraction of the codespace, typically 4K less                                                                        //
// 10x faster due to not using delays except when blocking, good code won't generally use blocking                            //
// Library by Keith Colson - http://www.edns.co.nz/index.php/about/ - September 2017...                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _NEXLITE_H_
#define _NEXLITE_H_

// *************************** Arduino Library Support ************************************************************************
#include <Arduino.h>

// *************************** Nextion Lite Header ****************************************************************************
#define NEX_RET_CMD_FINISHED         0x01    // status message
#define NEX_RET_EVENT_TOUCH_HEAD     0x65    // every touch seems to have this as the first byte
#define NEX_RET_NUMBER_HEAD          0x71    // get s.val slider value returns with this header

#define BUF_SIZE 10
struct nl_struct{
  uint8_t buff[BUF_SIZE];
  uint8_t prevId;
};

// Config
#define NL_SERIAL  Serial1
#define NL_BAUD    115200                             // baud rate for nextion touch screen

// ************************** Function prototypes ****************************************************************************
void   nl_Callback(struct nl_struct *);                 // put this callback in your file to get nextion events     
bool   nl_Init   (void (*funcPtr)(struct nl_struct *)); // connects the nextion uo to the serial port and returns true if successful
void   nl_Listen (void);                                // call Listen from your main loop to genrate the touch events

void   nl_SendCommand (const char*, uint8_t);           // send command to nextion with ID that will be returned with the response
void   nl_SendCommand2(const char*);                    // send command to nextion with out worrying about response matching
struct nl_struct *nl_SendCommandWait();                 // wait for send command response to be returned
void   nl_GetVal(const char* var, uint8_t id);
void   nl_SetPage(uint8_t page);


#endif

