// demo: CAN-BUS Shield, send data
#include <mcp_can.h>
#include <SPI.h>
#include <stdio.h>

#define BCM_REQUEST 0x400
#define BCM_DATA1 0x430
#define BCM_DATA2 0x440
#define BCM_STATUS 0x410

#define BCM_ENABLE 0x1
#define BCM_EPO 0x2
#define BCM_LEAK_ENA 0x8
#define BCM_MAINC_CLOSE 0x4

typedef union { 
  unsigned char msg[8];
  struct {
  byte bcm_ready : 1;
  byte bcm_epo : 1;
  byte bcm_on_plug : 1;
  byte bcm_hvil_mon : 1;
  byte bcm_mainC_stat : 2;
  byte bcm_chhgc_stat : 2;
  byte b7 : 4;
  byte bcm_cpwr_cmd : 1;
  byte bcm_cpwr_mon : 1;
  byte bcm_alarm : 2;
  byte bcm_gfld_h :8;
  byte bcm_SOC :8;
  int bcm_ibat :16;
  int             :16; //otherstuff. to be filled in from DBC file siegeljb 5/4/17
} parts;
} BCM_STATUSBitfield;

typedef union { 
  unsigned char msg[8];
  struct {
  byte bcm_eeprom_wip : 1;
  byte  : 1;
  byte bcm_mainc_step : 4;
  byte bcm_fault_mon : 1;
  byte bcm_balancing : 1;
  byte bcm_fdg : 8;
  int bcm_ibat : 16;
  int          :16;
  int          :16; //otherstuff.
} parts;
} BCM_Data2Bitfield;

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

BCM_STATUSBitfield BCM_status_reg;
BCM_Data2Bitfield BCM_DATA2_reg;

//byte msg[8];
unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
unsigned long msgtype;
const byte interruptPin = 2;
void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
//    attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
    attachInterrupt(digitalPinToInterrupt(interruptPin), MCP2515_ISR, FALLING); // start interrupt
    pinMode(interruptPin, INPUT_PULLUP);
    
}

void MCP2515_ISR()
{
    flagRecv = 1;
}

unsigned char stmp[8] = {0, 0, 0, 0};

void loop()
{
    // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
    
    
    CAN.sendMsgBuf(BCM_REQUEST, 0, 4, stmp);
    //while(CAN.
    delay(10);                       // send data per 10ms

   if(flagRecv) 
    {                                   // check if get data

        flagRecv = 0;                   // clear flag

        // iterate over all pending messages
        // If either the bus is saturated or the MCU is busy,
        // both RX buffers may be in use and reading a single
        // message does not clear the IRQ conditon.
        while (CAN_MSGAVAIL == CAN.checkReceive()) 
        {
            
            // read data,  len: data length, buf: data buf
            CAN.readMsgBuf(&len, buf);
            msgtype=CAN.getCanId();
           switch (msgtype) {
           case BCM_DATA1:
           //BCM_DATA1_reg=buf;
            break;
           case BCM_DATA2:
 
            for(int i = 0; i<len; i++)
            {
           BCM_DATA2_reg.msg[i]=buf[i];
            }
            Serial.print("MainC_Step : ");
            Serial.print(BCM_DATA2_reg.parts.bcm_mainc_step,HEX);
            Serial.println();
            break;
           case BCM_STATUS:
                       for(int i = 0; i<len; i++)
            {
           BCM_status_reg.msg[i]=buf[i];
            }
                     if( BCM_status_reg.parts.bcm_hvil_mon)
                     {
                        stmp[0]=BCM_ENABLE;
                        Serial.print("BCM_Enable \n");
                     }
                      if( BCM_status_reg.parts.bcm_cpwr_mon)
                        stmp[0]=BCM_ENABLE|BCM_MAINC_CLOSE;
                        if( BCM_status_reg.parts.bcm_alarm)
                        {
                        Serial.print("BCM_alarm state:");
                        Serial.print( BCM_status_reg.parts.bcm_alarm,DEC);
                        Serial.println();
                        }
            break;
           default:
           
           break;
           }

            // print the data
            for(int i = 0; i<len; i++)
            {
                Serial.print(buf[i],HEX);Serial.print(" ");             
            }
            Serial.println();
        }
    }


    
    
}

//int get_status(byte *msg)
//{
//  bool statusmsg;
//  bool epo;
//  byte mainCstep;
//statusmsg=msg[0]&&0x01;
//epo=
//
//}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
