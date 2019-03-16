#ifndef __SX127X_H__
#define __SX127X_H__

#define REG_LR_FIFO                                  0x00 
 // Common settings
#define REG_LR_OPMODE                                0x01 
#define REG_LR_BANDSETTING                           0x04
#define REG_LR_FRFMSB                                0x06 
#define REG_LR_FRFMID                                0x07
#define REG_LR_FRFLSB                                0x08 
 // Tx settings
#define REG_LR_PACONFIG                              0x09 
#define REG_LR_PARAMP                                0x0A 
#define REG_LR_OCP                                   0x0B 
 // Rx settings
#define REG_LR_LNA                                   0x0C 
 // LoRa registers
#define REG_LR_FIFOADDRPTR                           0x0D 
#define REG_LR_FIFOTXBASEADDR                        0x0E 
#define REG_LR_FIFORXBASEADDR                        0x0F 
#define REG_LR_FIFORXCURRENTADDR                     0x10 
#define REG_LR_IRQFLAGSMASK                          0x11 
#define REG_LR_IRQFLAGS                              0x12 
#define REG_LR_NBRXBYTES                             0x13 
#define REG_LR_RXHEADERCNTVALUEMSB                   0x14 
#define REG_LR_RXHEADERCNTVALUELSB                   0x15 
#define REG_LR_RXPACKETCNTVALUEMSB                   0x16 
#define REG_LR_RXPACKETCNTVALUELSB                   0x17 
#define REG_LR_MODEMSTAT                             0x18 
#define REG_LR_PKTSNRVALUE                           0x19 
#define REG_LR_PKTRSSIVALUE                          0x1A 
#define REG_LR_RSSIVALUE                             0x1B 
#define REG_LR_HOPCHANNEL                            0x1C 
#define REG_LR_MODEMCONFIG1                          0x1D 
#define REG_LR_MODEMCONFIG2                          0x1E 
#define REG_LR_SYMBTIMEOUTLSB                        0x1F 
#define REG_LR_PREAMBLEMSB                           0x20 
#define REG_LR_PREAMBLELSB                           0x21 
#define REG_LR_PAYLOADLENGTH                         0x22 
#define REG_LR_PAYLOADMAXLENGTH                      0x23 
#define REG_LR_HOPPERIOD                             0x24 
#define REG_LR_FIFORXBYTEADDR                        0x25
#define REG_LR_MODEMCONFIG3                          0x26
 // end of documented register in datasheet
 // I/O settings
#define REG_LR_DIOMAPPING1                           0x40
#define REG_LR_DIOMAPPING2                           0x41
 // Version
#define REG_LR_VERSION                               0x42
 // Additional settings
#define REG_LR_PLLHOP                                0x44
#define REG_LR_TCXO                                  0x4B
#define REG_LR_PADAC                                 0x4D
#define REG_LR_FORMERTEMP                            0x5B
#define REG_LR_BITRATEFRAC                           0x5D
#define REG_LR_AGCREF                                0x61
#define REG_LR_AGCTHRESH1                            0x62
#define REG_LR_AGCTHRESH2                            0x63
#define REG_LR_AGCTHRESH3                            0x64


#define GPIO_VARE_1                                  0X00
#define GPIO_VARE_2                                  0X00
#define RFLR_MODEMCONFIG2_SF_MASK                    0x0f
#define RFLR_MODEMCONFIG1_CODINGRATE_MASK            0xF1 
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK          0xFB 
#define RFLR_MODEMCONFIG1_BW_MASK                    0x0F 
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK        0xFE 
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK        0xfc
#define RFLR_MODEMCONFIG3_MOBILE_NODE_MASK           0xF7 

#define TIME_OUT_INT                                 0x80 
#define PACKET_RECVER_INT                            0x40 
#define CRC_ERROR_INT                                0x20 
#define RECVER_HEAR_INT                              0x10 
#define FIFO_SEND_OVER                               0x08 
#define RFLR_IRQFLAGS_CAD                            0x04 
#define RFLR_IRQFLAGS_FHSS                           0x02 
#define RFLR_IRQFLAGS_CADD                           0x01 

#define IRQN_TXD_Value                               0xF7
#define IRQN_RXD_Value                               0x9F
#define IRQN_CAD_Value                               0xFA
#define IRQN_SEELP_Value                             0xFF
#define PACKET_MIAX_Value                            0xff

typedef enum 
{
   Sleep_mode	        = (unsigned char)0x00, 
   Stdby_mode	        = (unsigned char)0x01, 
   TX_mode 	       	  = (unsigned char)0x02,
   Transmitter_mode		= (unsigned char)0x03,
   RF_mode 		   			= (unsigned char)0x04,
   Receiver_mode			= (unsigned char)0x05,
   receive_single			= (unsigned char)0x06,
   CAD_mode						= (unsigned char)0x07,
}RFMode_SET;

typedef enum
{
   FSK_mode           = (unsigned char)0x00, 
   LORA_mode          = (unsigned char)0x80, 
}  Debugging_fsk_ook;

typedef enum{false=0,true=1}BOOL_t;

typedef enum{enOpen,enClose}cmdEntype_t;

void gByteWritefunc(unsigned char buffer);
unsigned char gByteReadfunc(void);
void SX127xWriteBuffer( unsigned char addr, unsigned char buffer);
unsigned char SX127xReadBuffer(unsigned char addr);
void SX127xReset(void);//1278¸´Î»
void SX127xLoRaSetOpMode( RFMode_SET opMode);
void SX127xLoRaFsk( Debugging_fsk_ook opMode);
void SX127xLoRaSetRFFrequency(void);
void SX127xLoRaSetRFPower(unsigned char power);
void SX127xLoRaSetSpreadingFactor(unsigned char factor);
void SX127xLoRaSetNbTrigPeaks(unsigned char value);
void SX127xLoRaSetErrorCoding(unsigned char value);
void SX127xLoRaSetPacketCrcOn(BOOL_t enable);
void SX127xLoRaSetSignalBandwidth(unsigned char bw);
void SX127xLoRaSetImplicitHeaderOn(BOOL_t enable);
void SX127xLoRaSetSymbTimeout(unsigned int value);
void SX127xLoRaSetPayloadLength(unsigned char value);
void SX127xLoRaSetMobileNode(BOOL_t enable);
void SX127xLORA_INT(void);
void FUN_RF_SENDPACKET(unsigned char *RF_TRAN_P,unsigned char LEN);
void RF_RECEIVE(void);
//void gSwitchEnStatus(cmdEntype_t cmdcmd); //¿ØÖÆÆ¬Ñ¡
void SX127x_Interupt(void);
void fqcRecvData(unsigned char *lpbuf);

extern unsigned char   Frequency[];
extern unsigned char 	 recv[];
extern unsigned char   SX127x_RLEN;

#endif

