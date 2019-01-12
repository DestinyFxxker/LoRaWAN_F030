
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_spi.h"
#include "SX127x.h"
#include "string.h"
#include "function.h"

extern SPI_HandleTypeDef hspi1;


//unsigned char   Frequency[3]={0xE4,0xC0,0x26}; //915M
//unsigned char   Frequency[3]={0xD4,0x80,0x23}; //850M
unsigned char   Frequency[3]={0xD9,0x06,0x78}; //868.1M
//unsigned char   Frequency[3]={0xEC,0x40,0x27}; //945M
//unsigned char   Frequency[3]={0x7A,0x00,0x15};//510MHz
//unsigned char   Frequency[3]={0x6C,0x40,0x12};//433MHz
unsigned char   SpreadingFactor=11;    //7-12,��Ƶ����ѡСһЩ������ʱ������һЩ��
unsigned char   CodingRate=1;        //1-4
unsigned char   Bw_Frequency=7;      //6-9
unsigned char   powerValue = 14;       //�������ã����Ե�ʱ�����������Сһ��
unsigned char   power_data[16]={0X80,0X81,0X82,0X83,0X84,0X85,0X86,0X87,0X88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f};
unsigned char   RF_EX0_STATUS;
unsigned char   CRC_Value;
unsigned char   SX127x_RLEN;

void gByteWritefunc(unsigned char buffer)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//set NSS(PA_4) to low
    HAL_SPI_Transmit(&hspi1, &buffer, 1, 10);
}

unsigned char gByteReadfunc(void)
{
    unsigned char temp=0;
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//set NSS(PA_4) to low
    HAL_SPI_Receive(&hspi1, &temp, 1,10);
	
	  return temp;
}

void SX127xWriteBuffer( unsigned char addr, unsigned char buffer)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//set NSS(PA_4) to low
	gByteWritefunc(addr|0x80);
	gByteWritefunc(buffer);
//	while( SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);   //�ж�SPI�Ƿ�æ
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//set NSS(PA_4) to high
}

unsigned char SX127xReadBuffer(unsigned char addr)
{
	unsigned char Value;
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//set NSS(PA_4) to low
	gByteWritefunc(addr & 0x7f);
	Value = gByteReadfunc();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//set NSS(PA_4) to high
	
	return Value; 
}

void SX127xReset(void)//1278��λ
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);//set LoRa RESET(PA_1) to low
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(200);
}

void SX127xLoRaSetOpMode( RFMode_SET opMode )
{
	unsigned char opModePrev;
	opModePrev=SX127xReadBuffer(REG_LR_OPMODE);  //��0x01ģʽ�Ĵ���
	opModePrev &=0xf8;  //�������λ
	opModePrev |= (unsigned char)opMode; //�����β�
	SX127xWriteBuffer( REG_LR_OPMODE, opModePrev); //����д��ȥ	
}

void SX127xLoRaFsk( Debugging_fsk_ook opMode )
{
	unsigned char opModePrev;
	opModePrev=SX127xReadBuffer(REG_LR_OPMODE); //��0x01ģʽ�Ĵ���
	opModePrev &=0x7F; //�������λ
	opModePrev |= (unsigned char)opMode;  //�����β�
	SX127xWriteBuffer( REG_LR_OPMODE, opModePrev); //����д��ȥ		
}

void SX127xLoRaSetRFFrequency(void)
{
	SX127xWriteBuffer( REG_LR_FRFMSB, Frequency[0]);  //д0x06�Ĵ���
	SX127xWriteBuffer( REG_LR_FRFMID, Frequency[1]);  //д0x07�Ĵ���
	SX127xWriteBuffer( REG_LR_FRFLSB, Frequency[2]);  //д0x08�Ĵ���
}

void SX127xLoRaSetRFPower(unsigned char power )
{
	//Set Pmax to +20dBm for PA_HP, Must turn off PA_LF or PA_HF, and set RegOcp
	//SX1276WriteBuffer( REG_LR_PACONFIG,  power_data[power] ); //��û����һ�䣬Ƶ���Ǽ����ⲻ���ź�,���ǿ��Խ��������շ���
	//SX1276WriteBuffer( REG_LR_OCP, 0x3f);  //add by skay,20160810, д���������Ĵ�����
	SX127xWriteBuffer( REG_LR_PADAC, 0x87);  //high power
	SX127xWriteBuffer( REG_LR_PACONFIG,  power_data[power] ); //��û����һ�䣬Ƶ���Ǽ����ⲻ���ź�,���ǿ��Խ��������շ���
}

void SX127xLoRaSetSpreadingFactor(unsigned char factor )
{
	unsigned char RECVER_DAT;
	SX127xLoRaSetNbTrigPeaks( 3 ); //0x03-->SF7 to SF12
	RECVER_DAT=SX127xReadBuffer( REG_LR_MODEMCONFIG2); //��0x1E�Ĵ���  
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
	SX127xWriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT );	 
}

void SX127xLoRaSetNbTrigPeaks(unsigned char value )
{
	unsigned char RECVER_DAT;
	RECVER_DAT = SX127xReadBuffer( 0x31);  //Read RegDetectOptimize,
	RECVER_DAT = ( RECVER_DAT & 0xF8 ) | value; //process;
	SX127xWriteBuffer( 0x31, RECVER_DAT );  //write back;
}

void SX127xLoRaSetErrorCoding(unsigned char value )
{	
	unsigned char RECVER_DAT;
	RECVER_DAT=SX127xReadBuffer( REG_LR_MODEMCONFIG1); //��0x1D�Ĵ���
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
	SX127xWriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT);
}

void SX127xLoRaSetPacketCrcOn(BOOL_t enable )
{	
	unsigned char RECVER_DAT;
	RECVER_DAT= SX127xReadBuffer( REG_LR_MODEMCONFIG2);  //��0x1E�Ĵ��� 
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
	SX127xWriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT );
}

void SX127xLoRaSetSignalBandwidth(unsigned char bw )
{
	unsigned char RECVER_DAT;
	RECVER_DAT=SX127xReadBuffer( REG_LR_MODEMCONFIG1);  //��0x1D�Ĵ���
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
	SX127xWriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT );
}

void SX127xLoRaSetImplicitHeaderOn(BOOL_t enable )
{
	unsigned char RECVER_DAT;
	RECVER_DAT=SX127xReadBuffer( REG_LR_MODEMCONFIG1 );  //��0x1D�Ĵ���
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
	SX127xWriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT );
}

void SX127xLoRaSetSymbTimeout(unsigned int value )
{
	unsigned char RECVER_DAT[2];
	RECVER_DAT[0]=SX127xReadBuffer( REG_LR_MODEMCONFIG2 );    //��0x1E�Ĵ���
	RECVER_DAT[1]=SX127xReadBuffer( REG_LR_SYMBTIMEOUTLSB );  //��0x1F�Ĵ���
	RECVER_DAT[0] = ( RECVER_DAT[0] & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
	RECVER_DAT[1] = value & 0xFF;
	SX127xWriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT[0]);
	SX127xWriteBuffer( REG_LR_SYMBTIMEOUTLSB, RECVER_DAT[1]);
}

void SX127xLoRaSetPayloadLength(unsigned char value )
{
	SX127xWriteBuffer( REG_LR_PAYLOADLENGTH, value );  //д0x22�Ĵ���
} 

void SX127xLoRaSetMobileNode(BOOL_t enable )
{	 
	unsigned char RECVER_DAT;
	RECVER_DAT=SX127xReadBuffer( REG_LR_MODEMCONFIG3 );  //��0x26�Ĵ���
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG3_MOBILE_NODE_MASK ) | ( enable << 3 );
	SX127xWriteBuffer( REG_LR_MODEMCONFIG3, RECVER_DAT );
}

void SX127xLORA_INT(void)
{
	SX127xWriteBuffer(0x0A,0x33);
	//SX127xReadBuffer(0x0A);
	DebugPrintf("%x\r\n", SX127xReadBuffer(0x0A));
	
	SX127xLoRaSetOpMode(Sleep_mode);  //����˯��ģʽ
	SX127xLoRaFsk(LORA_mode);	     // ������Ƶģʽ
	SX127xLoRaSetOpMode(Stdby_mode);  // ����Ϊ��ͨģʽ
	SX127xWriteBuffer( REG_LR_DIOMAPPING1,GPIO_VARE_1); //д0x40�Ĵ���,DIO����ӳ������
	SX127xWriteBuffer( REG_LR_DIOMAPPING2,GPIO_VARE_2); //д0x41�Ĵ���
	SX127xLoRaSetRFFrequency();  //Ƶ������
	SX127xLoRaSetRFPower(powerValue);  //��������
	SX127xLoRaSetSpreadingFactor(SpreadingFactor);	 // ��Ƶ��������
	SX127xLoRaSetErrorCoding(CodingRate);		 //��Ч���ݱ�
	SX127xLoRaSetPacketCrcOn(true);			 //CRC У���
	SX127xLoRaSetSignalBandwidth( Bw_Frequency );	 //������Ƶ����, 125khz
	SX127xLoRaSetImplicitHeaderOn(false);		 //ͬ��ͷ������ģʽ
	SX127xLoRaSetPayloadLength( 0xff);
	SX127xLoRaSetSymbTimeout( 0x3FF );
	SX127xLoRaSetMobileNode(true); 			 // �����ݵ��Ż�
	DebugPrintf("LoRa Chip SX127x Initial done!\r\n");
	RF_RECEIVE();
}

void FUN_RF_SENDPACKET(unsigned char *RF_TRAN_P,unsigned char LEN)
{	
	unsigned char ASM_i;
	
	SX127xLoRaSetOpMode( Stdby_mode );
	SX127xWriteBuffer( REG_LR_HOPPERIOD, 0 );	//����Ƶ������
	SX127xWriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_TXD_Value);	//�򿪷����ж�
	SX127xWriteBuffer( REG_LR_PAYLOADLENGTH, LEN);	 //������ݰ�
	SX127xWriteBuffer( REG_LR_FIFOTXBASEADDR, 0); //дTx FIFO��ַ
	SX127xWriteBuffer( REG_LR_FIFOADDRPTR, 0 ); //SPI interface address pointer in FIFO data buffer
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //ѡ��SX127x
	gByteWritefunc( 0x80 );
	for( ASM_i = 0; ASM_i < LEN; ASM_i++ )
	{
		gByteWritefunc( *RF_TRAN_P );RF_TRAN_P++;
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //ȡ��ѡ��SX127x
	SX127xWriteBuffer(REG_LR_DIOMAPPING1,0x40);  //�����ж�ӳ�䵽DIO0����
	SX127xWriteBuffer(REG_LR_DIOMAPPING2,0x00);
	SX127xLoRaSetOpMode( Transmitter_mode );     //����Ϊ����ģʽ
}

void RF_RECEIVE(void)
{
	SX127xLoRaSetOpMode(Stdby_mode );
	SX127xWriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //0x11,�򿪽����ж�
	SX127xWriteBuffer(REG_LR_HOPPERIOD,	PACKET_MIAX_Value );//0x24�Ĵ���
	SX127xWriteBuffer( REG_LR_DIOMAPPING1, 0X00 ); //DIO����ӳ�����ã���Ĭ��
	SX127xWriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
	SX127xLoRaSetOpMode( Receiver_mode );  //����Ϊ��������ģʽ
}

//**************�������ж����洦��Ĵ���*******************************
unsigned char recv[10];
unsigned char RF_REC_RLEN_i;

void SX127x_Interupt(void)
{
	RF_EX0_STATUS=SX127xReadBuffer( REG_LR_IRQFLAGS ); 
	if((RF_EX0_STATUS&0x40)==0x40)  //�������
	{
		CRC_Value=SX127xReadBuffer( REG_LR_MODEMCONFIG2 );
		if((CRC_Value&0x04) == 0x04) //�Ƿ��CRCУ��
		{
			SX127xWriteBuffer (REG_LR_FIFOADDRPTR,0x00);
			SX127x_RLEN = SX127xReadBuffer(REG_LR_NBRXBYTES); //��ȡ���һ�������ֽ���
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //ѡ��SX127x
			gByteWritefunc( 0x00 );
			if(SX127x_RLEN > 10)  //���ղ�����10���ֽ�
				SX127x_RLEN = 10;
			for(RF_REC_RLEN_i=0;RF_REC_RLEN_i<SX127x_RLEN;RF_REC_RLEN_i++)
			{
				recv[RF_REC_RLEN_i]=gByteReadfunc();
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //ȡ��ѡ��SX127x
		}
		fqcRecvData(recv);  //�����½��յ�������

		SX127xLoRaSetOpMode( Stdby_mode );
		SX127xWriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);  //�򿪷����ж�
		SX127xWriteBuffer(REG_LR_HOPPERIOD,    PACKET_MIAX_Value);
		SX127xWriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
		SX127xWriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
		SX127xLoRaSetOpMode( Receiver_mode );

	}
	else if((RF_EX0_STATUS&0x08)==0x08)  //�������
	{
		SX127xLoRaSetOpMode( Stdby_mode );
		SX127xWriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	//�򿪷����ж�
		SX127xWriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
		SX127xWriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
		SX127xWriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
		SX127xLoRaSetOpMode( Receiver_mode );
	}
	else if((RF_EX0_STATUS&0x04)==0x04)  //cad���
	{  
		if((RF_EX0_STATUS&0x01)==0x01)
		{	 
		//��ʾCAD ��⵽��Ƶ�ź� ģ������˽���״̬����������
			SX127xLoRaSetOpMode( Stdby_mode );
			SX127xWriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	//�򿪷����ж�
			SX127xWriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
			SX127xWriteBuffer( REG_LR_DIOMAPPING1, 0X02 );
			SX127xWriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
			SX127xLoRaSetOpMode( Receiver_mode );
		}
		else
		{
		// û��⵽
			SX127xLoRaSetOpMode( Stdby_mode );
			SX127xWriteBuffer(REG_LR_IRQFLAGSMASK,	IRQN_SEELP_Value);	//�򿪷����ж�
			SX127xWriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
			SX127xWriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
			SX127xLoRaSetOpMode( Sleep_mode );
		}
	}
	else
	{
		SX127xLoRaSetOpMode( Stdby_mode );
		SX127xWriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	//�򿪷����ж�
		SX127xWriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
		SX127xWriteBuffer( REG_LR_DIOMAPPING1, 0X02 );
		SX127xWriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
		SX127xLoRaSetOpMode( Receiver_mode );
	}
	SX127xWriteBuffer( REG_LR_IRQFLAGS, 0xff  );
}

void fqcRecvData(unsigned char *lpbuf)
{
//	unsigned char i;
//	unsigned char Irq_flag=0;
	switch(lpbuf[1]&0x0f)
	{
		case 0x01: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);break;
		case 0x02: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);break;
	}
}



