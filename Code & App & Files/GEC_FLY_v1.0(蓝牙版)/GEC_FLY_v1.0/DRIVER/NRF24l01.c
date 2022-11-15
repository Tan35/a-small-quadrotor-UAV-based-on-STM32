/**********************************************
*�� �� �ţ�         v1.0
*�� �� �ߣ�         ��Ƕ�ɷ�
*����������         ����NRF24101����
**********************************************/

#include "nrf24l01.h"

u8 mode = 0;
uint8_t  TX_ADDRESS[TX_ADDR_LEN]= {0xE1,0xE1,0xE1,0xE1,0xE1};	
uint8_t  RX_ADDRESS[RX_ADDR_LEN]= {0xE1,0xE1,0xE1,0xE1,0xE1};	

uint8_t NRF_Write_Reg(uint8_t reg, uint8_t data)                //NRFдд�Ĵ���
{
	uint8_t status;
	SPI_CSN_L();					  
	status = SPI_RW(reg);  			
	SPI_RW(data);		  		
	SPI_CSN_H();					
  	return 	status;
}

uint8_t NRF_Read_Reg(uint8_t reg)															 //NRF���Ĵ���
{
	uint8_t date;
	SPI_CSN_L();					
	SPI_RW(reg);		
	date = SPI_RW(0xff);	 
	SPI_CSN_H();					
    	return date;
}

uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len) //NRFд������
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();				    
	status = SPI_RW(reg);	
	for(i=0; i<len; i++)
	{
		SPI_RW(pBuf[i]);		
	}
	SPI_CSN_H();						
    return status;	
}

uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len) //NRF�ӻ����ȡ
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();						
	status = SPI_RW(reg);	
	for(i=0; i<len; i++)
	{
		pBuf[i] = SPI_RW(0xff); 
	}
	SPI_CSN_H();						
    return status;
}

void NRF_SendPacket(uint8_t * tx_buf, uint8_t len)                     //NRF���Ͱ�
{	
	SPI_CE_L();		
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADDR_LEN);  // װ�ؽ��ն˵�ַ
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			                       // װ������	
	SPI_CE_H();		 
}
void NRF_BackPacket(uint8_t * tx_buf, uint8_t len)                     //NRF���հ�
{	
	SPI_CE_L();		 
	NRF_Write_Buf(0xa8, tx_buf, len); 			                            // װ������
	SPI_CE_H();		
}
u8 Nrf24l01_Check(void)                                               //NRFУ��
{ 
	u8 buf[5]; 
	u8 i; 
	/*д��5���ֽڵĵ�ַ. */ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5); 
	/*����д��ĵ�ַ */ 
	NRF_Read_Buf(TX_ADDR,buf,5); 
	
	for(i=0;i<5;i++) 
	{ 
		if(buf[i]!=TX_ADDRESS[i]) 
		    break; 
	} 
	if(i==5)
		return 0 ; 
	else
		return 1 ; 
}

void Nrf24l01_Init(void)                                            //NRF��ʼ��
{
	SPI_CE_L();
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADDR_LEN);		//дRX�ڵ��ַ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADDR_LEN); 		  //дTX�ڵ��ַ  
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//ʹ��ͨ��0���Զ�Ӧ�� 
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);											//�����Զ��ط����ʱ��:500us;����Զ��ط�����:10�� 
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,40);														//����RFͨ��ΪCHANAL
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 											//����TX�������,0db����,2Mbps,���������濪��
}


void NRF24l01_SetRxMode(void)                                       //NRF_RXģʽ����
{
	NRF_Write_Reg(FLUSH_TX,0xff);
	NRF_Write_Reg(FLUSH_RX,0xff);
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 
	
	SPI_RW(0x50);
	SPI_RW(0x73);
	NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
	NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
	SPI_CE_H();
	mode=2;
}

void NRF24l01_SetTxMode(void)                                       //NRF_TXģʽ����
{
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		
	NRF_Write_Reg(FLUSH_TX,0xff);
	NRF_Write_Reg(FLUSH_RX,0xff);
	
	SPI_RW(0x50);
	SPI_RW(0x73);
	NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
	NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
	SPI_CE_H();
	mode=1;
}

u8 NRF24l01_Recv(u8 *buf)	                                          //NRF����
{
	u8 sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
	u8 rx_len = 0;

	if(sta & (1<<RX_DR)) 
	{
		rx_len = NRF_Read_Reg(R_RX_PL_WID);	
		if(rx_len<33)
		{
			NRF_Read_Buf(RD_RX_PLOAD,buf,rx_len);
		}
		else 
		{
			NRF_Write_Reg(FLUSH_RX,0xff);
		}
	}
	if(sta & (1<<MAX_RT))
	{		
		if(sta & 0x01)	
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
	return rx_len;
}

void NRF_SendData(u8 *buf)                                        //NRF��������
{
	if(mode==1)
		NRF_SendPacket(buf,32);
	else if(mode==2)
		NRF_BackPacket(buf,32);
}

////////////////////////////////////////////////////////////////////////////////
