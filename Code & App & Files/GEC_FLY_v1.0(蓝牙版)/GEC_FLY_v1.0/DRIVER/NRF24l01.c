/**********************************************
*版 本 号：         v1.0
*创 建 者：         粤嵌股份
*功能描述：         四轴NRF24101函数
**********************************************/

#include "nrf24l01.h"

u8 mode = 0;
uint8_t  TX_ADDRESS[TX_ADDR_LEN]= {0xE1,0xE1,0xE1,0xE1,0xE1};	
uint8_t  RX_ADDRESS[RX_ADDR_LEN]= {0xE1,0xE1,0xE1,0xE1,0xE1};	

uint8_t NRF_Write_Reg(uint8_t reg, uint8_t data)                //NRF写写寄存器
{
	uint8_t status;
	SPI_CSN_L();					  
	status = SPI_RW(reg);  			
	SPI_RW(data);		  		
	SPI_CSN_H();					
  	return 	status;
}

uint8_t NRF_Read_Reg(uint8_t reg)															 //NRF读寄存器
{
	uint8_t date;
	SPI_CSN_L();					
	SPI_RW(reg);		
	date = SPI_RW(0xff);	 
	SPI_CSN_H();					
    	return date;
}

uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len) //NRF写给缓存
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

uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len) //NRF从缓存读取
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

void NRF_SendPacket(uint8_t * tx_buf, uint8_t len)                     //NRF发送包
{	
	SPI_CE_L();		
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADDR_LEN);  // 装载接收端地址
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			                       // 装载数据	
	SPI_CE_H();		 
}
void NRF_BackPacket(uint8_t * tx_buf, uint8_t len)                     //NRF接收包
{	
	SPI_CE_L();		 
	NRF_Write_Buf(0xa8, tx_buf, len); 			                            // 装载数据
	SPI_CE_H();		
}
u8 Nrf24l01_Check(void)                                               //NRF校验
{ 
	u8 buf[5]; 
	u8 i; 
	/*写入5个字节的地址. */ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5); 
	/*读出写入的地址 */ 
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

void Nrf24l01_Init(void)                                            //NRF初始化
{
	SPI_CE_L();
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADDR_LEN);		//写RX节点地址 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADDR_LEN); 		  //写TX节点地址  
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//使能通道0的自动应答 
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//使能通道0的接收地址 
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);											//设置自动重发间隔时间:500us;最大自动重发次数:10次 
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,40);														//设置RF通道为CHANAL
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 											//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
}


void NRF24l01_SetRxMode(void)                                       //NRF_RX模式设置
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

void NRF24l01_SetTxMode(void)                                       //NRF_TX模式设置
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

u8 NRF24l01_Recv(u8 *buf)	                                          //NRF接收
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

void NRF_SendData(u8 *buf)                                        //NRF发送数据
{
	if(mode==1)
		NRF_SendPacket(buf,32);
	else if(mode==2)
		NRF_BackPacket(buf,32);
}

////////////////////////////////////////////////////////////////////////////////
