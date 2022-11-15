#ifndef __WIFI_H
#define __WIFI_H
#define u8 unsigned char 

extern void Uart2Init(void);
extern void wifi_init(void);
extern void wifi_process_msg(void);
extern unsigned char send_sms(void);
extern unsigned char Serial_available(void);
extern unsigned char Serial_read(char *msg);
extern void AT_INIT(void);
extern unsigned char  Wifi_handle(unsigned char *Wifi);
extern unsigned char  GEC_handle(char *Wifi);
extern unsigned char  LookFor_Str_gec(void);
extern char GEC_FLY[18];
extern void WIFI_SendData(u8 *buf);
extern char USART1_RX_BUF[64];  
extern char USART2_RX_BUF[64]; 
extern unsigned short int  USART1_RX_STA ;
extern unsigned short int  USART2_RX_STA ;
#endif
