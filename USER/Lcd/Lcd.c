#include "main.h"

uint8_t runstop_comd_back;
extern Struct_InvParameter Para;
extern uint16_t Lcd_rx_data[];
void LcdDataGet(void);
uint16_t Lcd_rx_data[200];
uint8_t quick_runstop;
extern uint8_t usart5_txdata[200],usart5_rxdata[200];
extern uint8_t Wirte_Over_Flag;


/**
 * @brief 分时处理数据是否接收完毕
 * 
 * @param argument 
 */
void LcdDataGet(void)
{
  uint16_t *paraP = &Para.OnOff;
	int i;
	for (i = 1; i < 100; i++)
		*paraP++ = Lcd_rx_data[i];
	Para.PcsFaultClear = Lcd_rx_data[156];        // PCS故障清除
//	if((Sys_Fault.pcs_fault == SYSTEM_FAULT) ||
//		(Sys_Fault.mttp_fault == SYSTEM_FAULT) ||
//	  (Sys_Fault.bms1_fault == SYSTEM_FAULT) ||
//    (Sys_Fault.bms2_fault == SYSTEM_FAULT))	
//	{
//	  Para.OnOff = 0;                             // 故障关机
//	} 
	
	if(runstop_comd_back != Para.OnOff)
	{
			PcsData.PcsModbusSta  = PCS_DATD_READ_STEP6; 
      quick_runstop = 0x05f;
		  if(Para.OnOff == 1)
			{
				PcsData.Value[10] = PCS_POWER_ON;     // 开机PCS 
			}
			else
			{
				PcsData.Value[10] = PCS_POWER_OFF;          // 关PCS
			}
	}	
	runstop_comd_back = Para.OnOff;
} 



/*******************************************************************************
* Function Name  : void  ScibDeal(void)
* Description    : 液晶触摸屏通讯
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void ScibDeal(void)
{
    uint16_t i,k,n,m,bytecount,Crccheckadd;
    uint16_t rx_lcd_data,rx_lcd_addr;
    uint16_t TxbNumber;
    n =  usart5_rxdata[2] & 0x00ff;   // 获取地址
    n =  n << 8;
    m = n | usart5_rxdata[3];
    bytecount = usart5_rxdata[5];     // 获取读取数量
    if(usart5_rxdata[1] == 0x03)
    {
        if(((m + bytecount-1) <= 792 && m > 0 && m <= 1000))//0-216号可读
        {
            //取数据
            for(i = 0;i < bytecount;i++)
            {
                k = i * 2;
                n = *(&Para.PARA_ADDR + m + i);          //   
                usart5_txdata[3+k] = (n >> 8) & 0x00ff;
                usart5_txdata[4+k] = n & 0x00ff;
            }
            usart5_txdata[0] = usart5_rxdata[0];
            usart5_txdata[1] = usart5_rxdata[1];
            usart5_txdata[2] = 2 * bytecount;
            Crccheckadd = Crc8(usart5_txdata,2 * bytecount + 3);
            usart5_txdata[3 + 2 * bytecount] = Crccheckadd >> 8;
            usart5_txdata[4 + 2 * bytecount] = Crccheckadd & 0x00ff;
            TxbNumber = 5 + 2 * bytecount;
        }
    }
    else if(usart5_rxdata[1] == 0x06)
    {
        if(m < 100 && m > 0)//1-41功能号可写或2000H-2040H地址可写
        {
            usart5_txdata[0] = usart5_rxdata[0];
            usart5_txdata[1] = usart5_rxdata[1];
            usart5_txdata[2] =  usart5_rxdata[2];
            usart5_txdata[3] = usart5_rxdata[3];

            n =  usart5_rxdata[4] & 0x00ff;
            n =  n << 8;
            rx_lcd_data = n | usart5_rxdata[5];
            n =  usart5_rxdata[2] & 0x00ff;
            n =  n << 8;
            rx_lcd_addr = n | usart5_rxdata[3];
					  Lcd_rx_data[rx_lcd_addr] = rx_lcd_data;
            //SetParameter(m,n);
					  

            usart5_txdata[4] = (n >> 8) & 0x00ff;
            usart5_txdata[5] = n & 0x00ff;

            Crccheckadd = Crc8(usart5_txdata,6);
            usart5_txdata[6] = Crccheckadd >> 8;
            usart5_txdata[7] = Crccheckadd & 0x00ff;

            TxbNumber = 8;
						Wirte_Over_Flag = 0;                      // 写24C2456标志
        }
    }		
		if(TxbNumber == 0)
		{
		    return;
		}
		USARTB_TX_EN;
    HAL_UART_Transmit_IT(&huart5,usart5_txdata,TxbNumber);
}

