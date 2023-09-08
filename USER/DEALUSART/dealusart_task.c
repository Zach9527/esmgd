#include "main.h"


uint8_t usart5_txdata[200];
uint8_t usart5_rxdata[200];
uint8_t data1,data3,data5;

extern struct_PCSDATA PcsData;
int LenLcd;
#define FIFO_MAX_SIZE 200
#define STA_ERR 0
#define STA_OK 1
#define LCD_MAXLEN 200
typedef struct
{
	uint8_t dat[FIFO_MAX_SIZE];
	int dep;
	int enp;
}DAT_FIFO_TYP;

DAT_FIFO_TYP RS422_RECV_FIFO;

typedef uint16_t ERR_STA;



void InitFifo(DAT_FIFO_TYP *fifo)
{
	memset((void *)(fifo->dat), 0, FIFO_MAX_SIZE * sizeof(fifo->dat[0]));
	
	fifo->dep = 0;
	fifo->enp = 0;
}

uint32_t GetFifoNum(DAT_FIFO_TYP *fifo)
{
	uint32_t num = 0;

	num = (fifo->enp - fifo->dep + FIFO_MAX_SIZE) % FIFO_MAX_SIZE;

	return num;
}

 ERR_STA EnqueueFifo(DAT_FIFO_TYP *fifo, uint8_t *dat, uint32_t size)
{
	uint32_t tmp;
	uint32_t i;
	
	for(i = 0; i < size; i++)
	{	
		if(((fifo->enp + 1)%FIFO_MAX_SIZE) == fifo->dep)
		{
				return STA_ERR;
		}
		if(size > FIFO_MAX_SIZE)
		{
				return STA_ERR;
		}
		
		fifo->dat[fifo->enp] = dat[i];
		
		tmp = (fifo->enp + 1)%FIFO_MAX_SIZE;
		fifo->enp = tmp;
	}
	
	return STA_OK;
}

 ERR_STA DequeueFifo(DAT_FIFO_TYP *fifo, uint8_t *dat)
{
	uint32_t tmp;
	
	if(fifo->dep == fifo->enp)
	{
			return STA_ERR;
	}
	
	*dat = fifo->dat[fifo->dep];
	
	tmp = (fifo->dep + 1)%FIFO_MAX_SIZE;
	fifo->dep = tmp;
	
	return STA_OK;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 {
    if(huart->Instance==USART2)
    {
       osSignalSet(DEALUASRTHandle,0x01);
    }
    else if(huart->Instance==USART3)
    {
       osSignalSet(DEALUASRTHandle,0x01);
    }
     else if(huart->Instance==UART5)
    {
        osSignalSet(DEALUASRTHandle,0x01);
    }
} 

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==UART5)
    {
        EnqueueFifo(&RS422_RECV_FIFO,&data5,USART_BUFFERSIZE);
        HAL_UART_Receive_IT(&huart5, &data5, USART_BUFFERSIZE);
    }

}

void dealusart_task(void *argument)
{    
    uint16_t CrcR;
    uint8_t *buf5 = usart5_rxdata;
    USARTC_RX_EN;     
    USARTA_RX_EN;
		 if (HAL_UART_Receive_IT(&huart5, &data5, USART_BUFFERSIZE) != HAL_OK)
		{
			/* Transfer error in reception process */
		}
    while(1)
    {
            osDelay(10);
            while (GetFifoNum(&RS422_RECV_FIFO) > 0)
			{
                DequeueFifo(&RS422_RECV_FIFO,buf5);
                buf5++;
                LenLcd++;
				if(usart5_rxdata[0] == 0x01)
                {
                    if((usart5_rxdata[1] == 0x3) || (usart5_rxdata[1] == 0x6 ))
                    {
                    CrcR = Crc8(usart5_rxdata,LenLcd - 2);
					if(usart5_rxdata[LenLcd - 2] == (CrcR >> 8) && usart5_rxdata[LenLcd-1] == (CrcR & 0x00ff))
                    {
                        USARTB_TX_EN;
                        ScibDeal();
                        osSignalWait(0x1,osWaitForever);
                        memset(usart5_rxdata,0x0,200);
                        memset(usart5_txdata,0x0,200);
                        if(LenLcd == LCD_MAXLEN)
                        {
                            buf5 = usart5_rxdata;
                        }
                        else
                        {
                            buf5 = usart5_rxdata+LenLcd-LCD_MAXLEN;
                        }
                        LenLcd = 0;
						USARTB_RX_EN;
                    }
                    }
                    else if(LenLcd > 1)
                    {
                        buf5--;
                        LenLcd--;
                    }
                }
                else 
                {
                    buf5--;
                    LenLcd--;
                }
            }
    }
}

