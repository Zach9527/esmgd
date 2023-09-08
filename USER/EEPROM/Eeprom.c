#include "main.h"



uint16_t Read_24XX_Data(uint16_t);
void Write_24XX_Data(uint16_t,uint16_t);

Struct_Eeprom24xxData Eeprom_data;

uint16_t cyclic_date = 0;
uint16_t cyclic_start = 0;
uint16_t cyclic_end = EEPROM_CHIP;
uint16_t *Read_Data_Addr;		

uint8_t Wirte_Over_Flag;
uint16_t Write_Data;
uint16_t i;	
uint16_t *Write_Data_Addr;	
uint16_t Write_24c_Addr;
	
uint16_t *Read_Data;	
uint16_t Read_24c_Addr;
uint16_t Read_24c_Data;	
extern Struct_InvParameter Para;
extern I2C_HandleTypeDef hi2c1;

uint16_t test_data;

extern void User_Variable_Init(void);
/*******************************************************************************
* Function Name  : void  Eeprom_Control(void)
* Description    : 24C存储器控制程庿
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void eeprom_task(void *argument)
{
	 for(;;)
   {
		
 		  HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);         //喂硬件狗
			osDelay(100);	 
			if(Wirte_Over_Flag != 0x055)
			{
//					Para.Reserve137 = Eeprom_data.Reserve137 + 1;               // test date
//					Para.Reserve122 = Eeprom_data.Reserve122 + 1;
//					Para.Anti_Reflux_Enable = Eeprom_data.Anti_Reflux_Enable + 1;
//					Para.Reserve72 = Eeprom_data.Reserve72 + 1;
 				
					Write_Data_Addr = &Para.PARA_ADDR;
					for(i = 0; i < EEPROM_SUM; i++)
					{
						 Write_Data = *(Write_Data_Addr++);
						 Write_24c_Addr = i;
						 Write_24XX_Data(Write_24c_Addr,Write_Data);
					}
			 		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);         //喂硬件狗
					Wirte_Over_Flag = 0x055;
					
//					Read_Data_Addr = &Eeprom_data.PARA_ADDR;                         // 读首地址

			}
			// =====================================================
		#if 0
			for(cyclic_date = cyclic_start; cyclic_date < cyclic_end; cyclic_date++)
			{
				Read_24c_Addr = cyclic_date;
				
				*Read_Data_Addr++ = Read_24XX_Data(Read_24c_Addr);			
		 
			}
			cyclic_start = cyclic_end;                            // 每次诿2个字芿
			cyclic_end = cyclic_end + EEPROM_CHIP;
			if(cyclic_end > EEPROM_SUM)
			{
				cyclic_end = cyclic_end - EEPROM_SUM;
				cyclic_start = 0;
				Read_Data_Addr = &Eeprom_data.PARA_ADDR;           // 读首地址

			}
			
	 
			test_data = Eeprom_data.Reserve137;
			test_data = Eeprom_data.Reserve122;
			test_data = Eeprom_data.Anti_Reflux_Enable;
			test_data = Eeprom_data.Reserve72;
			test_data = Eeprom_data.BmsLevel3Warn2;
	 		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);         //喂硬件狗
		 #endif
		}		
	 

}



 
/*******************************************************************************
* Function Name  : void  Read_24XX_Data(void)
* Description    : 读取24C存储器数捿
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

uint16_t Read_24XX_Data(uint16_t read_24c_addr)
{
uint8_t Eeprom_Read_Data[1];
uint16_t Read_Addr;
uint16_t Read_data_Rturn;
 	
	  Read_Addr = read_24c_addr * 2;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);   
	  osDelay(1);  
 
		HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, Read_Addr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)(Eeprom_Read_Data),3,1000);
    Read_data_Rturn = (uint16_t)(Eeprom_Read_Data[0] << 8);
	  osDelay(1);  
	  Read_Addr = read_24c_addr * 2 +1;;
		HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, Read_Addr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)(Eeprom_Read_Data),3,1000);
	  Read_data_Rturn = (uint16_t)(Read_data_Rturn + Eeprom_Read_Data[0]);
	  return Read_data_Rturn;
} 



/*******************************************************************************
* Function Name  : void  Write_24XX_Data(void)
* Description    : 写人24C存储器数捿
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Write_24XX_Data(uint16_t write_24c_addr,uint16_t write_Data)
{

uint8_t Eeprom_Write_Data[1];
uint16_t Write_Addr;
	
	  Eeprom_Write_Data[0] = write_Data >> 8;
	  Write_Addr = write_24c_addr * 2;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);  
	  osDelay(5);  
//  4 存储器内部地址 usart2_txbuf-存的数据地址＿-数据个数＿000 超时时间
		HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, Write_Addr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)(Eeprom_Write_Data),3,1000); 
	  osDelay(5);  
	  Eeprom_Write_Data[0] = write_Data;
	  Write_Addr = write_24c_addr * 2 + 1;
		HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, Write_Addr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)(Eeprom_Write_Data),3,1000); 

} 


void User_Variable_Init()
{
	
	Para.Mppt_electric_quantity1_s = 0;
	Para.Mppt_electric_quantity1_m = 0;
	Wirte_Over_Flag = 0;	
}




