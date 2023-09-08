#ifndef  SYSCONTROL_H
#define  SYSCONTROL_H

#include "main.h"
#include "cmsis_os.h"
#include "string.h"

#define PCS_DATA_NOT_RECEIVED_OR_CRC_FAILED  4  // 未收到数据或者效验不通过
#define PCS_DATA_RECEIVED_AND_CRC_OK  3         // 收到数据恢复，且效验通过
#define PCS_DATA_RX_PREPARE           2         // 数据接收准备
#define PCS_DATA_START_SENDING        1         // 开始发送
#define PCS_DATA_RESTART_SEND         0         // 重新启动发送

#define MPPT_POWER_ON             0         //  MPPT开机
#define MPPT_POWER_OFF            1         //  MPPT关机
#define MPPT_No_DORMANCY          2         //  MPPT无休眠
#define MPPT_DORMANCY             3         //  MPPT休眠
#define MPPT_SET_POWER            4         //  设置MPPT功率

#define PCS_POWER_ON              1         // PCS开机
#define PCS_POWER_OFF             0         // PCS关机
typedef struct
{
		uint16_t PcsModbusSta;
		int16_t PcsModbusWait;
		uint16_t Err;
	  uint16_t Err_Count;
		uint16_t PcsWriteF;
		uint16_t PcsWriteO;
		uint16_t PcsReadF;    // 读取标志
		uint16_t Value[12];	
}struct_PCSDATA;

extern struct_PCSDATA PcsData;

typedef struct
{
		uint16_t SysSta;
		int16_t  TimeOut;
		uint16_t Reissued;
		uint16_t Wait;
		uint32_t Curr;
		uint16_t Pwr;  //写入MPPT的功率
		uint16_t PwrRt;//实时MPPT功率
	  uint16_t MpptRx_End_Flag; // 接收完成标志
}struct_MPPT;

typedef struct
{
		uint16_t HeartBeat;
		uint16_t Cnt;
		uint16_t Flag;
}struct_BMS;

typedef struct {
		uint16_t SysFlag;    //系统状态标志
		uint16_t WaitFlag;   //
		uint16_t WaitFlag1;  //
		uint16_t WaitCnt;
		uint16_t WaitCnt1;
		uint16_t WaitCnt2;
}Struct_SysControl;

extern Struct_SysControl SysControl;

#define MTTP_READ_STEP0     0 
#define MTTP_READ_STEP1     1
#define MTTP_READ_STEP2     2
#define MTTP_READ_STEP3     3
#define MTTP_READ_STEP4     4
#define MTTP_READ_STEP5     5
#define MTTP_READ_STEP6     6
#define MTTP_READ_STEP7     7
#define MTTP_READ_STEP8     8
#define MTTP_READ_STEP9     9
#define MTTP_READ_STEP10    10
#define MTTP_READ_STEP11    11
#define MTTP_READ_STEP12    12
#define MTTP_READ_STEP13    13
#define MTTP_READ_STEP14    14
#define MTTP_READ_STEP15    15
#define MTTP_READ_STEP16    16
#define MTTP_READ_STEP17    17
#define MTTP_READ_STEP18    18

#define PCS_DATD_READ_STEP0     0
#define PCS_DATD_READ_STEP1     1
#define PCS_DATD_READ_STEP2     2
#define PCS_DATD_READ_STEP3     3
#define PCS_DATD_READ_STEP4     4
#define PCS_DATD_READ_STEP5     5
#define PCS_DATD_READ_STEP6     6
#define AMMETER_DATD_READ_STEP7 7
#define AMMETER_DATD_READ_STEP8 8
#define AMMETER_DATD_READ_STEP9 9
#define AMMETER_DATD_READ_STEP10 10

// PCS状态
#define PCS_SHORT_REST                    4   // 4—短静置
#define PCS_STOP                          5   // 停机
#define PCS_FAULT                         6   // 故障
#define PCS_SOFT_START                   10   // 软起
#define PCS_CONSTANT_VOLTAGE_RUN         11   // 恒压运行
#define PCS_CONSTANT_CURRENT_RUN         12   // 恒流运行
#define PCS_IDLE_RUN                     13   // 待机
#define PCS_OFF_GRID_INVERTER_RUN        14   // 离网逆变运行
#define PCS_AC_CONSTANT_POWER_OPERATION  16   // AC 恒功率运行

#define PCS_USART_ERR1                   1
#define PCS_USART_ERR2                   2
#define PCS_USART_ERR3                   3
#define PCS_USART_ERR4                   4
#define PCS_USART_ERR5                   5

// PCS运行模式
#define PCS_DC_CONS_CUR_RUN              34    // PCS DC 恒功率
#define PCS_AC_CONS_POWER_RUN            64    // PCS Ac 恒功率



void SysStateManage(void);
void PcsDataGet_Tx(void);
void PcsDataGet_Rx(void);
void MpptDataGet(void);
void PowerOff(void);
void SetPcs(uint8_t Oder);
void StopState(void);
void PcsPupDeal(void);
void RunState(void);
void FailureState(void);
void SetMppt(uint8_t Oder);
void BmsData_Tx(void);

uint16_t Crc8(uint8_t *puchMsg, uint16_t usDataLen);
uint16_t ByteCom(uint8_t DataA,uint8_t DataB);
uint16_t Crc16(uint16_t *puchMsg, uint16_t usDataLen);
 
//void BmsDataGet(void);

#endif
