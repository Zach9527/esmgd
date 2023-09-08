#ifndef  EMS_H
#define  EMS_H

#include "main.h"
#include "cmsis_os.h"
#include "string.h"

//general function enable
#define FUNC_ENABLE 0x01 //功能启用
#define FUNC_DISABLE 0X00 //功能禁用

//EMS mode
#define	EMS_STG_LS 0x00  //本地调度，Local schedule
#define	EMS_STG_RS 0x01  //远程调度，Remote schedule
#define	EMS_STG_LF 0x02  //负载优先，Load first
#define	EMS_STG_LR 0x03	 //负载优先-保电，Load first reservation
#define	EMS_STG_BF 0x04  //电池优先，Battery fisrt
#define	EMS_STG_ECO 0x05 //经济模式，Economy mode
#define	EMS_STG_PLS 0x06 //削峰填谷，Peak load shifting
#define	EMS_STG_BKP 0x07 //后备电源，Backup power
#define	EMS_STG_EM 0x08  //外置电表，External meter

#define LS_MODE_ACCP 0x00 //交流调度，对应AC恒功率
#define LS_MODE_DCCP 0x01 //直流调度，对应DC恒功率

#define PCS_MODE_ACCP 0x40 //PCS交流恒功率
#define PCS_MODE_DCCP 0x22 //PCS直流恒功率
#define PCS_MODE_OFF 0x41 //PCS独立逆变

#define ECO_PRD_1 0x00//时间段1
#define ECO_PRD_2 0x01//时间段2

#define ECO_SEG_PEDI 0x00 //尖时
#define ECO_SEG_PEAK 0x01 //峰时
#define ECO_SEG_PLAIN 0x02 //平时
#define ECO_SEG_VALLEY 0x03 //谷时

//DI输入量位定义
#define DI_EM_STOP 0x01 //急停输入
#define DI_SPD_FAILURE 0x02 //浪涌串联反馈
#define DI_GRID_MCCB 0x04 //电网断路器反馈
#define DI_LOAD_MCCB 0x08 //负载断路器反馈
#define DI_EX_FAN 0x10 //外部风机反馈
#define DI_BYPASS_MCCB 0x20 //旁路断路器反馈

//DO输出量位定义，仅调试模式下有效
#define DO_CAB_FAN 0x00 //电源柜风机
#define DO_SYS_FAULT 0x01 //系统故障指示
#define DO_SYS_RUNNING 0x02 //系统运行指示
#define DO_AIR 0x10 //空调供电控制
#define DO_HVB 0x20 //高压箱供电控制
#define DO_EX_FAN 0x40 //柜外风机控制

//调试位定义，用于开启DI和DO口功能、模拟故障状态
#define DEBUG_DISABLE 0x00
#define DEBUG_ENABLE 0x01
#define DEBUG_BMS_FAULT_MILD 0x02
#define DEBUG_BMS_FAULT_MEDIUM 0x04
#define DEBUG_BMS_FAULT_SEVERE 0x08

typedef struct {                     //the capacity of the parameter table is 125kB

 /**********************************************************************
                          可读可写功能号，主要为参数及指令，长度为
    **********************************************************************/
  uint16_t PARA_ADDR;				 // 000: 功能号表首地址，主要针对参数表的操作
	uint16_t OnOff;					 // 001: 开关机 0 关机 1 开机
	uint16_t Grid_onoff;			 // 002: 并离网状态 0 并网 1离网 2 油机
	uint16_t Anti_Reflux_Enable;	 // 003: 防逆流使能   1使能 0禁止
	uint16_t GridPV_Both_Enable;	 // 004: 电网PV同时充电使能 1使能 0禁止
	uint16_t DieselEngine_Enable;	 // 005: 油机使能 1使能 0禁止
	uint16_t GridOnOffAuto_Enable;	 // 006: 并离网切换使能。  1使能 0禁止
	uint16_t AutoRun_Enable;		 // 007: 设备自启动使能 1使能 0禁止
	uint16_t PVGridBothChargeEnable; // 008：光伏和电网同时充电使能
	uint16_t TrickleChargeEnable;	 // 009:强制充电使能
	uint16_t GridOnMode;			 // 010: 0 负载优先 1 电池优先 2 经济模式 3 人工调度 4 EMS模式
	// Local schedule
	uint16_t LSMode;	 // 011:本地调度方式
	int16_t LSActiveP;	 // 012:本地调度交流有功功率或直流功率
	int16_t LSReactiveP; // 013:本地调度交流无功功率
	// Backup power
	uint16_t BKPVolt; // 014: pcs输出电压，默认230
	uint16_t BKPFreq; // 015: pcs输出电压频率，默认50
	// Economy mode
	struct
	{
		uint16_t Date; // 016:起始日期
		uint16_t Prd;  // 017:日历设置
	} ECOCldSet[4];  // 016~023
	struct
	{
		uint16_t Time; // 024:起始时间
		uint16_t Seg;  // 025:计费号
	} ECOSegSet[2][6];  //024~047
	// Peak load shifting
	uint16_t PLSup;	 // 048:电网功率上限
	uint16_t PLSlow; // 049:电网功率下限
	// PCS Setting
	uint16_t CPwrUp; // 050:PCS充电功率上限
	uint16_t DPwrUp; // 051:PCS放电功率上限
	// External meter
	uint16_t EMActiveP; // 052:外置电表有功功率
	uint16_t EMReactiveP; // 053:外置电表无功功率
	
	uint16_t PcsRunMode;  // 054:PCS工作模式
	uint16_t PcsN;		   // 055: pcS并机数量
	uint16_t ForcePower;   // 056：强制充电功率
	uint16_t PVPowerLimit; // 057: 光伏功率限值

	uint16_t BattUpV;	  // 058: 电池单体电压上限阈值
	uint16_t BattDownV;	  // 059: 电池单体电压下限阈值
	uint16_t BattMaxPwr;  // 060：电池最大充放电功率0.001
	uint16_t MpptMaxPwr;  // 061: MPPT最大功率
	uint16_t PcsMaxPwrH;  // 062：PCS最大充放电功率H
	uint16_t PcsMaxPwrL;  // 063：PCS最大充放电功率L
	uint16_t TrickleCurr; // 064：充电涓流电流
	uint16_t Year;        // 065
	uint16_t Month; //066
	uint16_t Day;  //067
	uint16_t Hour; //068
	uint16_t Minute; //069
	uint16_t Second; //070
	uint16_t Weekday; //071
	uint16_t CorrectTime; //072
	uint16_t Din; //073:数字输入
	uint16_t Dout; //074:数字输出
	uint16_t Debug; //075:调试预留接口
	uint16_t EmsReserve[24];

 
	/*********************************************************
	地址100~199为PCS数据存储区，该区域数据不存在e方中，掉电即消失
	********************************************************/
	uint16_t    PcsStatus;              //100: PCS状态
	uint16_t    PcsWarn;                //101：Pcs告警
	uint16_t    PcsFault;               //102：Pcs故障
	uint16_t    PcsDcVH;                //103：Pcs直流电压高16位
	uint16_t    PcsDcVL;                //104：Pcs直流电压低16位
	uint16_t    PcsDcCH;                //105: Pcs直流电流高16位
	uint16_t    PcsDcCL;                //106: Pcs直流电流低16位
	uint16_t    PcsDcPH;                //107：Pcs直流功率高16位
	uint16_t    PcsDcPL;                //108：Pcs直流功率低16位
	uint16_t    PcsVuH;                 //109: Pcs电网电压U高16位
	uint16_t    PcsVuL;                 //110: Pcs电网电压U低16位
	uint16_t    PcsVvH;                 //111: Pcs电网电压V高16位
	uint16_t    PcsVvL;                 //112: Pcs电网电压V低16位
	uint16_t    PcsVwH;                 //113: Pcs电网电压W高16位
	uint16_t    PcsVwL;                 //114: Pcs电网电压W低16位
	uint16_t    PcsCuH;                 //115: Pcs电网电流U高16位
	uint16_t    PcsCuL;                 //116: Pcs电网电流U低16位
	uint16_t    PcsCvH;                 //117: Pcs电网电流V高16位
	uint16_t    PcsCvL;                 //118: Pcs电网电流V低16位
	uint16_t    PcsCwH;                 //119: Pcs电网电流W高16位
	uint16_t    PcsCwL;                 //120: Pcs电网电流W低16位	
	uint16_t    PcsPH;                  //121: Pcs电网有功高16位
	uint16_t    PcsPL;                  //122: Pcs电网有功低16位
	uint16_t    PcsQH;                  //123: Pcs电网无功高16位
	uint16_t    PcsQL;                  //124: Pcs电网无功低16位
	uint16_t    PcsSH;                  //125: Pcs电网视在高16位
	uint16_t    PcsSL;                  //126: Pcs电网视在低16位
	uint16_t    PcsLoadVuH;             //127: Pcs负载电压U高16位
	uint16_t    PcsLoadVuL;             //128: Pcs负载电压U低16位
	uint16_t    PcsLoadVvH;             //129: Pcs负载电压V高16位
	uint16_t    PcsLoadVvL;             //130: Pcs负载电压V低16位
	uint16_t    PcsLoadVwH;             //131: Pcs负载电压W高16位
	uint16_t    PcsLoadVwL;             //132: Pcs负载电压W低16位
	uint16_t    PcsLoadCuH;             //133: Pcs负载电流U高16位
	uint16_t    PcsLoadCuL;             //134: Pcs负载电流U低16位
	uint16_t    PcsLoadCvH;             //135: Pcs负载电流V高16位
	uint16_t    PcsLoadCvL;             //136: Pcs负载电流V低16位
	uint16_t    PcsLoadCwH;             //137: Pcs负载电流W高16位
	uint16_t    PcsLoadCwL;             //138: Pcs负载电流W低16位	
	uint16_t    PcsLoadPH;              //139: Pcs负载有功高16位
	uint16_t    PcsLoadPL;              //140: Pcs负载有功低16位
	uint16_t    PcsLoadQH;              //141: Pcs负载无功高16位
	uint16_t    PcsLoadQL;              //142: Pcs负载无功低16位
	uint16_t    PcsLoadSH;              //143: Pcs负载视在高16位
	uint16_t    PcsLoadSL;              //144: Pcs负载视在低16位
	uint16_t    PcsModeH;               //145: Pcs模式高16位
	uint16_t    PcsModeL;               //146: Pcs模式低16位
	uint16_t    PcsPara1H;              //147: Pcs参数1高16位
	uint16_t    PcsPara1L;              //148: Pcs参数1低16位
	uint16_t    PcsPara2H;              //149: Pcs参数1高16位
	uint16_t    PcsPara2L;              //150: Pcs参数1低16位
	uint16_t    PcsPara3H;              //151: Pcs参数1高16位
	uint16_t    PcsPara3L;              //152: Pcs参数1低16位
	uint16_t    PcsPara4H;              //153: Pcs参数1高16位
	uint16_t    PcsPara4L;              //154: Pcs参数1低16位
	uint16_t    PcsStartStop;           //155: Pcs开关机
	uint16_t    PcsFaultClear;          //156: Pcs故障清除
  uint16_t    PcsIgbtIntakeTempe;     //157 : 模块进风口温度
  uint16_t    PcsIgbtOutletTemp;      //158 : 模块出风口温度
  uint16_t    Reserve82;                  //159 : 保留
  uint16_t    Reserve83;                  //160 : 保留
	
  uint16_t    Reserve84;                  //161 : 保留
  uint16_t    Reserve85;                  //162 : 保留
  uint16_t    Reserve86;                  //163 : 保留
  uint16_t    Reserve87;                  //164 : 保留
  uint16_t    Reserve88;                  //165 : 保留
  uint16_t    Reserve89;                  //166 : 保留
  uint16_t    Reserve90;                  //167 : 保留
  uint16_t    Reserve91;                  //168 : 保留
  uint16_t    Reserve92;                  //169 : 保留
  uint16_t    Reserve93;                 //170 : 保留
	
  uint16_t    Reserve94;                  //171 : 保留
  uint16_t    Reserve95;                  //172 : 保留
  uint16_t    Reserve96;                  //173 : 保留
  uint16_t    Reserve97;                  //174 : 保留
  uint16_t    Reserve98;                  //175 : 保留
  uint16_t    Reserve99;                  //176 : 保留
  uint16_t    Reserve100;                 //177 : 保留
  uint16_t    Reserve101;                 //178 : 保留
  uint16_t    Reserve102;                 //179 : 保留
  uint16_t    Reserve103;                 //180 : 保留
	
  uint16_t    Reserve104;                 //181 : 保留
  uint16_t    Reserve105;                 //182 : 保留
  uint16_t    Reserve106;                 //183 : 保留
  uint16_t    Reserve107;                 //184 : 保留
  uint16_t    Reserve108;                 //185 : 保留
  uint16_t    Reserve109;                 //186 : 保留
  uint16_t    Reserve110;                 //187 : 保留
  uint16_t    Reserve111;                 //188 : 保留
  uint16_t    Reserve112;                 //189 : 保留
  uint16_t    Reserve113;                 //190 : 保留
	
  uint16_t    Reserve114;                 //191 : 保留
  uint16_t    Reserve115;                 //192 : 保留
  uint16_t    Reserve116;                 //193 : 保留
  uint16_t    Reserve117;                 //194 : 保留
  uint16_t    Reserve118;                 //195 : 保留
  uint16_t    Reserve119;                 //196 : 保留
  uint16_t    Reserve120;                 //197 : 保留
  uint16_t    Reserve121;                 //198 : 保留
  uint16_t    Reserve122;                 //199 : 保留
  
	
	/*********************************************************
	地址200~399为Mppt数据存储区，该区域数据不存在e方中，掉电即消失
	********************************************************/
          uint16_t    MpptSysVolt;           //200: MPPT系统电压
          uint16_t    MpptSysCurr;           //201: MPPT系统电流
          uint16_t    MpptSysPwr;           //202: MPPT系统功率
          uint16_t    MpptDcBusVolt;        //203:MPPT母线电压
          int16_t     MpptDcBusCurr;        //204
          int16_t     MpptDcPwr;//          //205
          int16_t     MpptSingleT[6];        //206~211: MPPT6个模块环温
          int16_t     MpptInTemp;            //212
          int16_t     MpptoutTemp;           //213
          uint16_t    MpptSingleV[6];       //214~219: MPPT10个模块电压高16位 0-9 MpptSingleVH[10]和MpptSingleVL[0]地址一样
          int16_t     MpptSingleC[6];       //220~225: MPPT10个模块电流高16位
          int16_t     MpptSingleP[6];       //226~231: MPPT10个模块电流高16位
          uint16_t    MpptDevStutas[4];     //232~235:设备状态字
          uint16_t    MpptBFault;           //236:并机故障字
          uint16_t    MpptSysFault;         //237:MPPT系统故障字
          uint16_t    MpptCurrFault[6];     //238~243: MPPT6个模块直流接入故障值
          uint16_t    MpptReserve124[156];             //244~399: 保留


	/*********************************************************
	地址400~499为BMS数据存储区，该区域数据不存在e方中，掉电即消失
	********************************************************/
	uint16_t    Rechargeable_Battery_capacity; //400:电池可充电量
	uint16_t    Discharge_Battery_Capacity;    //401: 电池可放电量
	uint16_t    BmsTotalVolt;                  //402: 电池总电压  //SOC 电池剩余电量百分比
	uint16_t    BmsTotalCurr;                  //403: 电池总电流  SOH 电池健康度
	uint16_t    BmsSoc;                        //404: SOC  //高压继电器状态
	uint16_t    BmsSoh;                        //405：SOH// 总电压
	uint16_t    BmsBattStatus0;                //406: 电池状态 0  //总电流
	uint16_t    BmsBattStatus1;                //407: 电池状态 1 充电允许最大电流
	uint16_t    BmsBattFault0;                 //408: 电池故障 0  放电允许最大电流
	uint16_t    BmsBattFault1;                 //409: 电池故障 1  单体最大电压组号
	uint16_t    BmsBattFault2;                 //410: 电池故障 2  单体最大电压箱号
	
	uint16_t    BmsMinTem;                     //411:单体最低温度 单体最大电压编号
	uint16_t    BmsMaxTemRN;                   //412：最低温度编号   单体最大温度组号
	uint16_t    BmsMaxTem;                     //413: 单体最高温度  单体最大温度箱号
	uint16_t    BmsMaxTemCN;                   //414: 最高温度编号  最大温度
	
	uint16_t    BmsAvgTem;                     //415: 平均温度   单体最小电压组号
	uint16_t    BmsMinVolt;                    //416: 单体最低电压 单体最小电压箱号
	uint16_t    BmsMinVoltN;                   //417: 最低电压编号  单体最小电压编号
	uint16_t    BmsMaxVolt;                    //418：  单体最高电压   单体最小温度组号
	uint16_t    BmsMaxVoltCN;                  //419:  最高电压编号 单体最小温度箱号
	uint16_t    BmsAvgVolt;                    //420: 平均电压  最小温度
	
	uint16_t    BmsMaxChargeCurrent;           //421: 最大允许充电电流 电池状态
	uint16_t    BmsMaxDisChargeCurrent;        //422: 最大允许放电电流 系统状态标识
	uint16_t    BmsClusterState0;              //423：簇状态 0 一级报警标识1
	uint16_t    BmsClusterState1;              //424：簇状态 1  一级报警标识2
	uint16_t    BmsMinSoc;                     //425：单体最低 SOC 二级报警标识1
	uint16_t    BmsMinSocN;                    //426：最低 SOC 编号  二级报警标识2
	uint16_t    BmsMaxSoc;                     //427：单体最高 SOC  三级报警标识1
	uint16_t    BmsMaxSocN;                    //428：最高 SOC 编号  三级报警标识2
  uint16_t    BmsClusterCurrent;             //429:簇 1..簇 10 电流   保留
	
  uint16_t    Mppt_electric_quantity1_s;        //430  MPPT1秒电量
  uint16_t    Mppt_electric_quantity1_m;        //431  MPPT1分钟电量 
  uint16_t    Mppt_electric_quantity1_h;        //432  MPPT1小时电量	
	
	int16_t    Phase_A_Active_Ppower;            //433  A 相有功功率
	int16_t    Phase_B_Active_Ppower;            //434  B 相有功功率	
	int16_t    Phase_C_Active_Ppower;            //435  C 相有功功率	
	int16_t    Total_Active_Ppower;              //436  总有功功率
	int16_t    Phase_A_Reactive_Ppower;          //437  A 相无功功率
	int16_t    Phase_B_Reactive_Ppower;          //438  B 相无功功率
	int16_t    Phase_C_Reactive_Ppower;          //439  C 相无功功率  
	int16_t    Total_Rective_Ppower;             //440  总无功功率
	int16_t    Phase_A_Apparent_Power;           //441  A 相视在功率
	int16_t    Phase_B_Apparent_Power;           //442  B 相视在功率
	int16_t    Phase_C_Apparent_Power;           //443  C 相视在功率  
	int16_t    Total_Apparent_Power;             //444  总视在功率
	int16_t    Phase_A_Power_Factor;             //445  A 相功率因数 
	int16_t    Phase_B_Power_Factor;             //446  B 相功率因数 
	int16_t    Phase_C_Power_Factor;             //447  C 相功率因数 
	int16_t    Total_Power_Factor;               //448  总功率因数
	int16_t    VoltRatio;                    	//449 电压变比
	int16_t    CurrRatio;											//450 电流变比
	
	uint16_t    reservedx[18];
	
}Struct_InvParameter;


typedef struct {                     //the capacity of the parameter table is 125kB

  
  float       PcsDcVoltage;             // Pcs直流电压
  float       PcsDcCurrent;             // Pcs直流电流
	float       PcsDcPower;               // Pcs直流功率 
	float       PcsActivePower;           // Pcs电网有功
	float       PcsReActivePower;         // Pcs电网无功
  float       PcsLoadActivePower;       // Pcs负载有功
  float       PcsLoadReActivePower;     // Pcs负载无功
	float       PcsLoadApparentPower;     // Pcs负载视在功率
  float       PcsChargePowerMax;        // Pcs最大充电功率
  float       PcsInchargePowerMax;      // Pcs最大放电功率
	float       MpptSysVoltage;           // MPPT系统电压   
	float       MpptSysCurrent;           // MPPT系统电流
  float       MpptSysPower;             // MPPT系统功率
  float       MpptSysPowerMax;          // MPPT系统最大功率
	float       MpptRatePower;            // MPPT额定功率
	float       BattPowerMax;             // 电池最大功率
	float       TrickleCurr;              // 充电涓流电流
	float       GridLimitUp;              // 电网上限功率，削峰填谷时的取电限值
	float       GridLimitLow;             // 电网下限功率，削峰填谷
  float       AgcPower;                 // AGC模式功率
}Struct_FParameter;



enum ENUM_STATETYPE {
    STA_STOP = 0,
	  STA_START,
    STA_PCSCHECK,
    STA_RUN,
    STA_FAILURE,
    STA_EMSTOP
};

typedef struct { 
	
    uint8_t pcs_fault;
	  uint8_t mttp_fault;
	  uint8_t bms1_fault;
	  uint8_t bms2_fault;
	  uint8_t bms3_fault;	
}Struct_SystemFault;


//0可充可放 1 不可充可放  2 不可放可充 3 不可充不可放
#define CAN_CHARGE_CAN_DIDCHARGEE      0   // 0可充可放        / 可充    / 可放 
#define CANNOT_CHARGE_CAN_DISCHARGE    1   // 1不可充可放                / 可放 
#define CAN_CHARGE_CANNOT_DISCHARGE    2   // 2 不可放可充     / 可充
#define CANNOT_CHARGE_CANNOT_DISCHARGE 3   // 3 不可充不可放
#define CHARGE_CURRENT_LARGE           4   // 4 充电电流大
#define DISCHARGE_CURRENT_LARGE        5   // 5 放电电流大

#define ANTI_REFLUX_DISENABLE          0   // 防逆流禁止
#define ANTI_REFLUX_ENABLE             1   // 防逆流使能

#define PCS_MODE_INVERTER              0   // PCS逆变到负载

#define PCS_MODE_POWER_ON              1   // PCS开机
#define PCS_MODE_FAULT_CLEAR           2   // 清除PCS故障

#define PCS_SYSTEM_FAULT               1   // PCS系统有故障
#define PCS_SYSTEM_NO_FAULT            0   // PCS系统没有故障

#define MPPT_SYSTEM_FAULT              1   // MPPT系统有故障
#define MPPT_SYSTEM_NO_FAULT           0   // MPPT系统没有故障

#define BMS_SYSTEM_FAULT               1   // BMS系统有故障
#define BMS_SYSTEM_NO_FAULT            0   // BMS系统没有故障


void  LoadFirst(void);
void  BattFirst(void);
void  Economic_Models(void);
void  PeakLload_Shifting(void);
void  LoadFirst_PowerConservation(void);
void  Remote_scheduling(void);
void  Local_scheduling(void);
void  External_Electric_Meters(void);
void  GridAutoSwitchingMode(void);
void  GridOff(void);
uint16_t  PcsCheck(void);
void ScibDeal(void);

extern uint8_t Wirte_Over_Flag;
#endif




