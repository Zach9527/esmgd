#ifndef  EEPROM_H
#define  EEPROM_H

//#include "main.h"

#define EEPROM_CHIP  22              //  每次读eeprom字节

#define EEPROM_SUM  462

typedef struct {                     //the capacity of the parameter table is 125kB

  /**********************************************************************
                          可读可写功能号，主要为参数及指令，长度为
    **********************************************************************/
  uint16_t    PARA_ADDR;                 //000: 功能号表首地址，主要针对参数表的操作
	uint16_t    OnOff;                     //001: 开关机 0 关机 1 开机
  uint16_t    Grid_onoff;                //002: 并离网状态 0 并网 1离网 2 油机
	uint16_t    Anti_Reflux_Enable;        //003: 防逆流使能   1使能 0禁止
  uint16_t    GridPV_Both_Enable;        //004: 电网PV同时充电使能 1使能 0禁止
	uint16_t    DieselEngine_Enable;       //005: 油机使能 1使能 0禁止
	uint16_t    GridOnOffAuto_Enable;      //006: 并离网切换使能。  1使能 0禁止
	uint16_t    GridOnMode;                //007: 0 负载优先 1 电池优先 2 经济模式 3 人工调度 4 EMS模式
	uint16_t    PinkPlainValley;           //008: 谷峰平 0 谷 1 峰 2 平 
	uint16_t    PcsN;                      //009: pcS并机数量
	uint16_t    PcsVout;                   //010: pcs输出电压，默认230
	uint16_t    PcsVoutFre;                //011: pcs输出电压频率，默认50  ????????????????????????????
	uint16_t    AutoRun_Enable;            //012: 设备自启动使能 1使能 0禁止
	uint16_t    ForceV;                    //013: 强充电压
	uint16_t    ForceVRecover;             //014: 强充恢复电压
	uint16_t    ForcePower;                //015：强制充电功率
	uint16_t    FilledRecover;             //016: 充满恢复电压
	uint16_t    EvacuationRecover;         //017: 放空恢复电压
	uint16_t    PVPowerLimit;              //018: 光伏功率限值
	uint16_t    PVGridBothChargeEnable;    //019：光伏和电网同时充电使能
	uint16_t    GridLimitUp;               //020: 电网上限功率，削峰填谷时的取电限值
	
	uint16_t    BattUpV;                   //021: 电池单体电压上限阈值  
	uint16_t    BattDownV;                 //022: 电池单体电压下限阈值
	uint16_t    BattMaxPwr;                //023：电池最大充放电功率0.001
	uint16_t    MpptMaxPwr;                //024: MPPT最大功率
	uint16_t    PcsMaxPwrH;                //025：PCS最大充放电功率H
	uint16_t    PcsMaxPwrL;                //026：PCS最大充放电功率L
	uint16_t    TrickleCurr;               //027：充电涓流电流
	uint16_t    PcsRunMode;                //028: pcs运行模式
  uint16_t    Reserve0;                  //029 : 保留
  uint16_t    Reserve1;                  //030 : 保留
	
  uint16_t    Reserve2;                   //031 : 保留
  uint16_t    Reserve3;                   //032 : 保留
  uint16_t    Reserve4;                   //033 : 保留
  uint16_t    Reserve5;                   //034 : 保留
  uint16_t    Reserve6;                   //035 : 保留
  uint16_t    Reserve7;                   //036 : 保留
  uint16_t    Reserve8;                   //037 : 保留
  uint16_t    Reserve9;                   //038 : 保留
  uint16_t    Reserve10;                  //039 : 保留
  uint16_t    Reserve11;                  //040 : 保留
	
  uint16_t    Reserve12;                  //041 : 保留
  uint16_t    Reserve13;                  //042 : 保留
  uint16_t    Reserve14;                  //043 : 保留
  uint16_t    Reserve15;                  //044 : 保留
  uint16_t    Reserve16;                  //045 : 保留
  uint16_t    Reserve17;                  //046 : 保留
  uint16_t    Reserve18;                  //047 : 保留
  uint16_t    Reserve19;                  //048 : 保留
  uint16_t    Reserve20;                  //049 : 保留
  uint16_t    Reserve21;                  //050 : 保留
	
  uint16_t    Reserve22;                  //051 : 保留
  uint16_t    Reserve23;                  //052 : 保留
  uint16_t    Reserve24;                  //053 : 保留
  uint16_t    Reserve25;                  //054 : 保留
  uint16_t    Reserve27;                  //055 : 保留
  uint16_t    Reserve28;                  //056 : 保留
  uint16_t    Reserve29;                  //057 : 保留
  uint16_t    Reserve30;                  //058 : 保留
  uint16_t    Reserve31;                  //059 : 保留
  uint16_t    Reserve32;                  //060 : 保留
	
  uint16_t    Reserve33;                  //061 : 保留
  uint16_t    Reserve34;                  //062 : 保留
  uint16_t    Reserve35;                  //063 : 保留
  uint16_t    Reserve36;                  //064 : 保留
  uint16_t    Reserve37;                  //065 : 保留
  uint16_t    Reserve38;                  //066 : 保留
  uint16_t    Reserve39;                  //067 : 保留
  uint16_t    Reserve40;                  //068 : 保留
  uint16_t    Reserve41;                  //069 : 保留
  uint16_t    Reserve42;                  //070 : 保留
	
  uint16_t    Reserve43;                  //071 : 保留
  uint16_t    Reserve44;                  //072 : 保留
  uint16_t    Reserve45;                  //073 : 保留
  uint16_t    Reserve46;                  //074 : 保留
  uint16_t    Reserve47;                  //075 : 保留
  uint16_t    Reserve48;                  //076 : 保留
  uint16_t    Reserve49;                  //077 : 保留
  uint16_t    Reserve50;                  //078 : 保留
  uint16_t    Reserve51;                  //079 : 保留
  uint16_t    Reserve52;                  //080 : 保留
	
  uint16_t    Reserve53;                  //081 : 保留
  uint16_t    Reserve54;                  //082 : 保留
  uint16_t    Reserve55;                  //083 : 保留
  uint16_t    Reserve57;                  //084 : 保留
  uint16_t    Reserve58;                  //085 : 保留
  uint16_t    Reserve59;                  //086 : 保留
  uint16_t    Reserve60;                  //087 : 保留
  uint16_t    Reserve61;                  //088 : 保留
  uint16_t    Reserve62;                  //089 : 保留
  uint16_t    Reserve63;                  //090 : 保留
	
  uint16_t    Reserve64;                  //091 : 保留
  uint16_t    Reserve65;                  //092 : 保留
  uint16_t    Reserve66;                  //093 : 保留
  uint16_t    Reserve67;                  //094 : 保留
  uint16_t    Reserve68;                  //095 : 保留
  uint16_t    Reserve69;                  //096 : 保留
  uint16_t    Reserve70;                  //097 : 保留
  uint16_t    Reserve71;                  //098 : 保留
  uint16_t    Reserve72;                  //099 : 保留

 
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
  uint16_t    Reserve80;                  //157 : 保留
  uint16_t    Reserve81;                  //158 : 保留
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
        uint16_t    MpptSysVolt;           //200: MPPT绯荤粺鐢靛帇
        uint16_t    MpptSysCurr;           //201: MPPT绯荤粺鐢垫祦
        uint16_t    MpptSysPwr;           //202: MPPT绯荤粺鍔熺巼
        uint16_t    MpptDcBusVolt;        //203:MPPT姣嶇嚎鐢靛帇
        int16_t     MpptDcBusCurr;        //204
        int16_t     MpptDcPwr;//          //205
        int16_t     MpptSingleT[6];        //206~211: MPPT6涓ā鍧楃幆娓?
        int16_t     MpptInTemp;            //212
        int16_t     MpptoutTemp;           //213
        uint16_t    MpptSingleV[6];       //214~219: MPPT10涓ā鍧楃數鍘嬮珮16浣?0-9 MpptSingleVH[10]鍜孧pptSingleVL[0]鍦板潃涓€鏍?
        int16_t     MpptSingleC[6];       //220~225: MPPT10涓ā鍧楃數娴侀珮16浣?
        int16_t     MpptSingleP[6];       //226~231: MPPT10涓ā鍧楃數娴侀珮16浣?
        uint16_t    MpptDevStutas[4];     //232~235:璁惧鐘舵€佸瓧
        uint16_t    MpptBFault;           //236:骞舵満鏁呴殰瀛?
        uint16_t    MpptSysFault;         //237:MPPT绯荤粺鏁呴殰瀛?
        uint16_t    MpptCurrFault[6];     //238~243: MPPT6涓ā鍧楃洿娴佹帴鍏ユ晠闅滃€?
        uint16_t    MpptReserve124[156];             //244~399: 淇濈暀



	/*********************************************************
	地址400~499为BMS数据存储区，该区域数据不存在e方中，掉电即消失
	********************************************************/
	uint16_t    BmsMaxVolt;                       //400: 单体最大电压
	uint16_t    BmsMinVolt;                       //401: 单体最小电压
	uint16_t    BmsSoc;                           //402: SOC 电池剩余电量百分比
	uint16_t    BmsSoh;                           //403: SOH 电池健康度
	uint16_t    BmsHighVoltageRelay;              //404: 高压继电器状态
	uint16_t    BmsTotalVolt;                     //405：总电压
	uint16_t    BmsTotalCurr;                     //406: 总电流
	uint16_t    BmsMaxChargeCurr;                 //407: 充电允许最大电流
	uint16_t    BmsMaxDisCurr;                    //408: 放电允许最大电流
	uint16_t    BmsMaxVoltRN;                     //409: 单体最大电压组号
	uint16_t    BmsMaxVoltCN;                     //410: 单体最大电压箱号
	
	uint16_t    BmsMaxVoltN;                      //411: 单体最大电压编号
	uint16_t    BmsMaxTemRN;                      //412：单体最大温度组号
	uint16_t    BmsMaxTemCN;                      //413: 单体最大温度箱号
	uint16_t    BmsMaxTem;                        //414: 最大温度
	uint16_t    BmsMinVoltRN;                     //415: 单体最小电压组号
	uint16_t    BmsMinVoltCN;                     //416: 单体最小电压箱号
	uint16_t    BmsMinVoltN;                      //417: 单体最小电压编号
	uint16_t    BmsMinTemRN;                      //418：单体最小温度组号
	uint16_t    BmsMinTemCN;                      //419: 单体最小温度箱号
	uint16_t    BmsMinTem;                        //420: 最小温度
	
	uint16_t    BmsBattStatus;                    //421: 电池状态
	uint16_t    BmsSysStatus;                     //422: 系统状态标识
	uint16_t    BmsLevel1Warn1;                   //423：一级报警标识1
	uint16_t    BmsLevel1Warn2;                   //424：一级报警标识2
	uint16_t    BmsLevel2Warn1;                   //425：二级报警标识1
	uint16_t    BmsLevel2Warn2;                   //426：二级报警标识2
	uint16_t    BmsLevel3Warn1;                   //427：三级报警标识1
	uint16_t    BmsLevel3Warn2;                   //428：三级报警标识2
  uint16_t    Reserve139;                       //429: 保留	
  uint16_t    Mppt_electric_quantity1_s;        //430  MPPT1秒电量
  uint16_t    Mppt_electric_quantity1_m;        //431  MPPT1分钟电量 
  uint16_t    Mppt_electric_quantity1_h;        //432  MPPT1小时电量	
	
	uint16_t    reservedx[20];
}Struct_Eeprom24xxData;

 
 
extern Struct_Eeprom24xxData Eeprom_data;
#endif



