//int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300 };
//int name_arr[] = { 38400, 19200, 9600, 4800, 2400, 1200, 300, 38400, 19200, 9600, 4800, 2400, 1200, 300 };
typedef unsigned char BYTE;
#define TRUE 1
#define FALSE 0

struct timeval tpstart,tpend_1,tpstart2,tpend_2;
double timeuse;
int Lineshow;

//unsigned int serialflg;
BYTE rbuf[160];//存放正确数据
BYTE write_buf[30];//要发送的数据
int con;

pthread_mutex_t serial_msg_mutex;
pthread_mutex_t serial_msg_mutexw;

struct termios Opt; //定义termios结构

//短帧1
char Key1;
char Key2;
char Key3;
char Key4;
float Vspeed;
float Rspeed;
int Bpress1;
int Bpress2;
int Bpress3;
char Alarm[19];
char LDW;
float Thopen;
char KeyOK;
char KeyMode;
char KeyUp;
char KeyDown;
char KeyLeft;
char KeyRight;
char Cutheme;


//短帧2
float TraFuel;
float AveFuel;
char ECAS_Pos;
float ECAS_Load;
char ACC_Gear;
int ACC_Dist;
int TarVspeed;
int CruiseVspeed;
int VarVspeed;
float RetarderTorq;
char RetarderGear;
int PTOspeed;
char BgtMode;
char SectionSel;
char AlarmDis;
char A_M;
char Cmode;
char Lmode;
char EPmode;
char Tgear_S;
int Tgear;
int Cgear;
char AMTword;
char TyreWord;
char TyreMode;
char Lighrigion1;
char Lighrigion2;
char Lighrigion3;
char Lighrigion4;
float Vadjgoal;


//长帧1
int TotalTrip;
float SingleTrip;
float CellVoltage;
int CoolWater;
float FuelLevel;
int OilPress;
float UreaLevel;
char Sec;
char Min;
char Hour;
char Day;
char Month;
int Year;
char DriveTime_min;
int DriveTime_hour;
char Week;
float FuelConsump;
float Outertemp;
char Outertempsh;
float Score1;
float Score2;
float Score3;
float Score4;
float OverallScore;
float UnitScore;
int ScoreWord;



//长帧2
char Tyre_Pos;
int Tyre_Pres;
char Tyre_St;
char Pswordtip;
float SecOil_Level;
int AveVspeed;
int MaxVspeed;
int Range;
float Engtime;
float Tyre_Temp;

//特殊帧
char SMcu;


//燃油泄漏信息帧
int totalmile1;
int totalmile2;
int totalmile3;
int totalmile4;
int totalmile5;
int totalmile6;
int totalmile7;
int totalmile8;
float totmilepec1;
float totmilepec2;
float totmilepec3;
float totmilepec4;
float totmilepec5;
float totmilepec6;
float totmilepec7;
float totmilepec8;


//行驶信息帧
int avespeed1;
int maxspeed1;
char timemin1;
int timehour1;
int avespeed2;
int maxspeed2;
char timemin2;
int timehour2;
int avespeed3;
int maxspeed3;
char timemin3;
int timehour3;
int avespeed4;
int maxspeed4;
char timemin4;
int timehour4;
int avespeed5;
int maxspeed5;
char timemin5;
int timehour5;
int avespeed6;
int maxspeed6;
char timemin6;
int timehour6;
int avespeed7;
int maxspeed7;
char timemin7;
int timehour7;
int avespeed8;
int maxspeed8;
char timemin8;
int timehour8;


//油耗历史信息帧
float Oilconme1;
float Aveoilconme1;
float Subtomile1;
float Oilconme2;
float Aveoilconme2;
float Subtomile2;
float Oilconme3;
float Aveoilconme3;
float Subtomile3;
float Oilconme4;
float Aveoilconme4;
float Subtomile4;
float Oilconme5;
float Aveoilconme5;
float Subtomile5;
float Oilconme6;
float Aveoilconme6;
float Subtomile6;
float Oilconme7;
float Aveoilconme7;
float Subtomile7;
float Oilconme8;
float Aveoilconme8;
float Subtomile8;

//报警事件信息帧
char Alarmim;

//保养查询信息帧
int Tomile;
int Engmile;
int Gebmile;
int Draxmile;
int Hubmile;
int Fitmile;
int Afitmile;
int Ufitmile;
int Sfitmile;
int Dremile;


//EMS故障码信息帧
char Ems[61];

//VCU故障码信息帧
char Vcu[61];

//仪表版本信息帧
char SVersion[15];
char HVersion[15];

//用户自定义保养当前设置信息帧
int CTomile;
int CEngmile;
int CGebmile;
int CDraxmile;
int CHubmile;
int CFitmile;
int CAfitmile;
int CUfitmile;
int CSfitmile;
int CDremile;

//开发者模式信息帧
char Devr[36];


//IC故障码信息帧
char Ics[150];

//请求确认信息帧
char Resitem;
char Resval;




//核心板发送数据短帧
char Senden;
char Themseude;
char Itresh;
char KeyState;
char KeyMode_St;
char Code_Low;
char Code_High;
char Itset0;
char Itset1;
char Itset2;

//配置请求帧
char Requeitm;

//时间设置信息帧
char Yeset;
char Mtset;
char Daset;
char Hrset;
char Meset;

//保养设置信息帧
int STomile;
int SEngmile;
int SGebmile;
int SDraxmile;
int SHubmile;
int SFitmile;
int SAfitmile;
int SUfitmile;
int SSfitmile;
int SDremile;

//胎压自学习设置信息帧
char Tyrepos;

//胎压参考值设置信息帧
char Retypepos;
char Retyrepre;

//仪表自检设置信息帧
char Loudspk;
char LED;





typedef struct _SERIAL_MSG_NODE
{
	BYTE* data;//数组指针
	unsigned int len;//数组长度
	struct _SERIAL_MSG_NODE * next;
	struct _SERIAL_MSG_NODE * last;
}SERIAL_MSG_NODE;
SERIAL_MSG_NODE* serialMsgHead;
SERIAL_MSG_NODE* serialMsgLast;
SERIAL_MSG_NODE* serialMsgHeadw;
SERIAL_MSG_NODE* serialMsgLastw;










