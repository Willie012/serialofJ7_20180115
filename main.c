#include <stdio.h> //标准输入输出定义
#include <stdlib.h> //标准函数库定义
#include <string.h>
#include <unistd.h> //Unix标准函数定义
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> //文件控制定义
#include <termios.h> //POSIX中断控制定义
#include <errno.h> //错误号定义
#include <pthread.h>
#include <sys/time.h>
#include <sys/select.h>
#include <time.h>
#include <malloc.h>
#include "main.h"

void serial_init()
{
	if(pthread_mutex_init(&serial_msg_mutex, NULL) != 0) 	
	{		
	  	printf("serial_msg_mutex initialization error!\n");	
	}
//	serialflg=0;
	serialMsgLast = (SERIAL_MSG_NODE *)malloc(sizeof(SERIAL_MSG_NODE));
	serialMsgLast->next = NULL;
	
	serialMsgHead = (SERIAL_MSG_NODE *)malloc(sizeof(SERIAL_MSG_NODE));	
	serialMsgHead->next = serialMsgLast;
	serialMsgHead->last = NULL;
	serialMsgLast->last = serialMsgHead;
	
	if(pthread_mutex_init(&serial_msg_mutexw, NULL) != 0) 	
	{		
	  	printf("serial_msg_mutexw initialization error!\n");	
	}
//	serialflg=0;
	serialMsgLastw = (SERIAL_MSG_NODE *)malloc(sizeof(SERIAL_MSG_NODE));
	serialMsgLastw->next = NULL;
	
	serialMsgHeadw = (SERIAL_MSG_NODE *)malloc(sizeof(SERIAL_MSG_NODE));	
	serialMsgHeadw->next = serialMsgLastw;
	serialMsgHeadw->last = NULL;
	serialMsgLastw->last = serialMsgHeadw;

	con=0;
	Lineshow=0;
}

void SetSpeed(int fd, int speed)
{//设置波特率
	int i;
	
	if(tcgetattr(fd, &Opt) != 0)//用于获取与终端相关的参数
	{
		perror("tcgetattr fd");
		return;
	}
	if( speed == B115200 )
	{
		tcflush( fd, TCIOFLUSH);//清空终端未完成的输入/输出请求及数据
		cfsetispeed( &Opt, B115200);
		cfsetospeed( &Opt, B115200);
		if(tcsetattr(fd, TCSANOW, &Opt) != 0)//用于设置终端参数的函数
		{
			perror("tcsetattr fd");
			return;
		}
		tcflush(fd, TCIOFLUSH);
	}	
}

int SetParity(int fd, int databits, int stopbits, int parity)
{//设置数据位、停止位和校验位
	if(tcgetattr(fd, &Opt) != 0)
	{
		perror("tcgetattr fd");
		return FALSE;
	}
	Opt.c_cflag |= (CLOCAL | CREAD); //一般必设置的标志
	
	switch(databits) //设置数据位数
	{
	case 7:
		Opt.c_cflag &= ~CSIZE;
		Opt.c_cflag |= CS7;
		break;
	case 8:
		Opt.c_cflag &= ~CSIZE;
		Opt.c_cflag |= CS8;
		break;
		default:
		fprintf(stderr, "Unsupported data size.\n");
		return FALSE;
	}

		switch(stopbits) //设置停止位
		{
		case 1:
			Opt.c_cflag &= ~CSTOPB;
			break;
		case 2:
			Opt.c_cflag |= CSTOPB;
			break;
		default:
			fprintf(stderr, "Unsupported stopbits.\n");
			return FALSE;
		}

	switch(parity) //设置校验位
	{
	case 'n'://无校验
	case 'N':
		Opt.c_cflag &= ~PARENB; //清除校验位
		Opt.c_iflag &= ~INPCK; //enable parity checking
		break;
	case 'o':
	case 'O':
		Opt.c_cflag |= PARENB; //enable parity
		Opt.c_cflag |= PARODD; //奇校验
		Opt.c_iflag |= INPCK; //disable parity checking
		break;
	case 'e':
	case 'E':
		Opt.c_cflag |= PARENB; //enable parity
		Opt.c_cflag &= ~PARODD; //偶校验
		Opt.c_iflag |= INPCK; //disable pairty checking
		break;
	case 's'://space校验
	case 'S':
		Opt.c_cflag &= ~PARENB; //清除校验位
		Opt.c_cflag &= ~CSTOPB; //??????????????
		Opt.c_iflag |= INPCK; //disable pairty checking
		break;
	default:
		fprintf(stderr, "Unsupported parity.\n");
	return FALSE; 
	}
	


	Opt.c_cflag |= (CLOCAL | CREAD);

	Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//原始模式输入

	Opt.c_oflag &= ~OPOST;//原始输出
	Opt.c_oflag &= ~(ONLCR | OCRNL); ///////屏蔽在串口设置中c_iflag和c_oflag中存在的从NL-CR和CR-NL的映射，即串口能把回车和换行当成同一个字符
	Opt.c_iflag &= ~(ICRNL | INLCR );//| IGNCR
	Opt.c_iflag &= ~(IXON | IXOFF | IXANY); //把软件流控制屏蔽

	tcflush(fd, TCIFLUSH);
	Opt.c_cc[VTIME] = 50; //单位百毫秒,设置超时为15sec,只有设置为阻塞时这两个参数才有效，仅针对于读操作。
	Opt.c_cc[VMIN] =  1; //Update the Opt and do it now
	if(tcsetattr(fd, TCSANOW, &Opt) != 0)
	{
		perror("tcsetattr fd");
		return FALSE;
	}

	return TRUE;
}

void serial_msg_rev(unsigned int msgtype)
{//打包	
	int i;
//	gettimeofday(&tpstart,NULL);
	
	SERIAL_MSG_NODE* node=(SERIAL_MSG_NODE *)malloc(sizeof(SERIAL_MSG_NODE));
	node->len = msgtype;
	node->data = (BYTE*)malloc(sizeof(BYTE) * msgtype); 

	for(i=0;i<msgtype;i++)
	{
		node->data[i]=rbuf[i];
		//printf("node->data.para[%d] = %x, msg_save \n ",i,node->data.para[i]);
	} 
	
	SERIAL_MSG_NODE * tmp;
	pthread_mutex_lock(&serial_msg_mutex);

	serialMsgLast->last->next=node;
	node->next=serialMsgLast;//serialMsgLast总是指向最后一个分配的节点
	node->last=serialMsgLast->last;
	serialMsgLast->last=node;
//	printf("node->next:%p;serialMsgLast:%p;serialMsgLast:%p\n",serialMsgHead,node,serialMsgLast);
//	printf("node->last:%p;serialMsgLast->last:%p\n",node->last,serialMsgLast->last);
//	printf("serialMsgLast->last:%p;node:%p\n",serialMsgLast->last,node);

//	if(serialflg==0)
//	{
//		serialMsgHead->next=node;//serialMsgHead总是指向第一个分配的节点
		
//		serialflg=1;
//	}
//	printf("serialMsgHead:%p;node:%p;serialMsgLast:%p\n",serialMsgHead,node,serialMsgLast);


	pthread_mutex_unlock(&serial_msg_mutex);

//	gettimeofday(&tpend_1,NULL);

	//时间为微秒
//	timeuse=1000000*(tpend_1.tv_sec-tpstart.tv_sec)+ tpend_1.tv_usec-tpstart.tv_usec; 
    //时间为秒
//  timeuse/=1000000;
//    printf("Used Time of rev:%lf\n",timeuse);
	
}


void assign_funcr(SERIAL_MSG_NODE * node)
{//赋值函数
int i=0;
	switch(node->data[1])
	{//char类型未转换为十进制，其余类型都转换为十进制
		case 0x51:
			Key1= node->data[3]&0x03;
		    Key2= (node->data[3]&0x0c)>>2;
			Key3= (node->data[3]&0x30)>>4;
			Key4= (node->data[3]&0xc0)>>6;
			Vspeed=node->data[4]/256;
			if(Vspeed>250.996)
				Vspeed=250.996;
			Rspeed=(node->data[5]+node->data[6]*256)*0.125;
			if(Rspeed>8031.875)
				Rspeed=8031.875;
			Bpress1=node->data[7]*8;
			if(Bpress1>2000)
				Bpress1=2000;
			Bpress2=node->data[8]*8;
			if(Bpress2>2000)
				Bpress2=2000;
			Bpress3=node->data[9]*8;
			if(Bpress3>2000)
				Bpress3=2000;
			Alarm[0]=node->data[10];
			LDW=node->data[11];
			Thopen=node->data[12]*0.004;
			KeyOK=node->data[13]&0x03;
			KeyMode= (node->data[13]&0x04)>>2 ;
			KeyUp=	 (node->data[13]&0x08)>>3 ;
			KeyDown= (node->data[13]&0x10)>>4 ;
			KeyLeft= (node->data[13]&0x20)>>5 ;
			KeyRight= (node->data[13]&0x40)>>6 ;
			Alarm[1]=node->data[14];
			Alarm[2]=node->data[15];
			Alarm[3]=node->data[16];			
			Cutheme=node->data[17];
			Lineshow++;
		//	printf("%d Key1:%d,Tgear:%d,Rspeed:%f\n",Lineshow++,Key1,Tgear,Rspeed); 
			break;	
		case 0x52:	
			TraFuel=( node->data[3] + node->data[4]*256 )/10;
			AveFuel=( node->data[5] + node->data[6]*256 )/10;
			Alarm[4]=node->data[7];
			Alarm[5]=node->data[8];
			ECAS_Pos= node->data[9] ;
			ECAS_Load=( node->data[10] + node->data[11]*256 )*0.5;
			ACC_Gear= node->data[12] ;
			ACC_Dist= node->data[13] ;
			if(ACC_Dist>250)
				ACC_Dist=250;		
			TarVspeed= node->data[14] ;
			if(TarVspeed>250)
				TarVspeed=250;			
			CruiseVspeed= node->data[15] ;
			if(CruiseVspeed>250)
				CruiseVspeed=250;
			VarVspeed= node->data[16] ;
			if(VarVspeed>250)
				VarVspeed=250;
			RetarderTorq= node->data[17] *0.01-125;
			if(RetarderTorq>100)
				RetarderTorq=100;
			RetarderGear= node->data[18] ;
			PTOspeed=( node->data[19] + (node->data[20]&0x03)*256 )*10;
			if(PTOspeed>8000)
				PTOspeed=8000;
			BgtMode=(node->data[20]&0xc0)>>6;
			Alarm[6]=node->data[21];
			Alarm[7]=node->data[22];
			SectionSel=node->data[23];
			AlarmDis=node->data[24];
			Alarm[8]=node->data[25];
			Alarm[9]=node->data[26];
			Alarm[10]=node->data[27];
			Alarm[11]=node->data[28];
			Alarm[12]=node->data[29];
			Alarm[13]=node->data[30];
			Alarm[14]=node->data[31];	
			Alarm[15]=node->data[32];				
			A_M=node->data[33]&0x03;
			Cmode=(node->data[33]&0x04)>>2;
			Lmode=(node->data[33]&0x08)>>3;
			EPmode=(node->data[33]&0x30)>>4;
			Tgear_S=(node->data[33]&0xc0)>>6;
			Tgear=node->data[34]-125;
			Cgear=node->data[35]-125;
			AMTword=node->data[36];
			TyreWord=node->data[37]&0x0F;
			TyreMode=(node->data[37]&0xF0)>>4;
			Lighrigion1=node->data[38];
			Lighrigion2=node->data[39];
			Lighrigion3=node->data[40];
			Lighrigion4=node->data[41];	
			Alarm[16]=node->data[42];
			Vadjgoal=( node->data[43] + node->data[44]*256 )*0.125;
			if(Vadjgoal>8031.875)
				Vadjgoal=8031.875;	
			Alarm[17]=node->data[45];
			Lineshow++;
		//	printf("%d FuelUnit:%d,Alarm[0]:%x\n",Lineshow++,FuelUnit,Alarm[0]); 
			break;			
		case 0x61:
			SMcu=node->data[3];
			Lineshow++;
			//	printf("%d FuelUnit:%d,Alarm[0]:%x\n",Lineshow++,FuelUnit,Alarm[0]); 
			break;
		case 0x71:
			TotalTrip= node->data[3] + node->data[4] *256+ node->data[5] *256*256+ node->data[6]*256*256*256 ;
			if(TotalTrip>1999999)
			TotalTrip=1999999;
			SingleTrip=( node->data[7] + node->data[8] *256+ node->data[9] *256*256+ node->data[10]*256*256*256 )*0.1;
			if(SingleTrip>9999.9)
			SingleTrip=9999.9;
			CellVoltage=( node->data[11] + node->data[12]*256 )*0.1;
			CoolWater= node->data[13] -40;
			FuelLevel= node->data[14] *0.004;
			if(FuelLevel>1)
			FuelLevel=1;
			OilPress= node->data[15] *4;
			if(OilPress>1000)
			OilPress=1000;
			UreaLevel= node->data[16] *0.004;
			if(UreaLevel>1)
			UreaLevel=1;
			Sec= node->data[17] ;
			if(Sec>59)
			Sec=59;
			Min= node->data[18] ;
			if(Min>59)
			Min=59;
			Hour= node->data[19] ;
			if(Hour>23)
			Hour=23;
			Day= node->data[20] ;
			if(Day>31)
			Day=31;
			Month= node->data[21] ;
			if(Month>12)
			Month=12;
			Year= node->data[22] +2000;
			if(Year>2099)
			Year=2099;
			DriveTime_min= node->data[23] ;
			if(DriveTime_min>59)
				DriveTime_min=59;
			DriveTime_hour= node->data[24] ;
			if(DriveTime_hour>255)
				DriveTime_hour=255;
			Week= node->data[25] ;
			FuelConsump=(  node->data[28] + node->data[29] *256+ node->data[30] *256*256+ node->data[31]*256*256*256   )*0.1;
			Outertemp=( node->data[32] + node->data[33]*256 )/32-273;
			Outertempsh=node->data[34]&0x01;
			Score1= node->data[36] *0.01;
			if(Score1>1)
				Score1=1;
			Score2= node->data[37] *0.01;
			if(Score2>1)
				Score2=1;
			Score3= node->data[38] *0.01;
			if(Score3>1)
				Score3=1;
			Score4= node->data[39] *0.01;
			if(Score4>1)
				Score4=1;
			OverallScore= node->data[40] *0.01;
			if(OverallScore>1)
				OverallScore=1;
			UnitScore= node->data[41] *0.01;
			if(UnitScore>1)
				UnitScore=1;
			ScoreWord= node->data[42];		
			Lineshow++;
		//	printf("%d TotalTrip:%ld,Day:%d\n",Lineshow++,TotalTrip,Day); 
			break;
		case 0x72:
			Tyre_Pos=node->data[3];
			Tyre_Pres=node->data[4]+node->data[5]*256;
			if(Tyre_Pres> 64255 )
				Tyre_Pres= 64255 ;
			Tyre_St= node->data[6] ;
			Pswordtip=node->data[7];
			SecOil_Level= node->data[8] *0.004;
			if(SecOil_Level>1)
				SecOil_Level=1;
			AveVspeed= node->data[9];
			if(AveVspeed>250)
				AveVspeed=250;
			MaxVspeed= node->data[10];
			if(MaxVspeed>250)
				MaxVspeed=250;
			Alarm[18]=node->data[11];
			Range=node->data[12]+node->data[13]*256;
			if(Range> 999 )
				Range=999;			
			Engtime=(node->data[14] + node->data[15] *256+ node->data[16] *256*256+ node->data[17]*256*256*256)*0.05 ;
			if(Engtime>9999999.9)
				Engtime=9999999.9;	
			Tyre_Temp=(node->data[41]+node->data[42]*256)/32-273;
			if(Tyre_Temp>1734.96875)
				Tyre_Temp=1734.96875;			
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;		
		case 0xA0:
			totalmile1=node->data[3] + node->data[4] *256+ node->data[5] *256*256+ node->data[6]*256*256*256;
			totalmile2=node->data[7] + node->data[8] *256+ node->data[9] *256*256+ node->data[10]*256*256*256;
			totalmile3=node->data[11] + node->data[12] *256+ node->data[13] *256*256+ node->data[14]*256*256*256;
			totalmile4=node->data[15] + node->data[16] *256+ node->data[17] *256*256+ node->data[18]*256*256*256;
			totalmile5=node->data[19] + node->data[20] *256+ node->data[21] *256*256+ node->data[22]*256*256*256;
			totalmile6=node->data[23] + node->data[24] *256+ node->data[25] *256*256+ node->data[26]*256*256*256;
			totalmile7=node->data[27] + node->data[28] *256+ node->data[29] *256*256+ node->data[30]*256*256*256;
			totalmile8=node->data[31] + node->data[32] *256+ node->data[33] *256*256+ node->data[34]*256*256*256;
			totmilepec1=node->data[35]*0.004;
			if(totmilepec1>1)
				totmilepec1=1;
			totmilepec2=node->data[36]*0.004;
			if(totmilepec2>1)
				totmilepec2=1;
			totmilepec3=node->data[37]*0.004;
			if(totmilepec3>1)
				totmilepec3=1;
			totmilepec4=node->data[38]*0.004;
			if(totmilepec4>1)
				totmilepec4=1;
			totmilepec5=node->data[39]*0.004;
			if(totmilepec5>1)
				totmilepec5=1;
			totmilepec6=node->data[40]*0.004;
			if(totmilepec6>1)
				totmilepec6=1;
			totmilepec7=node->data[41]*0.004;
			if(totmilepec7>1)
				totmilepec7=1;
			totmilepec8=node->data[42]*0.004;
			if(totmilepec8>1)
				totmilepec8=1;
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;							
		case 0xA1:
			avespeed1=node->data[3];
			if(avespeed1>250)
				avespeed1=250;
			maxspeed1=node->data[4];
			if(maxspeed1>250)
				maxspeed1=250;
		    timemin1=node->data[5];
			if(timemin1>59)
				timemin1=59;
			timehour1=node->data[6] + node->data[7] *256+ node->data[8] *256*256;
			if(timehour1>99999)
				timehour1=99999;
			avespeed2=node->data[9];
			if(avespeed2>250)
				avespeed2=250;
			maxspeed2=node->data[10];
			if(avespeed2>250)
				avespeed2=250;
			timemin2=node->data[11];
			if(timemin2>59)
				timemin2=59;
			timehour2=node->data[12] + node->data[13] *256+ node->data[14] *256*256;
			if(timehour2>99999)
				timehour2=99999;
			avespeed3=node->data[15];
			if(avespeed3>250)
				avespeed3=250;
			maxspeed3=node->data[16];
			if(avespeed3>250)
				avespeed3=250;
			timemin3=node->data[17];
			if(timemin3>59)
				timemin3=59;
			timehour3=node->data[18] + node->data[19] *256+ node->data[20] *256*256;
			if(timehour3>99999)
				timehour3=99999;
			avespeed4=node->data[21];
			if(avespeed4>250)
				avespeed4=250;
			maxspeed4=node->data[22];
			if(avespeed4>250)
				avespeed4=250;
			timemin4=node->data[23];
			if(timemin4>59)
				timemin4=59;
			timehour4=node->data[24] + node->data[25] *256+ node->data[26] *256*256;
			if(timehour4>99999)
				timehour4=99999;
			avespeed5=node->data[27];
			if(avespeed5>250)
				avespeed5=250;
			maxspeed5=node->data[28];
			if(avespeed5>250)
				avespeed5=250;
			timemin5=node->data[29];
			if(timemin5>59)
				timemin5=59;
			timehour5=node->data[30] + node->data[31] *256+ node->data[32] *256*256;
			if(timehour5>99999)
				timehour5=99999;
			avespeed6=node->data[33];
			if(avespeed6>250)
				avespeed6=250;
			maxspeed6=node->data[34];
			if(avespeed6>250)
				avespeed6=250;
			timemin6=node->data[35];
			if(timemin6>59)
				timemin6=59;
			timehour6=node->data[36] + node->data[37] *256+ node->data[38] *256*256;
			if(timehour6>99999)
				timehour6=99999;
			avespeed7=node->data[39];
			if(avespeed7>250)
				avespeed7=250;
			maxspeed7=node->data[40];
			if(avespeed7>250)
				avespeed7=250;
			timemin7=node->data[41];
			if(timemin7>59)
				timemin7=59;
			timehour7=node->data[42] + node->data[43] *256+ node->data[44] *256*256;
			if(timehour7>99999)
				timehour7=99999;
			avespeed8=node->data[45];
			if(avespeed8>250)
				avespeed8=250;
			maxspeed8=node->data[46];
			if(avespeed8>250)
				avespeed8=250;
			timemin8=node->data[47];
			if(timemin8>59)
				timemin8=59;
			timehour8=node->data[48] + node->data[49] *256+ node->data[50] *256*256;
			if(timehour8>99999)
				timehour8=99999;		
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;							
		case 0xA2:
			Oilconme1=(node->data[3]+node->data[4]*256+node->data[5]*256*256+node->data[6]*256*256*256)*0.1;
			Aveoilconme1=(node->data[7]+node->data[8]*256)/10;
			Subtomile1=(node->data[9]+node->data[10]*256+node->data[11]*256*256+node->data[12]*256*256*256)*0.1;
			Oilconme2=(node->data[13]+node->data[14]*256+node->data[15]*256*256+node->data[16]*256*256*256)*0.1;
			Aveoilconme2=(node->data[17]+node->data[18]*256)/10;
			Subtomile2=(node->data[19]+node->data[20]*256+node->data[21]*256*256+node->data[22]*256*256*256)*0.1;
			Oilconme3=(node->data[23]+node->data[24]*256+node->data[25]*256*256+node->data[26]*256*256*256)*0.1;
			Aveoilconme3=(node->data[27]+node->data[28]*256)/10;
			Subtomile3=(node->data[29]+node->data[30]*256+node->data[31]*256*256+node->data[32]*256*256*256)*0.1;
			Oilconme4=(node->data[33]+node->data[34]*256+node->data[35]*256*256+node->data[36]*256*256*256)*0.1;
			Aveoilconme4=(node->data[37]+node->data[38]*256)/10;
			Subtomile4=(node->data[39]+node->data[40]*256+node->data[41]*256*256+node->data[42]*256*256*256)*0.1;
			Oilconme5=(node->data[43]+node->data[44]*256+node->data[45]*256*256+node->data[46]*256*256*256)*0.1;
			Aveoilconme5=(node->data[47]+node->data[48]*256)/10;
			Subtomile5=(node->data[49]+node->data[50]*256+node->data[51]*256*256+node->data[52]*256*256*256)*0.1;
			Oilconme6=(node->data[53]+node->data[54]*256+node->data[55]*256*256+node->data[56]*256*256*256)*0.1;
			Aveoilconme6=(node->data[57]+node->data[58]*256)/10;
			Subtomile6=(node->data[59]+node->data[60]*256+node->data[61]*256*256+node->data[62]*256*256*256)*0.1;
			Oilconme7=(node->data[63]+node->data[64]*256+node->data[65]*256*256+node->data[66]*256*256*256)*0.1;
			Aveoilconme7=(node->data[67]+node->data[68]*256)/10;
			Subtomile7=(node->data[69]+node->data[70]*256+node->data[71]*256*256+node->data[72]*256*256*256)*0.1;
			Oilconme8=(node->data[73]+node->data[74]*256+node->data[75]*256*256+node->data[76]*256*256*256)*0.1;
			Aveoilconme8=(node->data[77]+node->data[78]*256)/10;
			Subtomile8=(node->data[79]+node->data[80]*256+node->data[81]*256*256+node->data[82]*256*256*256)*0.1;
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;					
		case 0xA3:
			Alarmim=0;
			for(i=3;i<=9;i++)
			{
				if(node->data[i]!=0x00)
				{
					switch(node->data[i])
					{
						case 0x01:
							Alarmim=(i-3)*8+1;		
							break;
						case 0x02:
							Alarmim=(i-3)*8+2;
							break;
						case 0x04:
							Alarmim=(i-3)*8+3;
							break;
						case 0x08:
							Alarmim=(i-3)*8+4;
							break;
						case 0x10:
							Alarmim=(i-3)*8+5;
							break;
						case 0x20:
							Alarmim=(i-3)*8+6;
							break;
						case 0x40:
							Alarmim=(i-3)*8+7;
							break;
						case 0x80:
							Alarmim=(i-3)*8+8;
							break;
					}
					break;
				}				
			}			
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;	
		case 0xA4:
			Tomile=node->data[3]+node->data[4]*256+node->data[5]*256*256;
			if(Tomile>999000)
				Tomile=999000;
			Engmile=node->data[6]+node->data[7]*256+node->data[8]*256*256;
			if(Engmile>999000)
				Engmile=999000;
			Gebmile=node->data[9]+node->data[10]*256+node->data[11]*256*256;
			if(Gebmile>999000)
				Gebmile=999000;
			Draxmile=node->data[12]+node->data[13]*256+node->data[14]*256*256;
			if(Draxmile>999000)
				Draxmile=999000;
			Hubmile=node->data[15]+node->data[16]*256+node->data[17]*256*256;
			if(Hubmile>999000)
				Hubmile=999000;
			Fitmile=node->data[18]+node->data[19]*256+node->data[20]*256*256;
			if(Fitmile>999000)
				Fitmile=999000;
			Afitmile=node->data[21]+node->data[22]*256+node->data[23]*256*256;
			if(Afitmile>999000)
				Afitmile=999000;
			Ufitmile=node->data[24]+node->data[25]*256+node->data[26]*256*256;
			if(Ufitmile>999000)
				Ufitmile=999000;
			Sfitmile=node->data[27]+node->data[28]*256+node->data[29]*256*256;
			if(Sfitmile>999000)
				Sfitmile=999000;
			Dremile=node->data[30]+node->data[31]*256+node->data[32]*256*256;
			if(Dremile>999000)
				Dremile=999000;			
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;
							
		case 0xA5:
			for(i=0;i<=60;i++)
				Ems[i]=node->data[i+3];
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;
							
		case 0xA6:
			for(i=0;i<=60;i++)
				Vcu[i]=node->data[i+3];
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;
						
		case 0xA7:
			for(i=0;i<=14;i++)
				SVersion[i]=node->data[i+3];
			for(i=0;i<=14;i++)
				HVersion[i]=node->data[i+18];
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;
							
		case 0xA8:
			CTomile=(node->data[3]+node->data[4]*256)*1000;
			if(CTomile>999000)
				CTomile=999000;
			CEngmile=(node->data[5]+node->data[6]*256)*1000;
			if(CEngmile>999000)
				CEngmile=999000;
			CGebmile=(node->data[7]+node->data[8]*256)*1000;
			if(CGebmile>999000)
				CGebmile=999000;
			CDraxmile=(node->data[9]+node->data[10]*256)*1000;
			if(CDraxmile>999000)
				CDraxmile=999000;
			CHubmile=(node->data[11]+node->data[12]*256)*1000;
			if(CHubmile>999000)
				CHubmile=999000;
			CFitmile=(node->data[13]+node->data[14]*256)*1000;
			if(CFitmile>999000)
				CFitmile=999000;
			CAfitmile=(node->data[15]+node->data[16]*256)*1000;
			if(CAfitmile>999000)
				CAfitmile=999000;
			CUfitmile=(node->data[17]+node->data[18]*256)*1000;
			if(CUfitmile>999000)
				CUfitmile=999000;
			CSfitmile=(node->data[19]+node->data[20]*256)*1000;
			if(CSfitmile>999000)
				CSfitmile=999000;
			CDremile=(node->data[21]+node->data[22]*256)*1000;
			if(CDremile>999000)
				CDremile=999000;
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;
							
		case 0xA9:
			for(i=0;i<=35;i++)
				Devr[i]=node->data[i+3];
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;
							
		case 0xAA:
			for(i=0;i<=149;i++)
				Ics[i]=node->data[i+3];
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;
							
		case 0xB0:
			Resitem=node->data[3]&0x7F;
			Resval=(node->data[3]&0x80)>>7;
			Lineshow++;
		//	printf("%d Tyre_Pres:%d,AveVspeed:%d\n",Lineshow++,Tyre_Pres,AveVspeed); 
			break;		
		default:
			printf("feature_func error!\n");
			break;


	}

}

void serial_msg_decode()
{//解包
		SERIAL_MSG_NODE * tmp;			
		SERIAL_MSG_NODE * tofree;	
	//	int Line=0;
		
	//	gettimeofday(&tpstart2,NULL);

	
	pthread_mutex_lock(&serial_msg_mutex);
	tmp = serialMsgHead->next;
//	printf("tmp:%p;serialMsgHead->next:%p;serialMsgLast:%p\n",tmp,serialMsgHead->next,serialMsgLast);
	

	while(tmp != serialMsgLast)	
	{			
		assign_funcr(tmp);
		
	//	printf("\n%d",Line++);
	//	int j;
	//	for( j = 0; j < tmp->len; j++)
	//		printf( "%02X ", tmp->data[j]);

		tofree = tmp;	
		tmp=tmp->next;
		
		serialMsgHead->next = tmp;	
		
	//	if(tofree!=serialMsgLast)
	//	{
			free(tofree->data);
			free(tofree);
	//	}
	}	

	serialMsgLast->last=serialMsgHead;
	serialMsgHead->next=serialMsgLast;
	pthread_mutex_unlock(&serial_msg_mutex);	

//	gettimeofday(&tpend_2,NULL);
	//时间为微秒
//	timeuse=1000000*(tpend_2.tv_sec-tpstart2.tv_sec)+ tpend_2.tv_usec-tpstart2.tv_usec; 
	//时间为秒
	//	timeuse/=1000000;
//	printf("Used Time dec:%lf\n",timeuse);
}


void* threadr (void *arg)	
{
	pthread_detach(pthread_self());//将该子线程的状态设置为分离的
	int x7Eflag=0;//起始标志
	int fdr;
	int i = 0,j=0;

	int t=0,k=0;
	int n = 0;
	int Line = 0;
	
	BYTE sum=0x00;	
	BYTE read_buf[160];//暂存读取的数据,暂时先给一个初始长度20
	unsigned int size1=3000;
	BYTE READBUF[size1][160];
	unsigned int msgtype=20;
	//O_NOCTTY：告诉Unix这个程序不想成为“控制终端”控制的程序，不说明这个标志的话，任何输入都会影响你的程序。

//	FILE *fp;
	
//	if((fp=fopen("string","at+"))==NULL)
//	{
//		printf("Cannot open file strike any key exit!");
//		getch();
//		exit(1);
//	}
	
	fdr = open( "/dev/ttymxc2", O_RDWR | O_NOCTTY  );//阻塞模式
	if( fdr == -1 )
	{
		perror( "open serial 2\n" );
	}
	SetSpeed( fdr, B115200 );
	if(!SetParity( fdr, 8 , 1 , 'N' ))
		exit(1);	
	bzero(read_buf, sizeof(read_buf)); //置字节字符串s的前n个字节为零且包括‘\0’。
	bzero(rbuf, sizeof(rbuf));
	while(1)
	{ 
		while((n = read(fdr, read_buf, sizeof(read_buf)))>0)//读取数据，可能是一帧或一帧的一部分
		{ 
		//printf("comein\n");
			for( i = 0; i < n; i++)//逐个考察读取到的字符
			{		
				if(t>msgtype-1)//一帧长度为15,如果超出长度仍未读到结束符，则抛弃已读到的字符重新读取
				{
					x7Eflag=0;//重置开始标志
					t=0;
					sum=0;
					bzero(read_buf, sizeof(read_buf));
					bzero(rbuf, sizeof(rbuf));
					break;
				}
				if(t==2)
				{
					switch(rbuf[1])
					{
						case 0x51:
							msgtype=20;
							break;
						case 0x52:	
							msgtype=50;
							break;
						case 0x61:
							msgtype=10;
							break;
						case 0x71:
							msgtype=45;
							break;
						case 0x72:
							msgtype=45;
							break;
					//	case 0x81:
					//		msgtype=45;
					//		break;
					//	case 0x91:
					//		msgtype=45;
					//		break;
						case 0xA0:
							msgtype=45;
							break;
						case 0xA1:
							msgtype=53;
							break;
						case 0xA2:
							msgtype=85;
							break;
						case 0xA3:
							msgtype=12;
							break;
						case 0xA4:
							msgtype=35;
							break;
						case 0xA5:
							msgtype=66;
							break;
						case 0xA6:
							msgtype=66;
							break;
						case 0xA7:
							msgtype=35;
							break;
						case 0xA8:
							msgtype=25;
							break;
						case 0xA9:
							msgtype=41;
							break;
						case 0xAA:
							msgtype=155;
							break;
						case 0xB0:
							msgtype=6;
							break;
					/*	case 0x10:
							msgtype=15;
							break;
						case 0x20:
							msgtype=35;
							break;
					*/
						default:
							msgtype=0;
							break;
					}
					if(msgtype==0)
					{
						x7Eflag=0;
						t=0;
						sum=0x00;
						bzero(read_buf, sizeof(read_buf));
						bzero(rbuf, sizeof(rbuf));						
						break;
					}
				}
				if(read_buf[i]==0x7E && x7Eflag==0)//如果第一次遇到起始符
				{
					x7Eflag=1;//起始标志位置1
					rbuf[t++] = read_buf[i];				
				}
				else if(read_buf[i]==0x0D )//如果遇到结束符
				{		
					if (t==msgtype-1)//如果收到的字符数等于一帧的长度，则遇到的结束符是真正的结束符
					{
						if(sum==rbuf[msgtype-2]) //如果累加和也满足，则rbuf数组中的数据就是正确的，可输出
						{				  //如果累加和不满足，则rbuf数组中的数据就是错误的，不输出，抛弃已有数据重新接收
							rbuf[t]=0x0D; 
							
							serial_msg_rev(msgtype);//打包
						//////////////////////以下是把收到的数据包放在二维数组内，以便原样发回////////////////
						//	for( j = 0; j < msgtype; j++)
						//		READBUF[k][j]=rbuf[j];
						//	k++;
							
							printf("received_num:%d ",Line);
							Line++;
							for( j = 0; j < msgtype; j++)
								printf( "%02X ", rbuf[j]);
							printf("\n");
						//	printf("Alarmim:%d\n",Alarmim); 
						//	write(fdr, rbuf, sizeof(rbuf));
						//	fputs(rbuf,fp);	
						//	fwrite(rbuf,msgtype,1,fp);

						
						}
						x7Eflag=0;//重置开始标志
						t=0;
						sum=0;
						bzero(read_buf, sizeof(read_buf));
						bzero(rbuf, sizeof(rbuf));
						break;
					}
					
					else
					{
						rbuf[t++] = read_buf[i];
						if(t>=4)
							sum+=rbuf[t-1];//求累加和
					}
				}
				else//正常数据
				{					
					if(t == msgtype-2 ) //累加和位
						rbuf[t++] = read_buf[i];
					else if(t<msgtype-2)
					{
						rbuf[t++] = read_buf[i];
						if(t>=4)
							sum+=rbuf[t-1];//求累加和
					
					}
				}
       		}	
       		
		}
//		printf("%d\n",Lineshow);
//		Lineshow=0;
//		for(t=0;t<1000;t++)
//		{	for( j = 0; j < msgtype; j++)
//				printf( "%02X ", READBUF[t][j]);
//			printf("\n");
//			}
//		for( j = 0; j < msgtype; j++)
//		printf( "%02X ", READBUF[0][0]);
//		printf("k:%d\n",k);
		Line=0;
//		struct timeval temp1;
//		temp1.tv_sec = 0;//s
//		temp1.tv_usec = 50000;//us

	//////////////////////把收到的数据包原样发回//////////////////////////
	/*	if(READBUF[0][0]!=0x00)
		{
				for(k=0;k<size1;k++)
				{
					switch (READBUF[k][1])
					{
						case 0x51:
							msgtype=20;
							break;
						case 0x52:	
							msgtype=45;
							break;
						case 0x71:
							msgtype=45;
							break;
						case 0x72:
							msgtype=45;
							break;
						case 0x81:
							msgtype=45;
							break;
						case 0x91:
							msgtype=45;
							break;
						case 0xA0:
							msgtype=45;
							break;
						case 0xA1:
							msgtype=53;
							break;
						case 0xA2:
							msgtype=85;
							break;
						case 0xA3:
							msgtype=12;
							break;
						case 0xA4:
							msgtype=35;
							break;
						case 0xA5:
							msgtype=66;
							break;
						case 0xA6:
							msgtype=66;
							break;
						case 0xA7:
							msgtype=13;
							break;
						case 0xB0:
							msgtype=6;
							break;
					/*	case 0x10:
							msgtype=15;
							break;
						case 0x20:
							msgtype=35;
							break;
					*/
	/*					default:
							break;
					}
					if(write(fdr, READBUF[k], msgtype)==-1)
					{
						printf("write error\n");
						break;
					}
					usleep(50000);
				}
				bzero( READBUF, sizeof( READBUF));
				printf("finish_k:%d\n",k);
		}
		

*/
		
	}	
	close( fdr );
	Line=0;

	pthread_mutex_destroy(&serial_msg_mutex);
	return arg;
	
}


void serial_msg_fillw(BYTE datatype)
{//装填并打包
	int i;
	BYTE sum=0;
	unsigned int msgtype=25;
	bzero(write_buf, sizeof(write_buf)); 
	switch (datatype)
	{
		case 0x10:
			msgtype=15;
			write_buf[0]=0x7E;
			write_buf[1]=0x10;
			write_buf[2]=0x0A;
			write_buf[3]=(Senden|Themseude<<7);
			write_buf[4]=Itresh;
			write_buf[5]=KeyState;
			write_buf[6]=KeyMode_St;
			write_buf[7]=Code_Low;
			write_buf[8]=Code_High;
			write_buf[9]=0x00;
			write_buf[10]=Itset0;	
			write_buf[11]=Itset1;
			write_buf[12]=Itset2;
			for(i=0;i<msgtype-5;i++)
				sum=sum+write_buf[i+3];
			write_buf[13]=sum;
			write_buf[14]=0x0D;
			break;
		case 0x20:
			msgtype=6;
			write_buf[0]=0x7E;
			write_buf[1]=0x20;
			write_buf[2]=0x01;
			write_buf[3]=Requeitm;
			for(i=0;i<msgtype-5;i++)
				sum=sum+write_buf[i+3];
			write_buf[4]=sum;
			write_buf[5]=0x0D;
			break;
		case 0x30:
			msgtype=10;
			write_buf[0]=0x7E;
			write_buf[1]=0x30;
			write_buf[2]=0x05;
			write_buf[3]=Yeset;
			write_buf[4]=Mtset;
			write_buf[5]=Daset;
			write_buf[6]=Hrset;
			write_buf[7]=Meset;
			for(i=0;i<msgtype-5;i++)
				sum=sum+write_buf[i+3];
			write_buf[8]=sum;
			write_buf[9]=0x0D;
			break;
		case 0x31:
			msgtype=25;
			write_buf[0]=0x7E;
			write_buf[1]=0x31;
			write_buf[2]=0x14;
			write_buf[3]=(STomile/1000)%256;	
			write_buf[4]=( (STomile/1000) /256)%256;
			write_buf[5]=(SEngmile/1000)%256;	
			write_buf[6]=((SEngmile/1000)/256)%256;
			write_buf[7]=(SGebmile/1000)%256;	
			write_buf[8]=((SGebmile/1000)/256)%256;
			write_buf[9]=(SDraxmile/1000)%256;	
			write_buf[10]=((SDraxmile/1000)/256)%256;
			write_buf[11]=(SHubmile/1000)%256;	
			write_buf[12]=((SHubmile/1000)/256)%256;
			write_buf[13]=(SFitmile/1000)%256;	
			write_buf[14]=((SFitmile/1000)/256)%256;
			write_buf[15]=(SAfitmile/1000)%256;	
			write_buf[16]=((SAfitmile/1000)/256)%256;
			write_buf[17]=(SUfitmile/1000)%256;	
			write_buf[18]=((SUfitmile/1000)/256)%256;
			write_buf[19]=(SSfitmile/1000)%256;	
			write_buf[20]=((SSfitmile/1000)/256)%256;
			write_buf[21]=(SDremile/1000)%256;	
			write_buf[22]=((SDremile/1000)/256)%256;
			for(i=0;i<msgtype-5;i++)
				sum=sum+write_buf[i+3];
			write_buf[23]=sum;
			write_buf[24]=0x0D;
			break;
		case 0x32:
			msgtype=6;
			write_buf[0]=0x7E;
			write_buf[1]=0x32;
			write_buf[2]=0x01;
			write_buf[3]=0x00;
			for(i=0;i<msgtype-5;i++)
				sum=sum+write_buf[i+3];
			write_buf[4]=sum;
			write_buf[5]=0x0D;
			break;
		case 0x33:
			msgtype=6;
			write_buf[0]=0x7E;
			write_buf[1]=0x33;
			write_buf[2]=0x01;
			write_buf[3]=Tyrepos;
			for(i=0;i<msgtype-5;i++)
				sum=sum+write_buf[i+3];
			write_buf[4]=sum;
			write_buf[5]=0x0D;
			break;
		case 0x34:
			msgtype=7;
			write_buf[0]=0x7E;
			write_buf[1]=0x34;
			write_buf[2]=0x02;
			write_buf[3]=Retypepos;
			write_buf[4]=Retyrepre/8;
			for(i=0;i<msgtype-5;i++)
				sum=sum+write_buf[i+3];
			write_buf[5]=sum;
			write_buf[6]=0x0D;
			break;
		case 0x35:
			msgtype=6;
			write_buf[0]=0x7E;
			write_buf[1]=0x35;
			write_buf[2]=0x01;
			write_buf[3]=Loudspk|(LED<<1);
			for(i=0;i<msgtype-5;i++)
				sum=sum+write_buf[i+3];
			write_buf[4]=sum;
			write_buf[5]=0x0D;
			break;
		default:
			printf("serial_msg_fillw!\n");
			break;
	}
	
	SERIAL_MSG_NODE* node=(SERIAL_MSG_NODE *)malloc(sizeof(SERIAL_MSG_NODE));
	node->len = msgtype;
	node->data = (BYTE*)malloc(sizeof(BYTE) * msgtype);

	for(i=0;i<msgtype;i++)
	{
		node->data[i]=write_buf[i];
	} 
	
	SERIAL_MSG_NODE * tmp;
	pthread_mutex_lock(&serial_msg_mutexw);

	serialMsgLastw->last->next=node;
	node->next=serialMsgLastw;//serialMsgLastw总是指向最后一个分配的节点
	node->last=serialMsgLastw->last;
	serialMsgLastw->last=node;
	
	pthread_mutex_unlock(&serial_msg_mutexw);


}

void serial_msg_decodew()
{//解包并发送
	int fdw;
	fdw = open( "/dev/ttymxc2", O_RDWR | O_NOCTTY  );//阻塞模式
	if( fdw == -1 )
	{
		perror( "open serial 2\n" );
	}
	SetSpeed( fdw, B115200 );
	if(!SetParity( fdw, 8 , 1 , 'N' ))
		exit(1);	
	
	SERIAL_MSG_NODE * tmp;			
	SERIAL_MSG_NODE * tofree;	
	unsigned int msgtype=25;
	
	pthread_mutex_lock(&serial_msg_mutexw);
	tmp = serialMsgHeadw->next;

	while(tmp != serialMsgLastw)	
	{			
		switch (tmp->data[1])
		{
			
			case 0x10:
				msgtype=15;
				break;
			case 0x20:
				msgtype=6;
				break;
			case 0x30:
				msgtype=10;
				break;
			case 0x31:
				msgtype=25;
				break;
			case 0x32:
				msgtype=6;
				break;
			case 0x33:
				msgtype=6;
				break;
			case 0x34:
				msgtype=7;
				break;
			case 0x35:
				msgtype=6;
				break;
			default:
				printf("serial_msg_decodew!\n");
				break;
			
		}
	
		write(fdw, write_buf, msgtype);
		
		printf("write_num:%d\n",con++);
		
		tofree = tmp;	
		tmp=tmp->next;
		
		serialMsgHeadw->next = tmp;	
		
//		if(tofree!=serialMsgLastw)
//		{
		free(tofree->data);
		free(tofree);
//		}
	}	

	serialMsgLastw->last=serialMsgHeadw;
	serialMsgHeadw->next=serialMsgLastw;
	pthread_mutex_unlock(&serial_msg_mutexw);	
	
	close( fdw );
}


void serial_msg_spsend(BYTE datatype)
{//发送指定帧
	serial_msg_fillw(datatype);//装填并打包函数
	serial_msg_decodew();//解包并发送

}



void* threadw (void *arg)
{//线程固定频率发送0x10短帧
	pthread_detach(pthread_self());//将该子线程的状态设置为分离的	
	BYTE datatype=0x01;//需要发送的帧的数据类型
	int k=0;
//	while(1)
//	{//每隔100ms发送一次数据
//	for(k=0;k<100;k++)
//	{
		serial_msg_spsend(datatype);				
		usleep(100000);
//		}
//	}	
	return arg;
}





void main()
{
	serial_init();
	pthread_t th;
	int retread,retwrite;
	retread = pthread_create(&th,NULL,threadr,NULL);
	if( retread != 0 ){
		printf( "Create?threadr?error!\n");
	}
//	retwrite = pthread_create(&th,NULL,threadw,NULL);
//	if( retwrite != 0 ){
//		printf( "Create?threadw?error!\n");
//	}



////////////对要发送的数据进行赋值////////
	//核心板发送数据短帧
	Senden=1;
	Themseude=2;
	Itresh=3;
	KeyState=4;
	KeyMode_St=5;
	Code_Low=6;
	Code_High=7;
	Itset0=8;
	Itset1=9;
	Itset2=10;

//配置请求帧
	Requeitm=11; 	
//时间设置信息帧
	Yeset=12;
	Mtset=1;
	Daset=2;
	Hrset=3;
	Meset=4;

//保养设置信息帧
	STomile=1;
	SEngmile=2;
	SGebmile=3;
	SDraxmile=4;
	SHubmile=5;
	SFitmile=6;
	SAfitmile=7;
	SUfitmile=8;
	SSfitmile=9;
	SDremile=1;

//胎压自学习设置信息帧
	Tyrepos=2;

//胎压参考值设置信息帧
	Retypepos=3;
	Retyrepre=4;

//仪表自检设置信息帧
	Loudspk=5;
	LED=6;
/////////////////////////////////////
//	while(1)
//	{
	//	serial_msg_spsend(0x10);
	//	usleep(100000);
		Requeitm=4;
		serial_msg_spsend(0x20);
		usleep(100000);
		Requeitm=4;
		serial_msg_spsend(0x20);
		usleep(100000);
//	}

/*
char c;
	while(1)
	{
		c=getchar();
		switch(c)
		{
			case '0'://核心板发送短帧
				serial_msg_spsend(0x10);
				break;
			case '1'://燃油泄漏信息
				Requeitm=1;
				serial_msg_spsend(0x20);
				break;				
			case '2'://行驶信息
				Requeitm=2;
				serial_msg_spsend(0x20);
				break;		
			case '3'://油耗历史信息
				Requeitm=3;
				serial_msg_spsend(0x20);
				break;		
			case '4'://报警事件信息
				Requeitm=4;
				serial_msg_spsend(0x20);
				break;		
			case '5'://保养查询信息
				Requeitm=5;
				serial_msg_spsend(0x20);
				break;				
			case '6'://EMS故障码信息
				Requeitm=6;
				serial_msg_spsend(0x20);
				break;	
			case '7'://VCU故障码信息
				Requeitm=7;
				serial_msg_spsend(0x20);
				break;	
			case '8'://底板软件版本信息
				Requeitm=8;
				serial_msg_spsend(0x20);
				break;				
			case '9'://用户自定义保养当前设置信息
				Requeitm=9;
				serial_msg_spsend(0x20);
				break;				
			case 'p'://开发者模式信息
				Requeitm=10;
				serial_msg_spsend(0x20);
				break;
			case 'o'://IC故障码信息
				Requeitm=11;
				serial_msg_spsend(0x20);
				break;
			case 't'://时间设置信息帧
				serial_msg_spsend(0x30);
				break;
			case 'r'://保养设置信息帧
				serial_msg_spsend(0x31);
				break;
			case 'x'://预留
				serial_msg_spsend(0x32);
				break;
			case 's'://胎压自学习设置信息帧
				serial_msg_spsend(0x33);
				break;
			case 'f'://胎压参考值设置信息帧
				serial_msg_spsend(0x34);
				break;
			case 'v'://仪表自检设置信息帧
				serial_msg_spsend(0x35);
				break;	
			
			}

		serial_msg_decode();//收到后，在threadr里面打包，在此处解包
		}
		*/


}























