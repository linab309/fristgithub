#include "DS18B20.h"

UINT8 id[8];
UINT8 sensor_data_value[2];
UINT8 ch[9];
UINT8 flag; 

/*
cc2430的1个机器周期 = 1个振荡周期
在32MHz主频下工作：
1个振荡周期 = 1/32  uS ，即 1个机器周期也是1/32 uS
所以如果执行一条减法指令需要1个机器周期，那么就可以通过计数来实现定时
*/

#pragma optimize=none
void Delay_nus(UINT16 s) 
{  
  while (s--)
  {
    asm("NOP");
    asm("NOP");
    asm("NOP");
    
    asm("NOP");
    asm("NOP");
    asm("NOP");
  }

}

void init_1820(void) //初始化18b20
{
  SET_OUT; 
  SET_DQ;//输出1
  CL_DQ; 
  Delay_nus(550);//拉低一段时间 550
  SET_DQ;//释放 
  SET_IN;//输入 
  Delay_nus(20); //释放总线后等待15-60us 40
  while(IN_DQ)  {;}//等待回复 
  Delay_nus(240);//回复的低电平在60到240us  
  SET_OUT; 
  SET_DQ;//回到初始DQ=1；
}

void write_1820(UINT8 x)//往18b20写一个字节数据
{  
  UINT8 m;    
  SET_OUT;
  for(m=0;m<8;m++)  
  {  
     CL_DQ;
     if(x&(1<<m))    //写数据，从低位开始  
     {
      SET_DQ; 
     }
     else  
     {
      CL_DQ;
     }
     Delay_nus(40);   //15~60us  
     SET_DQ;  
  }
  SET_DQ;  
} 




UINT8 read_1820(void)  //从18b20读一个数据
{   
  UINT8 temp,k,n;    
  temp=0;  
  for(n=0;n<8;n++)  
  {  
    CL_DQ;     
    SET_DQ; 
    SET_IN;  
    k=IN_DQ;    //读数据,从低位开始  
    if(k)  
    temp|=(1<<n);  
    else  
    temp&=~(1<<n);  
    Delay_nus(70); //60~120us      70
    SET_OUT;
  
  }  
  return (temp);  
}  
//搜索ID
void get_id()//获得18b20的id号
{     
  init_1820();        //resert
  write_1820(0x33); 
  
  for(unsigned char ii=0;ii<8;ii++)
  {
    id[ii]=read_1820();
  }
}

/*功能匹配*/
UINT8 match_rom(UINT8 *rom);
UINT8 match_rom(UINT8 *rom)
{
  unsigned char i;
  init_1820();
  write_1820(0x55);
  for(i=8;i>0;i--)
  {
          write_1820(*(rom+i-1));
  }
  return 1;
}
 
void read_data(UINT8 * pSensorValue)//读取温度数据
{
  UINT8 temh,teml; 
  UINT8 a,b,c;
  int j=0;

  init_1820();  //复位18b20  
  write_1820(0xcc);   // 发出转换命令 搜索器件 
  write_1820(0x44);     //启动  
  //Delay_nus(50000);
  for(j=20;j>1;j--)  
    Delay_nus(500);
  init_1820();  
  write_1820(0xcc);
  write_1820(0xbe); 
       
  teml=read_1820();  //读数据  
  temh=read_1820();  
  if(temh&0x80)//判断正负
  {
    flag=1;
    c=0;
    c=c|temh;
    c=c&0x00ff;
    c=c<<8;
    a=c;

    a=c|teml;

    a=(a^0xffff);//异或
    a=a+1;  //取反加1
    teml=a&0x0f;
    temh=a>>4;
  }
  else
  {
    flag=0;   //为正
    a=temh<<4;
    a+=(teml&0xf0)>>4; //得到高位的值
    b=teml&0x0f;
    temh=a;
    teml=b&0x00ff;
  }
  pSensorValue[0]=teml;
  pSensorValue[1]=temh;
}

    
////////////处理数据部分/////////////////
void DataChange(UINT8 * pSensorValue,UINT8 * pChBuf)
{
  UINT8 temh,teml;
  //UINT8 ch[9];
  UINT16 num;
  //UINT8 i;
  
  teml=pSensorValue[0];
  temh=pSensorValue[1];
  //i=(Temp_channel-0x31)*2;
  pChBuf[0]='T';//当前显示的传感器的编号
  pChBuf[1]=':';
  num=teml*625;          //小数部分的取值每位代表0.0625（精度）
  if(flag==1)            //判断正负温度
  {
    pChBuf[2]='-';              //+0x2d 为变"-"ASCII码
  }
  else 
    pChBuf[2]='+';   
  if(temh/100==0)
    pChBuf[3]=' ';
  else
    pChBuf[3]=temh/100+0x30;      //+0x30 为变 0~9 ASCII码
  if((temh/10%10==0)&&(temh/100==0))
   pChBuf[4]=' ';
  else
    pChBuf[4]=temh/10%10+0x30;
  pChBuf[5]=temh%10+0x30;
  pChBuf[6]='.';
  pChBuf[7]=num/1000+0x30;//忽略小数点后1位的数
  pChBuf[8]='\0';
  //Print(i,15,ch,1);   //显示温度值
  //Print(i,80,"℃",1); //显示摄氏度标示  ℃
  //此处添加具体的显示代码
  //LCD_write_english_string(8,1,(char *)ch);
}

void ds18b20_main()
{
 
  while(1)
  {
    read_data(sensor_data_value);// 读取温度
    DataChange(sensor_data_value,ch); //数据处理
    //LCD_write_english_string(8,1,(char *)ch);
    
    Delay_nus(10000);
  }
}



