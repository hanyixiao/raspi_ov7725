/********************************************************************************
 * raspi_ov7725.cpp
 * Create by hanyixiao on 2020 4 23
 * Copyright (c) 2020 yixiaoHan.  All rights reserved.
 * hanyixiao@github.com
 * target arm-v7a rasbberypi-4B
 *******************************************************************************/    
#include "raspi_ov7725.h"
#include "stdio.h"
extern "C"
{
#include "raspi_io.h"   
#include "convert.h"
}
#include "raspi_sccb.h"
#include <wiringPi.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static int DataLine[8]={
    17,27,22,10,9,11,0,5
};
#define FIFO_OE_L set_gpio_value(OV7725_FIFO_OE_PIN,0)
#define FIFO_OE_H set_gpio_value(OV7725_FIFO_OE_PIN,1)
#define FIFO_WE_L set_gpio_value(OV7725_FIFO_WE_PIN,0)
#define FIFO_WE_H set_gpio_value(OV7725_FIFO_WE_PIN,1)
#define FIFO_RCLK_L  set_gpio_value(OV7725_FIFO_RCLK_PIN,0)
#define FIFO_RCLK_H  set_gpio_value(OV7725_FIFO_RCLK_PIN,1)
#define FIFO_RRST_L  set_gpio_value(OV7725_FIFO_RRST_PIN,0)
#define FIFO_RRST_H  set_gpio_value(OV7725_FIFO_RRST_PIN,1)
#define FIFO_WRST_L  set_gpio_value(OV7725_FIFO_WRST_PIN,0)
#define FIFO_WRST_H  set_gpio_value(OV7725_FIFO_WRST_PIN,1)
int Ov7725_vsync=0;
regval_list sensor_config[]={
    {REG_CLKRC,     0x00}, /*clock config*/
	{REG_COM7,      0x46}, /*QVGA RGB565 */
	{REG_HSTART,    0x3f},
	{REG_HSIZE,     0x50},
	{REG_VSTRT,     0x03},
	{REG_VSIZE,     0x78},
	{REG_HREF,      0x00},
	{REG_HOutSize,  0x50},
	{REG_VOutSize,  0x78},
	{REG_EXHCH,     0x00},
	

	/*DSP control*/
	{REG_TGT_B,     0x7f},
	{REG_FixGain,   0x09},
	{REG_AWB_Ctrl0, 0xe0},
	{REG_DSP_Ctrl1, 0xff},
	{REG_DSP_Ctrl2, 0x20},
	{REG_DSP_Ctrl3,	0x00},
	{REG_DSP_Ctrl4, 0x00},

	/*AGC AEC AWB*/
	{REG_COM8,		0xf0},
	{REG_COM4,		0x81}, /*Pll AEC CONFIG*/
	{REG_COM6,		0xc5},
	{REG_COM9,		0x21},
	{REG_BDBase,	0xFF},
	{REG_BDMStep,	0x01},
	{REG_AEW,		0x34},
	{REG_AEB,		0x3c},
	{REG_VPT,		0xa1},
	{REG_EXHCL,		0x00},
	{REG_AWBCtrl3,  0xaa},
	{REG_COM8,		0xff},
	{REG_AWBCtrl1,  0x5d},

	{REG_EDGE1,		0x0a},
	{REG_DNSOff,	0x01},
	{REG_EDGE2,		0x01},
	{REG_EDGE3,		0x01},

	{REG_MTX1,		0x5f},
	{REG_MTX2,		0x53},
	{REG_MTX3,		0x11},
	{REG_MTX4,		0x1a},
	{REG_MTX5,		0x3d},
	{REG_MTX6,		0x5a},
	{REG_MTX_Ctrl,  0x1e},

	{REG_BRIGHT,	0x00},
	{REG_CNST,		0x25},
	{REG_USAT,		0x65},
	{REG_VSAT,		0x65},
	{REG_UVADJ0,	0x81},
	//{REG_SDE,		  0x20},	//黑白
	{REG_SDE,		  0x06},	//彩色	调节SDE这个寄存器还可以实现其他效果
	
    /*GAMMA config*/
	{REG_GAM1,		0x0c},
	{REG_GAM2,		0x16},
	{REG_GAM3,		0x2a},
	{REG_GAM4,		0x4e},
	{REG_GAM5,		0x61},
	{REG_GAM6,		0x6f},
	{REG_GAM7,		0x7b},
	{REG_GAM8,		0x86},
	{REG_GAM9,		0x8e},
	{REG_GAM10,		0x97},
	{REG_GAM11,		0xa4},
	{REG_GAM12,		0xaf},
	{REG_GAM13,		0xc5},
	{REG_GAM14,		0xd7},
	{REG_GAM15,		0xe8},
	{REG_SLOP,		0x20},

	{REG_HUECOS,	0x80},
	{REG_HUESIN,	0x80},
	{REG_DSPAuto,	0xff},
	{REG_DM_LNL,	0x00},
	{REG_BDBase,	0x99},
	{REG_BDMStep,	0x03},
	{REG_LC_RADI,	0x00},
	{REG_LC_COEF,	0x13},
	{REG_LC_XC,		0x08},
	{REG_LC_COEFB,  0x14},
	{REG_LC_COEFR,  0x17},
	{REG_LC_CTR,	0x05},
	
	{REG_COM3,		0xd0},/*Horizontal mirror image*/

	/*night mode auto frame rate control*/
	{REG_COM5,		0xf5},	 /*在夜视环境下，自动降低帧率，保证低照度画面质量*/
	//{REG_COM5,
    {0xff,         0xff},/*END marker*/		
};
static void WriteSensorRegs(const struct regval_list reglist[])
{
    const struct regval_list *next = reglist;
	for(;;){
		uint8_t reg_addr = (next->reg_num);
		uint8_t reg_val = (next->value);
		if((reg_addr==255)&&(reg_val==255))
			break;
		while(0==SCCB_Write_Reg(reg_addr, reg_val))
        {
            printf("[error]:failed while write the reg\n");
            printf("[error]:reg is %x data is %x \n",reg_addr,reg_val);
        }
		next++;
	}
}
static void FIFO_GPIO_config()
{
	gpio_init();
	set_gpio_fsel(OV7725_FIFO_OE_PIN,FUNC_OP);
	set_gpio_fsel(OV7725_FIFO_RCLK_PIN,FUNC_OP);
	set_gpio_fsel(OV7725_FIFO_RRST_PIN,FUNC_OP);
	set_gpio_fsel(OV7725_FIFO_WE_PIN,FUNC_OP);
	set_gpio_fsel(OV7725_FIFO_WRST_PIN,FUNC_OP);

	set_gpio_value(OV7725_FIFO_OE_PIN,0);
	set_gpio_value(OV7725_FIFO_RCLK_PIN,0);
	set_gpio_value(OV7725_FIFO_RRST_PIN,0);
	set_gpio_value(OV7725_FIFO_WE_PIN,0);
	set_gpio_value(OV7725_FIFO_WRST_PIN,0);
	FIFO_OE_L;
	FIFO_WE_H;	
}
static void All_gpio_set_Op()
{
	gpio_init();	
    for(int i=0;i<27;i++)
    {
        set_gpio_fsel(i,FUNC_OP);
		gpio_set_pull(i,PULL_DOWN);
        delay_ms(100);
        set_gpio_value(i,0);
    }
}
static uint8_t Ov7725_init(void)
{
    #define OV7725_ID 0x21
    uint8_t id=0x00;
    SCCB_GPIO_init();
    if(0 == SCCB_Write_Reg(0x12,0x80))
    {
        return 0;
    }
    if(0 == SCCB_Read_Reg(REG_VER,&id))
    {
        return 0;
    }
    if(id == OV7725_ID)
    {
		printf("[info]:ID is %x\n",id);
        WriteSensorRegs(sensor_config);
    }

    return 1;
} 

static void VSYNC_ISR()
{
	printf("VSYNC interrupts\n");
    if(Ov7725_vsync == 0 )
    {
        FIFO_WRST_L; 	                      //拉低使FIFO写(数据from摄像头)指针复位
        FIFO_WE_H;	                        //拉高使FIFO写允许
            
        Ov7725_vsync = 1;	   	
        FIFO_WE_H;                          //使FIFO写允许
        FIFO_WRST_H;                        //允许使FIFO写(数据from摄像头)指针运动
    }
    else if(Ov7725_vsync == 1 )
    {
        FIFO_WE_L;                          //拉低使FIFO写暂停
        Ov7725_vsync = 2;
    }    	
}
static void set_wiringpi_isr()
{
	//setup to wiringpi library
	if(wiringPiSetup()<0){
		fprintf(stderr,"Uable to setup wiringPi:%s\n",strerror(errno));
		return ;
	}
	pinMode(OV7725_VSYNC_PIN,INPUT);
	pullUpDnControl(OV7725_VSYNC_PIN,PUD_UP);
	if(wiringPiISR(OV7725_VSYNC_PIN,INT_EDGE_FALLING,&VSYNC_ISR)<0){
		fprintf(stderr,"Uable to setup isr\n",strerror(errno));
		return ;
	}

}
static void data_gpio_set()
{
	for(int i=0;i<8;i++)
	{
		set_gpio_fsel(DataLine[i],FUNC_IP);
		gpio_set_pull(DataLine[i],PULL_DOWN);
	}
}
static void OV7725_Special_Effect(uint8_t eff)
{
	switch(eff)
	{
		case 0://正常
			SCCB_Write_Reg(REG_SDE,0x06);
			SCCB_Write_Reg(REG_UFix,0x80);
			SCCB_Write_Reg(REG_VFix,0x80);
			break;
		case 1://黑白
			//SCCB_Write_Reg(REG_SDE,0x26);
			SCCB_Write_Reg(REG_SDE,0x20);
			SCCB_Write_Reg(REG_UFix,0x80);
			SCCB_Write_Reg(REG_VFix,0x80);
			break;
		case 2://偏蓝
			SCCB_Write_Reg(REG_SDE,0x1e);
			SCCB_Write_Reg(REG_UFix,0xa0);
			SCCB_Write_Reg(REG_VFix,0x40);
			break;
		case 3://复古
			SCCB_Write_Reg(REG_SDE,0x1e);
			SCCB_Write_Reg(REG_UFix,0x40);
			SCCB_Write_Reg(REG_VFix,0xa0);
			break;
		case 4:
			SCCB_Write_Reg(REG_SDE,0x46);
			break;
		default:
			fprintf(stderr,"Specail Effect Error\n",strerror(errno));
			break;
	}
}
static void OV7725_Light_Mode(uint8_t mode)
{
	switch(mode)
	{
		case 0:	//Auto，自动模式
			SCCB_Write_Reg(0x13, 0xff); //AWB on 
			SCCB_Write_Reg(0x0e, 0x65);
			SCCB_Write_Reg(0x2d, 0x00);
			SCCB_Write_Reg(0x2e, 0x00);
			break;
		case 1://sunny，晴天
			SCCB_Write_Reg(0x13, 0xfd); //AWB off
			SCCB_Write_Reg(0x01, 0x5a);
			SCCB_Write_Reg(0x02, 0x5c);
			SCCB_Write_Reg(0x0e, 0x65);
			SCCB_Write_Reg(0x2d, 0x00);
			SCCB_Write_Reg(0x2e, 0x00);
			break;	
		case 2://cloudy，多云
			SCCB_Write_Reg(0x13, 0xfd); //AWB off
			SCCB_Write_Reg(0x01, 0x58);
			SCCB_Write_Reg(0x02, 0x60);
			SCCB_Write_Reg(0x0e, 0x65);
			SCCB_Write_Reg(0x2d, 0x00);
			SCCB_Write_Reg(0x2e, 0x00);
			break;	
		case 3://office，办公室
			SCCB_Write_Reg(0x13, 0xfd); //AWB off
			SCCB_Write_Reg(0x01, 0x84);
			SCCB_Write_Reg(0x02, 0x4c);
			SCCB_Write_Reg(0x0e, 0x65);
			SCCB_Write_Reg(0x2d, 0x00);
			SCCB_Write_Reg(0x2e, 0x00);
			break;	
		case 4://home，家里
			SCCB_Write_Reg(0x13, 0xfd); //AWB off
			SCCB_Write_Reg(0x01, 0x96);
			SCCB_Write_Reg(0x02, 0x40);
			SCCB_Write_Reg(0x0e, 0x65);
			SCCB_Write_Reg(0x2d, 0x00);
			SCCB_Write_Reg(0x2e, 0x00);
			break;	
		
		case 5://night，夜晚
			SCCB_Write_Reg(0x13, 0xff); //AWB on
			SCCB_Write_Reg(0x0e, 0xe5);
			break;	
		
		default:
			printf("Light Mode parameter error!"); 

			break;
	}

}	

/**
  * @brief  设置饱和度
  * @param  sat:饱和度,参数范围[-4 ~ +4]             	
  * @retval 无
  */
static void OV7725_Color_Saturation(int8_t sat)
{

 	if(sat >=-4 && sat<=4)
	{	
		SCCB_Write_Reg(REG_USAT, (sat+4)<<4); 
		SCCB_Write_Reg(REG_VSAT, (sat+4)<<4);
	}
	else
	{
		printf("Color Saturation parameter error!");
	}
	
}
/**
  * @brief  设置光照度
  * @param  bri:光照度，参数范围[-4~+4]
  * @retval 无
  */
static void OV7725_Brightness(int8_t bri)
{
	uint8_t BRIGHT_Value,SIGN_Value;	
	
	switch(bri)
	{
		case 4:
				BRIGHT_Value = 0x48;
				SIGN_Value = 0x06;
			break;
		
		case 3:
				BRIGHT_Value = 0x38;
				SIGN_Value = 0x06;		
		break;	
		
		case 2:
				BRIGHT_Value = 0x28;
				SIGN_Value = 0x06;			
		break;	
		
		case 1:
				BRIGHT_Value = 0x18;
				SIGN_Value = 0x06;			
		break;	
		
		case 0:
				BRIGHT_Value = 0x08;
				SIGN_Value = 0x06;			
		break;	
		
		case -1:
				BRIGHT_Value = 0x08;
				SIGN_Value = 0x0e;		
		break;	
		
		case -2:
				BRIGHT_Value = 0x18;
				SIGN_Value = 0x0e;		
		break;	
		
		case -3:
				BRIGHT_Value = 0x28;
				SIGN_Value = 0x0e;		
		break;	
		
		case -4:
				BRIGHT_Value = 0x38;
				SIGN_Value = 0x0e;		
		break;	
		
		default:
			printf("Brightness parameter error!");
			break;
	}

		SCCB_Write_Reg(REG_BRIGHT, BRIGHT_Value); //AWB on
		SCCB_Write_Reg(REG_SIGN, SIGN_Value);
}		
/**
  * @brief  设置对比度
  * @param  cnst:对比度，参数范围[-4~+4]
  * @retval 无
  */
static void OV7725_Contrast(int8_t cnst)
{
	if(cnst >= -4 && cnst <=4)
	{
		SCCB_Write_Reg(REG_CNST, (0x30-(4-cnst)*4));
	}
	else
	{
		printf("Contrast parameter error!");
	}
}

/**
  * @brief  设置图像输出窗口（分辨率）QVGA
	* @param  sx:窗口x起始位置
	* @param  sy:窗口y起始位置
	* @param  width:窗口宽度
	* @param  height:窗口高度
	* @param QVGA_VGA:0,QVGA模式，1，VGA模式
  *
	* @note 	QVGA模式 参数要求，sx + width <= 320 ,sy+height <= 240
	* 			 	VGA模式参数要求，sx + width <= 640 ,sy+height <= 480
	*					但由于 液晶屏分辨率 和 FIFO空间 的限制，本工程不适用于超过320*240的配置
	*         使用VGA模式主要是因为OV7725无法直接交换XY方向，QVGA不方便使用竖平显示，
	*					设置成VGA模式，可以使用竖屏显示，
	*					相对QVGA模式，同样分辨率下 VGA模式 图像采样帧率稍慢
  * @retval 无
  */
void OV7725_Window_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height,uint8_t QVGA_VGA)
{
	uint8_t reg_raw,cal_temp;

	/***********QVGA or VGA *************/
	if(QVGA_VGA == 0)
	{
		/*QVGA RGB565 */
		SCCB_Write_Reg(REG_COM7,0x46); 
	}
	else
	{
			/*VGA RGB565 */
		SCCB_Write_Reg(REG_COM7,0x06); 
	}

	/***************HSTART*********************/
	//读取寄存器的原内容，HStart包含偏移值，在原始偏移植的基础上加上窗口偏移	
	SCCB_Read_Reg(REG_HSTART,&reg_raw);
	
	//sx为窗口偏移，高8位存储在HSTART，低2位在HREF
	cal_temp = (reg_raw + (sx>>2));	
	SCCB_Write_Reg(REG_HSTART,cal_temp ); 
	
	/***************HSIZE*********************/
	//水平宽度，高8位存储在HSIZE，低2位存储在HREF
	SCCB_Write_Reg(REG_HSIZE,width>>2);//HSIZE左移两位 
	
	
	/***************VSTART*********************/
	//读取寄存器的原内容，VStart包含偏移值，在原始偏移植的基础上加上窗口偏移	
	SCCB_Read_Reg(REG_VSTRT,&reg_raw);	
	//sy为窗口偏移，高8位存储在HSTART，低1位在HREF
	cal_temp = (reg_raw + (sy>>1));	
	
	SCCB_Write_Reg(REG_VSTRT,cal_temp);
	
	/***************VSIZE*********************/
	//垂直高度，高8位存储在VSIZE，低1位存储在HREF
	SCCB_Write_Reg(REG_VSIZE,height>>1);//VSIZE左移一位
	
	/***************VSTART*********************/
	//读取寄存器的原内容	
	SCCB_Read_Reg(REG_HREF,&reg_raw);	
	//把水平宽度的低2位、垂直高度的低1位，水平偏移的低2位，垂直偏移的低1位的配置添加到HREF
	cal_temp = (reg_raw |(width&0x03)|((height&0x01)<<2)|((sx&0x03)<<4)|((sy&0x01)<<6));	
	
	SCCB_Write_Reg(REG_HREF,cal_temp);
	
	/***************HOUTSIZIE /VOUTSIZE*********************/
	SCCB_Write_Reg(REG_HOutSize,width>>2);
	SCCB_Write_Reg(REG_VOutSize,height>>1);
	
	//读取寄存器的原内容	
	SCCB_Read_Reg(REG_EXHCH,&reg_raw);	
	cal_temp = (reg_raw |(width&0x03)|((height&0x01)<<2));	

	SCCB_Write_Reg(REG_EXHCH,cal_temp);	
}
static void ov7725_Output_Set()
{
	/*Special Effect*/
	OV7725_Special_Effect(0);
	/*light mode*/
	OV7725_Light_Mode(0);//auto
	/*Color Saturation*/
	OV7725_Color_Saturation(0);
	/*brightness*/
	OV7725_Contrast(0);

	//OV7725_Special_Effect(0);

	OV7725_Window_Set(0,0,320,240,0);
}
static void FIFO_READ_PRE(void)
{
	FIFO_RRST_L;
	FIFO_RCLK_L;
	FIFO_RCLK_H;
	FIFO_RRST_H;
	FIFO_RCLK_L;
	FIFO_RCLK_H;
}
static uint16_t BGR2RGB(uint16_t c)
{
  uint16_t  r, g, b, rgb;

  b = (c>>0)  & 0x1f;
  g = (c>>5)  & 0x3f;
  r = (c>>11) & 0x1f;
  
  rgb =  (b<<11) + (g<<5) + (r<<0);

  return( rgb );
}
static void READ_FIFO_PIXEL(uint16_t *data)
{
	uint16_t rgb565=0x0000;
	uint8_t data_high=0x00;
	uint8_t data_low=0x00;
	FIFO_RCLK_L;
	for(uint8_t i=0;i<8;i++)
	{
		data_high |=(get_gpio_level(DataLine[i])<<i);
	/*	printf("num %d bit %x datahigh %x\n",i,
				get_gpio_level(DataLine[i]),data_high);
	*/
	}
	rgb565=(uint16_t)data_high&0x00ff;
	rgb565=rgb565<<8;
	FIFO_RCLK_H;
	FIFO_RCLK_L;
	for(uint8_t i=0;i<8;i++)
	{
		data_low |=(get_gpio_level(DataLine[i])<<i);
	/*	printf("num %d bit %x datalow %x\n",i,
				get_gpio_level(DataLine[i]),data_low);
	*/
	}
	rgb565|=(uint16_t)data_low&0x00ff;
	FIFO_RCLK_H;
	*data =rgb565;
}
static void ImagDisp(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height,FILE* fp)
{
	uint16_t i, j; 
	uint16_t Camera_Data;
	
	for(i = 0; i < width; i++)
	{
		for(j = 0; j < height; j++)
		{
			READ_FIFO_PIXEL(&Camera_Data);
			fwrite(&Camera_Data,sizeof(Camera_Data),1,fp);
		//	printf("number %d %d pix data %x \n",i,j,Camera_Data);
		}
	}
}
int CreateDir(const char *sPathName)  
{  
    char DirName[256];  
    strcpy(DirName, sPathName);  
    int i,len = strlen(DirName);
    for(i=1; i<len; i++)  
    {  
        if(DirName[i]=='/')  
        {  
            DirName[i] = 0; 
            if(access(DirName, NULL)!=0)  
            {  
                if(mkdir(DirName, 0755)==-1)  
                {   
                      printf("mkdir   error\n");   
                      return -1;   
                }  
            }  
            DirName[i] = '/';  

        }  
      }  

    return 0;  
} 
int main(int argc,char **argv)
{
	All_gpio_set_Op();
	FIFO_GPIO_config();
	set_wiringpi_isr();
	data_gpio_set();
	while(0==Ov7725_init())
    {
        printf("[info]:ov7725 init failed\n");
    }
	delay_ms(1000);
	ov7725_Output_Set();
	ov7725_Output_Set();
	ov7725_Output_Set();
	delay_ms(1000);
    printf("[info]:init success\n");
	int frameCount=0;
	//char picFolder[]="picture";
	//CreateDir(picFolder);
	while(1)
	{
		if(Ov7725_vsync==2)
		{
			char fileName[1024];
			sprintf(fileName,"picture/camerRawdata-%d",frameCount);
			FILE *fp = fopen(fileName,"wb");
			if(!fp)
			{
				fprintf(stderr,"open file failed\n",strerror(errno));
				return 0;
			}
			frameCount++;
			FIFO_READ_PRE();
			ImagDisp(0,0,320,240,fp);
			fclose(fp);
			img_init(240,320);
			uint8_t* Dat;
			int debayer=4;
			Dat=(uint8_t *)calloc(320*240,bytesPerPixel(4));
			printf("[info] calloc success\n");
			uint8_t* outImg =(uint8_t *) malloc(320*240* 3); 
			int useNum=0;
			processImg(Dat,outImg,useNum,debayer,0,0,0,fileName,0);
			printf("[info] proccss success\n");
			char *saveName=(char *)malloc(128);
			
			saveName=(char *)realloc(saveName,strlen(fileName)+64);
			sprintf(saveName,"%s.png",fileName);
			if(savePNG(saveName,320,240,outImg))
			{
				puts("Error while saving the PNG");
				return 0;
			}
			free(Dat);
			free(outImg);
			free(saveName);
			Ov7725_vsync=0;
			printf("采集帧数　%d\n",frameCount);
			if(frameCount>=30) break;
		}
	}
    return 0;
}