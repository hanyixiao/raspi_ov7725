/********************************************************************************
 * raspi_sccb.cpp
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
}

#define SCCB_DELAY 100
#define SCCB_MIN 50

#define SCCB_SIOD_H set_gpio_value(SCCB_SIOD,1)
#define SCCB_SIOC_H set_gpio_value(SCCB_SIOC,1)
#define SCCB_SIOD_L set_gpio_value(SCCB_SIOD,0)
#define SCCB_SIOC_L set_gpio_value(SCCB_SIOC,0)

#define SCCB_SIOD_IN  set_gpio_fsel(SCCB_SIOD,FUNC_IP)
#define SCCB_SIOD_OUT set_gpio_fsel(SCCB_SIOD,FUNC_OP)
#define SCCB_SIOD_STATE get_gpio_level(SCCB_SIOD)


//数据总线初始化
void SCCB_GPIO_init()
{
    gpio_init();
    set_gpio_fsel(SCCB_SIOC,FUNC_OP);
    set_gpio_fsel(SCCB_SIOD,FUNC_OP);
    gpio_set_pull(SCCB_SIOC,PULL_UP);
    gpio_set_pull(SCCB_SIOD,PULL_UP);
}
/*
--------------------
    功能：start命令
    参数：无
    返回值：无
--------------------
*/
static void Start_SCCB()
{
    SCCB_SIOD_H;
    delay_us(SCCB_DELAY);
    SCCB_SIOC_H;
    delay_us(SCCB_DELAY);
    SCCB_SIOD_L;
    delay_us(SCCB_DELAY);
    SCCB_SIOC_L;
    delay_us(SCCB_DELAY);

}
/*
--------------------
    功能：stop命令
    参数：无
    返回值：无
--------------------
*/
static void Stop_SCCB()
{
    SCCB_SIOD_L;
    delay_us(SCCB_DELAY);
    SCCB_SIOC_H;
    delay_us(SCCB_DELAY);
    SCCB_SIOD_H;
    delay_us(SCCB_DELAY);
}

/*
--------------------
    功能：noACK用于连续读取中最后一个结束周期
    参数：无
    返回值：无
--------------------
*/
static void NoAck()
{
    SCCB_SIOD_H;
    delay_us(SCCB_DELAY);

    SCCB_SIOC_H;
    delay_us(SCCB_DELAY);

    SCCB_SIOC_L;
    delay_us(SCCB_DELAY);

    SCCB_SIOD_L;
    delay_us(SCCB_DELAY);

}
/*
------------------
    功能：写入一个字节到SCBB
    参数：写入数据mdate
    返回值：发送成功返回１　失败返回０
-------------------
*/
uint8_t SCCBWriteB(uint8_t m_data)
{
    uint8_t j,temp;
    SCCB_SIOD_OUT;
    for(j=0;j<8;j++)
    {
        if((m_data<<j)&0x80)
        {
            SCCB_SIOD_H;
        }
        else
        {
            SCCB_SIOD_L;
        }
        delay_us(SCCB_DELAY);
        SCCB_SIOC_H;
        delay_us(SCCB_DELAY);
        SCCB_SIOC_L;
        delay_us(SCCB_DELAY);
    }
    delay_us(SCCB_DELAY);
    SCCB_SIOD_IN;//设置ＳＤＡ为输入
    delay_us(SCCB_DELAY);
    SCCB_SIOC_H;
    delay_us(SCCB_DELAY);
  /*  printf("[info]:this value of SCCB_SIOD_STATE %d\n",
    SCCB_SIOD_STATE);
    */
    if(SCCB_SIOD_STATE) temp=0;//SDA发送失败
    else temp=1;//ＳＤＡ发送成功
    SCCB_SIOC_L;
    delay_us(SCCB_DELAY);
    SCCB_SIOD_OUT;

    return temp;

}
/*
-----------------------------------------------
   功能: 一个字节数据读取并且返回
   参数: 无
 返回值: 读取到的数据
-----------------------------------------------
*/
uint8_t SCCBReadB(void)
{
    uint8_t read,j;
    read=0x00;

    SCCB_SIOD_IN;
    delay_us(SCCB_DELAY);
    for(j=8;j>0;j--)
    {
        delay_us(SCCB_DELAY);
        SCCB_SIOC_H;
        delay_us(SCCB_DELAY);
        read=read<<1;
        if(SCCB_SIOD_STATE)
        {
            read=read+1;
        }
        SCCB_SIOC_L;
        delay_us(SCCB_DELAY);
    }
    SCCB_SIOD_OUT;
    return (read);
}

/***********************
 * 功能：写寄存器
 * 参数：uint8_t reg uint8_t data
 * 返回值：1 success 0 failed
 ***********************/
uint8_t SCCB_Write_Reg(uint8_t reg,uint8_t data)
{
    Start_SCCB();
    if(0==SCCBWriteB(0x42))
    {
        Stop_SCCB();
        printf("[info]:write reg fail\n");
        return 0;
    }
    delay_us(SCCB_MIN);
    if(0==SCCBWriteB(reg))
    {
        Stop_SCCB();
        printf("[info]:write reg fail\n");
        return 0;
    }
    delay_us(SCCB_MIN);
    if(0==SCCBWriteB(data))
    {
        Stop_SCCB();
        printf("[info]:write reg fail\n");
        return 0;
    }
    Stop_SCCB();
    return 1;
}
/**********************
 * 功能：读取OV7670的寄存器
 * 参数：uint8_t reg,uint8_t data
 * 返回值：1-success 0-fail
 ***********************/
uint8_t SCCB_Read_Reg(uint8_t reg,uint8_t *data)
{
    //通过写操作设置寄存器
    Start_SCCB();
    if(0==SCCBWriteB(0x42))
    {
        Stop_SCCB();
        printf("[info]:Re_Ov7670_Reg fail\n");
        return 0;
    }
    delay_us(SCCB_MIN);
    if(0==SCCBWriteB(reg))
    {
        Stop_SCCB();
        return 0;
    }
    Stop_SCCB();
    delay_us(SCCB_MIN);

    //设置寄存器地址之后才是读寄存器
    Start_SCCB();
    if(0==SCCBWriteB(0x43))
    {
        Stop_SCCB();
        printf("[info:] reWrite failed\n");
        return 0;
    }
    delay_us(SCCB_MIN);
    *data = SCCBReadB();
    NoAck();
    Stop_SCCB();
    return 1;
}