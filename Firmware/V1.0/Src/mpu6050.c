#include "mpu6050.h"

/* 引脚 定义 */
#define IIC_SCL_GPIO_PORT               GPIOA
#define IIC_SCL_GPIO_PIN                GPIO_PIN_4
#define IIC_SCL_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

#define IIC_SDA_GPIO_PORT               GPIOA
#define IIC_SDA_GPIO_PIN                GPIO_PIN_5
#define IIC_SDA_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

/* IO操作 */
#define IIC_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define IIC_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */

#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN) /* 读取SDA */

/* IIC所有操作函数 */
void IIC_Init(void);            /* 初始化IIC的IO口 */
void IIC_Start(void);           /* 发送IIC开始信号 */
void IIC_Stop(void);            /* 发送IIC停止信号 */
void IIC_Ack(void);             /* IIC发送ACK信号 */
void IIC_Nack(void);            /* IIC不发送ACK信号 */
uint8_t IIC_Wait_Ack(void);     /* IIC等待ACK信号 */
void IIC_Send_Byte(uint8_t txd);/* IIC发送一个字节 */
uint8_t IIC_Read_Byte(uint8_t ack);/* IIC读取一个字节 */

/**
* @brief       初始化IIC
* @param       无
* @retval      无
*/
void IIC_Init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL引脚时钟使能 */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA引脚时钟使能 */

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* 开漏输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH; /* 高速 */
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    IIC_Stop();     /* 停止总线上所有设备 */
}

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void IIC_Delay(void)
{
    uint16_t t = 1;    /* 2us的延时, 读写速度在250Khz以内 */
    while(t--)
    {
        (void)t;
    }
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void IIC_Start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    IIC_Delay();
    IIC_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    IIC_Delay();
    IIC_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    IIC_Delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void IIC_Stop(void)
{
    IIC_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    IIC_Delay();
    IIC_SCL(1);
    IIC_Delay();
    IIC_SDA(1);     /* 发送I2C总线结束信号 */
    IIC_Delay();
}

/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t IIC_Wait_Ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    IIC_Delay();
    IIC_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    IIC_Delay();

    while (IIC_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 250)
        {
            IIC_Stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, 结束ACK检查 */
    IIC_Delay();
    return rack;
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void IIC_Ack(void)
{
    IIC_SDA(0);     /* SCL 0 -> 1  时 SDA = 0,表示应答 */
    IIC_Delay();
    IIC_SCL(1);     /* 产生一个时钟 */
    IIC_Delay();
    IIC_SCL(0);
    IIC_Delay();
    IIC_SDA(1);     /* 主机释放SDA线 */
    IIC_Delay();
}

/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void IIC_Nack(void)
{
    IIC_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    IIC_Delay();
    IIC_SCL(1);     /* 产生一个时钟 */
    IIC_Delay();
    IIC_SCL(0);
    IIC_Delay();
}

/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void IIC_Send_Byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        IIC_Delay();
        IIC_SCL(1);
        IIC_Delay();
        IIC_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC_SDA(1);         /* 发送完成, 主机释放SDA线 */
}

/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC_SCL(1);
        IIC_Delay();

        if (IIC_READ_SDA)
        {
            receive++;
        }

        IIC_SCL(0);
        IIC_Delay();
    }

    if (!ack)
    {
        IIC_Nack();     /* 发送nACK */
    }
    else
    {
        IIC_Ack();      /* 发送ACK */
    }

    return receive;
}


__IO uint8_t mpu_address;

void Set_MPU(uint8_t id)
{
    if(id == 1)
    {
        mpu_address = 0x68;
    }
    else if(id == 2)
    {
        mpu_address = 0x69;
    }
}

void Single_WriteI2C(uint8_t REG_Address,uint8_t REG_data)
{
    IIC_Start();                  //起始信号
    IIC_Send_Byte(0xD0);   //发送设备地址+写信号
    IIC_Send_Byte(REG_Address);    //内部寄存器地址，
    IIC_Send_Byte(REG_data);       //内部寄存器数据，
    IIC_Stop();                   //发送停止信号
}

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Init(void)
{
    uint8_t res;
    IIC_Init();
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);  //复位MPU6050
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);  //唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);          //陀螺仪传感器dps
    MPU_Set_Accel_Fsr(2);          //加速度传感器g
    MPU_Set_Rate(50);            //设置采样率50Hz
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);  //关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);  //I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);  //关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);  //INT引脚低电平有效
    res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res==MPU_ADDR)//器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);  //设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);  //加速度与陀螺仪都工作
        MPU_Set_Rate(200);            //设置采样率Hz
    }
    else
        return res;

//    #define SMPLRT_DIV      0x19    //陀螺仪采样率，典型值：0x07(125Hz)
//    #define CONFIG          0x1A    //低通滤波频率，典型值：0x06(5Hz)
//    #define GYRO_CONFIG     0x1B    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
//    #define ACCEL_CONFIG    0x1C    //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
//    #define PWR_MGMT_1      0x6B    //电源管理，典型值：0x00(正常启用)
//   
//    Single_WriteI2C(PWR_MGMT_1, 0x00);  //解除休眠状态
//    Single_WriteI2C(SMPLRT_DIV, 0x07);  //陀螺仪125hz
//    Single_WriteI2C(CONFIG, 0x04);      //21HZ滤波 延时A8.5ms G8.3ms  此处取值应相当注意，延时与系统周期相近为宜
//    Single_WriteI2C(GYRO_CONFIG, 0x08); //陀螺仪500度/S 65.5LSB/g
//    Single_WriteI2C(ACCEL_CONFIG, 0x08);//加速度+-4g  8192LSB/g
    return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);  //设置数字低通滤波器
    return MPU_Set_LPF(rate/2);  //自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
int16_t MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    int16_t raw;
    float temp;
    MPU_Read_Len(mpu_address,MPU_TEMP_OUTH_REG,2,buf);
    raw=((uint16_t)buf[0]<<8)|buf[1];
    temp=36.53f+((float)raw)/340.0f;
    return temp*100;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(int16_t *gx,int16_t *gy,int16_t *gz)
{
    uint8_t buf[6],res;
    res=MPU_Read_Len(mpu_address,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=((uint16_t)buf[0]<<8)|buf[1];
        *gy=((uint16_t)buf[2]<<8)|buf[3];
        *gz=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(int16_t *ax,int16_t *ay,int16_t *az)
{
    uint8_t buf[6],res;
    res=MPU_Read_Len(mpu_address,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=((uint16_t)buf[0]<<8)|buf[1];
        *ay=((uint16_t)buf[2]<<8)|buf[3];
        *az=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}

uint8_t MPU_Get_Data(int16_t *mem)
{
    uint8_t buf[6],res;
    res=MPU_Read_Len(mpu_address,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *(mem + 3)=((uint16_t)buf[0]<<8)|buf[1];
        *(mem + 4)=((uint16_t)buf[2]<<8)|buf[3];
        *(mem + 5)=((uint16_t)buf[4]<<8)|buf[5];
    }
    res=MPU_Read_Len(mpu_address,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *(mem + 0)=((uint16_t)buf[0]<<8)|buf[1];
        *(mem + 1)=((uint16_t)buf[2]<<8)|buf[3];
        *(mem + 2)=((uint16_t)buf[4]<<8)|buf[5];
    }
}

//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
    if(IIC_Wait_Ack())  //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //写寄存器地址
    IIC_Wait_Ack();    //等待应答
    for(i=0; i<len; i++)
    {
        IIC_Send_Byte(buf[i]);  //发送数据
        if(IIC_Wait_Ack())    //等待ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
    if(IIC_Wait_Ack())  //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //写寄存器地址
    IIC_Wait_Ack();    //等待应答
    IIC_Start();
    IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令
    IIC_Wait_Ack();    //等待应答
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK
        else *buf=IIC_Read_Byte(1);    //读数据,发送ACK
        len--;
        buf++;
    }
    IIC_Stop();  //产生一个停止条件
    return 0;
}
//IIC写一个字节
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte((mpu_address<<1)|0);//发送器件地址+写命令
    if(IIC_Wait_Ack())  //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //写寄存器地址
    IIC_Wait_Ack();    //等待应答
    IIC_Send_Byte(data);//发送数据
    if(IIC_Wait_Ack())  //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}
//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    IIC_Start();
    IIC_Send_Byte((mpu_address<<1)|0);//发送器件地址+写命令
    IIC_Wait_Ack();    //等待应答
    IIC_Send_Byte(reg);  //写寄存器地址
    IIC_Wait_Ack();    //等待应答
    IIC_Start();
    IIC_Send_Byte((mpu_address<<1)|1);//发送器件地址+读命令
    IIC_Wait_Ack();    //等待应答
    res=IIC_Read_Byte(0);//读取数据,发送nACK
    IIC_Stop();      //产生一个停止条件
    return res;
}

#include "stdio.h"
#include "math.h"

static float angle, angle_dot;
const float Q_angle = 0.002, Q_gyro = 0.002, R_angle = 0.5, dt = 0.01;
static float P[2][2]= {
    { 1, 0 },
    { 0, 1 }
};
static float Pdot[4] = {0, 0, 0, 0};
const uint8_t C_0 = 1;
static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
/*************************************************

名称：void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)
功能：陀螺仪数据与加速度计数据通过滤波算法融合
输入参数：
  float angle_m   加速度计计算的角度
	float gyro_m    陀螺仪角速度
	float *angle_f  融合后的角度
	float *angle_dot_f  融合后的角速度
输出参数：滤波后的角度及角速度
返回值：无
**************************************************/
void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)
{
    angle += (gyro_m - q_bias) * dt;

    Pdot[0]  =Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;

    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    angle_err = angle_m - angle;

    PCt_0=C_0 * P[0][0];
    PCt_1=C_0 * P[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    angle += K_0 * angle_err;
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m - q_bias;

    *angle_f = angle;
    *angle_dot_f = angle_dot;
}
