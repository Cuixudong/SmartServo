#include "mpu6050.h"

/* ���� ���� */
#define IIC_SCL_GPIO_PORT               GPIOA
#define IIC_SCL_GPIO_PIN                GPIO_PIN_4
#define IIC_SCL_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PB��ʱ��ʹ�� */

#define IIC_SDA_GPIO_PORT               GPIOA
#define IIC_SDA_GPIO_PIN                GPIO_PIN_5
#define IIC_SDA_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PB��ʱ��ʹ�� */

/* IO���� */
#define IIC_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define IIC_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */

#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN) /* ��ȡSDA */

/* IIC���в������� */
void IIC_Init(void);            /* ��ʼ��IIC��IO�� */
void IIC_Start(void);           /* ����IIC��ʼ�ź� */
void IIC_Stop(void);            /* ����IICֹͣ�ź� */
void IIC_Ack(void);             /* IIC����ACK�ź� */
void IIC_Nack(void);            /* IIC������ACK�ź� */
uint8_t IIC_Wait_Ack(void);     /* IIC�ȴ�ACK�ź� */
void IIC_Send_Byte(uint8_t txd);/* IIC����һ���ֽ� */
uint8_t IIC_Read_Byte(uint8_t ack);/* IIC��ȡһ���ֽ� */

/**
* @brief       ��ʼ��IIC
* @param       ��
* @retval      ��
*/
void IIC_Init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL����ʱ��ʹ�� */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA����ʱ��ʹ�� */

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* ��©��� */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH; /* ���� */
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA����ģʽ����,��©���,����, �����Ͳ���������IO������, ��©�����ʱ��(=1), Ҳ���Զ�ȡ�ⲿ�źŵĸߵ͵�ƽ */

    IIC_Stop();     /* ֹͣ�����������豸 */
}

/**
 * @brief       IIC��ʱ����,���ڿ���IIC��д�ٶ�
 * @param       ��
 * @retval      ��
 */
static void IIC_Delay(void)
{
    uint16_t t = 1;    /* 2us����ʱ, ��д�ٶ���250Khz���� */
    while(t--)
    {
        (void)t;
    }
}

/**
 * @brief       ����IIC��ʼ�ź�
 * @param       ��
 * @retval      ��
 */
void IIC_Start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    IIC_Delay();
    IIC_SDA(0);     /* START�ź�: ��SCLΪ��ʱ, SDA�Ӹ߱�ɵ�, ��ʾ��ʼ�ź� */
    IIC_Delay();
    IIC_SCL(0);     /* ǯסI2C���ߣ�׼�����ͻ�������� */
    IIC_Delay();
}

/**
 * @brief       ����IICֹͣ�ź�
 * @param       ��
 * @retval      ��
 */
void IIC_Stop(void)
{
    IIC_SDA(0);     /* STOP�ź�: ��SCLΪ��ʱ, SDA�ӵͱ�ɸ�, ��ʾֹͣ�ź� */
    IIC_Delay();
    IIC_SCL(1);
    IIC_Delay();
    IIC_SDA(1);     /* ����I2C���߽����ź� */
    IIC_Delay();
}

/**
 * @brief       �ȴ�Ӧ���źŵ���
 * @param       ��
 * @retval      1������Ӧ��ʧ��
 *              0������Ӧ��ɹ�
 */
uint8_t IIC_Wait_Ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);     /* �����ͷ�SDA��(��ʱ�ⲿ������������SDA��) */
    IIC_Delay();
    IIC_SCL(1);     /* SCL=1, ��ʱ�ӻ����Է���ACK */
    IIC_Delay();

    while (IIC_READ_SDA)    /* �ȴ�Ӧ�� */
    {
        waittime++;

        if (waittime > 250)
        {
            IIC_Stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, ����ACK��� */
    IIC_Delay();
    return rack;
}

/**
 * @brief       ����ACKӦ��
 * @param       ��
 * @retval      ��
 */
void IIC_Ack(void)
{
    IIC_SDA(0);     /* SCL 0 -> 1  ʱ SDA = 0,��ʾӦ�� */
    IIC_Delay();
    IIC_SCL(1);     /* ����һ��ʱ�� */
    IIC_Delay();
    IIC_SCL(0);
    IIC_Delay();
    IIC_SDA(1);     /* �����ͷ�SDA�� */
    IIC_Delay();
}

/**
 * @brief       ������ACKӦ��
 * @param       ��
 * @retval      ��
 */
void IIC_Nack(void)
{
    IIC_SDA(1);     /* SCL 0 -> 1  ʱ SDA = 1,��ʾ��Ӧ�� */
    IIC_Delay();
    IIC_SCL(1);     /* ����һ��ʱ�� */
    IIC_Delay();
    IIC_SCL(0);
    IIC_Delay();
}

/**
 * @brief       IIC����һ���ֽ�
 * @param       data: Ҫ���͵�����
 * @retval      ��
 */
void IIC_Send_Byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);    /* ��λ�ȷ��� */
        IIC_Delay();
        IIC_SCL(1);
        IIC_Delay();
        IIC_SCL(0);
        data <<= 1;     /* ����1λ,������һ�η��� */
    }
    IIC_SDA(1);         /* �������, �����ͷ�SDA�� */
}

/**
 * @brief       IIC��ȡһ���ֽ�
 * @param       ack:  ack=1ʱ������ack; ack=0ʱ������nack
 * @retval      ���յ�������
 */
uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* ����1���ֽ����� */
    {
        receive <<= 1;  /* ��λ�����,�������յ�������λҪ���� */
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
        IIC_Nack();     /* ����nACK */
    }
    else
    {
        IIC_Ack();      /* ����ACK */
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
    IIC_Start();                  //��ʼ�ź�
    IIC_Send_Byte(0xD0);   //�����豸��ַ+д�ź�
    IIC_Send_Byte(REG_Address);    //�ڲ��Ĵ�����ַ��
    IIC_Send_Byte(REG_data);       //�ڲ��Ĵ������ݣ�
    IIC_Stop();                   //����ֹͣ�ź�
}

//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Init(void)
{
    uint8_t res;
    IIC_Init();
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);  //��λMPU6050
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);  //����MPU6050
    MPU_Set_Gyro_Fsr(3);          //�����Ǵ�����dps
    MPU_Set_Accel_Fsr(2);          //���ٶȴ�����g
    MPU_Set_Rate(50);            //���ò�����50Hz
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);  //�ر������ж�
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);  //I2C��ģʽ�ر�
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);  //�ر�FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);  //INT���ŵ͵�ƽ��Ч
    res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res==MPU_ADDR)//����ID��ȷ
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);  //����CLKSEL,PLL X��Ϊ�ο�
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);  //���ٶ��������Ƕ�����
        MPU_Set_Rate(200);            //���ò�����Hz
    }
    else
        return res;

//    #define SMPLRT_DIV      0x19    //�����ǲ����ʣ�����ֵ��0x07(125Hz)
//    #define CONFIG          0x1A    //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
//    #define GYRO_CONFIG     0x1B    //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
//    #define ACCEL_CONFIG    0x1C    //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
//    #define PWR_MGMT_1      0x6B    //��Դ��������ֵ��0x00(��������)
//   
//    Single_WriteI2C(PWR_MGMT_1, 0x00);  //�������״̬
//    Single_WriteI2C(SMPLRT_DIV, 0x07);  //������125hz
//    Single_WriteI2C(CONFIG, 0x04);      //21HZ�˲� ��ʱA8.5ms G8.3ms  �˴�ȡֵӦ�൱ע�⣬��ʱ��ϵͳ�������Ϊ��
//    Single_WriteI2C(GYRO_CONFIG, 0x08); //������500��/S 65.5LSB/g
//    Single_WriteI2C(ACCEL_CONFIG, 0x08);//���ٶ�+-4g  8192LSB/g
    return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);  //�������ֵ�ͨ�˲���
    return MPU_Set_LPF(rate/2);  //�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
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
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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

//IIC����д
//addr:������ַ
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0);//����������ַ+д����
    if(IIC_Wait_Ack())  //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //д�Ĵ�����ַ
    IIC_Wait_Ack();    //�ȴ�Ӧ��
    for(i=0; i<len; i++)
    {
        IIC_Send_Byte(buf[i]);  //��������
        if(IIC_Wait_Ack())    //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0);//����������ַ+д����
    if(IIC_Wait_Ack())  //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //д�Ĵ�����ַ
    IIC_Wait_Ack();    //�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((addr<<1)|1);//����������ַ+������
    IIC_Wait_Ack();    //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK
        else *buf=IIC_Read_Byte(1);    //������,����ACK
        len--;
        buf++;
    }
    IIC_Stop();  //����һ��ֹͣ����
    return 0;
}
//IICдһ���ֽ�
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte((mpu_address<<1)|0);//����������ַ+д����
    if(IIC_Wait_Ack())  //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //д�Ĵ�����ַ
    IIC_Wait_Ack();    //�ȴ�Ӧ��
    IIC_Send_Byte(data);//��������
    if(IIC_Wait_Ack())  //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}
//IIC��һ���ֽ�
//reg:�Ĵ�����ַ
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    IIC_Start();
    IIC_Send_Byte((mpu_address<<1)|0);//����������ַ+д����
    IIC_Wait_Ack();    //�ȴ�Ӧ��
    IIC_Send_Byte(reg);  //д�Ĵ�����ַ
    IIC_Wait_Ack();    //�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((mpu_address<<1)|1);//����������ַ+������
    IIC_Wait_Ack();    //�ȴ�Ӧ��
    res=IIC_Read_Byte(0);//��ȡ����,����nACK
    IIC_Stop();      //����һ��ֹͣ����
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

���ƣ�void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)
���ܣ���������������ٶȼ�����ͨ���˲��㷨�ں�
���������
  float angle_m   ���ٶȼƼ���ĽǶ�
	float gyro_m    �����ǽ��ٶ�
	float *angle_f  �ںϺ�ĽǶ�
	float *angle_dot_f  �ںϺ�Ľ��ٶ�
����������˲���ĽǶȼ����ٶ�
����ֵ����
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
