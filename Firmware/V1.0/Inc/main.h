/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MPU_SCL_Pin GPIO_PIN_4
#define MPU_SCL_GPIO_Port GPIOA
#define MPU_SDA_Pin GPIO_PIN_5
#define MPU_SDA_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_1
#define MPU_INT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#if defined(__CC_ARM)
#pragma anon_unions
#endif

#define DEG2RAD 0.017453293f    /* 度转弧度 π/180 */
#define RAD2DEG 57.29578f       /* 弧度转度 180/π */

#define MPU6500_DEG_PER_LSB_250  (float)((2 * 250.f) / 65536.f)
#define MPU6500_DEG_PER_LSB_500  (float)((2 * 500.f) / 65536.f)
#define MPU6500_DEG_PER_LSB_1000 (float)((2 * 1000.f) / 65536.f)
#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.f) / 65536.f)

#define MPU6500_G_PER_LSB_2      (float)((2 * 2) / 65536.f)
#define MPU6500_G_PER_LSB_4      (float)((2 * 4) / 65536.f)
#define MPU6500_G_PER_LSB_8      (float)((2 * 8) / 65536.f)
#define MPU6500_G_PER_LSB_16     (float)((2 * 16) / 65536.f)
    
#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES 1024    /* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE          4000    /* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES   200     /* 加速计采样个数 */

// MPU9250主机模式读取数据 缓冲区长度
#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN        8
#define SENSORS_BARO_STATUS_LEN     1
#define SENSORS_BARO_DATA_LEN       6
#define SENSORS_BARO_BUFF_LEN       (SENSORS_BARO_STATUS_LEN + SENSORS_BARO_DATA_LEN)

typedef union
{
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis[3];
} Axis3i16;

typedef union
{
    struct
    {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    int32_t axis[3];
} Axis3i32;

typedef union
{
    struct
    {
        int64_t x;
        int64_t y;
        int64_t z;
    };
    int64_t axis[3];
} Axis3i64;

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

typedef struct
{
    uint32_t timestamp; /*时间戳*/

    float roll;
    float pitch;
    float yaw;
} attitude_t;

struct  vec3_s
{
    uint32_t timestamp; /*时间戳*/

    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;
/* Orientation as a quaternion */
typedef struct quaternion_s
{
    uint32_t timestamp;

    union
    {
        struct
        {
            float q0;
            float q1;
            float q2;
            float q3;
        };
        struct
        {
            float x;
            float y;
            float z;
            float w;
        };
    };
} quaternion_t;

typedef struct
{
    attitude_t attitude;
    quaternion_t attitudeQuaternion;
    point_t position;
    velocity_t velocity;
    acc_t acc;
} state_t;

typedef struct zRange_s
{
    uint32_t timestamp; //时间戳
    float distance;     //测量距离
    float quality;      //可信度
} zRange_t;

typedef struct
{
    float pressure;
    float temperature;
    float asl;
} baro_t;

typedef struct
{
    Axis3f acc;
    Axis3f gyro;
    Axis3f mag;
    baro_t baro;
    point_t position;
    zRange_t zrange;
} sensorData_t;

typedef struct
{
    Axis3f     bias;
    bool       isBiasValueFound;
    bool       isBufferFilled;
    Axis3i16*  bufHead;
    Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

#define ACCZ_SAMPLE  350

typedef struct
{
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;
} kalman_obj;

#define IIR_SHIFT         8

typedef struct 
{
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float delay_element_1;
    float delay_element_2;
} lpf2pData;

int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);

void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2pData* lpfData, float sample);
float lpf2pReset(lpf2pData* lpfData, float sample);

void SimpleKalmanFilter_init(kalman_obj obj,float mea_e, float est_e, float q);
float updateEstimate(kalman_obj obj,float mea);
void setMeasurementError(kalman_obj obj,float mea_e);
void setEstimateError(kalman_obj obj,float est_e);
void setProcessNoise(kalman_obj obj,float q);
float getKalmanGain(kalman_obj obj);


void serial_write(char s);
void printString(const char *s);
void printIntegerInBase(unsigned long n, unsigned long base);
void print_uint8_base10(uint8_t n);
void print_uint8_base2_ndigit(uint8_t n, uint8_t digits);
void print_uint32_base10(uint32_t n);
void printInteger(long n);
void printFloat(float n, uint8_t decimal_places);
void usart1_niming_report(uint8_t fun,uint8_t *data,uint8_t len);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void usart1_report_imu(short roll,short pitch,short yaw,short csb,int prs);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
