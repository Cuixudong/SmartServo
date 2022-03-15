/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <math.h>

#define GYRO_SCALE 0.06097609f 

static float angle, angle_dot, f_angle, f_angle_dot;
int16_t gx, gy, gz, ax, ay, az, temperature;

extern void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f);

#define ADC_LEN 32
uint16_t adc_val[ADC_LEN];
uint16_t adc_flite;

int16_t mpu_data[6];

#define FILTER_COUNT  16
int16_t gx_buf[FILTER_COUNT], ax_buf[FILTER_COUNT], ay_buf[FILTER_COUNT],az_buf[FILTER_COUNT];
void acc_filter(void)
{
    uint8_t i,j;
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;

    for(i = 1 ; i < FILTER_COUNT; i++)
    {
        ax_buf[i - 1] = ax_buf[i];
        ay_buf[i - 1] = ay_buf[i];
        az_buf[i - 1] = az_buf[i];
    }

    ax_buf[FILTER_COUNT - 1] = ax;
    ay_buf[FILTER_COUNT - 1] = ay;
    az_buf[FILTER_COUNT - 1] = az;

    for(i = 0 ; i < FILTER_COUNT; i++)
    {
        ax_sum += ax_buf[i];
        ay_sum += ay_buf[i];
        az_sum += az_buf[i];
    }

    ax = (int16_t)(ax_sum / FILTER_COUNT);
    ay = (int16_t)(ay_sum / FILTER_COUNT);
    az = (int16_t)(az_sum / FILTER_COUNT);
}

void set_pwm(int16_t pwm);

uint16_t tar_ang = 0;
float tar_dir = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int i = 0;
    if(!(i%5))
    {
        MPU_Get_Data(mpu_data);
        //angle_get(mpu_data);
        ax = *(mpu_data + 0);
        ay = *(mpu_data + 1);
        az = *(mpu_data + 2);
        gx = *(mpu_data + 3);//������ת��
        gy = *(mpu_data + 4);//���ƽ����ת
        gz = *(mpu_data + 5);
        acc_filter();
        #if 1
        angle_dot = gx * GYRO_SCALE;  //+-2000  0.060975 ��/LSB
        angle = atan(ay / sqrt(ax * ax + az * az ));
        angle = angle * 57.295780;    //180/pi
        #endif
        kalman_filter(angle, angle_dot, &f_angle, &f_angle_dot);//�������˲�
        
        tar_dir += gx * GYRO_SCALE * 0.005f;
    }
    if(i == 10)
    {
        i = 0;
        #define EN_PID 1
        #if EN_PID
        float cur_err = (adc_flite - 1024 + (angle * 20));
        static float last_err = 0;
        static float err_sum = 0;
        
        err_sum += cur_err;
        #define PID_I_SUM_NAX 200
        #define PID_OUT_MAX 2390
        if(cur_err > PID_I_SUM_NAX)cur_err = PID_I_SUM_NAX;
        if(cur_err < -PID_I_SUM_NAX)cur_err =  -PID_I_SUM_NAX;
        float pid_res = cur_err * 12 + err_sum * 0.01 + (cur_err - last_err) * 0.1;
        if(pid_res > PID_OUT_MAX)pid_res = PID_OUT_MAX;
        if(pid_res < -PID_OUT_MAX)pid_res = -PID_OUT_MAX;
        set_pwm(pid_res);
        last_err = cur_err;
        #endif
    }
    i++;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint8_t i;
    adc_flite = 0;
    for(i=0;i<ADC_LEN;i++)
    {
        adc_flite += adc_val[i];
    }
    adc_flite = (adc_flite >> 5);
}

void set_pwm(int16_t pwm)
{
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,2400 - pwm);
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,2400 + pwm);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    extern float Angle,angleAy;
    int i;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
    //printf("system start\r\n");
MPUinit:
    Set_MPU(1);
    i = MPU_Init();
    if(i != 0)
    {
        //printf("MPU1:%3d",i);
        HAL_Delay(100);
        goto MPUinit;
    }
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,2400);
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,2400);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    
    HAL_ADC_Start_DMA(&hadc,(uint32_t *)adc_val,ADC_LEN);
    
    set_pwm(1200);
    HAL_Delay(600);
    set_pwm(-1200);
    HAL_Delay(600);
    
    __HAL_TIM_CLEAR_FLAG(&htim14,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim14,TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim14);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        HAL_Delay(10);
        #if 1
        usart1_report_imu(angle * 100,0,0,0,0);
        #endif
        mpu6050_send_data(mpu_data[0],mpu_data[1],mpu_data[2],mpu_data[3],mpu_data[4],mpu_data[5]);
        #if 1
        printString("\r\n Angle:");printFloat(angle,2);
        printString(" AD:");printInteger(adc_flite);
        #endif
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#if 0
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}
#endif

void serial_write(char s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&s, 1, 0xFF);
}

void printString(const char *s)
{
    while (*s)
        serial_write(*s++);
}

void printIntegerInBase(unsigned long n, unsigned long base)
{
    unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
    unsigned long i = 0;

    if (n == 0) {
        serial_write('0');
        return;
    }

    while (n > 0) {
        buf[i++] = n % base;
        n /= base;
    }

    for (; i > 0; i--)
        serial_write(buf[i - 1] < 10 ?
                     '0' + buf[i - 1] :
                     'A' + buf[i - 1] - 10);
}


// Prints an uint8 variable in base 10.
void print_uint8_base10(uint8_t n)
{
    uint8_t digit_a = 0;
    uint8_t digit_b = 0;
    if (n >= 100) { // 100-255
        digit_a = '0' + n % 10;
        n /= 10;
    }
    if (n >= 10) { // 10-99
        digit_b = '0' + n % 10;
        n /= 10;
    }
    serial_write('0' + n);
    if (digit_b) {
        serial_write(digit_b);
    }
    if (digit_a) {
        serial_write(digit_a);
    }
}


// Prints an uint8 variable in base 2 with desired number of desired digits.
void print_uint8_base2_ndigit(uint8_t n, uint8_t digits)
{
    unsigned char buf[digits];
    uint8_t i = 0;

    for (; i < digits; i++) {
        buf[i] = n % 2 ;
        n /= 2;
    }

    for (; i > 0; i--)
        serial_write('0' + buf[i - 1]);
}


void print_uint32_base10(uint32_t n)
{
    if (n == 0) {
        serial_write('0');
        return;
    }

    unsigned char buf[10];
    uint8_t i = 0;

    while (n > 0) {
        buf[i++] = n % 10;
        n /= 10;
    }

    for (; i > 0; i--)
        serial_write('0' + buf[i-1]);
}


void printInteger(long n)
{
    if (n < 0) {
        serial_write('-');
        print_uint32_base10(-n);
    } else {
        print_uint32_base10(n);
    }
}


// Convert float to string by immediately converting to a long integer, which contains
// more digits than a float. Number of decimal places, which are tracked by a counter,
// may be set by the user. The integer is then efficiently converted to a string.
// NOTE: AVR '%' and '/' integer operations are very efficient. Bitshifting speed-up
// techniques are actually just slightly slower. Found this out the hard way.
void printFloat(float n, uint8_t decimal_places)
{
    if (n < 0) {
        serial_write('-');
        n = -n;
    }

    uint8_t decimals = decimal_places;
    while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
        n *= 100;
        decimals -= 2;
    }
    if (decimals) {
        n *= 10;
    }
    n += 0.5; // Add rounding factor. Ensures carryover through entire value.

    // Generate digits backwards and store in string.
    unsigned char buf[13];
    uint8_t i = 0;
    uint32_t a = (long)n;
    while(a > 0) {
        buf[i++] = (a % 10) + '0'; // Get digit
        a /= 10;
    }
    while (i < decimal_places) {
        buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
    }
    if (i == decimal_places) { // Fill in leading zero, if needed.
        buf[i++] = '0';
    }

    // Print the generated string.
    for (; i > 0; i--) {
        if (i == decimal_places) {
            serial_write('.');    // Insert decimal point in right place.
        }
        serial_write(buf[i-1]);
    }
}

//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0X01~0X1C
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(uint8_t fun,uint8_t *data,uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    if(len>28)return;	//���28�ֽ�����
    send_buf[len+3]=0;	//У��������
    send_buf[0]=0XAA;	//֡ͷ
    send_buf[1]=0XAA;	//֡ͷ
    send_buf[2]=fun;	//������
    send_buf[3]=len;	//���ݳ���
    for(i=0; i<len; i++)send_buf[4+i]=data[i];			//��������
    for(i=0; i<len+4; i++)send_buf[len+4]+=send_buf[i];	//����У���
    for(i=0; i<len+5; i++)serial_write(send_buf[i]);	//�������ݵ�����1
}
//���ͼ��ٶȴ���������+����������(������֡)
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
    uint8_t tbuf[18];
    tbuf[0]=(aacx>>8)&0XFF;
    tbuf[1]=aacx&0XFF;
    tbuf[2]=(aacy>>8)&0XFF;
    tbuf[3]=aacy&0XFF;
    tbuf[4]=(aacz>>8)&0XFF;
    tbuf[5]=aacz&0XFF;
    tbuf[6]=(gyrox>>8)&0XFF;
    tbuf[7]=gyrox&0XFF;
    tbuf[8]=(gyroy>>8)&0XFF;
    tbuf[9]=gyroy&0XFF;
    tbuf[10]=(gyroz>>8)&0XFF;
    tbuf[11]=gyroz&0XFF;
    tbuf[12]=0;//��Ϊ����MPL��,�޷�ֱ�Ӷ�ȡ����������,��������ֱ�����ε�.��0���.
    tbuf[13]=0;
    tbuf[14]=0;
    tbuf[15]=0;
    tbuf[16]=0;
    tbuf[17]=0;
    usart1_niming_report(0X02,tbuf,18);//������֡,0X02
}
//ͨ������1�ϱ���������̬���ݸ�����(״̬֡)
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
//csb:�������߶�,��λ:cm
//prs:��ѹ�Ƹ߶�,��λ:mm
void usart1_report_imu(short roll,short pitch,short yaw,short csb,int prs)
{
    uint8_t tbuf[12];
    tbuf[0]=(roll>>8)&0XFF;
    tbuf[1]=roll&0XFF;
    tbuf[2]=(pitch>>8)&0XFF;
    tbuf[3]=pitch&0XFF;
    tbuf[4]=(yaw>>8)&0XFF;
    tbuf[5]=yaw&0XFF;
    tbuf[6]=(csb>>8)&0XFF;
    tbuf[7]=csb&0XFF;
    tbuf[8]=(prs>>24)&0XFF;
    tbuf[9]=(prs>>16)&0XFF;
    tbuf[10]=(prs>>8)&0XFF;
    tbuf[11]=prs&0XFF;
    usart1_niming_report(0X01,tbuf,12);//״̬֡,0X01
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
