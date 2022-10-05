// MPU6050 

#ifndef MPU6050_H_
#define MPU6050_H_

//=============================================================================
// Библиотеки 
#include <stdio.h>											// стандартная библиотека ввода/вывода Си
#include <stdlib.h>											// для модуля числа
#include <math.h>											// библиотека математических функций Си
#include "stm32f4xx.h"
#include "delay.h"
#include "LCD1602_I2C.h"

// Системные константы
#define TRUE		1
#define FALSE		0

#define gyro_error	M_PI * (5.f / 180.f)			// gyroscope measurement error in rad/s (shown as 5 °/s)
#define gyro_drift	M_PI * (0.2f / 180.f)			// gyroscope measurement error in rad/s/s (shown as 0.2f °/s/s)
#define BETA		sqrt(3.f / 4.f) * gyro_error	// compute beta
#define ZETA		sqrt(3.f / 4.f) * gyro_drift	// compute zeta


//=============================================================================
// Установка параметров MPU
#define DMP2_FIFO_RATE_DIVISOR	0x01

#define GYRO_FS				500						// GYRO_FS: ±250°/сек; ±500°/сек; ±1000°/сек; ±2000°/сек
#define ACCEL_FS			4						// ACCEL_FS: ±2g; ±4g; ±8g; ±16g
#define SAMPLE_RATE			200						// частота выборки, Гц
#define DLPF				1						// частота фильтра низких частот
/*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | 8kHz
 */

// На модуле GY-521, вывод AD0 по умолчанию подключен к 0 через резистор 4k7,
// при этом 7-битный адрес модуля 110100, если AD0 = 0 и 110101, если AD0 = 1.
#define AD0					0
#if	AD0
#define MPU6050_addr		0x69			// Device address when ADO = 1
#else
#define MPU6050_addr		0x68			// Device address when ADO = 0
#endif

//=========================================================================
// Описание регистров MPU6050
// * - is not mentioned in manual
#define XG_OFFS_TC			0x00					// *[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define YG_OFFS_TC			0x01					// *[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define ZG_OFFS_TC			0x02					// *[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
// _G_OFFS_TC bits:
#define PWR_MODE			7
#define XG_OFFS_TC_0		1
#define YG_OFFS_TC_0		1
#define ZG_OFFS_TC_0		1
#define OTP_BNK_VLD			0
//===========================
	
#define X_FINE_GAIN			0x03					// *[7:0] X_FINE_GAIN
#define Y_FINE_GAIN			0x04					// *[7:0] Y_FINE_GAIN
#define Z_FINE_GAIN			0x05					// *[7:0] Z_FINE_GAIN

													// User-defined trim values for accelerometer
													// ±16g Offset cancellation in all Full Scale modes,
													// 15 bit 0,98-mg per steps
#define XA_OFFSET_H			0x06					// *[7:0] bit 14:7 XA_OFFS_USR
#define XA_OFFSET_L			0x07					// *[7:1] bit  6:0 XA_OFFS_USR; [0] - reserved
#define YA_OFFSET_H			0x08					// *[7:0] bit 14:7 YA_OFFS_USR
#define YA_OFFSET_L			0x09					// *[7:1] bit  6:0 YA_OFFS_USR; [0] - reserved
#define ZA_OFFSET_H			0x0A					// *[7:0] bit 14:7 ZA_OFFS_USR
#define ZA_OFFSET_L			0x0B					// *[7:1] bit  6:0 ZA_OFFS_USR; [0] - reserved
#define MPU_PROD_ID_REG		0X0C					// *PROD_ID register

#define SELF_TEST_X			0x0D
#define SELF_TEST_Y			0x0E
#define SELF_TEST_Z			0x0F
#define SELF_TEST_A			0x10

/* Регистры смещения гироскопа
 * Следующие регистры используются для удаления смещения постоянного тока с выхода датчика. 
 * Значение в этом регистре добавляется к значению датчика гироскопа перед тем, 
 * как попасть в регистр датчика.
 */
#define XG_OFFSET_H			0x13					// *[15:8] XG_OFFS_USR
#define XG_OFFSET_L			0x14					// *[7:0]  XG_OFFS_USR
#define YG_OFFSET_H			0x15					// *[15:8] YG_OFFS_USR
#define YG_OFFSET_L			0x16					// *[7:0]  YG_OFFS_USR
#define ZG_OFFSET_H			0x17					// *[15:8] ZG_OFFS_USR
#define ZG_OFFSET_L			0x18					// *[7:0]  ZG_OFFS_USR

#define SMPLRT_DIV			0x19					// делитель частоты дискретизации
#define CONFIG				0x1A					
/* 
 * CONFIG отвечает за настройку внешнего сигнала синхронизации (в GY-521, не используется)
 * и за настройку ФНЧ:
 * bit7:6 - резерв;
 * bit5:3 = 000 - внешняя синхронизация отсутствует;
 * bit2:0 - фильтр низких частот.
 *
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | 8kHz
 */
// CONFIG bits:
#define EXT_SYNC_SET		3
#define DLPF_CFG		0
// EXT_SYNC_SET:
#define INPUT_DISABLED		0
#define TEMP_OUT_L0		1
#define GYRO_XOUT_L0		2
#define GYRO_YOUT_L0		3
#define GYRO_ZOUT_L0		4
#define ACCEL_XOUT_L0		5
#define ACCEL_YOUT_L0		6
#define ACCEL_ZOUT_L0		7
//===========================

#define GYRO_CONFIG			0x1B
/*
 * Настройка гироскопа
 * Три старших бита, по одному на каждую ось гироскопа для самотестирования.
 * Два бита для настройки шкалы гироскопа от 250 до 2000 °/сек.
 * bit7   - XG_ST;
 * bit6   - YG_ST;
 * bit5   - ZG_ST;
 * bit4:3 - GFS_SEL_[1:0]: 0 - ±250°/сек; 1 - ±500°/сек; 2 - ±1000°/сек; 3 - ±2000°/сек
 */
// GYRO_CONFIG bits:
#define XG_ST				7						// запуск самотестирования гироскопа
#define YG_ST				6
#define ZG_ST				5
#define GFS_SEL				3						// чувствительность гироскопа
//===========================

#define ACCEL_CONFIG		0x1C
/*
 * Отвечает за настройку акселерометра. Три бита на каждую ось акселерометра для
 * самотестирования. Два бита для настройки шкалы акселерометра от ±2 до ±16g:
 * bit7   - XA_ST; 
 * bit6   - YA_ST;
 * bit5   - ZA_ST;
 * bit4:3 - AFS_SEL_[1:0] 0 - ±2g; 1 - ±4g; 2 - ±8g; 3 - ±16g
 */
// ACCEL_CONFIG bits:
#define XA_ST				7
#define YA_ST				6
#define ZA_ST				5
#define AFS_SEL				3
//===========================

#define FF_THR				0x1D					// *Free-fall
#define FF_DUR				0x1E					// *Free-fall
#define MOT_THR				0x1F					// *Motion detection threshold bits [7:0]
// Wake-on Motion Threshold - this register holds the threshold value for the 
// Wake on Motion Interrupt for accel x/y/z axes. LSB = 4mg. Range is 0mg to 1020mg.
#define MOT_DUR				0x20					// *Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR			0x21					// *Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR			0x22					// *Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN				0x23
/*
Отвечает за буфер обмена FIFO. Можно указать какие данные с датчиков будут
загружаться в буфер FIFO.
*/
// FIFO_EN bits:
#define TEMP_FIFO_EN		7
#define XG_FIFO_EN			6
#define YG_FIFO_EN			5
#define ZG_FIFO_EN			4
#define ACCEL_FIFO_EN		3
#define SLV2_FIFO_EN		2
#define SLV1_FIFO_EN		1
#define SLV0_FIFO_EN		0
//===========================

#define I2C_MST_CTRL		0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV1_ADDR		0x28
#define I2C_SLV1_REG		0x29
#define I2C_SLV1_CTRL		0x2A
#define I2C_SLV2_ADDR		0x2B
#define I2C_SLV2_REG		0x2C
#define I2C_SLV2_CTRL		0x2D
#define I2C_SLV3_ADDR		0x2E
#define I2C_SLV3_REG		0x2F
#define I2C_SLV3_CTRL		0x30
#define I2C_SLV4_ADDR		0x31
#define I2C_SLV4_REG		0x32
#define I2C_SLV4_DO			0x33
#define I2C_SLV4_CTRL		0x34
#define I2C_SLV4_DI			0x35
#define I2C_MST_STATUS		0x36

#define INT_PIN_CFG			0x37					// Настройка вывода INT
// INT_PIN_CFG bits:
#define INT_LEVEL			7
#define INT_OPEN			6
#define LATCH_INT_EN		5
#define INT_RD_CLEAR		4
#define FSYNC_INT_LEVEL		3
#define FSYNC_INT_EN		2
#define I2C_BYPASS_EN		1
#define INTCFG_CLKOUT_EN	0
//===========================

#define INT_ENABLE			0x38					// Отвечает за прерывания INT
// INT_ENABLE bits:
#define FF_EN				7
#define MOT_EN				6
#define ZMOT_EN				5
#define FIFO_OFLOW_EN		4
#define I2C_MST_INT_EN		3
#define PLL_RDY_INT_EN		2
#define DMP_INT_EN			1
#define DATA_RDY_EN			0


#define FIFO_OFLOW_EN		4			
#define I2C_MST_INT_EN		3
#define DATA_RDY_EN			0
//===========================

#define DMP_INT_STATUS		0x39					// *Check DMP interrupt
#define INT_STATUS			0x3A					// Флаги прерывания INT
// INT_STATUS bits:
#define FIFO_OFLOW_INT		4
#define I2C_MST_INT			3
#define DATA_RDY_INT		0
//===========================

#define ACCEL_XOUT_H		0x3B					// Старший байт акселерометра по оси X
#define ACCEL_XOUT_L		0x3C					// Младший байт акселерометра по оси X
#define ACCEL_YOUT_H		0x3D					// Старший байт акселерометра по оси Y
#define ACCEL_YOUT_L		0x3E					// Младший байт акселерометра по оси Y
#define ACCEL_ZOUT_H		0x3F					// Старший байт акселерометра по оси Z
#define ACCEL_ZOUT_L		0x40					// Младший байт акселерометра по оси Z
#define TEMP_OUT_H			0x41					// Старший байт температурного датчика
#define TEMP_OUT_L			0x42					// Младший байт температурного датчика
#define GYRO_XOUT_H			0x43					// Старший байт гироскопа по оси X
#define GYRO_XOUT_L			0x44					// Младший байт гироскопа по оси X
#define GYRO_YOUT_H			0x45					// Старший байт гироскопа по оси Y
#define GYRO_YOUT_L			0x46					// Младший байт гироскопа по оси Y
#define GYRO_ZOUT_H			0x47					// Старший байт гироскопа по оси Z
#define GYRO_ZOUT_L			0x48					// Младший байт гироскопа по оси Z
#define EXT_SENS_DATA_00	0x49
#define EXT_SENS_DATA_01	0x4A
#define EXT_SENS_DATA_02	0x4B
#define EXT_SENS_DATA_03	0x4C
#define EXT_SENS_DATA_04	0x4D
#define EXT_SENS_DATA_05	0x4E
#define EXT_SENS_DATA_06	0x4F
#define EXT_SENS_DATA_07	0x50
#define EXT_SENS_DATA_08	0x51
#define EXT_SENS_DATA_09	0x52
#define EXT_SENS_DATA_10	0x53
#define EXT_SENS_DATA_11	0x54
#define EXT_SENS_DATA_12	0x55
#define EXT_SENS_DATA_13	0x56
#define EXT_SENS_DATA_14	0x57
#define EXT_SENS_DATA_15	0x58
#define EXT_SENS_DATA_16	0x59
#define EXT_SENS_DATA_17	0x5A
#define EXT_SENS_DATA_18	0x5B
#define EXT_SENS_DATA_19	0x5C
#define EXT_SENS_DATA_20	0x5D
#define EXT_SENS_DATA_21	0x5E
#define EXT_SENS_DATA_22	0x5F
#define EXT_SENS_DATA_23	0x60
#define MOT_DETECT_STATUS	0x61
#define I2C_SLV0_DO			0x63
#define I2C_SLV1_DO			0x64
#define I2C_SLV2_DO			0x65
#define I2C_SLV3_DO			0x66
#define I2C_MST_DELAY_CTRL	0x67

#define SIGNAL_PATH_RESET	0x68		
// Cброс аналоговых и цифровых сигналов датчиков
// SIGNAL_PATH_RESET bits:
#define GYRO_RESET			2
#define ACCEL_RESET			1
#define TEMP_RESET			0
//===========================

#define MOT_DETECT_CTRL		0x69					// *
#define USER_CTRL			0x6A
/*
Предназначен для включения/отключения буфера FIFO, I2C мастер режима 
и общий сброс регистров датчиков
*/
// USER_CTRL bits:
#define DMP_EN_BIT			7
#define FIFO_EN_BIT			6
#define I2C_MST_EN			5
#define I2C_IF_DIS			4
#define DMP_RESET			3
#define FIFO_RESET			2
#define I2C_MST_RESET		1
#define SIG_COND_RESET		0
//===========================

#define PWR_MGMT_1			0x6B
/*
 * Этот регистр позволяет настроить режим питания и источник синхронизации. 
 * Он также предоставляет бит для сброса всего устройства и бит
 * для отключения датчика температуры.
 * 
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator 8 MHz 
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 *
 */
// PWR_MGMT_1 bits:
#define DEVICE_RESET		7						// сбрасывает все внутренние регистры к значениям по умолчанию
#define SLEEP				6						// спящий режим
#define CYCLE				5						// циклическое чередование режимов ожидания и пробуждения 
#define TEMP_DIS			3						// отключает датчик температуры
#define CLKSEL				0						// источник синхронизации устройства
//===========================

#define PWR_MGMT_2			0x6C
/*
Настройка частоты пробуждения акселерометра в режиме пониженного
энергопотребления и установка отдельных осей в режим ожидания
*/
// PWR_MGMT_2 bits:
#define LP_WAKE_CTRL_1		7						// 
#define LP_WAKE_CTRL_0		6						// 
#define DIS_XA				5						// 
#define DIS_YA				4						// 
#define DIS_ZA				3						// 
#define DIS_XG				2
#define DIS_YG				1
#define DIS_ZG				0
//===========================

#define DMP_BANK			0x6D					// *Activates a specific bank in the DMP
#define DMP_RW_PNT			0x6E					// *Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG				0x6F					// *Register in DMP from which to read or to which to write
#define DMP_REG_1			0x70					// *
#define DMP_REG_2			0x71					// *

#define FIFO_COUNT_H		        0x72
#define FIFO_COUNT_L		        0x73
#define FIFO_R_W			0x74
#define WHO_AM_I			0x75					// Отвечает за идентификацию устройства
													// Значение по умолчанию для регистра - 0x68
//=========================================================================
// DMP
//=========================================================================
#define DMP2_MEMORY_BANKS		8
#define DMP2_MEMORY_BANK_SIZE		256
#define DMP2_MEMORY_CHUNK_SIZE		16
    
#define MPU6050_FIFO_DEFAULT_TIMEOUT	11000
#define DMP2_CODE_SIZE					1929	// DMP2_memory[]
#define DMP6_CODE_SIZE					3062    // DMP6_memory[]
extern const uint8_t DMP2_memory[DMP2_CODE_SIZE] @ "FLASH";
extern const uint8_t DMP2_update[2] @ "FLASH" ;
extern const uint8_t DMP6_memory[DMP6_CODE_SIZE] @ "FLASH";

//=============================================================================
// Переменные
extern float a_norm, g_norm;						// нормирующие коэффициенты акселерометра и гироскопа
extern volatile uint8_t DMP_packet_size;

//=========================================================================
// Прототипы функций
uint8_t read_MPU_ID(void);							// чтение адреса MPU из регистра WHO_AM_I
void MPU6050_reset_wakeup(void);					// сброс MPU6050, выход из режима сна
void MPU6050_init(void);							// инициализация MPU6050
void MPU6050_set_sample_rate(void);					// установка частоты дискретизации
void MPU6050_get_norm(float* p_a_norm, float* p_g_norm);
void MPU6050_set_gyro_scale(void);					// расчёт разрешения гироскопа
void MPU6050_set_accel_scale(void);					// расчёт разрешения акселерометра													// расчёт разрешения акселерометра и гироскопа
void MPU6050_read_raw(int16_t* p_ax, int16_t* p_ay, int16_t* p_az, int16_t* p_gx, int16_t* p_gy, int16_t* p_gz); // чтение даных датчика MPU6050
void MPU6050_read_raw_acc(int16_t* p_ax, int16_t* p_ay, int16_t* p_az);
													// чтение данных акселерометра датчика MPU6050
void MPU6050_self_test(void);						// самотестирование MPU6050
void MPU_calibrate(void);							// калибровка MPU6050 (PID)
void gyro_calib(uint8_t loops);						// калибровка гироскопа
void accel_calib(uint8_t loops);					// калибровка акселерометра
float map(float x, float in_min, float in_max, float out_min, float out_max);
													// масштабирование диапазона
void PID(uint8_t read_addr, float kP, float kI, uint8_t loops);		// ПИД-регулятор

void DMP2_init(void);						// инициализация DMP MotionApp_v2
void DMP6_init(void);						// инициализация DMP MotionApp_v6
void set_memory_bank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank);
											// выбор банка памяти
uint8_t write_memory_block(const uint8_t* data, uint16_t dataSize, uint8_t bank, uint8_t addr, uint8_t verify);
											// запись блока памяти
void DMP2_get_quat_16(float* q);			// получение кватернионов (16 бит)
void DMP2_get_quat_32(float* q);			// получение кватернионов (32 бита)
void DMP2_get_euler(float* e, float* q);	// преобразование кватернионов в углы Эйлера
void DMP2_get_gravity(float* g, float* q);	// преобразование кватернионов в проекции вектора силы тяжести G

#endif /* MPU6050_ */


