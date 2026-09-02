/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU     STM32H743

#define BOARD_NAME        ELARIONH743
#define MANUFACTURER_ID   ELARION

#define USE_ACC
#define USE_GYRO

#define ACCGYRO_SPI_LSM6DSO
#define USE_ACCGYRO_LSM6DSO

// #define USE_ACC_SPI_ICM42688P
// #define USE_GYRO_SPI_ICM42688P

#define USE_BARO
#define USE_BARO_LPS22BH
#define USE_BARO_BMP581
#define USE_MAX7456
#define USE_MAG
#define USE_MAG_LIS2MDL

//Both use PC11, so choose: flash or sdcard blackbox
#define USE_FLASH
#define USE_FLASH_M25P16
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH

// #define USE_SDCARD
// #define USE_SDCARD_SPI
// #define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD


#define BEEPER_PIN           PA15
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PA0
#define MOTOR4_PIN           PA1
#define MOTOR5_PIN           PA2
#define MOTOR6_PIN           PA3
#define MOTOR7_PIN           PD12
#define MOTOR8_PIN           PD13
#define SERVO1_PIN           PE5
#define SERVO2_PIN           PE6
#define RX_PPM_PIN           PC7
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PB9
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PB8
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0
#define I2C1_SCL_PIN         PB6
#define I2C2_SCL_PIN         PB10
#define I2C1_SDA_PIN         PB7
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PE3
// #define LED1_PIN             PE4
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB3
#define SPI4_SCK_PIN         PE12
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PB4
#define SPI4_SDI_PIN         PE13
#define SPI1_SDO_PIN         PD7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5
#define SPI4_SDO_PIN         PE14
#define ADC_VBAT_PIN         PC0
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC1
#define ADC_EXTERNAL1_PIN    PC4  //ADC12  AirS
#define ADC_EXTERNAL2_PIN    PA4  //ADC12  VB2 
#define ADC_EXTERNAL3_PIN    PA7  //ADC12  CU2


#define FLASH_CS_PIN         PC11
#define FLASH_SPI_INSTANCE SPI3

// #define SDCARD_SPI_CS_PIN    PC11
// #define SDCARD_SPI_INSTANCE  SPI3
// #define SDCARD_DETECT_PIN    NONE


#define PINIO2_PIN           PD10
#define PINIO1_PIN           PD11

#define MAX7456_SPI_CS_PIN   PC14

#define GYRO_1_EXTI_PIN      PC2
#define GYRO_2_EXTI_PIN      PE9
#define GYRO_1_CS_PIN        PC10
#define GYRO_2_CS_PIN        PC3


#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB0 , 2,  0) \
    TIMER_PIN_MAP( 1, PB1 , 2,  0) \
    TIMER_PIN_MAP( 2, PA0 , 2,  0) \
    TIMER_PIN_MAP( 3, PA1 , 2,  0) \
    TIMER_PIN_MAP( 4, PA2 , 2,  0) \
    TIMER_PIN_MAP( 5, PA3 , 2,  0) \
    TIMER_PIN_MAP( 6, PD12, 1,  0) \
    TIMER_PIN_MAP( 7, PD13, 1,  0) \
    TIMER_PIN_MAP( 8, PE5 , 1, -1) \
    TIMER_PIN_MAP( 9, PE6 , 1, -1) \
    TIMER_PIN_MAP(10, PA8 , 1,  0) \
    TIMER_PIN_MAP(11, PA15, 1, -1) \
    TIMER_PIN_MAP(12, PC7 , 2, -1) 



#define ADC1_DMA_OPT        8
#define ADC3_DMA_OPT        9
#define TIMUP1_DMA_OPT      0
#define TIMUP2_DMA_OPT      0
#define TIMUP3_DMA_OPT      2
#define TIMUP4_DMA_OPT      1
#define TIMUP5_DMA_OPT      0
#define TIMUP8_DMA_OPT      0

#define BARO_I2C_INSTANCE I2CDEV_1
#define MAG_I2C_INSTANCE I2CDEV_2

#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL
#define SERIALRX_UART SERIAL_PORT_USART7


#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE_DEFAULT 110
#define DEFAULT_CURRENT_METER_SCALE 250
#define BEEPER_INVERTED
#define BEEPER_PWM_HZ 2500


#define MAX7456_SPI_INSTANCE SPI1

// The OSD is an STM32G431 running MAX7456 emulation firmware (SW-OSD), not the real
// chip. Its host SPI slave takes one interrupt per byte at a lower NVIC priority than
// its own video sync and pixel DMA, so it drops bytes out of long uninterrupted
// bursts - missing characters on screen, no error reported. Betaflight's DMA path
// sends up to 250 bytes back to back; the polled path sends 12 with gaps.
#define MAX7456_DISABLE_DMA


#define PINIO1_BOX 40
#define PINIO2_BOX 41

#define GYRO_1_SPI_INSTANCE SPI2
#define GYRO_1_ALIGN CW0_DEG_FLIP
#define GYRO_2_SPI_INSTANCE SPI2
#define GYRO_2_ALIGN CW0_DEG_FLIP
#define DEFAULT_GYRO_TO_USE GYRO_CONFIG_USE_GYRO_1

#define GPS_UART SERIAL_PORT_USART3
#define VTX_TRAMP_UART SERIAL_PORT_USART6