/**
 * File: stm32f767xx.h
 *
 * Last reviewed and updated: 2025/06/25
 * Author: VanTungDinh
 */

#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

#include <stdio.h>
#include <stdint.h>


/**
 * Name:                            System memory base addresses
 * Last reviewed and updated:       2025/06/25
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
#define NVIC_BASE           0xE000E100UL

#define FLASH_BASE          0x08000000UL
#define SRAM1_SIZE			(368 * 1024)					/* 368 KB * 1014 = 367832 B */
#define SRAM1_BASE          0x20020000UL
#define SRAM2_SIZE			(16 * 1024)						/* 16 KB * 1024 = 16384 B */
#define SRAM2_BASE          0x2007C000UL
#define DTCM_SIZE			(128 * 1024)					/* 128 KB * 1024 = 131072 B */
#define DTCM_BASE			0x20000000UL
#define ITCM_SIZE			(16 * 1024)						/* 16 KB * 1024 = 16384 B */
#define ITCM_BASE			0x00000000UL
#define SRAM_BASE           SRAM1_BASE
#define ROM_BASE            0x1FF00000UL			       	/* System memory */

#define PERIPHERAL_BASE     0x40000000UL
#define APB1_BASE           0x40000000UL
#define APB2_BASE           0x40010000UL
#define AHB1_BASE           0x40020000UL
#define AHB2_BASE           0x50000000UL
#define AHB3_BASE           0xA0000000UL


/**
 * Name:                            AHB1 peripheral base addresses
 * Last reviewed and updated:       2025/06/25
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
#define GPIOA_BASE      (AHB1_BASE + 0x0000)
#define GPIOB_BASE      (AHB1_BASE + 0x0400)
#define GPIOC_BASE      (AHB1_BASE + 0x0800)
#define GPIOD_BASE      (AHB1_BASE + 0x0C00)
#define GPIOE_BASE      (AHB1_BASE + 0x1000)
#define GPIOF_BASE      (AHB1_BASE + 0x1400)
#define GPIOG_BASE      (AHB1_BASE + 0x1800)
#define GPIOH_BASE      (AHB1_BASE + 0x1C00)
#define GPIOI_BASE		(AHB1_BASE + 0x2000)
#define GPIOJ_BASE		(AHB1_BASE + 0x2400)
#define GPIOK_BASE		(AHB1_BASE + 0x2800)
#define RCC_BASE        (AHB1_BASE + 0x3800)


/**
 * Name:                            APB1 peripheral base addresses
 * Last reviewed and updated:       2025/06/25
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
#define I2C1_BASE        	(APB1_BASE + 0x5400)
#define I2C2_BASE        	(APB1_BASE + 0x5800)
#define I2C3_BASE        	(APB1_BASE + 0x5C00)
#define I2C4_BASE			(APB1_BASE + 0x6000)
#define SPI2_BASE        	(APB1_BASE + 0x3800)
#define SPI3_BASE        	(APB1_BASE + 0x3C00)
#define USART2_BASE			(APB1_BASE + 0x4400)
#define USART3_BASE			(APB1_BASE + 0x4800)
#define UART4_BASE       	(APB1_BASE + 0x4C00)
#define UART5_BASE       	(APB1_BASE + 0x5000)


/**
 * Name:                            APB2 peripheral base addresses
 * Last reviewed and updated:       2025/06/25
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
#define SPI1_BASE        	(APB2_BASE + 0x3000)
#define SPI4_BASE			(APB2_BASE + 0x3400)
#define SPI5_BASE			(APB2_BASE + 0x5000)
#define SPI6_BASE			(APB2_BASE + 0x5400)
#define USART1_BASE      	(APB2_BASE + 0x1000)
#define USART6_BASE      	(APB2_BASE + 0x1400)
#define EXTI_BASE        	(APB2_BASE + 0x3C00)
#define SYSCFG_BASE      	(APB2_BASE + 0x3800)


#endif /* INC_STM32F767XX_H_ */
