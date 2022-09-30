/*
 * Copyright (c) 2021 hpmicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef HPM_SOC_FEATURE_H
#define HPM_SOC_FEATURE_H

#include "hpm_soc.h"

/*
 * I2C Section
 */
#define I2C_SOC_FIFO_SIZE (4U)

/*
 * PMIC Section
 */
#define PCFG_SOC_LDO1P1_MIN_VOLTAGE_IN_MV (700U)
#define PCFG_SOC_LDO1P1_MAX_VOLTAGE_IN_MV (1320U)
#define PCFG_SOC_LDO2P5_MIN_VOLTAGE_IN_MV (2125)
#define PCFG_SOC_LDO2P5_MAX_VOLTAGE_IN_MV (2900U)
#define PCFG_SOC_DCDC_MIN_VOLTAGE_IN_MV (600U)
#define PCFG_SOC_DCDC_MAX_VOLTAGE_IN_MV (1375U)

/*
 * I2S Section
 */
#define I2S_SOC_MAX_CHANNEL_NUM (16U)
#define I2S_SOC_MAX_TX_CHANNEL_NUM (8U)
#define PDM_I2S HPM_I2S0
#define DAO_I2S HPM_I2S1
#define PDM_SOC_SAMPLE_RATE_IN_HZ (16000U)
#define VAD_SOC_SAMPLE_RATE_IN_HZ (16000U)
#define DAO_SOC_SAMPLE_RATE_IN_HZ (48000U)
#define DAO_SOC_PDM_SAMPLE_RATE_RATIO (3U)
#define DAO_SOC_VAD_SAMPLE_RATE_RATIO (3U)

/*
 * PLLCTL Section
 */
#define PLLCTL_SOC_PLL_MAX_COUNT (5U)
/* PLL reference clock in hz */
#define PLLCTL_SOC_PLL_REFCLK_FREQ (24U * 1000000UL)
/* only PLL1 and PLL2 have DIV0, DIV1 */
#define PLLCTL_SOC_PLL_HAS_DIV0(x) ((((x) == 1) || ((x) == 2)) ? 1 : 0)
#define PLLCTL_SOC_PLL_HAS_DIV1(x) ((((x) == 1) || ((x) == 2)) ? 1 : 0)


/*
 * PWM Section
 */
#define PWM_SOC_PWM_MAX_COUNT (8U)
#define PWM_SOC_CMP_MAX_COUNT (24U)
#define PWM_SOC_OUTPUT_TO_PWM_MAX_COUNT (8U)
#define PWM_SOC_OUTPUT_MAX_COUNT (24U)

/*
 * DMA Section
 */
#define DMA_SOC_TRANSFER_WIDTH_MAX(x) (((x) == HPM_XDMA) ? DMA_TRANSFER_WIDTH_DOUBLE_WORD : DMA_TRANSFER_WIDTH_WORD)
#define DMA_SOC_TRANSFER_PER_BURST_MAX(x) (((x) == HPM_XDMA) ? DMA_NUM_TRANSFER_PER_BURST_1024T : DMA_NUM_TRANSFER_PER_BURST_128T)
#define DMA_SOC_BUS_NUM (1U)
#define DMA_SOC_CHANNEL_NUM (8U)
#define DMA_SOC_MAX_COUNT (2U)
#define DMA_SOC_CHN_TO_DMAMUX_CHN(x, n) (((x) == HPM_XDMA) ? (DMAMUX_MUXCFG_XDMA_MUX0 + n) : (DMAMUX_MUXCFG_HDMA_MUX0 + n))

/*
 * PDMA Section
 */
#define PDMA_SOC_PS_MAX_COUNT (2U)

/*
 * LCDC Section
 */
#define LCDC_SOC_MAX_LAYER_COUNT         (8U)
#define LCDC_SOC_MAX_CSC_LAYER_COUNT     (2U)
#define LCDC_SOC_LAYER_SUPPORTS_CSC(x) ((x) < 2)
#define LCDC_SOC_LAYER_SUPPORTS_YUV(x) ((x) < 2)

/*
 * USB Section
 */
#define USB_SOC_MAX_COUNT                          (2U)

#define USB_SOC_DCD_QTD_NEXT_INVALID               (1U)
#define USB_SOC_DCD_QHD_BUFFER_COUNT               (5U)
#define USB_SOC_DCD_QTD_ALIGNMENT                  (32U)
#define USB_SOC_DCD_QHD_ALIGNMENT                  (64U)
#define USB_SOC_DCD_MAX_ENDPOINT_COUNT             (8U)
#define USB_SOC_DCD_MAX_QTD_COUNT                  (USB_SOC_DCD_MAX_ENDPOINT_COUNT * 2U)
#define USB_SOS_DCD_MAX_QHD_COUNT                  (USB_SOC_DCD_MAX_ENDPOINT_COUNT * 2U)
#define USB_SOC_DCD_DATA_RAM_ADDRESS_ALIGNMENT     (2048U)

#define USB_SOC_HCD_QTD_BUFFER_COUNT               (5U)
#define USB_SOC_HCD_QTD_ALIGNMENT                  (32U)
#define USB_SOC_HCD_QHD_ALIGNMENT                  (32U)
#define USB_SOC_HCD_FRAMELIST_MAX_ELEMENTS         (1024U)
#define USB_SOC_HCD_DATA_RAM_ADDRESS_ALIGNMENT     (4096U)

/*
 * ENET Section
 */
#define ENET_SOC_DESC_ADDR_ALIGNMENT               (16U)
#define ENET_SOC_BUFF_ADDR_ALIGNMENT               (4U)
#define ENET_SOC_ADDR_MAX_COUNT                    (5U)
#define ENET_SOC_ALT_EHD_DES_MIN_LEN               (4U)
#define ENET_SOC_ALT_EHD_DES_MAX_LEN               (8U)
#define ENET_SOC_ALT_EHD_DES_LEN                   (8U)

/*
 * ADC Section
 */
#define ADC_SOC_SEQ_MAX_LEN                        (16U)
#define ADC_SOC_MAX_TRIG_CH_LEN                    (4U)
#define ADC_SOC_DMA_ADDR_ALIGNMENT                 (4U)
#define ADC_SOC_CONFIG_INTEN_CHAN_BIT_SIZE         (8U)
#define ADC_SOC_PREEMPT_ENABLE_CTRL_SUPPORT        (0U)
#define ADC_SOC_SEQ_MAX_DMA_BUFF_LEN_IN_4BYTES     (1024U)
#define ADC_SOC_PMT_MAX_DMA_BUFF_LEN_IN_4BYTES     (48U)

#define ADC12_SOC_CLOCK_CLK_DIV                    (2U)
#define ADC12_SOC_CALIBRATION_WAITING_LOOP_CNT     (10)
#define ADC12_SOC_MAX_CH_NUM                       (17U)
#define ADC12_SOC_TEMP_CH_NUM                      (18U)
#define ADC12_SOC_INVALID_TEMP_BASE                (0xF0010000UL)

#define ADC16_SOC_PARAMS_LEN                       (34U)
#define ADC16_SOC_MAX_CH_NUM                       (7U)
#define ADC16_SOC_TEMP_CH_NUM                      (14U)

/*
 * SYSCTL Section
 */
#define SYSCTL_SOC_CPU_GPR_COUNT (14U)
#define SYSCTL_SOC_MONITOR_SLICE_COUNT (4U)

/*
 * PTPC Section
 */
#define PTPC_SOC_TIMER_MAX_COUNT       (2U)

/*
 * CAN Section
 */
#define CAN_SOC_MAX_COUNT       (4U)

/*
 * UART Section
 */
#define UART_SOC_FIFO_SIZE       (16U)

/*
 * SPI Section
 */
#define SPI_SOC_TRANSFER_COUNT_MAX  (512U)

/*
 * SDXC Section
 */
#define SDXC_SOC_MAX_COUNT      (2)

/**
 * @brief Hpmicro clock src exist
 * 
 */
#define HPM_CLK_HAS_SRC_OSC24M        1
#define HPM_CLK_HAS_SRC_PLL0_CLK0        1
#define HPM_CLK_HAS_SRC_PLL1_CLK0        1
#define HPM_CLK_HAS_SRC_PLL1_CLK1        1
#define HPM_CLK_HAS_SRC_PLL2_CLK0        1
#define HPM_CLK_HAS_SRC_PLL2_CLK1        1
#define HPM_CLK_HAS_SRC_PLL3_CLK0        1
#define HPM_CLK_HAS_SRC_PLL4_CLK0        1
#define HPM_CLK_HAS_SRC_OSC32K        1
#define HPM_CLK_HAS_ADC_SRC_AHB0        1
#define HPM_CLK_HAS_ADC_SRC_ANA0        1
#define HPM_CLK_HAS_ADC_SRC_ANA1        1
#define HPM_CLK_HAS_ADC_SRC_ANA2        1
#define HPM_CLK_HAS_I2S_SRC_AHB0        1
#define HPM_CLK_HAS_I2S_SRC_AUD0        1
#define HPM_CLK_HAS_I2S_SRC_AUD1        1
#define HPM_CLK_HAS_I2S_SRC_AUD2        1

/**
 * @brief Hpmicro clock name exist
 * 
 */
#define HPM_CLOCK_HAS_CPU0        1
#define HPM_CLOCK_HAS_CPU1        1
#define HPM_CLOCK_HAS_MCHTMR0        1
#define HPM_CLOCK_HAS_MCHTMR1        1
#define HPM_CLOCK_HAS_AXI0        1
#define HPM_CLOCK_HAS_AXI1        1
#define HPM_CLOCK_HAS_AXI2        1
#define HPM_CLOCK_HAS_AHB        1
#define HPM_CLOCK_HAS_DRAM        1
#define HPM_CLOCK_HAS_XPI0        1
#define HPM_CLOCK_HAS_XPI1        1
#define HPM_CLOCK_HAS_GPTMR0        1
#define HPM_CLOCK_HAS_GPTMR1        1
#define HPM_CLOCK_HAS_GPTMR2        1
#define HPM_CLOCK_HAS_GPTMR3        1
#define HPM_CLOCK_HAS_GPTMR4        1
#define HPM_CLOCK_HAS_GPTMR5        1
#define HPM_CLOCK_HAS_GPTMR6        1
#define HPM_CLOCK_HAS_GPTMR7        1
#define HPM_CLOCK_HAS_UART0        1
#define HPM_CLOCK_HAS_UART1        1
#define HPM_CLOCK_HAS_UART2        1
#define HPM_CLOCK_HAS_UART3        1
#define HPM_CLOCK_HAS_UART4        1
#define HPM_CLOCK_HAS_UART5        1
#define HPM_CLOCK_HAS_UART6        1
#define HPM_CLOCK_HAS_UART7        1
#define HPM_CLOCK_HAS_UART8        1
#define HPM_CLOCK_HAS_UART9        1
#define HPM_CLOCK_HAS_UART10        1
#define HPM_CLOCK_HAS_UART11        1
#define HPM_CLOCK_HAS_UART12        1
#define HPM_CLOCK_HAS_UART13        1
#define HPM_CLOCK_HAS_UART14        1
#define HPM_CLOCK_HAS_UART15        1
#define HPM_CLOCK_HAS_I2C0        1
#define HPM_CLOCK_HAS_I2C1        1
#define HPM_CLOCK_HAS_I2C2        1
#define HPM_CLOCK_HAS_I2C3        1
#define HPM_CLOCK_HAS_SPI0        1
#define HPM_CLOCK_HAS_SPI1        1
#define HPM_CLOCK_HAS_SPI2        1
#define HPM_CLOCK_HAS_SPI3        1
#define HPM_CLOCK_HAS_CAN0        1
#define HPM_CLOCK_HAS_CAN1        1
#define HPM_CLOCK_HAS_CAN2        1
#define HPM_CLOCK_HAS_CAN3        1
#define HPM_CLOCK_HAS_DISPLAY        1
#define HPM_CLOCK_HAS_SDXC0        1
#define HPM_CLOCK_HAS_SDXC1        1
#define HPM_CLOCK_HAS_CAMERA0        1
#define HPM_CLOCK_HAS_CAMERA1        1
#define HPM_CLOCK_HAS_NTMR0        1
#define HPM_CLOCK_HAS_NTMR1        1
#define HPM_CLOCK_HAS_PTPC        1
#define HPM_CLOCK_HAS_REF0        1
#define HPM_CLOCK_HAS_REF1        1
#define HPM_CLOCK_HAS_WATCHDOG0        1
#define HPM_CLOCK_HAS_WATCHDOG1        1
#define HPM_CLOCK_HAS_WATCHDOG2        1
#define HPM_CLOCK_HAS_WATCHDOG3        1
#define HPM_CLOCK_HAS_PUART        1
#define HPM_CLOCK_HAS_PWDG        1
#define HPM_CLOCK_HAS_ETH0        1
#define HPM_CLOCK_HAS_ETH1        1
#define HPM_CLOCK_HAS_PTP0        1
#define HPM_CLOCK_HAS_PTP1        1
#define HPM_CLOCK_HAS_SDP        1
#define HPM_CLOCK_HAS_XDMA        1
#define HPM_CLOCK_HAS_ROM        1
#define HPM_CLOCK_HAS_RAM0        1
#define HPM_CLOCK_HAS_RAM1        1
#define HPM_CLOCK_HAS_USB0        1
#define HPM_CLOCK_HAS_USB1        1
#define HPM_CLOCK_HAS_JPEG        1
#define HPM_CLOCK_HAS_PDMA        1
#define HPM_CLOCK_HAS_KMAN        1
#define HPM_CLOCK_HAS_GPIO        1
#define HPM_CLOCK_HAS_MBX0        1
#define HPM_CLOCK_HAS_MBX1        1
#define HPM_CLOCK_HAS_HDMA        1
#define HPM_CLOCK_HAS_RNG        1
#define HPM_CLOCK_HAS_MOT0        1
#define HPM_CLOCK_HAS_MOT1        1
#define HPM_CLOCK_HAS_MOT2        1
#define HPM_CLOCK_HAS_MOT3        1
#define HPM_CLOCK_HAS_ACMP        1
#define HPM_CLOCK_HAS_PDM        1
#define HPM_CLOCK_HAS_DAO        1
#define HPM_CLOCK_HAS_MSYN        1
#define HPM_CLOCK_HAS_LMM0        1
#define HPM_CLOCK_HAS_LMM1        1
#define HPM_CLOCK_HAS_ANA0        1
#define HPM_CLOCK_HAS_ANA1        1
#define HPM_CLOCK_HAS_ANA2        1
#define HPM_CLOCK_HAS_ADC0        1
#define HPM_CLOCK_HAS_ADC1        1
#define HPM_CLOCK_HAS_ADC2        1
#define HPM_CLOCK_HAS_ADC3        1
#define HPM_CLOCK_HAS_AUD0        1
#define HPM_CLOCK_HAS_AUD1        1
#define HPM_CLOCK_HAS_AUD2        1
#define HPM_CLOCK_HAS_I2S0        1
#define HPM_CLOCK_HAS_I2S1        1
#define HPM_CLOCK_HAS_I2S2        1
#define HPM_CLOCK_HAS_I2S3        1
#define HPM_CLOCK_HAS_OSC0CLK0        1
#define HPM_CLOCK_HAS_PLL0CLK0        1
#define HPM_CLOCK_HAS_PLL1CLK0        1
#define HPM_CLOCK_HAS_PLL1CLK1        1
#define HPM_CLOCK_HAS_PLL2CLK0        1
#define HPM_CLOCK_HAS_PLL2CLK1        1
#define HPM_CLOCK_HAS_PLL3CLK0        1
#define HPM_CLOCK_HAS_PLL4CLK0        1

#endif /* HPM_SOC_FEATURE_H */
