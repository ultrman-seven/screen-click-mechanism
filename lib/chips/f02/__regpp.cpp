#include "mm32_reg.h"
#include "wyGpio.hpp"
#include "cppHalReg.hpp"
#include "wySys.hpp"

#define __RCC_APB2ENR_OFFSET 0x18
#define __RCC_APB1ENR_OFFSET 0x1c

const uint32_t __GPIO_PORT_BASEs[] = {GPIOA_BASE, GPIOB_BASE, GPIOC_BASE};
const uint32_t __GPIO_PORT_RCC_EN[] = {RCC_AHBENR_GPIOA, RCC_AHBENR_GPIOB, RCC_AHBENR_GPIOC};

const uint32_t __TIM_BASEs[] = {TIM1_BASE, TIM3_BASE, TIM14_BASE};
const uint32_t __TIM_RCC_EN[] = {RCC_APB2ENR_TIM1, RCC_APB1ENR_TIM3, RCC_APB2ENR_TIM14};

const uint32_t __UART_BASEs[] = {UART1_BASE, UART2_BASE, UART3_BASE, UART4_BASE};
// const uint32_t __UART_RCC_EN[] = {RCC_APB2ENR_UART1, RCC_APB1ENR_UART2};
const uint16_t __UART_RCC_ENR_CFG_OFFSETs[] = {
    ((__RCC_APB2ENR_OFFSET) << 8) + RCC_APB2ENR_UART1_Pos,
    ((__RCC_APB1ENR_OFFSET) << 8) + RCC_APB1ENR_UART2_Pos,
    ((__RCC_APB1ENR_OFFSET) << 8) + RCC_APB1ENR_UART3_Pos,
    ((__RCC_APB1ENR_OFFSET) << 8) + RCC_APB1ENR_UART4_Pos};

const uint32_t __TIM_IDXs[] = {
    0, 6, 1, 6, 6,
    6, 6, 6, 6, 6,
    6, 6, 6, 2};

const uint16_t __UART1_Rx_GPIO_AFs[] = {
    __GPIO_AF_Val(0, 9, 3), __GPIO_AF_Val(0, 10, 1), // a
    __GPIO_AF_Val(1, 7, 0),                          // b
    0xffff};
const uint16_t __UART1_Tx_GPIO_AFs[] = {
    __GPIO_AF_Val(0, 9, 1), __GPIO_AF_Val(0, 10, 3), // a
    __GPIO_AF_Val(1, 6, 0),                          // b
    0xffff};

const uint16_t __UART2_Rx_GPIO_AFs[] = {
    __GPIO_AF_Val(0, 3, 1), __GPIO_AF_Val(0, 15, 1), // a
    __GPIO_AF_Val(3, 6, 0),                          // d
    0xffff};
const uint16_t __UART2_Tx_GPIO_AFs[] = {
    __GPIO_AF_Val(0, 2, 1), __GPIO_AF_Val(0, 14, 1), // a
    __GPIO_AF_Val(3, 5, 0),                          // d
    0xffff};

const uint16_t __UART3_Rx_GPIO_AFs[] = {
    __GPIO_AF_Val(1, 11, 4),                         // b
    __GPIO_AF_Val(2, 5, 1), __GPIO_AF_Val(2, 11, 1), // c
    __GPIO_AF_Val(3, 9, 0),                          // d
    0xffff};
const uint16_t __UART3_Tx_GPIO_AFs[] = {
    __GPIO_AF_Val(1, 10, 4),                         // b
    __GPIO_AF_Val(2, 4, 1), __GPIO_AF_Val(2, 10, 1), // c
    __GPIO_AF_Val(3, 8, 0),                          // d
    0xffff};

const uint16_t __UART4_Rx_GPIO_AFs[] = {
    __GPIO_AF_Val(0, 1, 4),  // a
    __GPIO_AF_Val(2, 11, 0), // c
    0xffff};
const uint16_t __UART4_Tx_GPIO_AFs[] = {
    __GPIO_AF_Val(0, 0, 4),  // a
    __GPIO_AF_Val(2, 10, 0), // c
    0xffff};

#define __UART1_BUS sys::_MCU_BUS_APB2
#define __UART2_BUS sys::_MCU_BUS_APB1
#define __UART3_BUS sys::_MCU_BUS_APB1
#define __UART4_BUS sys::_MCU_BUS_APB1

uint32_t const __UART_BUS_CFG = __UART1_BUS + (__UART2_BUS << 2) + (__UART3_BUS << 4) + (__UART4_BUS << 6);

uint16_t const *const __UART_Rx_GPIO_AFs[] = {__UART1_Rx_GPIO_AFs, __UART2_Rx_GPIO_AFs, __UART3_Rx_GPIO_AFs, __UART4_Rx_GPIO_AFs};
uint16_t const *const __UART_Tx_GPIO_AFs[] = {__UART1_Tx_GPIO_AFs, __UART2_Tx_GPIO_AFs, __UART3_Tx_GPIO_AFs, __UART4_Tx_GPIO_AFs};

IRQn_Type const __UART_IRQ[] = {UART1_IRQn, UART2_IRQn, UART3_4_IRQn};
IRQn_Type const __TIM_IRQ[] = {TIM1_BRK_UP_TRG_COM_IRQn, TIM3_IRQn, TIM14_IRQn};