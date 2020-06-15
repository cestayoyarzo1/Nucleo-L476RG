/*
  FILE: hal.h
  Created on: April 24th, 2020, by Carlos Estay
  Hardware Abstraction Layer(HAL) header file for STM32L476
*/
#ifndef HAL_H
#define HAL_H

#define RTC_ENABLED true;
#define RUN true
#define BACKUO_DOMAIN_REGISTER_DEFAULT_VALUE 0x00008103
//#define SYSTEM_CLOCK 48000000
#define SYSTEM_CLOCK 80000000

class HAL
{
public:
  static void init(System* sys);
  static uint32_t oneMilliSecondSnapshot();
  //static void UART_ISR(UART_Id uartId);
  static uint32_t usCountDelay();
  static void EXTI_ISR();
  static void CAN1_RX0_IRQHandler();
  static void CAN1_RX1_IRQHandler(); 
  static void usCountTick();
  static void usDelay(uint32_t delay);
  static void setUsCount(bool option); 
  static uint32_t getUsCount();
  
private: 
  
  static void rcc(); //Runs the system clock configuration
  static void uart();
  static void can();
  static void adc();
  static void timer();
  static void exti(); //External interrupts and events
};

#endif /* HAL_H */