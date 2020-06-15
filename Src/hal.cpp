/*
  FILE: hal.cpp
  Created on: April 24th, 2020, by Carlos Estay
  Hardware Abstraction Layer(HAL) source file for STM32L476
*/

#include "includes.h"
 
static volatile uint32_t masterCount;
static volatile uint32_t usCounter;
System* p_SysPtr;
bool m_ToggleState;

//--------------------------------------------------------------------------Init
void HAL::init(System* sys)
{
  p_SysPtr = sys;
  __disable_irq();
  rcc();
  uart();
  timer();
  exti();
  __enable_irq();
  //adc has to be enabled after interruptas as uS counter uses TIM6 interrupt
  adc();
}
//--------------------------------------------------------------------------Tick
uint32_t HAL::oneMilliSecondSnapshot()
{
  return masterCount;
}
//----------------------------------------------------------------------UART ISR

//----------------------------------------------------------------------ESTI ISR
void HAL::EXTI_ISR()
{ //A switch statement must be added for more than 1 EXTI Interrupt
  //Right now, it's only available on BUtton (PC13) for testing purposes
  if(GPIOC->IDR & GPIO_IDR_ID4)
  {//GPIO lvl is high (1)


  }
  else
  {//GPIO lvl is low (0)

  } 
  EXTI->PR1 |= EXTI_PR1_PIF4;                  //clear pending request for EXTI0
  NVIC_ClearPendingIRQ(EXTI4_IRQn);         //clear flag  
}
//-----------------------------------------------------------------------us Tick
void HAL::usCountTick()
{
  ++usCounter;
}
//----------------------------------------------------------------------us Delay
void HAL::usDelay(uint32_t delay)
{
  setUsCount(true);
  usCounter = 0;
  while(usCounter < delay);
  setUsCount(false);
}
//-----------------------------------------------------Set us Count (start/stop)
void HAL::setUsCount(bool option)
{
  if(option)
  {
    TIM7->CR1 |= TIM_CR1_CEN;
    
  }
  else
  {
    TIM7->CR1 &= ~TIM_CR1_CEN;
  }
}
//------------------------------------------------------------------------------
uint32_t HAL::getUsCount()
{
  return usCounter;
}
//**********************PRIVATE METHODS*****************************************
void HAL::rcc()
{
  uint32_t BDCR = 0;
  
  //Enable peripherals
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;  //Enable power interface
  
  //Enable instruction prefetch, data and instruction cache
  FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
  
  //Slowdown FLASH
  FLASH->ACR &= ~FLASH_ACR_LATENCY;
  FLASH->ACR |= FLASH_ACR_LATENCY_1WS;
  
  //System will be running from MSI trimmed from LSE
  
  
  //Configure LSE
  BDCR = RCC->BDCR;
  if(BDCR != BACKUO_DOMAIN_REGISTER_DEFAULT_VALUE)
  {
    PWR->CR1 |= PWR_CR1_DBP;                      //Enable backup domain write
    
    RCC->BDCR &= ~RCC_BDCR_BDRST;                 //Do NOT reset backup domain
    RCC->BDCR &= ~RCC_BDCR_LSEBYP;                //No LSE Bypass (use crystal)
    RCC->BDCR |= RCC_BDCR_LSEON;                  //LSE ON
    while(!(RCC->BDCR & RCC_BDCR_LSERDY));        //Wait until LSE is Ready
    
    
    bool test = RTC_ENABLED;
    
    if(test)
    {
      RCC->BDCR |= RCC_BDCR_RTCSEL_0;             //LSE as RTC clock
      RCC->BDCR |= RCC_BDCR_RTCEN;                //Enable RTC
    }
    
    PWR->CR1 &= ~PWR_CR1_DBP;                     //Disable backup domain write
  }
  
  //Configure MSI
  RCC->CR &= ~RCC_CR_MSION;             //Turn MSI Off
  RCC->CR &= ~RCC_CR_MSIPLLEN;          //Disable MSI PLL
  RCC->CR |= RCC_CR_MSIRGSEL;           //Take Frequency Range from RCC->CR
  RCC->CFGR &= ~RCC_CFGR_MCOSEL;        //Clear MCOSEL
  //RCC->CFGR |= RCC_CFGR_MCOSEL_1;       //MSI as MCO
  RCC->CFGR |= RCC_CFGR_MCOSEL_0;       //SysCLK as MCO
  RCC->CFGR |= RCC_CFGR_MCOPRE_2;        //MCO is divided by 16, should see SYSTEM_CLOCK / 16
  RCC->CR &= ~RCC_CR_MSIRANGE;          //Clear Range
  RCC->CR |= RCC_CR_MSIRANGE_8;        //16Mhz Range
  RCC->CR |= RCC_CR_MSIPLLEN;           //Enable MSI PLL (Auto trim by LSE)
  RCC->CR |= RCC_CR_MSION;              //Turn On MSI
  while(!(RCC->CR & RCC_CR_MSIRDY));    //Wait until MSI is stable
  
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_MSI; //MSI as PLL clock Source
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;    //Clear PLLM
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;     //Clear PLLN
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0; //PLLM = 4
  //RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_3; //PLLN = 24 , 16/4 X 24/2 = 48[MHZ]
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_3; //PLLN = 24 , 16/4 X 40/2 = 80[MHZ]
                                        // PLLR is 2 after reset
  
  RCC->CR |= RCC_CR_PLLON;              //Turn On PLL
  while(!(RCC->CR & RCC_CR_PLLRDY));      //Wait until PLL is locked
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;   //Enable PLL Output
  
  RCC->CFGR &= ~RCC_CFGR_SW;            //Clear SySClk selection (fallback to MSI)
  RCC->CFGR |= RCC_CFGR_SW_0 | RCC_CFGR_SW_1; //PLL as SYSCLK
  
  SysTick_Config(SYSTEM_CLOCK / 1000); //1ms Ticks
  NVIC_EnableIRQ(SysTick_IRQn);
  
  //Enabling Peripherals
  //PORTS
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;  //Enable PORT A
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;  //Enable PORT B
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;  //Enable PORT C
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;  //Enable PORT D
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;  //Enable PORT E
  //USARTS/UARTS
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;         //Enable USART1
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;       // Enable USART2

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //SYSCFG+COMP+VEREFBUF,necessary for GPIO(EXTI) interrupts
  
  //Set PA5 to GPIO Outpput
//  GPIOA->MODER &= ~GPIO_MODER_MODER5;           //Clear Moder
//  GPIOA->MODER |= GPIO_MODER_MODER5_0;          //GP Output mode 
  
  //Set PA8 as MCO
  GPIOA->MODER &= ~GPIO_MODER_MODER8;           //Clear Moder
  GPIOA->MODER |= GPIO_MODER_MODER8_1;          //Set to alternate function   
}
//-------------------------------------------------------------------------Uarts
void HAL::uart()
{
  //USART 1 is connected to the ouside world
  
  //PA9 -> USART1_TX
  GPIOA->MODER &= ~GPIO_MODER_MODER9;           //Clear moder for PA2
  GPIOA->MODER |= GPIO_MODER_MODER9_1;         //Set Moder to Alternate (10b)
  GPIOA->AFR[1] |= 7 << (1 * 4);                //Set PA2 to AF7 (USART2_TX) 
   
  //PA10 -> USART1_RX
  GPIOA->MODER &= ~GPIO_MODER_MODER10;           //Clear moder for PA3
  GPIOA->MODER |= GPIO_MODER_MODER10_1;         //Set Moder to Alternate (10b)
  GPIOA->AFR[1] |= 7 << (2 * 4);                //Set PA3 to AF7 (USART2_RX)  
  
  NVIC_EnableIRQ(USART1_IRQn);                  // CM4 Intrinsic
  //NVIC_SetPriority(USART1_IRQn, 4);
  USART1->CR1 |= USART_CR1_RXNEIE;              //Enable USART 2 Receiving Interrupt
  
  //USART 2 is connected to ESP32 Module
  
  //PA2 -> USART2_TX  
  GPIOA->MODER &= ~GPIO_MODER_MODER2;           //Clear moder for PA2
  GPIOA->MODER |= GPIO_MODER_MODER2_1;         //Set Moder to Alternate (10b)
  GPIOA->AFR[0] |= 7 << (2 * 4);                //Set PA2 to AF7 (USART2_TX)
    
  //PA3 -> USART2_RX 
  GPIOA->MODER &= ~GPIO_MODER_MODER3;           //Clear moder for PA3
  GPIOA->MODER |= GPIO_MODER_MODER3_1;         //Set Moder to Alternate (10b)
  GPIOA->AFR[0] |= 7 << (3 * 4);                //Set PA3 to AF7 (USART2_RX)
  
  NVIC_EnableIRQ(USART2_IRQn);                  // CM4 Intrinsic
  //NVIC_SetPriority(USART2_IRQn, 4);
  USART2->CR1 |= USART_CR1_RXNEIE;              //Enable USART 2 Receiving Interrupt
    
}

//---------------------------------------------------------------------------ADC
void HAL::adc()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;    //Enable ADC on AHB2 (Clock)
  
  GPIOC->ASCR |= GPIO_ASCR_ASC1;        //connect PC1 to to ADC input
  ADC1->CR &= ~ADC_CR_DEEPPWD;          //Disable deep-power down
  ADC1->CR |= ADC_CR_ADVREGEN;          //Enable ADC Voltage regulator
 
  // TADCVREG_STUP is 20 uS, lets give it 100us
  HAL::usDelay(100);
  
  ADC123_COMMON->CCR |= ADC_CCR_CKMODE_1;       // 40 Mhz
  ADC123_COMMON->CCR |= ADC_CCR_PRESC_3;        //PCLK2 /8 = 5Mhz
  //ADC123_COMMON->CCR |= ADC_CCR_TSEN | ADC_CCR_VBATEN | ADC_CCR_VREFEN; //Battery and temperature channels
  
  
  //-------------------------ADC1 Auto Calibration------------------------------
  if( ADC1->CR & ADC_CR_ADEN)   // If adc is enabled
  {
    ADC1->CR |= ADC_CR_ADDIS;   // Disable it
    while(ADC1->CR & ADC_CR_ADEN); // Wait until it is disabled
  }
  ADC1->CR &= ~ADC_CR_ADCALDIF;         // Single ended channels calibration
  ADC1->CR |= ADC_CR_ADCAL;             // Start calibration
  while(ADC1->CR & ADC_CR_ADCAL);      // Wait until it is done
  //p_SysPtr->analogInput.init(ADC1->CALFACT & ADC_CALFACT_CALFACT_S);//get the calibration factor
  
  ADC1->CR |= ADC_CR_ADEN; // adc on
  while(!(ADC1->ISR & ADC_ISR_ADRDY));  //Wait till ready
  //ADC1->ISR |= ADC_ISR_ADRDY;  
  
//  ADC1->SQR1 |= ANALOG_CHANNELS - 1;                      // Sequence of 1 channels
//  ADC1->SQR1 |= 2 << (ANALOG_CHANNEL_SHIFT * 1);        //ADC1_IN2;
  
  //ADC1 channel configured for 640.5 clock cycles of sampling
  ADC1->SMPR1 |= 7 << 6;                                //ADC1_IN2 setting;      
  
  ADC1->CFGR |= ADC_CFGR_OVRMOD; //Register is overwritten when overrun is detected
  ADC1->CFGR &= ~ADC_CFGR_CONT; //Only one conversion of every channel
  
  ADC1->IER |= ADC_IER_EOCIE;   //End of conversion interrupt enable
  NVIC_EnableIRQ(ADC1_2_IRQn);
  
  TIM6->CR1 |= TIM_CR1_CEN;             //Start Timer for triggering
  //ADC1->CFGR |= ADC_CFGR_DMAEN;  //Enable DMA
//  ADC1->CR |= ADC_CR_ADSTART; // Start conversion
}
//------------------------------------------------------------------------Timer
void HAL::timer()
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN; // Enable Timer 6 on APB1 (ADC trigger)
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN; // Enable Timer 7 on APB1 (uS Counter)
  
  //uint32_t timer6Presc = SYSTEM_CLOCK / 250000 - 1;  //Prescaler at 250KHZ
  //uint32_t timer7Presc = SYSTEM_CLOCK / 8000000 - 1; //pre-scaler at 10MHZ
  //uint32_t timer7Presc = 1;                            //pre-scaler at 40MHZ
  
//  //TIM6, used for ADC sampling rate
  TIM6->CR1 &= ~TIM_CR1_CEN;            //Stop timer
  TIM6->CR1 |= TIM_CR1_ARPE;            //Auto-reload preload enable
  TIM6->CR1 |= TIM_CR1_URS;             //Only counter overflow/underflow generates Interupt
  TIM6->CR2 |= TIM_CR2_MMS_1;           //Master mode ->Update (TRGO)
  TIM6->PSC = 9;                        //prescaler at 8MHZ
  TIM6->ARR = 500;                      //500 counts, 16KHZ update;
  //TIM6->EGR |= TIM_EGR_UG;            //Enable register update
  TIM6->DIER |= TIM_DIER_UIE;           //Update Interrupt Enable
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  //TIM6->CR1 |= TIM_CR1_CEN;             //Start Timer
  //TIM6->SR &= ~TIM_SR_UIF;            //Clear update event      
  
  //Timer7, used for uS Counter
  TIM7->CR1 &= ~TIM_CR1_CEN;            //Stop timer
  TIM7->CR1 |= TIM_CR1_ARPE;            //Auto-reload preload enable
  TIM7->CR1 |= TIM_CR1_URS;             //Only counter overflow/underflow generates Interupt
  TIM7->CR2 |= TIM_CR2_MMS_1;           //Master mode ->Update (TRGO)
  //TIM7->PSC = timer7Presc;              //TIM6 prescaler at 4MHZ
  TIM7->PSC = 1;                        //prescaler at 40MHZ
  TIM7->ARR = 40;                       //40 counts, 1MHZ update;
  //TIM7->EGR |= TIM_EGR_UG;            //Enable register update
  TIM7->DIER |= TIM_DIER_UIE;           //Update Interrupt Enable
  NVIC_EnableIRQ(TIM7_IRQn);
  TIM7->CR1 |= TIM_CR1_CEN;             //Start Timer
  //TIM7->SR &= ~TIM_SR_UIF;            //Clear update event      
}
//------------------------------------------------------External Interrupt/Event
void HAL::exti()
{
  GPIOC->MODER &= ~GPIO_MODER_MODER4;           //Clear moder for PC4, set it to Input mode
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC; //EXTI0 PC0 enabled       
  EXTI->IMR1 |= EXTI_IMR1_IM4;                 //Interrupt request not mask on line 0
  EXTI->RTSR1 |= EXTI_RTSR1_RT4;               //Rising Trigger enabled on line 0
  EXTI->FTSR1 |= EXTI_FTSR1_FT4;               //Falling Trigger enabled on line 0
  NVIC_EnableIRQ(EXTI4_IRQn);                 //Enable IRQ
}

//**********************Native C interrupts*************************************

  //Systick
  #pragma call_graph_root="interrupt"
  void SysTick_Handler(void)
  {
    masterCount++;
  }
  //USART1
  #pragma call_graph_root="interrupt"
  void USART1_IRQHandler(void)
  {

    NVIC_ClearPendingIRQ(USART1_IRQn);   // Clear Flag
  }  
  //USART2
  #pragma call_graph_root="interrupt"
  void USART2_IRQHandler(void)
  {

    NVIC_ClearPendingIRQ(USART2_IRQn);   // Clear Flag
  }
  //USART3
  #pragma call_graph_root="interrupt"
  void USART3_IRQHandler(void)
  {

    NVIC_ClearPendingIRQ(USART3_IRQn);   // Clear Flag
  }    
  //UART4
  #pragma call_graph_root="interrupt"
  void UART4_IRQHandler(void)
  {

    NVIC_ClearPendingIRQ(UART4_IRQn);   // Clear Flag
  } 
  //UART5
  #pragma call_graph_root="interrupt"
  void UART5_IRQHandler(void)
  {
    NVIC_ClearPendingIRQ(UART5_IRQn);   // Clear Flag
  }  
  //EXTI 
  #pragma call_graph_root="interrupt"
  void EXTI4_IRQHandler(void)   //EXT4 handler
  {
     HAL::EXTI_ISR();
  } 
  
//  #pragma call_graph_root="interrupt"
//  void EXTI15_10_IRQHandler(void)  //EXT 5-10 Hanlder
//  {
//    //HAL::EXTI_ISR();
//  } 

  //Timer 6
  #pragma call_graph_root="interrupt"
  void TIM6_DAC_IRQHandler(void)
  {
    if(!m_ToggleState)
    {
      m_ToggleState = true;
      GPIOC->BSRR |= GPIO_BSRR_BS_5;        // Set High     
    }
    else
    {
      m_ToggleState = false;
      GPIOC->BSRR |= GPIO_BSRR_BR_5;        // Set Low 
    }   
    ADC1->CR |= ADC_CR_ADSTART; // Start conversion
    TIM6->SR &= ~TIM_SR_UIF;             //Clear flag
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);    //Clear IRQ
  }  
  //Timer 7
  #pragma call_graph_root="interrupt"
  void TIM7_IRQHandler(void)
  {
    ++usCounter;
    TIM7->SR &= ~TIM_SR_UIF;             //Clear flag
    NVIC_ClearPendingIRQ(TIM7_IRQn);    //Clear IRQ
  }  
  
  // ADC end of conversion
  #pragma call_graph_root="interrupt"
  void ADC1_2_IRQHandler(void)
  {
   ADC1->ISR |= ADC_ISR_EOC;   
  }
 
  //Hard Fault
  #pragma call_graph_root="interrupt"
  void HardFault_Handler(void)
  {
    //p_SysPtr->usart2.writeString("Hard Fault\r\n");
    while(true){}
  } 