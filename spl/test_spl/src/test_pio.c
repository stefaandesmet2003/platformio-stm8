//#include "stm8l.h"
#include "stm8s.h"

#ifdef STM8S_BLACK
#define Led_Init GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST)
#define Led_ON   GPIO_WriteHigh    (GPIOE,GPIO_PIN_5)
#define Led_OFF  GPIO_WriteLow     (GPIOE,GPIO_PIN_5)
#define Led_TOG  GPIO_WriteReverse (GPIOE,GPIO_PIN_5)
#endif 

#ifdef STM8S_BLUE
#define Led_Init GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST)
#define Led_ON   GPIO_WriteHigh    (GPIOB,GPIO_PIN_5)
#define Led_OFF  GPIO_WriteLow     (GPIOB,GPIO_PIN_5)
#define Led_TOG  GPIO_WriteReverse (GPIOB,GPIO_PIN_5)
#endif 

void main(void) 
{

   // Init LED Port, Pin
   Led_Init;

   // Set LED ON
   Led_ON;

   // Loop
   while(1){
      // Toggle LED ON/OFF
      Led_TOG;

      // White moment
      for(uint16_t d = 0; d<10000; d++){
         for(uint8_t c = 0; c<5; c++);
      }
   }
}
