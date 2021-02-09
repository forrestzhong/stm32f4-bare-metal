/*
 * touch.c
 *
 * author: Furkan Cayci & Lin Zhong
 * description:
 *   green, orange and red light up sequentially 
 *  after reset or user button is pressed.
 *
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

enum GPIO_pin_state {OUTPUT_HIGH, INPUT_X, INPUT_LOW};
enum bool {true,false};

/* Global indicator whether the user button is pressed */
enum bool pushed = false;

/*************************************************
* function declarations
*************************************************/
int main(void);

/*************************************************
* external interrupt handler
*************************************************/
void EXTI0_IRQHandler(void)
{
    
    // Check if the interrupt came from exti0
    if (EXTI->PR & (1 << 0)){

        /* Stop counter */

        pushed = true; //Indicate measurement is ready
        GPIOD->ODR |= (uint16_t)(0x8000); //blue on
        
        // Clear pending bit
        EXTI->PR = (1 << 0);

    }
}


/*************************************************
* timer 2 interrupt handler
*************************************************/
void TIM2_IRQHandler(void)
{
    static enum GPIO_pin_state state = INPUT_LOW;
    
    // clear interrupt status
    if (TIM2->DIER & 0x01) {
        if (TIM2->SR & 0x01) {
            TIM2->SR &= ~(1U << 0);
        }
    }
     
    switch(state) {

        case INPUT_LOW:
            // Set the pin 7 as output, corresponding to 15:14=01
            //RM0090 8.4.1
            //MODER is the port mode register
            GPIOC->MODER &= 0xFFFF7FFF; //set bit 15 to 0
            GPIOC->MODER |= 0x00004000; //set bit 14 to 1
            GPIOC->ODR = (1U<<7); //output high
            
            GPIOD->ODR |= 0x1000; //green
            state = OUTPUT_HIGH;

            break;
        case OUTPUT_HIGH:
            pushed = false; // measurement not ready
            GPIOD->ODR &= (uint16_t)0x7FFF;//blue off


            GPIOD->ODR |= 0x2000;//orange
            state = INPUT_X;
            break;
        case INPUT_X:

            if(pushed==true) {
                GPIOD->ODR = 0x00000000;
                state = INPUT_LOW;
            } else{
                GPIOD->ODR |= 0x4000;//red

            }
            break;
    } 
    
}


/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    /* Set up LED display */
    // setup LEDs
    RCC->AHB1ENR |= (1 << 3);
    GPIOD->MODER &= 0x00FFFFFF;
    GPIOD->MODER |= 0x55000000;
    GPIOD->ODR = 0x0000;

    /* Set up timer interrupt */
    // enable TIM2 clock (bit0)
    RCC->APB1ENR |= (1 << 0);
     
    // Timer clock runs at ABP1 * 2
    //   since ABP1 is set to /4 of fCLK
    //   thus 168M/4 * 2 = 84Mhz
    // set prescaler to 83999
    //   it will increment counter every prescalar cycles
    // fCK_PSC / (PSC[15:0] + 1)
    // 84 Mhz / 8399 + 1 = 10 khz timer clock speed
    TIM2->PSC = 8399;

    // Set the auto-reload value to 10000
    //   which should give 1 second timer interrupts
    // 10000 measn 1000 ms timer interrupt
    TIM2->ARR = 4000;

    // Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    NVIC_SetPriority(TIM2_IRQn, 2); // Priority level 2
    // enable TIM2 IRQ from NVIC
    NVIC_EnableIRQ(TIM2_IRQn);

    // Enable Timer 2 module (CEN, bit0)
    TIM2->CR1 |= (1 << 0);


    // enable SYSCFG clock (APB2ENR: bit 14)
    RCC->APB2ENR |= (1 << 14);

    /* tie push button at PA0 to EXTI0 */
    // EXTI0 can be configured for each GPIO module.
    //   EXTICR1: 0b XXXX XXXX XXXX 0000
    //               pin3 pin2 pin1 pin0
    //
    //   Writing a 0b0000 to pin0 location ties PA0 to EXT0
    SYSCFG->EXTICR[0] |= 0x00000000; // Write 0000 to map PA0 to EXTI0

    // Choose either rising edge trigger (RTSR) or falling edge trigger (FTSR)
    EXTI->RTSR |= 0x00001;   // Enable rising edge trigger on EXTI0

    // Mask the used external interrupt numbers.
    EXTI->IMR |= 0x00001;    // Mask EXTI0

    // Set Priority for each interrupt request
    NVIC_SetPriority(EXTI0_IRQn, 1); // Priority level 1

    // enable EXT0 IRQ from NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);


    while(1)
    {
        // Do nothing.
    }

    return 0;
}
