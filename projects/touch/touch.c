/*
 * touch.c
 *
 * author: Furkan Cayci & Lin Zhong
 * description:
 *   blinks LEDs one at a time using timer interrupt
 *   timer2 is used as the source, and it is setup
 *   to run at 10 kHz. LED blinking rate is set to
 *   1 second.
 *
 * timer and timer interrupt setup steps:
 *   1. Enable TIMx clock from RCC
 *   2. Set prescaler for the timer from PSC
 *   3. Set auto-reload value from ARR
 *   4. (optional) Enable update interrupt from DIER bit 0
 *   5. (optional) Enable TIMx interrupt from NVIC
 *   6. Enable TIMx module from CR1 bit 0
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

enum GPIO_pin_state {OUTPUT_HIGH, INPUT_X, INPUT_LOW};

/* Global indicator whether a capicitance measurement is ready */
int measured = 0;

/*************************************************
* function declarations
*************************************************/
int main(void);
void start_input(void);

/*************************************************
* external interrupt handler
*************************************************/
void EXTI0_IRQHandler(void)
{
    
    // Check if the interrupt came from exti0
    if (EXTI->PR & (1 << 7)){

        /* Stop counter */

        measured = 1; //Indicate measurement is ready
        GPIOD->ODR |= (uint16_t)(0x8000); //blue on

        /* wait little bit */
        for(int j=0; j<1000000; j++);

        
        // Clear pending bit
        EXTI->PR = (1 << 7);



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
    
    /* Start Measurement */
    
    
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
            start_input(); 

            GPIOD->ODR |= 0x2000;//orange
            state = INPUT_X;
            break;
        case INPUT_X:

            if(measured==1) {
                GPIOD->ODR = 0x0000;
                state = INPUT_LOW;
            } else{
                GPIOD->ODR |= 0x4000;//red

            }
            break;
    } 
    

}

/******************************************
 * Set PC.7 as input and interruptible
 ******************************************/

void start_input(void)
{
    //set the PIN as input;
    // Set the pin 7 as input, corresponding to 15:14=00
    //RM0090 8.4.1
    GPIOC->MODER &= 0xFFFF3FFF; //set bit 15 to 0

    measured = 0; // measurement not ready
    GPIOD->ODR &= (uint16_t)0x7FFF;//blue off

    //Start timer counter

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
    TIM2->ARR = 10000;

    // Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    NVIC_SetPriority(TIM2_IRQn, 2); // Priority level 2
    // enable TIM2 IRQ from NVIC
    NVIC_EnableIRQ(TIM2_IRQn);

    // Enable Timer 2 module (CEN, bit0)
    TIM2->CR1 |= (1 << 0);



    measured = 0;

    /* Configure the GPIO C.7 for touch sensing */
    // RM0090, 8.4.4
    // PUPDR [15:14] corresponds to Pin 7; 0b10 as pull down.
    GPIOC->PUPDR |= 0x00008000;// set bit 15 to 1 for pull down


    /* Tie PC.7 to EXTI7 */
    // EXTI7 can be configured for each GPIO module.
    //   EXTICR2: 0b 0010 XXXX XXXX XXXX
    //               pin7 pin6 pin5 pin4
    //
    //   Writing a 0b0010 to pin7 location ties PC.70 to EXT0
    SYSCFG->EXTICR[1] = 0x2000;// Write 0010 to map PC7 to EXTI0

    // Choose either rising edge trigger (RTSR) or falling edge trigger (FTSR)
    EXTI->FTSR |= 0x00001<<7;

    // Unmask the corresponding interrupt line (line 7).
    EXTI->IMR |= 0x00001<<7;   

    // Set Priority for each interrupt request
    NVIC_SetPriority(EXTI0_IRQn, 1); // Priority level 1

    // enable EXT0 IRQ from NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);


    // Output, high
    RCC->AHB1ENR |= (1 << 2); //GPIOC Clock enabled
    GPIOC->MODER &= 0xFFFF7FFF; //set bit 15 to 0
    GPIOC->MODER |= 0x00004000; //set bit 14 to 1
    GPIOC->ODR = (1U<<7);


    // enable SYSCFG clock (APB2ENR: bit 14)
    RCC->APB2ENR |= (1 << 14);



    while(1)
    {
        // Do nothing.
    }

    return 0;
}
