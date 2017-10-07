#include "gpio.h"
//#include "stm32f4xx_hal.h"
#define GPIO_MODE             0x00000003U
#define EXTI_MODE             0x10000000U
#define GPIO_MODE_IT          0x00010000U
#define GPIO_MODE_EVT         0x00020000U
#define RISING_EDGE           0x00100000U
#define FALLING_EDGE          0x00200000U
#define GPIO_OUTPUT_TYPE      0x00000010U

#define GPIO_NUMBER           16U

void gpio_mode_setup(GPIO_TypeDef* GPIOx, uint8_t mode, uint8_t pull_up_down,
		     uint16_t gpios)
{
	
	uint32_t position;
  uint32_t ioposition = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t temp = 0x00U;
	/*
	uint16_t i;
	uint32_t moder, pupd;

	
	 // We want to set the config only for the pins mentioned in gpios,
	 // but keeping the others, so read out the actual config first.
	 
	moder = GPIO_MODER(gpioport);
	pupd = GPIO_PUPDR(gpioport);

	for (i = 0; i < 16; i++) {
		if (!((1 << i) & gpios)) {
			continue;
		}

		moder &= ~GPIO_MODE_MASK(i);
		moder |= GPIO_MODE(i, mode);
		pupd &= ~GPIO_PUPD_MASK(i);
		pupd |= GPIO_PUPD(i, pull_up_down);
	}
	*/
	for(position = 0U; position < GPIO_NUMBER; position++)
  {
    /* Get the IO position */
    ioposition = 0x01U << position;
    /* Get the current IO position */
    iocurrent = (uint32_t)(gpios) & ioposition;

    if(iocurrent == ioposition)
    {
			/* Configure IO Direction mode (Input, Output, Alternate or Analog) */
			temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
      temp |= ((mode & GPIO_MODE) << (position * 2U));
      GPIOx->MODER = temp;
			/* Activate the Pull-up or Pull down resistor for the current IO */
			temp = GPIOx->PUPDR;
      temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
      temp |= ((pull_up_down) << (position * 2U));
      GPIOx->PUPDR = temp;
		}
	}
}
