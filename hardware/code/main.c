#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


uint8_t nextSend = 'a';
uint8_t messageLen = 0;
#define SENDBUFLEN 64
uint8_t message[64];
uint8_t messageIndex = 0;

uint8_t rxIndex = 0;
#define RECVBUFLEN 8
uint8_t rxBuf[RECVBUFLEN];

static void clock_setup(void);
static void gpio_setup(void);







static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOC);
}


static void gpio_setup(void)
{
	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); //PC13 is the LED
}


int main(void) {

	
	clock_setup();
	gpio_setup();

	while (1) {
		for (int i=0; i<90000; i++){
			__asm__("nop");
		}
		gpio_toggle(GPIOC, GPIO13);

	}

	return 0;
}
