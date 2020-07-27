#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
	LEDS_OFF(LEDS_MASK);
    
	while (true)
	{
        __WFE();
        __SEV();
        __WFE();
	}
}