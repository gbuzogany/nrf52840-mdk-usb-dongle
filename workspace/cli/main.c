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
        // Make sure any pending events are cleared
        __SEV();
        __WFE();
        // Enter System ON sleep mode
        __WFE();
	}
}