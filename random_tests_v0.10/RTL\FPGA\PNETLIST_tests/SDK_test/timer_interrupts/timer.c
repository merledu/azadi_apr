#include "gpio.h"
#include "trap.h"
#include "plic.h"
#include "platform.h"

int main(void){
while(1)
{
	gpio_direct_write_enable(5);
	gpio_direct_write(5, 1);
		
	delay(2000);

	gpio_direct_write_enable(7);
	gpio_direct_write(7, 1);

	delay(2000);

	// Enable Global (PLIC) interrupts.
	asm volatile("li      t0, 8\t\n"
		     "csrrs   zero, mstatus, t0\t\n"
		    );

	// Enable Local (PLIC) interrupts.
	asm volatile("li      t0, 0x800\t\n"
		     "csrrs   zero, mie, t0\t\n"
		    );

}

	return 0;
}
