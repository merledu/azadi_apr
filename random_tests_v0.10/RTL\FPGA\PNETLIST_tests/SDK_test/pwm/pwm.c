
#include "gpio.h"
#include "trap.h"
#include "plic.h"
#include "platform.h"
#include "pwm.h"

int main(void){
while(1)
{

	PWM_DUTYCYCLE(1,20);
	PWM_DUTYCYCLE(2,20);

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
