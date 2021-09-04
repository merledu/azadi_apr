#include "gpio.h"
#include "trap.h"
#include "plic.h"
#include "platform.h"

uint32_t state ;

void handle_button_press(__attribute__((unused)) uint32_t num);


void handle_button_press(__attribute__((unused)) uint32_t num)
{
 state = gpio_read_pin(23);
 	if(state == 1){
		gpio_direct_write_enable(5);
		gpio_direct_write(5, 1);
		
	} 
	
		
}


int main(){
while(1)
{
	gpio_intr_enable(23);
	gpio_intr_type(23);

	plic_init(24, 0);

	isr_table[24] = handle_button_press;


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
