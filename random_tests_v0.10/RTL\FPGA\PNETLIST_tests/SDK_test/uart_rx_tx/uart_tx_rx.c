#include "uart.h"
#include "utils.h"
#include <stdio.h>
int main() 
{ 
uart_init(9600 , 8000000);
char a = uart_polled_data();
uart_send_char(a);

}

