#include "spi.h"
#include "plic.h"

int main()
{

    Set_Speed(0xFFFF);
    Spi_Write(1, 0, 1);

    return 0;
}
