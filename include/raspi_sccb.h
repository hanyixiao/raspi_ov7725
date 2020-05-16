#include <stdint.h>
void SCCB_GPIO_init();
uint8_t SCCB_Read_Reg(uint8_t reg,uint8_t *data);
uint8_t SCCB_Write_Reg(uint8_t reg,uint8_t data);
