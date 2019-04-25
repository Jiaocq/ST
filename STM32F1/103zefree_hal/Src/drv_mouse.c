#include "drv_mouse.h"
#include "main.h"

//struct Control * MouseControl ;

int32_t PutMouseControl_Init(struct Control * MouseControl)   //(u8 Button, u8 x_axis, u8 y_axis, u8 whell){
{
    MouseControl->array[0] = 0x08; //
    MouseControl->array[1] = 0x00; //
    MouseControl->array[2] = 0xa1; //
    MouseControl->array[3] = 0x02; //
    MouseControl->array[4] = 0x00; //Button;
    MouseControl->array[5] = 0x00; //x_axis;//
    MouseControl->array[6] = 0x00; //y_axis;
    MouseControl->array[7] = 0x00; //whell;
		MouseControl->length=8;
    return HAL_OK;
}






















