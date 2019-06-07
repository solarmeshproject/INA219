#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "c:\Users\Master\Documents\peripheral\twi_sensor\librarys\ina219.h"


int main(void){
	
ina219_init(0.2, 10,5,0.5,RANGE_32V, GAIN_1_40MV, ADC_128SAMP, ADC_128SAMP, CONT_SH);
	
    while(true){
			
        nrf_delay_ms(100);
			
    }
}
