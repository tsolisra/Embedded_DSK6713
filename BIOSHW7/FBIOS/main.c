/*
 * main.c
 */
#include "FBIOScfg.h"
#include "dsk6713_dip.h"
#include "dsk6713_led.h"
#include "dsk6713.h"
#include <sem.h>
#include <tsk.h>
#include <prd.h>

int sw_val;

int prd_state = 1;

int main(void) {

	DSK6713_init();
	CSL_init();
	DSK6713_LED_init();
	DSK6713_DIP_init();

	DSK6713_LED_off(0);
	DSK6713_LED_off(1);
	DSK6713_LED_off(2);
	DSK6713_LED_off(3);
}

void TSK0_ftn(int arg0, int arg1){
	for(;;){
		SEM_pendBinary(&SEM0, SYS_FOREVER);
		sw_val = DSK6713_DIP_get(0) +(DSK6713_DIP_get(1)*2) +(DSK6713_DIP_get(2)*4) +(DSK6713_DIP_get(3)*8) ;
		if((sw_val != 0) && (!prd_state)){
			prd_state = 1;
			PRD_start(&PRD0);
		}
		SEM_postBinary(&SEM0);
	}
}
void TSK2_ftn(int arg0, int arg1){
	SEM_postBinary(&SEM0);
}

void PRD0_ftn(int arg0, int arg1)
{
	sw_val = DSK6713_DIP_get(0) +(DSK6713_DIP_get(1)*2) +(DSK6713_DIP_get(2)*4) +(DSK6713_DIP_get(3)*8) ;
	if(sw_val==0){
		PRD_stop(&PRD0);
		prd_state = 0;
	}
	else{
		PRD0.period= 500/sw_val;
		DSK6713_LED_toggle(0);
	}

}

