/*
 * main.c
 */
#include "FBIOScfg.h"
#include "dsk6713_dip.h"
#include "dsk6713_led.h"
#include "dsk6713.h"
#include <sem.h>
#include <tsk.h>


int main(void) {
	DSK6713_init();
	CSL_init();
	DSK6713_LED_init();
	DSK6713_DIP_init();
}

void TSK0_ftn(int arg0, int arg1){
	for(;;){
		SEM_pendBinary(&SEM0, SYS_FOREVER);
		if(DSK6713_DIP_get(0)==0){
			DSK6713_LED_on(0);
			while(DSK6713_DIP_get(1)==1){

			}
			DSK6713_LED_off(0);
		}
		SEM_postBinary(&SEM0);
	}
}

void TSK1_ftn(int arg0, int arg1){
	for(;;){

		SEM_pendBinary(&SEM0, SYS_FOREVER);
		if(DSK6713_DIP_get(3)==0){
			DSK6713_LED_on(3);
			while(DSK6713_DIP_get(2)==1){

			}
			DSK6713_LED_off(3);
		}
		SEM_postBinary(&SEM0);
}
}

void TSK2_ftn(int arg0, int arg1){
	SEM_postBinary(&SEM0);
}

