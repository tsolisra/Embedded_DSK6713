//-------------------------------------------------------------------
#include "dsk6713.h"
#include "dsk6713_dip.h"
#include "dsk6713_led.h"
#include "dsk6713_aic23.h"  	  						//codec-dsk support file
#include "csl_edma.h"
#include "app_codec.h"
//#include <c67fastMath.h>
//-------------------------------------------------------------------
//-------------------------------------------------------------------
//

//Putting any other combination than PASSTHROUGH, MUTE, LOW, MID, HIGH will result in automatic mute to ensure the sound is not affected



//-------------------------------------------------------------------
#define BUFLEN 0x80				// if you change this number, you have to go to the EDMA parameters   - 128
								// and change the number of reloads, the skip forward, and the skip backward
#define BUFWIDTH 2
#define LEFT  1                  //data structure for union of 32-bit data
#define RIGHT 0                  //into two 16-bit data


#pragma DATA_SECTION (rx_ping, ".buffers");
#pragma DATA_ALIGN   (rx_ping, 0x80);
short rx_ping[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (rx_pong, ".buffers");
#pragma DATA_ALIGN   (rx_pong, 0x80);
short rx_pong[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (rx_pang, ".buffers");
#pragma DATA_ALIGN   (rx_pang, 0x80);
short rx_pang[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (tx_ping, ".buffers");
#pragma DATA_ALIGN   (tx_ping, 0x80);
short tx_ping[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (tx_pong, ".buffers");
#pragma DATA_ALIGN   (tx_pong, 0x80);
short tx_pong[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (tx_pang, ".buffers");
#pragma DATA_ALIGN   (tx_pang, 0x80);
short tx_pang[BUFWIDTH][BUFLEN];

//-------------------------------------------------------------------
typedef struct _Globals
{
	volatile int move_ping;
	volatile int move_pong;
	volatile int move_pang;
} Globals;

Globals Glo = {FALSE, FALSE, FALSE};

//-------------------------------------------------------------------
// declare the CSL objects
EDMA_Handle hEdma_rx;     // Handle for the EDMA channel
EDMA_Handle hEdmaPing_rx; // Handle for the ping EDMA reload parameters
EDMA_Handle hEdmaPong_rx; // Handle for the pong EDMA reload parameters
EDMA_Handle hEdmaPang_rx; // Handle for the pong EDMA reload parameters
EDMA_Config cfgEdma_rx;   //EDMA configuration structure

EDMA_Handle hEdma_tx;
EDMA_Handle hEdmaPing_tx;
EDMA_Handle hEdmaPong_tx;
EDMA_Handle hEdmaPang_tx;
EDMA_Config cfgEdma_tx;

#include "app_edma.h"      // structures are filled in the header


//-------------------------------------------------------------------
void main(void);
void forever_loop(void);
void output_sample(int out_data);
Uint32 input_sample(void);
void init(void);
long Mult_fp(short data, short coeff);
long rounding(long result);
short scale_down(unsigned long result);

long test();

short convolution(int i, int N, int SIDE,int POS, short dip);
//-------------------------------------------------------------------
//-------------------------------------------------------------------
extern far void vectors();         //external function
//-------------------------------------------------------------------
//-----FILTERS-------
short Filters[3][81] = {{
		0	,
		3	,
		6	,
		9	,
		11	,
		13	,
		13	,
		12	,
		9	,
		4	,
		-2	,
		-10	,
		-19	,
		-28	,
		-36	,
		-42	,
		-46	,
		-45	,
		-38	,
		-23	,
		0	,
		33	,
		78	,
		134	,
		202	,
		282	,
		373	,
		475	,
		585	,
		702	,
		823	,
		946	,
		1068	,
		1185	,
		1294	,
		1392	,
		1477	,
		1546	,
		1597	,
		1628	,
		1638	,
		1628	,
		1597	,
		1546	,
		1477	,
		1392	,
		1294	,
		1185	,
		1068	,
		946	,
		823	,
		702	,
		585	,
		475	,
		373	,
		282	,
		202	,
		134	,
		78	,
		33	,
		0	,
		-23	,
		-38	,
		-45	,
		-46	,
		-42	,
		-36	,
		-28	,
		-19	,
		-10	,
		-2	,
		4	,
		9	,
		12	,
		13	,
		13	,
		11	,
		9	,
		6	,
		3	,
		0	,


}, {
		0	,
		10	,
		13	,
		5	,
		-11	,
		-26	,
		-29	,
		-21	,
		-9	,
		-1	,
		0	,
		3	,
		19	,
		49	,
		80	,
		85	,
		46	,
		-25	,
		-84	,
		-82	,
		0	,
		117	,
		173	,
		74	,
		-202	,
		-563	,
		-834	,
		-851	,
		-585	,
		-199	,
		0	,
		-269	,
		-1068	,
		-2125	,
		-2893	,
		-2785	,
		-1477	,
		862	,
		3571	,
		5730	,
		6554	,
		5730	,
		3571	,
		862	,
		-1477	,
		-2785	,
		-2893	,
		-2125	,
		-1068	,
		-269	,
		0	,
		-199	,
		-585	,
		-851	,
		-834	,
		-563	,
		-202	,
		74	,
		173	,
		117	,
		0	,
		-82	,
		-84	,
		-25	,
		46	,
		85	,
		80	,
		49	,
		19	,
		3	,
		0	,
		-1	,
		-9	,
		-21	,
		-29	,
		-26	,
		-11	,
		5	,
		13	,
		10	,
		0	,

}, {
		0	,
		-13	,
		-19	,
		-14	,
		0	,
		13	,
		16	,
		9	,
		0	,
		-3	,
		2	,
		7	,
		0	,
		-22	,
		-44	,
		-42	,
		0	,
		69	,
		122	,
		105	,
		0	,
		-150	,
		-251	,
		-208	,
		0	,
		282	,
		461	,
		377	,
		0	,
		-503	,
		-823	,
		-677	,
		0	,
		940	,
		1599	,
		1392	,
		0	,
		-2408	,
		-5168	,
		-7358	,
		24576	,
		-7358	,
		-5168	,
		-2408	,
		0	,
		1392	,
		1599	,
		940	,
		0	,
		-677	,
		-823	,
		-503	,
		0	,
		377	,
		461	,
		282	,
		0	,
		-208	,
		-251	,
		-150	,
		0	,
		105	,
		122	,
		69	,
		0	,
		-42	,
		-44	,
		-22	,
		0	,
		7	,
		2	,
		-3	,
		0	,
		9	,
		16	,
		13	,
		0	,
		-14	,
		-19	,
		-13	,
		0	,

}};





//-------------------------------------------------------------------

//-------------------------------------------------------------------
void main(void)
{
	init();
	forever_loop();
} // void main(void)
//-------------------------------------------------------------------


//-------------------------------------------------------------------
void forever_loop(void)
{
int i;
int DIP =0;
int Dip_s =0;

///*
	for(;;)
	  {
		int a = DSK6713_DIP_get(0);
		Dip_s = DSK6713_DIP_get(1) + DSK6713_DIP_get(2) + DSK6713_DIP_get(3);


	  	if(Glo.move_ping == TRUE)
	  	  {
			DIP = DSK6713_DIP_get(1) + DSK6713_DIP_get(2)*2 + DSK6713_DIP_get(3)*3;
	  		Glo.move_ping = FALSE;
	  	  	for(i=0;i<BUFLEN;i++)
	  	  	  {
	  	  		if(a || (Dip_s > 1)){
	  	  		tx_ping[RIGHT][i] = 0;
	  	  		tx_ping[LEFT][i] = 0;
	  	  		}
	  	  		else if (DIP){
	  	  		tx_ping[RIGHT][i] = convolution(0, 81,RIGHT, i,DIP);
	  	  		tx_ping[LEFT][i] =convolution(0, 81,LEFT, i,DIP);
	  	  		}
	  	  		else{
	  	  		tx_ping[RIGHT][i] = rx_ping[RIGHT][i];
	  	  	    tx_ping[LEFT][i] = rx_ping[LEFT][i];
	  	  		}
	  	  	  }
	  	  }
	  	if(Glo.move_pong == TRUE)
	  	  {
	  		Glo.move_pong = FALSE;
	  	  	for(i=0;i<BUFLEN;i++)
	  	  	  {
	  	  		if(a|| (Dip_s > 1)){
	  	  		tx_pong[RIGHT][i] = 0;
	  	  		tx_pong[LEFT][i] = 0;
	  	  		}
	  	  	else if (DIP){
	  	  		tx_pong[RIGHT][i] =convolution(1, 81, RIGHT, i,DIP);
	  	  		tx_pong[LEFT][i] =convolution(1, 81,LEFT, i,DIP);
	  	  		}
	  	  	else{
	  	  		tx_pong[RIGHT][i] = rx_pong[RIGHT][i];
	  	  	    tx_pong[LEFT][i] = rx_pong[LEFT][i];
	  	  	}
	  	  	  }

	  	  }
	  	if(Glo.move_pang == TRUE)
	  	  {
	  		Glo.move_pang = FALSE;
	  	  	for(i=0;i<BUFLEN;i++)
	  	  	  {
	  	  		if(a|| (Dip_s > 1)){
	  	  		tx_pang[RIGHT][i] = 0;
	  	  		tx_pang[LEFT][i] = 0;
	  	  		}
	  	  	else if (DIP){
	  	  		tx_pang[RIGHT][i] =convolution(2, 81,RIGHT, i,DIP);
	  	  		tx_pang[LEFT][i] =convolution(2, 81,LEFT, i,DIP);
	  	  		}
	  	  		else{
		  	  		tx_pang[RIGHT][i] = rx_pang[RIGHT][i];
		  	  	    tx_pang[LEFT][i] = rx_pang[LEFT][i];
	  	  		}
	  	  	  }
	  	  }
	  } // ever
	 //*/

	/*
	for(;;)
	{
		long r = test();
	}
*/
} // void forever_loop(void)
//-------------------------------------------------------------------
long test(){
	return Mult_fp(-1400,100);

}


short convolution(int i, int N, int SIDE,int POS, short dip){
	int len = 0;
	int holder = 0;
	long result=0;
	int mul=0;
	short final_res=0;

	short filter = dip -1;
	//PING
	if(i == 0){
		for(len = 0;len < N ; len++){
			holder =POS - len;
				if(holder < 0){
					mul = Mult_fp(rx_pang[SIDE][128+holder],Filters[filter][len]);
					result = _lsadd(mul, result);
				}
				else{
					mul = Mult_fp(rx_ping[SIDE][holder],Filters[filter][len]);
					result = _lsadd(mul, result);
				}
		}
	}
	//PONG
	else if(i == 1){
		for(len = 0;len < N ; len++){
			holder =POS - len;
				if(holder < 0){
					mul = Mult_fp(rx_ping[SIDE][128+holder],Filters[filter][len]);
					result = _lsadd(mul, result);
				}
				else{
					mul = Mult_fp(rx_pong[SIDE][holder],Filters[filter][len]);
					result = _lsadd(mul, result);
				}
		}
	}
	//PANG
	else if(i == 2){
		for(len = 0;len < N ; len++){
			holder =POS - len;
				if(holder < 0){
					mul = Mult_fp(rx_pong[SIDE][128+holder],Filters[filter][len]);
					result = _lsadd(mul, result);
				}
				else{
					mul = rx_pang[SIDE][holder]*Filters[filter][len];
					result = _lsadd(mul, result);
				}
			}
	}
	result = rounding(result);
	final_res = scale_down(result);
	return final_res;
}
//-------------------------------------------------------------------
long Mult_fp(short data, short coeff){
	long data_scale = data;
	long coeff_scale = coeff;
	long answer = data_scale * coeff_scale;
	return answer;
}

long rounding(long result){
	long num;
	if ((result >> 31) &  1){
		num = result + (2^7);
	}
	else{
		num = result - (2^7);
	}
	return num;
}

short scale_down(unsigned long result){
	short num;
	num = result >> 15;
	return num;
}



void output_sample(int out_data)    //for out to Left and Right channels
{

	AIC_data.uint=out_data;          //32-bit data -->data structure

    MCBSP_write(DSK6713_AIC23_DATAHANDLE, AIC_data.uint);//write/output data

} // void output_sample(int out_data)


//-------------------------------------------------------------------


//-------------------------------------------------------------------


Uint32 input_sample(void)                      	  	//for 32-bit input
{

	AIC_data.uint=MCBSP_read(DSK6713_AIC23_DATAHANDLE);//read data

	return(AIC_data.uint);
} // Uint32 input_sample(void)




//-------------------------------------------------------------------
interrupt void c_int8()            // interrupt service routine
{
	if(EDMA_intTest(0xF))          // rx_ping is full
	{
		EDMA_intClear(0xF);
		Glo.move_ping = TRUE;
	}

	if(EDMA_intTest(0xE))          // tx_ping has been emptied
	{
		EDMA_intClear(0xE);
	}

	if(EDMA_intTest(0xD))          // rx_pong is full
	{
		EDMA_intClear(0xD);
		Glo.move_pong = TRUE;
	}

	if(EDMA_intTest(0xC))          // tx_pong has been emptied
	{
		EDMA_intClear(0xC);
	}

	if(EDMA_intTest(0xB))          // rx_pang is full
	{
		EDMA_intClear(0xB);
		Glo.move_pang = TRUE;
	}

	if(EDMA_intTest(0xA))          // tx_pang has been emptied
	{
		EDMA_intClear(0xA);
	}
	return;

} // c_int8
//-------------------------------------------------------------------


//-------------------------------------------------------------------
void init(void)	     	 	//for communication/init using interrupt
{
	int i;
	for(i=0; i<BUFLEN; i++)
	  {
	  	rx_ping[RIGHT][i] = 0;
	  	rx_ping[LEFT][i] = 0xffff;
	  	rx_pong[RIGHT][i] = 0;
	  	rx_pong[LEFT][i] = 0xffff;
	  	rx_pang[RIGHT][i] = 0;
	  	rx_pang[LEFT][i] = 0xffff;
	  	tx_ping[RIGHT][i] = 0;
	  	tx_ping[LEFT][i] = 0xffff;
	  	tx_pong[RIGHT][i] = 0;
	  	tx_pong[LEFT][i] = 0xffff;
	  	tx_pang[RIGHT][i] = 0;
	  	tx_pang[LEFT][i] = 0xffff;
	  }
	DSK6713_init();                   	//call BSL to init DSK-EMIF,PLL)
	CSL_init();
    IRQ_globalDisable();           	    //disable interrupts
   	hCodec = DSK6713_AIC23_openCodec(0, &config1);
                                        // codec has to be open before handle is referenced
   	EDMA_clearPram(0x00000000);

    hEdma_rx = EDMA_open(EDMA_CHA_REVT1, EDMA_OPEN_RESET);
    hEdmaPing_rx = EDMA_allocTable(-1);
    hEdmaPong_rx = EDMA_allocTable(-1);
    hEdmaPang_rx = EDMA_allocTable(-1);

    cfgEdmaPing_rx.rld = EDMA_RLD_RMK(2,hEdmaPong_rx);
    cfgEdmaPong_rx.rld = EDMA_RLD_RMK(2,hEdmaPang_rx);
    cfgEdmaPang_rx.rld = EDMA_RLD_RMK(2,hEdmaPing_rx);

    cfgEdma_rx = cfgEdmaPing_rx;        // start with ping

    EDMA_config(hEdma_rx, &cfgEdma_rx);
    EDMA_config(hEdmaPing_rx, &cfgEdmaPing_rx);
    EDMA_config(hEdmaPong_rx, &cfgEdmaPong_rx);
    EDMA_config(hEdmaPang_rx, &cfgEdmaPang_rx);

    hEdma_tx = EDMA_open(EDMA_CHA_XEVT1, EDMA_OPEN_RESET);
    hEdmaPing_tx = EDMA_allocTable(-1);
    hEdmaPong_tx = EDMA_allocTable(-1);
    hEdmaPang_tx = EDMA_allocTable(-1);

    cfgEdmaPing_tx.rld = EDMA_RLD_RMK(2,hEdmaPong_tx);
    cfgEdmaPong_tx.rld = EDMA_RLD_RMK(2,hEdmaPang_tx);
    cfgEdmaPang_tx.rld = EDMA_RLD_RMK(2,hEdmaPing_tx);

    cfgEdma_tx = cfgEdmaPing_tx;        // start with ping

    EDMA_config(hEdma_tx, &cfgEdma_tx);
    EDMA_config(hEdmaPing_tx, &cfgEdmaPing_tx);
    EDMA_config(hEdmaPong_tx, &cfgEdmaPong_tx);
    EDMA_config(hEdmaPang_tx, &cfgEdmaPang_tx);

    EDMA_intDisable(0xF);               // use 4 EDMA events
    EDMA_intClear(0xF);                 // on one EDMA interrupt
    EDMA_intEnable(0xF);

    EDMA_intDisable(0xE);
    EDMA_intClear(0xE);
    EDMA_intEnable(0xE);

    EDMA_intDisable(0xD);
    EDMA_intClear(0xD);
    EDMA_intEnable(0xD);

    EDMA_intDisable(0xC);
    EDMA_intClear(0xC);
    EDMA_intEnable(0xC);

    EDMA_intDisable(0xB);
    EDMA_intClear(0xB);
    EDMA_intEnable(0xB);

    EDMA_intDisable(0xA);
    EDMA_intClear(0xA);
    EDMA_intEnable(0xA);

    EDMA_enableChannel(hEdma_rx);       // start on the copy of ping
    EDMA_enableChannel(hEdma_tx);       // start on the copy of ping


  	IRQ_setVecs(vectors);     			//point to the IRQ vector table
    IRQ_map(IRQ_EVT_EDMAINT, 8);
    IRQ_reset(IRQ_EVT_EDMAINT);
    IRQ_enable(IRQ_EVT_EDMAINT);        // c_int8


    IRQ_globalEnable();       			//globally enable interrupts
  	IRQ_nmiEnable();          			//enable NMI interrupt

  	output_sample(0);                   // start TX to launch EDMA
} // init
//-------------------------------------------------------------------

// EOF

