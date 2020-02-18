//-------------------------------------------------------------------
#include "dsk6713.h"
#include "dsk6713_dip.h"
#include "dsk6713_led.h"
#include "dsk6713_aic23.h"  	  						//codec-dsk support file
#include "csl_edma.h"
#include "app_codec.h"

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
short convolution(int i, int N, int SIDE,int POS, short dip);
//-------------------------------------------------------------------
//-------------------------------------------------------------------
extern far void vectors();         //external function
//-------------------------------------------------------------------
//-----FILTERS-------
float Filters[3][81] = {{
		1.33E-19	,
		8.96E-05	,
		0.00018321	,
		0.00027148	,
		0.00034397	,
		0.00038983	,
		0.00039869	,
		0.0003618	,
		0.00027308	,
		0.00013031	,
		-6.39E-05	,
		-0.00030124	,
		-0.00056701	,
		-0.00083994	,
		-0.0010922	,
		-0.0012898	,
		-0.0013939	,
		-0.0013615	,
		-0.0011479	,
		-0.00070824	,
		6.67E-19	,
		0.0010146	,
		0.002366	,
		0.0040755	,
		0.0061526	,
		0.0085938	,
		0.011381	,
		0.014482	,
		0.017849	,
		0.021421	,
		0.025125	,
		0.028876	,
		0.032583	,
		0.036152	,
		0.039486	,
		0.042491	,
		0.045083	,
		0.047184	,
		0.048732	,
		0.049681	,
		0.05	,
		0.049681	,
		0.048732	,
		0.047184	,
		0.045083	,
		0.042491	,
		0.039486	,
		0.036152	,
		0.032583	,
		0.028876	,
		0.025125	,
		0.021421	,
		0.017849	,
		0.014482	,
		0.011381	,
		0.0085938	,
		0.0061526	,
		0.0040755	,
		0.002366	,
		0.0010146	,
		6.67E-19	,
		-0.00070824	,
		-0.0011479	,
		-0.0013615	,
		-0.0013939	,
		-0.0012898	,
		-0.0010922	,
		-0.00083994	,
		-0.00056701	,
		-0.00030124	,
		-6.39E-05	,
		0.00013031	,
		0.00027308	,
		0.0003618	,
		0.00039869	,
		0.00038983	,
		0.00034397	,
		0.00027148	,
		0.00018321	,
		8.96E-05	,
		1.33E-19	,
}, {
		5.30E-19	,
		0.0003155	,
		0.00040967	,
		0.00015136	,
		-0.00034397	,
		-0.00077966	,
		-0.0008915	,
		-0.00064892	,
		-0.00027308	,
		-3.70E-05	,
		0	,
		8.56E-05	,
		0.00056701	,
		0.0015065	,
		0.0024422	,
		0.0025797	,
		0.0013939	,
		-0.0007591	,
		-0.0025668	,
		-0.0024931	,
		2.67E-18	,
		0.0035714	,
		0.0052906	,
		0.0022722	,
		-0.0061526	,
		-0.017188	,
		-0.025449	,
		-0.025975	,
		-0.017849	,
		-0.0060853	,
		0	,
		-0.008203	,
		-0.032583	,
		-0.064842	,
		-0.088293	,
		-0.084983	,
		-0.045083	,
		0.026307	,
		0.10897	,
		0.17488	,
		0.2	,
		0.17488	,
		0.10897	,
		0.026307	,
		-0.045083	,
		-0.084983	,
		-0.088293	,
		-0.064842	,
		-0.032583	,
		-0.008203	,
		0	,
		-0.0060853	,
		-0.017849	,
		-0.025975	,
		-0.025449	,
		-0.017188	,
		-0.0061526	,
		0.0022722	,
		0.0052906	,
		0.0035714	,
		2.67E-18	,
		-0.0024931	,
		-0.0025668	,
		-0.0007591	,
		0.0013939	,
		0.0025797	,
		0.0024422	,
		0.0015065	,
		0.00056701	,
		8.56E-05	,
		0	,
		-3.70E-05	,
		-0.00027308	,
		-0.00064892	,
		-0.0008915	,
		-0.00077966	,
		-0.00034397	,
		0.00015136	,
		0.00040967	,
		0.0003155	,
		5.30E-19	,
}, {
		1.99E-18	,
		-0.00040513	,
		-0.00059288	,
		-0.00042284	,
		-3.42E-18	,
		0.00038983	,
		0.00049281	,
		0.00028712	,
		7.59E-19	,
		-9.33E-05	,
		6.39E-05	,
		0.00021566	,
		6.18E-18	,
		-0.00066658	,
		-0.00135	,
		-0.0012898	,
		-1.73E-17	,
		0.0021206	,
		0.0037147	,
		0.0032013	,
		1.00E-17	,
		-0.004586	,
		-0.0076566	,
		-0.0063477	,
		-1.30E-17	,
		0.0085938	,
		0.014068	,
		0.011493	,
		1.46E-16	,
		-0.015336	,
		-0.025125	,
		-0.020673	,
		-2.78E-17	,
		0.02869	,
		0.048807	,
		0.042491	,
		2.78E-17	,
		-0.073491	,
		-0.1577	,
		-0.22456	,
		0.75	,
		-0.22456	,
		-0.1577	,
		-0.073491	,
		2.78E-17	,
		0.042491	,
		0.048807	,
		0.02869	,
		-2.78E-17	,
		-0.020673	,
		-0.025125	,
		-0.015336	,
		1.46E-16	,
		0.011493	,
		0.014068	,
		0.0085938	,
		-1.30E-17	,
		-0.0063477	,
		-0.0076566	,
		-0.004586	,
		1.00E-17	,
		0.0032013	,
		0.0037147	,
		0.0021206	,
		-1.73E-17	,
		-0.0012898	,
		-0.00135	,
		-0.00066658	,
		6.18E-18	,
		0.00021566	,
		6.39E-05	,
		-9.33E-05	,
		7.59E-19	,
		0.00028712	,
		0.00049281	,
		0.00038983	,
		-3.42E-18	,
		-0.00042284	,
		-0.00059288	,
		-0.00040513	,
		1.99E-18	,
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
	for(;;)
	  {
		int a = DSK6713_DIP_get(0);
		Dip_s = DSK6713_DIP_get(1) + DSK6713_DIP_get(2) + DSK6713_DIP_get(3);

		//float l = Mid_pass[0] + High_Pass[0]+Low_pass[0];

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


} // void forever_loop(void)
//-------------------------------------------------------------------

short convolution(int i, int N, int SIDE,int POS, short dip){
	int len = 0;
	int holder =0;
	int nB=0;
	int result=0;

	long mul=0;

	short filter = dip -1;
	//PING
	if(i == 0){
		nB=0;
		for(len = 0;len < N ; len++){
			holder =POS - len;

				if(holder < 0){
					mul = (int)(rx_pang[SIDE][127-nB]*Filters[filter][len]);
					result = _lsadd(mul,result);
					nB++;
				}
				else{
					mul = (int)(rx_ping[SIDE][holder]*Filters[filter][len]);
					result = _lsadd(mul,result);
				}
		}
	}
	//PONG
	else if(i == 1){
		nB=0;
		for(len = 0;len < N ; len++){
			holder =POS - len;

				if(holder < 0){
					mul = (int)(rx_ping[SIDE][127-nB]*Filters[filter][len]);
					result = _lsadd(mul,result);
					nB++;
				}
				else{
					mul = (int)(rx_pong[SIDE][holder]*Filters[filter][len]);
					result = _lsadd(mul,result);
				}
		}
	}
	//PANG
	else if(i == 2){
		nB=0;
		for(len = 0;len < N ; len++){
			holder =POS - len;
				if(holder < 0){
					mul = (int)(rx_pong[SIDE][127-nB]*Filters[filter][len]);
					result = _lsadd(mul,result);
					nB++;
				}
				else{
					mul = (int)(rx_pang[SIDE][holder]*Filters[filter][len]);
					result = _lsadd(mul,result);
				}
			}
	}
	return (short)(result);
}
//-------------------------------------------------------------------

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

