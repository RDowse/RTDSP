/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		  LAB 3: Interrupt I/O

 				            ********* I N T I O. C **********

  Demonstrates inputing and outputing data from the DSK's audio port using interrupts. 

 *************************************************************************************
 				Updated for use on 6713 DSK by Danny Harvey: May-Aug 2006
				Updated for CCS V4 Sept 10
 ************************************************************************************/
/*
 *	You should modify the code so that interrupts are used to service the 
 *  audio port.
 */
/**************************** Pre-processor statements ******************************/

#include <stdlib.h>
#include <stdio.h>
//  Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This 
   example also includes dsk6713_aic23.h because it uses the 
   AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

// MATLAB output coefficients
#include "fir_coeff.txt" 

// math library (trig functions)
#include <math.h>

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

// PI defined here for use in your code 
#define PI 3.141592653589793

// define a delay buffer of size BUFFER_SIZE (order of filter)
#define BUFFER_SIZE 206

//sampling frequency as defind in Config
#define SAMP_FREQ 8000

/******************************* Global declarations ********************************/

/* Audio port configuration settings: these values set registers in the AIC23 audio 
   interface to configure it. See TI doc SLWS106D 3-3 to 3-10 for more info. */
DSK6713_AIC23_Config Config = { \
			 /**********************************************************************/
			 /*   REGISTER	            FUNCTION			      SETTINGS         */ 
			 /**********************************************************************/\
    0x0017,  /* 0 LEFTINVOL  Left line input channel volume  0dB                   */\
    0x0017,  /* 1 RIGHTINVOL Right line input channel volume 0dB                   */\
    0x01f9,  /* 2 LEFTHPVOL  Left channel headphone volume   0dB                   */\
    0x01f9,  /* 3 RIGHTHPVOL Right channel headphone volume  0dB                   */\
    0x0011,  /* 4 ANAPATH    Analog audio path control       DAC on, Mic boost 20dB*/\
    0x0000,  /* 5 DIGPATH    Digital audio path control      All Filters off       */\
    0x0000,  /* 6 DPOWERDOWN Power down control              All Hardware on       */\
    0x0043,  /* 7 DIGIF      Digital audio interface format  16 bit                */\
    0x008d,  /* 8 SAMPLERATE Sample rate control             8 KHZ                 */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
			 /**********************************************************************/
};


// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;

// creates boolean type
typedef enum { false, true } bool;

// enum for buffer types. Change the value of select_buffer to pick the type used.
typedef enum {
	NONCIRCULAR_FILTER, CIRCULAR_FILTER, CIRCULAR_FILTER_MODULO,
	CIRCULAR_FILTER_SYMM
} BufferType;
BufferType select_buffer = CIRCULAR_FILTER_MODULO;
 
// Current x[i] and y[i]
double dInput;
double dOutput;
// signal buffer of size M 
double x[BUFFER_SIZE] = {0};
// ptr to most recent element in buffer
short pBuf = 0;

//circular 1 or shift buffer 0
//short selectFIR = 2;

 /******************************* Function prototypes ********************************/
void   init_hardware(void);     
void   init_HWI(void);
void   ISR_AIC(void);  
double noncircular_filter(double sample_in);
void   shift_buffer(double sample_in);
double circular_filter_modulo(double sample_in);
double circular_filter(double sample_in);
double circular_filter_symm(double sample_in);
double convolution(double currentInput);
/********************************** Main routine ************************************/
void main()
{
	
	// initialize board and the audio port
	init_hardware();
	
	/* initialize hardware interrupts */
	init_HWI();
  	 		
	/* loop indefinitely, waiting for interrupts */  					
	while(1) 
  	{};
  
}
        
/********************************** init_hardware() **********************************/  
void init_hardware()
{
    // Initialize the board support library, must be called first 
    DSK6713_init();
    
    // Start the AIC23 codec using the settings defined above in config 
    H_Codec = DSK6713_AIC23_openCodec(0, &Config);

	/* Function below sets the number of bits in word used by MSBSP (serial port) for 
	receives from AIC23 (audio port). We are using a 32 bit packet containing two 
	16 bit numbers hence 32BIT is set for  receive */
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);	

	/* Configures interrupt to activate on each consecutive available 32 bits 
	from Audio port hence an interrupt is generated for each L & R sample pair */	
	MCBSP_FSETS(SPCR1, RINTM, FRM);

	/* These commands do the same thing as above but applied to data transfers to  
	the audio port */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);	
	MCBSP_FSETS(SPCR1, XINTM, FRM);	
	

}

/********************************** init_HWI() **************************************/  
void init_HWI(void)
{
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt (used by the debugger)
	IRQ_map(IRQ_EVT_XINT1,4);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_XINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts
} 

/*************************** Signal processing functions ******************************/  

double noncircular_filter(double sample_in){
	short i;
	double sum = 0;
	// Perform convolution
	sum += sample_in * b[0]; // convolution with coefficient 0
	for (i=0; i<BUFFER_SIZE; i++)
	{
		sum += b[BUFFER_SIZE-i] * x[i]; // convolution with coefficients 1 to M
	}
	// Perform buffer shift (delay)
	shift_buffer(sample_in);
	return sum;
}

void shift_buffer(double sample_in)
{
	short i;
	// shifts buffer x[]
	for (i=BUFFER_SIZE-1; i>0; i--)
	{
		x[i]=x[i-1]; /* move data along buffer from lower */
	} /* element to next higher */
	x[0] = sample_in; 
}

double circular_filter_modulo(double sample_in){
	short i;
	double sum = 0;
	// do convolution sequentially from coefficient b0 to bM
	sum += b[0] * sample_in ; // convolution with coefficient 0
	for (i=0; i<BUFFER_SIZE; i++)
	{
		// convolution with coefficients 1 to M
		sum += b[i+1] * x[ (BUFFER_SIZE+pBuf-i) % BUFFER_SIZE ];
	}
	// then store current sample in incremented position in buffer,
	// wrapping around if attempting to go past last entry in buffer
	++pBuf;
	pBuf = pBuf % BUFFER_SIZE;
	x[pBuf] = sample_in;
	return sum;
}

double circular_filter(double sample_in){
	short i;
	double sum = 0;
	double *pCoeff = b;
	// do convolution sequentially from coefficient b0 to bM
	sum += *(pCoeff++) * sample_in; // convolution with coefficient 0
	for (i=pBuf; i>=0; i--)
	{
		sum += *(pCoeff++) * x[i]; // convolution with left part of buffer
	}
	for (i=BUFFER_SIZE-1; i>pBuf; i--)
	{
		sum += *(pCoeff++) * x[i]; // convolution with right part of buffer
	}
	// increment buffer ptr, wrapping around if 0
	if (++pBuf == BUFFER_SIZE) pBuf = 0;
	// then store current sample in incremented position in buffer
	x[pBuf] = sample_in;
	return sum;
}

double circular_filter_symm(double sample_in){
	short i;
	double sum = 0;
	double *pCoeff = b;
	short pBufR = pBuf + 1; // initially points to x[M-1], runs to x[BUFFER_SIZE/2]
	short pBufL = pBuf; // initially points to x[1], runs to x[BUFFER_SIZE/2] 
	// do convolution in pairs, leveraging on symmetry of b[] coefficients
	if (pBufR == BUFFER_SIZE) pBufR = 0; // wraparound
	sum += *(pCoeff++) * (sample_in + x[pBufR++]); // convolution with coefficient 0 and M-1
	for (i=1; i<BUFFER_SIZE/2; i++)
	{
		// deal with wraparound of pBufL and pBufR
		if (pBufR == BUFFER_SIZE) pBufR = 0;
		if (pBufL == -1) pBufL = BUFFER_SIZE-1;
		// convolve with coefficients i and BUFFER_SIZE-1-i
		// also increment buffer pointers
		// (decrement pBufL because it moves leftwards through buffer)
		sum += *(pCoeff++) * (x[pBufL--] + x[pBufR++]);
	}
	// special case for coefficient BUFFER_SIZE/2
	if (pBufR == BUFFER_SIZE) pBufR = 0;
	sum += *(pCoeff) * x[pBufR];
	// increment buffer ptr, wrapping around if 0
	if (++pBuf == BUFFER_SIZE) pBuf = 0;
	// then store current sample in incremented position in buffer
	x[pBuf] = sample_in;
	return sum;
}

/******************** INTERRUPT SERVICE ROUTINE ***********************/  

void ISR_AIC(void)
{
	// get new sample
	dInput = (double) mono_read_16Bit();
	// filtering using method selected by selectFIR
	switch (select_buffer) {
		case NONCIRCULAR_FILTER:
			dOutput = noncircular_filter(dInput);
			break;
		case CIRCULAR_FILTER_MODULO:
			dOutput = circular_filter_modulo(dInput);
			break;
		case CIRCULAR_FILTER:
			dOutput = circular_filter(dInput);
			break;
		case CIRCULAR_FILTER_SYMM:
			dOutput = circular_filter_symm(dInput);
			break;
	}
	// output result to both L/R channels
	mono_write_16Bit( (short) dOutput );
}
