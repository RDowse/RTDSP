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

// define a delay buffer of size M (order of filter)
#define M 206

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
// has interrupt occured?
bool outReady = false;

 /******************************* Function prototypes ********************************/
void   init_hardware(void);     
void   init_HWI(void);
void   ISR_AIC(void);  
void shift_buffer(double sample_in, short* x);
double convolution(short currentInput, short* x);
/********************************** Main routine ************************************/
void main()
{
	// Current x[i] and y[i]
	short sInput;
	double dOutput;
	// signal buffer of size M 
	short x[M] = {0};
	
	// initialize board and the audio port
	init_hardware();
	
	/* initialize hardware interrupts */
	init_HWI();
  	 		
	/* loop indefinitely, waiting for interrupts */  					
	while(1) 
  	{
  		// interrupt just occurred. 
  		if (outReady)
  		{
  			// get new sample
  			sInput = mono_read_16Bit();
  			// convolve and shift buffer
			dOutput = convolution(sInput, x);
			shift_buffer(sInput, x);
			// output result to both L/R channels
			mono_write_16Bit( (short) dOutput );
			// unflag; processor now waits for next interrupt
			outReady = false;
	  	}
  	};
  
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

//void sine_init(void)
//{	
///*  Initialises an array (table[]) of sine values. */
//	int i;
//	for (i = 0; i < SINE_TABLE_SIZE; i++) {
//		table[i] = (short) ((1<<14) * sin(2*i*PI/SINE_TABLE_SIZE));
//	}
//}
//
//short advance_sample(double* index)
//{
///* Returns sine table value pointed by current index.
// * Then advances sine table index. */ 
// 	short res = table[(int) floor(*index)];
//	*index += ((double) (sineFreq*SINE_TABLE_SIZE)) / SAMP_FREQ;
//	// Ensures that count loops back if it exceeds the table size
//	while (*index >= SINE_TABLE_SIZE) *index -= SINE_TABLE_SIZE;
//	return res;
//}
//
//short rectify_sample(short sample)
//{
///* rectifies input sample */
//	short temp = sample >> 15;
//	sample ^= temp;
//	sample -= temp;
//	return sample;
//}

void shift_buffer(double sample_in, short* x)
{
	int i;
	// shifts buffer x[]
	for (i=M-1; i>0; i--)
	{
		x[i]=x[i-1]; /* move data along buffer from lower */
	} /* element to next higher */
	x[0] = sample_in; 
}

double convolution(short currentInput, short* x) {
	int i;
	double sum = 0;
	sum += currentInput * b[0]; // convolution with coefficient 0
	for (i=0; i<M; i++)
	{
		sum += b[M-i] * x[i]; // convolution with coefficients 1 to M
	}
	return sum;
}

/******************** INTERRUPT SERVICE ROUTINE ***********************/  

void ISR_AIC(void)
{
	//set flag to show that the output is ready.
	outReady = true;
}
