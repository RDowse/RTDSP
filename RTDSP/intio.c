/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		LAB 5: IIR Filter Design

 				            ********* I N T I O. C **********

 *************************************************************************************
 				Updated for use on 6713 DSK by Danny Harvey: May-Aug 2006
				Updated for CCS V4 Sept 10
 ************************************************************************************/
 
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
#include "iir_coeff.txt" 

// math library (trig functions)
#include <math.h>

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

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

// filter order and dynamic buffers
short order;
double* x;
double* y;

 /******************************* Function prototypes ********************************/
void   init_hardware(void);     
void   init_HWI(void);
void   ISR_AIC(void);  
double iir_filter(double input);
void shift_buffer(double sample, double buffer);
/********************************** Main routine ************************************/
void main()
{
	
	// initialize board and the audio port
	init_hardware();
	
	/* initialize hardware interrupts */
	init_HWI();
	
	/* initialise x/y buffers */
	order = sizeof(a)/sizeof(a[0])-1; /* Find the order of the filter. */
	x = (double *)calloc(order+1, sizeof(double)); 
	y = (double *)calloc(order+1, sizeof(double)); 
  	 		
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

double iir_filter(double input) {
	
	double sum = 0;
	int size = sizeof(a);
	int i;
	for(i = 0; i < size; i++){
		sum += b[i]*x[size-i]-a[i]*y[size-i]
	}
	shift_buffer(input, x);
	shift_buffer(sum, y);
	return sum;
}

void shift_buffer(double sample, double buffer)
{
	short i;
	int size = sizeof(buffer);
	// shifts buffer
	for (i=BUFFER_SIZE-1; i>0; i--)
	{
		buffer[i]=buffer[i-1]; /* move data along buffer from lower */
	} /* element to next higher */
	buffer[0] = sample; 
}

/******************** INTERRUPT SERVICE ROUTINE ***********************/  

void ISR_AIC(void)
{
	// get new sample
	dInput = (double) mono_read_16Bit();
	// filtering using IIR
	short dOutput = iir_filter(dInput);
	// output result to both L/R channels
	mono_write_16Bit( (short) dOutput );
}
