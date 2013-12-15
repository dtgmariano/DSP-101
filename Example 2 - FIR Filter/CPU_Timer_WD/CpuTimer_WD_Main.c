// EXEMPLO DE APLICACAO COM INTERRUPÇÃO DE HARDWARE(CPUTIMER0) E WATCHDOG
// MAY 5, 2013   - Baseado do exemplo CPUTimer0 do headers v131 (xample_2833xCpuTimer.c)
//###########################################################################
//
//
//
// DESCRIPTION:
//
//    This example configures CPU Timer0, 
//
//       Watch Variables:
//          CpuTimer0.InterruptCount
//
//###########################################################################


#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#define sizeArray 200
#define N 32
// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);

void Toggle_LED1(void);
void Toggle_LED2(void);
void Read(void);
void Write(void);

int count_led;
/*float X[sizeArray] = {0	,
		2.5056, 3.89543, 3.65716, 2.15825,
		0.445392,
		-0.324167	,
		0.488755	,
		2.65754	,
		5.21141	,
		6.96045	,
		7.13956	,
		5.8221	,
		3.8792	,
		2.50662	,
		2.57579	,
		4.16071	,
		6.4972	,
		8.39659	,
		8.88963	,
		7.75152	,
		5.62353	,
		3.66526	,
		2.92604	,
		3.77615	,
		5.70634	,
		7.6041	,
		8.35713	,
		7.4567	,
		5.27404	,
		2.85972	,
		1.37499	,
		1.46309	,
		2.90151	,
		4.72363	,
		5.74369	,
		5.20073	,
		3.17223	,
		0.533338	,
		-1.51622	,
		-2.10117	,
		-1.1438	,
		0.595289	,
		1.93081	,
		1.89672	,
		0.271217	,
		-2.30166	,
		-4.65978	,
		-5.75156	,
		-5.20469	,
		-3.52671	,
		-1.82869	,
		-1.22182	,
		-2.21397	,
		-4.4334	,
		-6.82932	,
		-8.24061	,
		-8.02443	,
		-6.40401	,
		-4.34703	,
		-3.04055	,
		-3.24719	,
		-4.89136	,
		-7.10011	,
		-8.67615	,
		-8.74689	,
		-7.23755	,
		-4.91187	,
		-2.95846	,
		-2.34689	,
		-3.30261	,
		-5.1854	,
		-6.83705	,
		-7.20998	,
		-5.93598	,
		-3.52671	,
		-1.09741	,
		0.236601	,
		-0.0367022	,
		-1.54966	,
		-3.25549	,
		-3.997	,
		-3.13515	,
		-0.89685	,
		1.74673	,
		3.60517	,
		3.91274	,
		2.74209	,
		0.963484	,
		-0.228228	,
		0.0373524	,
		1.8331	,
		4.39365	,
		6.52905	,
		7.26872	,
		6.38644	,
		4.52204	,
		2.83362	,
		2.36897	,
		3.49682	,
		5.70634	,
		7.88343	,
		8.9142	,
		8.28834	,
		6.37553	,
		4.2248	,
		2.99591	,
		3.33062	,
		5.00506	,
		7.05123	,
		8.28213	,
		7.93558	,
		6.08795	,
		3.61336	,
		1.7106	,
		1.25411	,
		2.32089	,
		4.14968	,
		5.55472	,
		5.56956	,
		3.9721	,
		1.40625	,
		-0.965915	,
		-2.09273	,
		-1.60168	,
		-2.58E-07	,
		1.60168	,
		2.09273	,
		0.965915	,
		-1.40625	,
		-3.9721	,
		-5.56956	,
		-5.55472	,
		-4.14968	,
		-2.32089	,
		-1.25411	,
		-1.7106	,
		-3.61336	,
		-6.08795	,
		-7.93558	,
		-8.28213	,
		-7.05123	,
		-5.00506	,
		-3.33063	,
		-2.99591	,
		-4.2248	,
		-6.37553	,
		-8.28834	,
		-8.9142	,
		-7.88343	,
		-5.70634	,
		-3.49682	,
		-2.36897	,
		-2.83362	,
		-4.52204	,
		-6.38643	,
		-7.26872	,
		-6.52906	,
		-4.39365	,
		-1.83311	,
		-0.0373527	,
		0.228228	,
		-0.963484	,
		-2.74208	,
		-3.91274	,
		-3.60517	,
		-1.74673	,
		0.89685	,
		3.13515	,
		3.997	,
		3.2555	,
		1.54966	,
		0.0367025	,
		-0.236601	,
		1.09741	,
		3.52671	,
		5.93598	,
		7.20998	,
		6.83705	,
		5.1854	,
		3.30261	,
		2.34689	,
		2.95846	,
		4.91187	,
		7.23755	,
		8.74689	,
		8.67615	,
		7.10011	,
		4.89136	,
		3.24719	,
		3.04055	,
		4.34703	,
		6.40401	,
		8.02443	,
		8.24061	,
		6.82932	,
		4.4334	,
		2.21397	,
		1.22182	,
		1.82869	}; */
float X[sizeArray];
float Y[sizeArray];
float yn = 0;

float h[N] = {7.721061893433e-05,0.0005722734696623, 0.001426237191501, 0.003052593168767,
			  0.005674100021448, 0.009558090186268,  0.01488990809316,   0.0217362872309,
			  0.02999783988332,  0.03938382675721,  0.04941269219336,  0.05944344335524,
			  0.06873701141268,  0.07653832561754,  0.08217073615837,  0.08512397533896,
			  0.08512397533896,  0.08217073615837,  0.07653832561754,  0.06873701141268,
			  0.05944344335524,  0.04941269219336,  0.03938382675721,  0.02999783988332,
			  0.0217362872309,   0.01488990809316, 0.009558090186268, 0.005674100021448,
			  0.003052593168767, 0.001426237191501,0.0005722734696623,7.721061893433e-05};
float x[N] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2833x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.TINT0 = &cpu_timer0_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripheral. This function can be
//         found in DSP2833x_CpuTimers.c
   InitCpuTimers();   // For this example, only initialize the Cpu Timers

// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 150MHz CPU Freq, 1 second Period (in uSeconds)

   ConfigCpuTimer(&CpuTimer0, 150, 100000);

// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in DSP2833x_CpuTimers.h), the
// below settings must also be updated.

   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0

// Step 5. User specific code, enable interrupts:
// Configura Portas dos LEDS no Control Card do 28335 - Experimenter Kit
// LED2-> GPIO31   e  LED3-> GPIO34

   EALLOW;
   GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   // Reset output latch
   GpioCtrlRegs.GPAMUX2.bit.GPIO31  = 0;   // GPIO31 = GPIO31
   GpioCtrlRegs.GPADIR.bit.GPIO31   = 1;   // GPIO31 = output
   
   GpioDataRegs.GPBSET.bit.GPIO34 = 1;     // Set output latch
   GpioCtrlRegs.GPBMUX1.bit.GPIO34  = 0;   // GPIO34 = GPIO34
   GpioCtrlRegs.GPBDIR.bit.GPIO34   = 1;   // GPIO34 = output
   EDIS;

// Enable CPU int1 which is connected to CPU-Timer 0,

   IER |= M_INT1;

// Enable TINT0 in the PIE: Group 1 interrupt 7
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Reset the watchdog counter
   ServiceDog();
      
// Enable the watchdog
// 		Watchdog Control Register (WDCR) - pag54 - Sprufb0d.pdf
// 		|  7   |  6   | 5 4 3 | 2 1 0|
// 		|WDFLAG| WDDIS| WDCHK | WDPS |
//         0      0     1 0 1   1 1 1 b
// 		WDPS=111b -> WDCLK = OSCCLK/512/64=150Mhz/512/64 = 4477/6Hz 
// 		-> 256passos - estouro em 56ms
   EALLOW;
   SysCtrlRegs.WDCR = 0x002f;   
   EDIS;

// Step 6. IDLE loop. Just sit and loop forever (optional):
   count_led = 0;
   Read();
   int i;
   for(i=0; i<sizeArray; i++)
       {
           int k;                                      //  Alternative implementation
           for(k=0; k < N-1; k++)               //  for(int k=N-1; k>0; k--)
           {                                        //  {
             x[N-k-1] = x[N-k-2];//shift the data   //    x[k] = x[k-1];
           }                                        //  }

           x[0] = X[i]; // move input sample to buffer
           yn = 0; // clear output sample

           for(k=0; k < N; k++)
           {
               yn += h[k]*x[k]; // multiply data on coefficients with accumulation
           }

           Y[i] = yn;
       }
   Write();
   for(;;);

}

void Toggle_LED1(void)
  {
   GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
  }
  
void Toggle_LED2(void)
  {
   GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
  }

void Read(void)
{

}

void Write(void)
{

}
interrupt void cpu_timer0_isr(void)
{
	count_led++;
   if(count_led>=2) {
              Toggle_LED1();
              Toggle_LED2();
              count_led=0;
              }
   // Reset the watchdog counter
   ServiceDog();
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


//===========================================================================
// No more.
//===========================================================================
