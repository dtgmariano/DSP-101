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

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);
void Toggle_LED1(void);
void Toggle_LED2(void);

int k;

float x[200];
float y[200];

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
   k = 0;
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
  

interrupt void cpu_timer0_isr(void)
{
   k++;
   if(k>=2) {
              Toggle_LED1();
              Toggle_LED2();
              k=0;
              }
   // Reset the watchdog counter
   ServiceDog();
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


//===========================================================================
// No more.
//===========================================================================
