// Exemplo ePWM - 14/01/2013
// Baseado no exemplo "Example_2833xGpioSetup.c" do headers - SPRUC530 - V131
//###########################################################################
//
// FILE:    ePWM_teste.c
//
// TITLE:   DSP2833x Device GPIO Setup
//
// ASSUMPTIONS:
//
//    This program requires the DSP2833x header files.
//
//    Two different examples are included. Select the example
//    to execute before compiling using the #define statements
//    found at the top of the code.
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The 2833x Boot Mode table is shown below.
//    For information on configuring the boot mode of an eZdsp,
//    please refer to the documentation included with the eZdsp,
//
//       $Boot_Table:
//
//         GPIO87   GPIO86     GPIO85   GPIO84
//          XA15     XA14       XA13     XA12
//           PU       PU         PU       PU
//        ==========================================
//            1        1          1        1    Jump to Flash
//            1        1          1        0    SCI-A boot
//            1        1          0        1    SPI-A boot
//            1        1          0        0    I2C-A boot
//            1        0          1        1    eCAN-A boot
//            1        0          1        0    McBSP-A boot
//            1        0          0        1    Jump to XINTF x16
//            1        0          0        0    Jump to XINTF x32
//            0        1          1        1    Jump to OTP
//            0        1          1        0    Parallel GPIO I/O boot
//            0        1          0        1    Parallel XINTF boot
//            0        1          0        0    Jump to SARAM       <- "boot to SARAM"
//            0        0          1        1    Branch to check boot mode
//            0        0          1        0    Boot to flash, bypass ADC cal
//            0        0          0        1    Boot to SARAM, bypass ADC cal
//            0        0          0        0    Boot to SCI-A, bypass ADC cal
//                                              Boot_Table_End$
//
// DESCRIPTION:
//
//
//    Configures the 2833x GPIO into two different configurations
//    This code is verbose to illustrate how the GPIO could be setup.
//    In a real application, lines of code can be combined for improved
//    code size and efficency.
//
//    This example only sets-up the GPIO.. nothing is actually done with
//    the pins after setup.
//
//    In general:
//
//       All pullup resistors are enabled.  For ePWMs this may not be desired.
//       Input qual for communication ports (eCAN, SPI, SCI, I2C) is asynchronous
//       Input qual for Trip pins (TZ) is asynchronous
//       Input qual for eCAP and eQEP signals is synch to SYSCLKOUT
//       Input qual for some I/O's and interrupts may have a sampling window
//
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include<math.h>

  float phase,w, Ts;
  int amplitude;
  int offset, comparacao1,comparacao2;
  int j,k;


// Prototype statements for functions found within this file.
void Toggle_LED1(void);
void Toggle_LED2(void);
void InitEPwm1Example(void);
void InitEPwm2Example(void);
interrupt void epwm1_isr(void);
interrupt void epwm2_isr(void);


void main(void)
{

   phase=0;
   w=377;
   Ts=1/10000.0; // Interrupção do ePWM1 =10kHz -> taxa de amostragem
   offset=7500;
   comparacao1=1500;
   comparacao2=13500;
   j=0;
   k=0;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio(); Skipped for this example// Os pinos são programados diretamente - veja o passo 5


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize PIE control registers to their default state.
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
   PieVectTable.EPWM1_INT = &epwm1_isr;
   PieVectTable.EPWM2_INT = &epwm2_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals(); // Not required for this example

// For this example, only initialize the ePWM
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; //desabilita clk para ePWMx
   EDIS;

   InitEPwm1Example(); //Configura ePWM1
   InitEPwm2Example(); //Configura ePWM2

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;//habilita clk para ePWMx
   EDIS;


// Step 5. User specific code, enable interrupts:

// Configura Portas dos LEDS no Control Card do 28335 - Experimenter Kit
// LED2-> GPIO31   e  LED3-> GPIO34

   EALLOW;
   GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   // Reset output latch
   GpioCtrlRegs.GPAMUX2.bit.GPIO31  = 0;   // GPIO31 = GPIO31
   GpioCtrlRegs.GPADIR.bit.GPIO31   = 1;   // GPIO31 = output
   
   GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   // Reset output latch
   GpioCtrlRegs.GPBMUX1.bit.GPIO34  = 0;   // GPIO34 = GPIO34
   GpioCtrlRegs.GPBDIR.bit.GPIO34   = 1;   // GPIO34 = output

  //Configure ePWM-1,ePWM-2,ePWM-3 pins using GPIO regs
  //Enable internal pull-up for the selected pins */
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO0 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO1 (EPWM2B)

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO0 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO1 as EPWM2B

    EDIS;

   // Enable CPU INT3 which is connected to EPWM1-3 INT:
   IER |= M_INT3;

  // Enable EPWM INTn in the PIE: Group 3 interrupt 1-2
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx2 = 1;


  // Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM


  // Step 6. IDLE loop. Just sit and loop forever :

   while(1) //loop infinito
   {
        asm(" NOP");
   }

}

void Toggle_LED1(void)
  {
   GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
  }
  
void Toggle_LED2(void)
  {
   GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
  }

void InitEPwm1Example()
{
   //Veja a definicao destas constantes em DSP2833x_EPwm_defines.h
   //    ou na página 18 do manual sprug04a.pdf
   // Setup TBCLK -
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBPRD = 14999;                   // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;         // Clock = SYSCLKOUT/(CLKDIV*HSPCLKDIV)  0->HSPCLKDIV=1
   EPwm1Regs.TBCTL.bit.CLKDIV = 0;            //                                       0->ClkDiv=1
                                              //CLKPWM=SYSCLKOUT=150MHz
                                              //fref_pwm=150MHz/15000=10kHz
   // Setup shadow register load on ZERO
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW; // Modo de duplo buffer de comparação ativado p/ CMPA
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW; // Modo de duplo buffer de comparação ativado p/ CMPB
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;// Recarga do CMPA ocorre qdo TBcounter=0
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//Recarga do CMPB ocorre qdo TBcounter=0

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = 7500;    // Set compare A value (50% do período)
   EPwm1Regs.CMPB = 7500;              // Set Compare B value (50% do período)

   // Set actions
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;           // Set EPWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;         // Clear EPWM1A on event A (TBCTR=CMPA), up count

   EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;        // Set EPWM1B on Zero
   EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;          // Clear EPWM1B on event B (TBCTR=CMPA) up count
                                               // EPWM1A e EPWM1B->simétricos

   // Interrupt where we will change the Compare Values
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event (qdo contador =0)
   EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate an interrupt on the first event

}

void InitEPwm2Example()
{
  //Veja a definicao destas constantes em DSP2833x_EPwm_defines.h
   //    ou na página 18 do manual sprug04a.pdf
   // Setup TBCLK
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm2Regs.TBPRD = 14999;       // Set timer period
   EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;//5;         // Clock = SYSCLKOUT/(CLKDIV*HSPCLKDIV)  5->HSPCLKDIV=10
   EPwm2Regs.TBCTL.bit.CLKDIV = 0;            //                                       0->ClkDiv=1
                                              //CLKPWM=SYSCLKOUT/10=15MHz
                                              //fref_pwm=15MHz/15000=1kHz
   // Setup shadow register load on ZERO
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW; // Modo de duplo buffer de comparação ativado p/ CMPA
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW; // Modo de duplo buffer de comparação ativado p/ CMPB
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;// Recarga do CMPA ocorre qdo TBcounter=0
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//Recarga do CMPB ocorre qdo TBcounter=0

   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = 1500;    // Set compare A value (10% do período)
   EPwm2Regs.CMPB = 13500;              // Set Compare B value (90% do período)

   // Set actions
   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;           // Set EPwm2A on Zero
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;         // Clear EPwm2A on event A (TBCTR=CMPA), up count

   EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;        // Set EPwm2B on Zero
   EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;          // Clear EPwm2B on event B (TBCTR=CMPA) up count
                                               // EPwm2A e EPWM1B->simétricos

   // Interrupt where we will change the Compare Values
   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event (qdo contador =0)
   EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate an interrupt on the first event

}

interrupt void epwm1_isr(void)  //taxa de requisicao=10kHz
{
   // Update the CMPA and CMPB values
      phase=phase + w*Ts; //integral de w
      if (phase > 6.2831853) phase=phase-6.2831853;
      amplitude=sin(phase)*6000; //Amplitude de 6000 para uma contagem do timer de 15000 - offset em 7500
      EPwm1Regs.CMPA.half.CMPA =amplitude + offset;
      EPwm1Regs.CMPB= offset - amplitude;

      j++;
      if(j==10000)
      {
      	j=0;
        Toggle_LED1();
      }

   // Clear INT flag for this timer
   EPwm1Regs.ETCLR.bit.INT = 1;
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


interrupt void epwm2_isr(void) //taxa de requisicao=1kHz
{

   // Update the CMPA and CMPB values
  // comparacao1++;
   if (comparacao1==13500) comparacao1=1500;
   EPwm2Regs.CMPA.half.CMPA =comparacao1;
   comparacao2--;
   if (comparacao2==1500) comparacao2=13500;
   EPwm2Regs.CMPB=comparacao2;
   k++;
   if(k==1000)
      {
      	k=0;
        Toggle_LED2();
        comparacao1++;
      }

   // Clear INT flag for this timer
   EPwm2Regs.ETCLR.bit.INT = 1;
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//===========================================================================
// No more.
//===========================================================================
