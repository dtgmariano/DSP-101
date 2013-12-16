// ADC_teste.c
// Baseado no exemplo do headers 131 - Example_2833xAdc.c
//###########################################################################
//
// ASSUMPTIONS:
//
//   This program requires the DSP2833x header files.
//
//   Make sure the CPU clock speed is properly defined in
//   DSP2833x_Examples.h before compiling this example.
//
//   Connect signals to be converted to A2 and A3.
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
//            0        1          0        0    Jump to SARAM	    <- "boot to SARAM"
//            0        0          1        1    Branch to check boot mode
//            0        0          1        0    Boot to flash, bypass ADC cal
//            0        0          0        1    Boot to SARAM, bypass ADC cal
//            0        0          0        0    Boot to SCI-A, bypass ADC cal
//                                              Boot_Table_End$
//
// DESCRIPTION:
//
//   This example sets up the PLL in x10/2 mode.
//
//   For 150 MHz devices (default)
//   divides SYSCLKOUT by six to reach a 25.0Mhz HSPCLK
//   (assuming a 30Mhz XCLKIN).
//
//   Interrupts are enabled and the ePWM1 is setup to generate a periodic
//   ADC SOC on SEQ1. Two channels are converted, ADCINA0 and ADCINA1.
//
//   Watch Variables:
//
//         Voltage1[100]     Last 100 ADCRESULT0 values
//         Voltage2[100]     Last 100 ADCRESULT1 values
//         ConversionCount  Current result number 0-99
//         LoopCount        Idle loop counter
//
//
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <math.h>

// Prototype statements for functions found within this file.
interrupt void adc_isr(void);
void Toggle_LED1(void);
void Toggle_LED2(void);
void InitEPwm1Example(void);
void Config_ADC(void);

// Global variables used in this example:
  int Voltage1[100];
  int Voltage2[100];
  float PWM_signal[100];
  int  ConversionCount;
  float phase,w, Ts;
  float amplitude;
  int comparacao,offset, A3,tensao;
  int j,segundos_count;


main()
{
   phase=0;
   w=2*3.1415927*100; // Frequencia fundamental do sinal gerado = 100Hz
   Ts=1/10000.0;      // Interrupção do ePWM1 =10kHz -> taxa de amostragem
   offset=7500;
   j=0;
   segundos_count=0;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
   InitSysCtrl(); //-> config SYSCLKOUT para 150Mhz (DIVSEL=2;DIV=0xA) - ver pag 46 sprufb0d.pdf
                  //-> frequência do cristal do control card 28335 Delfino = OSCCLK = 30Mhz
                  //-> config high-speed peripheral clock (HSPCLK)=SYSCLKOUT/2 = 75Mhz (usado pelo ADC module)
                  //-> config low-speed peripheral clock  (LSPCLK)=SYSCLKOUT/4 = 37.5MHz

// Step 2. Initialize GPIO:
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
   EALLOW;  // This is needed to write to EALLOW protected register
   PieVectTable.ADCINT = &adc_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals(); // Not required for this example
// For this example, only initialize the ePWM and ADC and Config_ADC
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; //desabilita clk para ePWMx
   EDIS;

   InitEPwm1Example(); //Configura ePWM1

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;//habilita clk para ePWMx
   EDIS;
   InitAdc();    // For this example, init the ADC
   Config_ADC(); // For this example, config the ADC

// Step 5. User specific code, enable interrupts:
// Configura Portas dos LEDS no Control Card do 28335 - Experimenter Kit
// LED2-> GPIO31   e  LED3-> GPIO34

   EALLOW;
   GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   // Reset output latch -  Turn on LED1
   GpioCtrlRegs.GPAMUX2.bit.GPIO31  = 0;   // GPIO31 = GPIO31
   GpioCtrlRegs.GPADIR.bit.GPIO31   = 1;   // GPIO31 = output

   GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   // Reset output latch -  Turn on LED2
   GpioCtrlRegs.GPBMUX1.bit.GPIO34  = 0;   // GPIO34 = GPIO34
   GpioCtrlRegs.GPBDIR.bit.GPIO34   = 1;   // GPIO34 = output

  //Configure ePWM-1 pins using GPIO regs
  //Enable internal pull-up for the selected pins */
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;

// Enable ADCINT in PIE
   PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
   IER |= M_INT1; // Enable CPU Interrupt 1
   EINT;          // Enable Global interrupt INTM
   ERTM;          // Enable Global realtime interrupt DBGM

// Wait for ADC interrupt
    ConversionCount = 0;
    A3=0; //Amplitude do terceiro harmônico
     while(1) //loop infinito
   {
    asm(" NOP");
    if(segundos_count >=2)
           {
         	segundos_count=0;     //Zera contador de segundos
         	ConversionCount = 0;  // habilita gravação da aquisição nos vetores a cada 2 segundos
         	A3=A3+200; //Incrementa amplitude do terceiro harmônico - sinal de controle PWM
         	if (A3>=2000)  A3=0;
           }
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

void InitEPwm1Example(void)
{
   //Veja a definicao destas constantes em DSP2833x_EPwm_defines.h
   //    ou na página 18 do manual sprug04a.pdf
   // Setup TBCLK -
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBPRD = 15000;       // Set timer period
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
   EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;          // Clear EPWM1B on event B (TBCTR=CMPA) up count                                              // EPWM1A e EPWM1B->simétricos

   // Interrupts are disable
   // Only SOCA (ADC trigger) is enable
   EPwm1Regs.ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
   EPwm1Regs.ETSEL.bit.SOCASEL = 2;       // Select SOC on period
   EPwm1Regs.ETPS.bit.SOCAPRD = 1;        // Generate pulse on 1st event
}

void Config_ADC(void)
{
// Configure ADC
// Assumes ePWM1 clock is already enabled in InitSysCtrl()
// SOCA é configurado no ePWM1
   AdcRegs.ADCTRL1.bit.CPS = 0;           // CPS=0    pag. 38 SPRU812A.pdf
   AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x3;    // Setup ADCCLK = HSPCLK/[6*(CPS + 1)] pag. 38 SPRU812A.pdf
                                          // -> HSPCLK=75Mhz -> ADCCLK=12.5Mhz
   AdcRegs.ADCTRL1.bit.ACQ_PS=3;          // Acquisition window size (duration the sampling switch is closed)
                                          // sample time = ACQ_PS(ADCTRL1[11:8]) + 1 times the ADCLK period.
                                          // 4*1/12.5MHz = 320nS
   AdcRegs.ADCMAXCONV.all = 0x0001;       // Setup 2 conv's on SEQ1
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // Setup ADCINA0 as 1st SEQ1 conv.
   AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // Setup ADCINA1 as 2nd SEQ1 conv.

   AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;// Enable SOCA from ePWM to start SEQ1
   AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)

}


interrupt void  adc_isr(void)
{
  if(ConversionCount < 100)
      {
       Voltage1[ConversionCount] = AdcRegs.ADCRESULT0 - 0x8000; //Convert amostras Q15/16-bits  (p.u.)
       Voltage2[ConversionCount] = AdcRegs.ADCRESULT1 - 0x8000; //Convert amostras Q15/16-bits  (p.u.)
       PWM_signal[ConversionCount]= amplitude;
       ConversionCount++;
      }
      else  GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   // Turn on LED2 - fim do  preenchimento dos vetores de amostras

   // Update the CMPA and CMPB values
  // phase=phase + w*Ts; //integral de w
//   if (phase > 6.2831853) phase=phase-6.2831853;
//   amplitude=sin(phase)*4500 + sin(3*phase)*A3; // Calculo do sinal a ser modulado em ePWM1 (A3=terceito harmonico)
//   comparacao =(int)amplitude;
     tensao= AdcRegs.ADCRESULT0 - 0x8000;
     amplitude=(float)tensao/5.0;
     comparacao =(int)amplitude;
   EPwm1Regs.CMPA.half.CMPA = comparacao + offset;
   EPwm1Regs.CMPB= offset - comparacao;               // Saida B em 180 graus com saida A


   if(tensao>14000)
   {
	   j++;
	   if(j==10000)
	         {
	           j=0;
	           segundos_count++;
	           Toggle_LED1();
	         }
   }





  // Reinitialize for next ADC sequence
  AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1 ;        // reset sequencer to state CONV00 - pag. 35 SPRU812A.pdf
  AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

  return;
}



