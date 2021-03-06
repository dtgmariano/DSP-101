#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <math.h>

// Prototype statements for functions found within this file.
interrupt void adc_isr(void);
void Toggle_LED1(void);
void Toggle_LED2(void);
void InitEPwm1Example(void);
void Config_ADC(void);


#define N 32
#define sizeArray 100

#define freqAmostragem 10000

// Global variables used in this example:
int Voltage1[sizeArray], Voltage2[sizeArray];
float PWM_signal[sizeArray];

int  ConversionCount;
float phase,w, Ts;
float amplitude;
int comparacao,offset, A3,tensao;
int j,segundos_count;

float X[sizeArray], Y[sizeArray];
float yn = 0;
/*float h[N] = {	0.01088851098307,  0.00826166872286,  0.01108546704709,  0.01428820677656,
				0.01782254652812,  0.02161186645364,  0.02556816195059,  0.02960025792067,
				0.03359639112285,  0.03743328713064,  0.04099288718132,  0.04417223870103,
				0.04684548673088,  0.04894102049304,  0.05037578751974,  0.05110892113801,
				0.05110892113801,  0.05037578751974,  0.04894102049304,  0.04684548673088,
				0.04417223870103,  0.04099288718132,  0.03743328713064,  0.03359639112285,
				0.02960025792067,  0.02556816195059,  0.02161186645364,  0.01782254652812,
				0.01428820677656,  0.01108546704709,  0.00826166872286,  0.01088851098307};*/
float h[N] = {
	    0.01457030917627, 0.003074439193362,-0.0003420519590572,-0.006217750677177,
	   -0.01349789894126,  -0.0203265553426, -0.02431538117982, -0.02301575815946,
	   -0.01450464494714,  0.00206919243524,  0.02610549020306,  0.05544112985627,
	    0.08658177194417,   0.1152761443351,   0.1373120968659,   0.1492739480982,
	     0.1492739480982,   0.1373120968659,   0.1152761443351,  0.08658177194417,
	    0.05544112985627,  0.02610549020306,  0.00206919243524, -0.01450464494714,
	   -0.02301575815946, -0.02431538117982,  -0.0203265553426, -0.01349789894126,
	  -0.006217750677177,-0.0003420519590572, 0.003074439193362,  0.01457030917627
	};
float x[N] = {	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

main()
{
   phase=0;
   w=2*3.1415927*100; // Frequencia fundamental do sinal gerado = 100Hz
   Ts=1/10000.0;      // Interrup��o do ePWM1 =10kHz -> taxa de amostragem
   offset=7500;
   j=0;
   segundos_count=0;

// Step 1.
   InitSysCtrl();

// Step 2. Initialize GPIO:
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example

// Step 3.
   DINT;

// Initialize the PIE control registers to their default state.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected register
   PieVectTable.ADCINT = &adc_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
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

   while(1) //loop infinito
   {
	   asm(" NOP");
	   if(segundos_count >=2)
	   {
		   segundos_count=0;     //Zera contador de segundos
		   ConversionCount = 0;  // habilita grava��o da aquisi��o nos vetores a cada 2 segundos
		   A3=A3+200; //Incrementa amplitude do terceiro harm�nico - sinal de controle PWM
		   if (A3>=2000)
			   A3=0;
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
   //    ou na p�gina 18 do manual sprug04a.pdf
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
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW; // Modo de duplo buffer de compara��o ativado p/ CMPA
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW; // Modo de duplo buffer de compara��o ativado p/ CMPB
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;// Recarga do CMPA ocorre qdo TBcounter=0
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//Recarga do CMPB ocorre qdo TBcounter=0

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = 7500;    // Set compare A value (50% do per�odo)
   EPwm1Regs.CMPB = 7500;              // Set Compare B value (50% do per�odo)

   // Set actions
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;           // Set EPWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;         // Clear EPWM1A on event A (TBCTR=CMPA), up count

   EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;        // Set EPWM1B on Zero
   EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;          // Clear EPWM1B on event B (TBCTR=CMPA) up count                                              // EPWM1A e EPWM1B->sim�tricos

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
// SOCA � configurado no ePWM1
   AdcRegs.ADCTRL1.bit.CPS = 0;           // CPS=0    pag. 38 SPRU812A.pdf
   AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x3;    // Setup ADCCLK = HSPCLK/[6*(CPS + 1)] pag. 38 SPRU812A.pdf
                                          // -> HSPCLK=75Mhz -> ADCCLK=12.5Mhz
   AdcRegs.ADCTRL1.bit.ACQ_PS=3;          // Acquisition window size (duration the sampling switch is closed)
                                          // sample time = ACQ_PS(ADCTRL1[11:8]) + 1 times the ADCLK period.
                                          // 4*1/12.5MHz = 320nS
   AdcRegs.ADCMAXCONV.all = 0x0000;       // Setup 2 conv's on SEQ1
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // Setup ADCINA0 as 1st SEQ1 conv.
   AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // Setup ADCINA1 as 2nd SEQ1 conv.

   AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;// Enable SOCA from ePWM to start SEQ1
   AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)
}


interrupt void  adc_isr(void)
{
	if(ConversionCount < sizeArray)
    {
		Voltage1[ConversionCount] = AdcRegs.ADCRESULT0 - 0x8000; //Convert amostras Q15/16-bits  (p.u.)
        //Voltage2[ConversionCount] = AdcRegs.ADCRESULT1 - 0x8000; //Convert amostras Q15/16-bits  (p.u.)
        //PWM_signal[ConversionCount]= amplitude;
        ConversionCount++;
    }
    else
    {
    	ConversionCount = 0;
    	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   // Turn on LED2 - fim do  preenchimento dos vetores de amostras
    }

	j++;
	if(j==10000)
	{
		j=0;
	    segundos_count++;
	    Toggle_LED1();
	}


	int i;

	for(i=0; i<sizeArray; i++)
	{
		int k;                                      //  Alternative implementation
	    for(k=0; k < N-1; k++)               //  for(int k=N-1; k>0; k--)
	    {                                        //  {
	    	x[N-k-1] = x[N-k-2];//shift the data   //    x[k] = x[k-1];
	    }                                        //  }

	    x[0] = Voltage1[i]; // move input sample to buffer
	    yn = 0; // clear output sample

	    for(k=0; k < N; k++)
	    {
	        yn += h[k]*x[k]; // multiply data on coefficients with accumulation
	    }

	    Voltage2[i] = yn;
    }


    tensao= AdcRegs.ADCRESULT0 - 0x8000;
    amplitude=(float)tensao/5.0;
    comparacao =(int)amplitude;
    EPwm1Regs.CMPA.half.CMPA = comparacao + offset;
    EPwm1Regs.CMPB= offset - comparacao;

   	if(tensao>14000)
   		GpioDataRegs.GPASET.bit.GPIO31 = 1;
   	else
   		GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;

//  Reinitialize for next ADC sequence
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1 ;        // reset sequencer to state CONV00 - pag. 35 SPRU812A.pdf
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}



