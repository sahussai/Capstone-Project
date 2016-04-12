#include <stdio.h>
#include <file.h>

#include "DSP28x_Project.h"     // DSP28x Headerfile
#include "ti_ascii.h"

#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/sci.h"
#include "f2802x_common/include/sci_io.h"
#include "f2802x_common/include/wdog.h"
#include "f2802x_common/include/pwm.h"

extern void DSP28x_usDelay(Uint32 Count);

uint16_t LoopCount;
//Sensor 1
uint16_t VoltageSOC2;
uint16_t VoltageSOC4;
uint16_t VoltageSOC5;

//Sensor 2
uint16_t VoltageSOC3;
uint16_t VoltageSOC6;
uint16_t VoltageSOC10;

//Sensor 3
uint16_t VoltageSOC1;
uint16_t VoltageSOC7;
uint16_t VoltageSOC11;

//Sensor 4
uint16_t VoltageSOC8;
uint16_t VoltageSOC9;
uint16_t VoltageSOC12;


ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
SCI_Handle mySci;
PWM_Handle myPwm;



__interrupt void adc_isr(void);
//send A after sending ADC value for synchronizing purposes
void Transmit(void)
{
    // Channel 1
	printf("%dA", VoltageSOC2);
	printf("%dA", VoltageSOC4);
    printf("%dA", VoltageSOC5);


    //Channel 2
    printf("%dA", VoltageSOC3 + 4096);
    printf("%dA", VoltageSOC6 + 4096);
    printf("%dA", VoltageSOC10 + 4096);

    //Channel 3
    printf("%dA", VoltageSOC1 + 8192);
    printf("%dA", VoltageSOC7 + 8192);
    printf("%dA", VoltageSOC11 + 8192);
}

// SCIA  8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scia_init()
{

    CLK_enableSciaClock(myClk);

    // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    SCI_disableParity(mySci);
    SCI_setNumStopBits(mySci, SCI_NumStopBits_One);
    SCI_setCharLength(mySci, SCI_CharLength_8_Bits);
    
    SCI_enableTx(mySci);
    SCI_enableRx(mySci);
    SCI_enableTxInt(mySci);
    SCI_enableRxInt(mySci);

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
    // Configured for 115.2kbps
#if (CPU_FRQ_60MHZ)
    SCI_setBaudRate(mySci, SCI_BaudRate_9_6_kBaud);
#elif (CPU_FRQ_50MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)13);
#elif (CPU_FRQ_40MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)10);
#endif

    SCI_enableFifoEnh(mySci);
    SCI_resetTxFifo(mySci);
    SCI_clearTxFifoInt(mySci);
    SCI_resetChannels(mySci);
    SCI_setTxFifoIntLevel(mySci, SCI_FifoLevel_Empty);

    SCI_resetRxFifo(mySci);
    SCI_clearRxFifoInt(mySci);
    SCI_setRxFifoIntLevel(mySci, SCI_FifoLevel_4_Words);

    SCI_setPriority(mySci, SCI_Priority_FreeRun);
    
    SCI_enable(mySci);
  
    return;
}

void main()
	{
    volatile int status = 0;
    volatile FILE *fid;
    
    CPU_Handle myCpu;
    PLL_Handle myPll;
    WDOG_Handle myWDog;
    
    // Initialize all the handles needed for this application    
    myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    mySci = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
    myPwm = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));


    // Perform basic system initialization    
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    (*Device_cal)();
    


    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);
    
    // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 10 / 2
    PLL_setup(myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);
    
    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);
        
    // If running from flash copy RAM only functions to RAM   
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif      

    // Initalize GPIO
    // Enable XCLOCKOUT to allow monitoring of oscillator 1
    GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_XCLKOUT);
    CLK_setClkOutPreScaler(myClk, CLK_ClkOutPreScaler_SysClkOut_by_1);

    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);   

    // Register interrupt handlers in the PIE vector table
       PIE_registerPieIntHandler(myPie, PIE_GroupNumber_10, PIE_SubGroupNumber_1, (intVec_t)&adc_isr);


    // Initialize SCIA
    scia_init();
    
    // Initialize the ADC
    ADC_enableBandGap(myAdc);
    ADC_enableRefBuffers(myAdc);
    ADC_powerUp(myAdc);
    ADC_enable(myAdc);
    ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);

    // Enable ADCINT1 in PIE
       PIE_enableAdcInt(myPie, ADC_IntNumber_1);
       // Enable CPU Interrupt 1
       CPU_enableInt(myCpu, CPU_IntNumber_10);
       // Enable Global interrupt INTM
       CPU_enableGlobalInts(myCpu);
       // Enable Global realtime interrupt DBGM
       CPU_enableDebugInt(myCpu);

    ADC_setIntPulseGenMode(myAdc, ADC_IntPulseGenMode_Prior);
    ADC_enableInt(myAdc, ADC_IntNumber_1);                                  //Enable ADCINT1
    ADC_setIntMode(myAdc, ADC_IntNumber_1, ADC_IntMode_ClearFlag);          //Disable ADCINT1 Continuous mode
    ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC2);                 //setup EOC2 to trigger ADCINT1 to fire


    //A2(2) and A4(4) and B2(5) Sample the same Signal (id0)
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_2, ADC_SocChanNumber_A2);    //set SOC2 channel select to ADCINA2
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_4, ADC_SocChanNumber_A4);    //set SOC1 channel select to ADCINA4 //Use another channel to sample the same signal
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_5, ADC_SocChanNumber_B2);

    //A3(3) and A6(6) and B3(10) Sample the same signal (id1)
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_3, ADC_SocChanNumber_A3);
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_6, ADC_SocChanNumber_A6);
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_10, ADC_SocChanNumber_B3);

    //A1(1) and A7(7) and B4(11) Sample the Same signal (id2)
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A1);
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_7, ADC_SocChanNumber_A7);
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_11, ADC_SocChanNumber_B4);

    //B1(8) and B7(9) and B6(12) sample the same signal (id3)
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_8, ADC_SocChanNumber_B1);
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_9, ADC_SocChanNumber_B7);
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_12, ADC_SocChanNumber_B6);


    //A2 and A4 and B2(5)
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_2, ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_4, ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_5, ADC_SocTrigSrc_EPWM1_ADCSOCA);

    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_4, ADC_SocSampleWindow_14_cycles);   //Set SOC1 acquisition period to 7 ADCCLK
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_2, ADC_SocSampleWindow_14_cycles);
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_5, ADC_SocSampleWindow_14_cycles);//set SOC2 S/H Window to 14 ADC Clock Cycles, (13 ACQPS plus 1)

    //A3 and A6 and B3(10)
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_3, ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_6, ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_10, ADC_SocTrigSrc_EPWM1_ADCSOCA);

    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_3, ADC_SocSampleWindow_14_cycles);   //Set SOC1 acquisition period to 7 ADCCLK
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_6, ADC_SocSampleWindow_14_cycles);   //set SOC2 S/H Window to 14 ADC Clock Cycles, (13 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_10, ADC_SocSampleWindow_14_cycles);


    //A1 and A7 and B4(11)
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_1, ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_7, ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_11, ADC_SocTrigSrc_EPWM1_ADCSOCA);

    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_14_cycles);
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_7, ADC_SocSampleWindow_14_cycles);
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_11, ADC_SocSampleWindow_14_cycles);


    //B1(8) and B7(9) and B6(12)
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_8, ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_9, ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_12, ADC_SocTrigSrc_EPWM1_ADCSOCA);

    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_8, ADC_SocSampleWindow_14_cycles);   //Set SOC1 acquisition period to 7 ADCCLK
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_9, ADC_SocSampleWindow_14_cycles);   //set SOC2 S/H Window to 14 ADC Clock Cycles, (13 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_12, ADC_SocSampleWindow_14_cycles);

    // Enable PWM clock
    CLK_enablePwmClock(myClk, PWM_Number_1);

    // Setup PWM
    PWM_enableSocAPulse(myPwm);                                         // Enable SOC on A group
    PWM_setSocAPulseSrc(myPwm, PWM_SocPulseSrc_CounterEqualZero);   // Select SOC from from CPMA on upcount
    PWM_setSocAPeriod(myPwm, PWM_SocPeriod_FirstEvent);                 // Generate pulse on 1st event
    PWM_setPeriod(myPwm, 200);
    PWM_setCounterMode(myPwm, PWM_CounterMode_Up);                      // count up and start
    CLK_enableTbClockSync(myClk);


    // Set the flash OTP wait-states to minimum. This is important
    // for the performance of the temperature conversion function.
    FLASH_setup(myFlash);

    // Initalize GPIO
    GPIO_setPullUp(myGpio, GPIO_Number_28, GPIO_PullUp_Enable);
    GPIO_setPullUp(myGpio, GPIO_Number_29, GPIO_PullUp_Disable);
    GPIO_setQualification(myGpio, GPIO_Number_28, GPIO_Qual_ASync);
    GPIO_setMode(myGpio, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
    GPIO_setMode(myGpio, GPIO_Number_29, GPIO_29_Mode_SCITXDA);
    

    // Don't need these extra GPIOs
    // Configure GPIO 0-3 as outputs
    GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_1, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_2, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_3, GPIO_0_Mode_GeneralPurpose);
    
    GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_2, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output);
    
    GPIO_setMode(myGpio, GPIO_Number_12, GPIO_12_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, GPIO_Number_12, GPIO_Direction_Input);
    GPIO_setPullUp(myGpio, GPIO_Number_12, GPIO_PullUp_Disable);
    

    //Redirect STDOUT to SCI
    status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
    fid = fopen("scia","w");
    freopen("scia:", "w", stdout);
    setvbuf(stdout, NULL, _IONBF, 0);


    for(;;) {
        LoopCount++;
    	//Wait for ADC interrupt
    }
}

__interrupt void adc_isr(void)
{
   //Channel 1
    VoltageSOC2 = ADC_readResult(myAdc, ADC_ResultNumber_2);
    VoltageSOC4 = ADC_readResult(myAdc, ADC_ResultNumber_4);
    VoltageSOC5 = ADC_readResult(myAdc, ADC_ResultNumber_5);

    //Channel 2

    VoltageSOC3 = ADC_readResult(myAdc, ADC_ResultNumber_3);
    VoltageSOC6 = ADC_readResult(myAdc, ADC_ResultNumber_6);
    VoltageSOC10 = ADC_readResult(myAdc, ADC_ResultNumber_10);

    //Channel 3
    VoltageSOC1 = ADC_readResult(myAdc, ADC_ResultNumber_1);
    VoltageSOC7 = ADC_readResult(myAdc, ADC_ResultNumber_7);
    VoltageSOC11 = ADC_readResult(myAdc, ADC_ResultNumber_11);

    //Channel 4
   VoltageSOC8 = ADC_readResult(myAdc, ADC_ResultNumber_8);
   VoltageSOC9 = ADC_readResult(myAdc, ADC_ResultNumber_9);
   VoltageSOC12 = ADC_readResult(myAdc, ADC_ResultNumber_12);

   Transmit();

    // Clear ADCINT1 flag reinitialize for next SOC
    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_10);

    return;
}
