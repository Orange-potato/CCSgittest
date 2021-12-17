//============================================================================================
// 선행처리 지시
//--------------------------------------------------------------------------------------------
#include "F28x_Project.h"						// Device Headerfile and Examples Include File
#include "F2837xD_epwm.h"
//============================================================================================

#define SYSTEM_CLOCK        200E6   // 200MHz
#define TBCLK               100E6   // 100MHz
#define PWM_CARRIER         60E3    // 60kHz
#define PWM_DUTY_RATIO_A    5.472E-1    // 0.5472, 54%

//============================================================================================
//	함수 선언
//--------------------------------------------------------------------------------------------
interrupt void adcb1_isr(void);

// Prototype statements for functions found within this Example
void InitEPwm4Module(void);
interrupt void EPwm4Isr(void);
//============================================================================================


//============================================================================================
// 시스템에서 사용할 전역 변수 선언
//--------------------------------------------------------------------------------------------
Uint16 ADC_value01, ADC_value02;
Uint16 Loop_cnt, ADC_cnt;
Uint16  BackTicker;
Uint16  EPwm4IsrTicker;

float32 PwmCarrierFrequency;
float32 PwmDutyRatioA;
float32 FallingEdgeDelay;
float32 RisingEdgeDelay;
//============================================================================================


//============================================================================================
//	메인 함수 - 시작 
//============================================================================================
void main(void)
{
//============================================================================================
// Step 1. 전역 인터럽트 해제
//--------------------------------------------------------------------------------------------
	DINT;
    IER = 0x0000;
    IFR = 0x0000;
//============================================================================================


//============================================================================================
// Step 2. 시스템 클럭 초기화:
//--------------------------------------------------------------------------------------------
    //  2.1 InitSysCtrl()
    //      2.1.1 Disables the watchdog
    //      2.1.2 Set the PLLCR for proper SYSCLKOUT frequency
    //      2.1.3 Set the pre-scaler for the high and low frequency peripheral clocks
    //      2.1.4 Enable the clocks to the peripherals
    //  2.2 Initialize GPIO MUX

    InitSysCtrl();

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;  /* Enable pull-up on GPIO6 (EPWM4A) */
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;  /* Enable pull-up on GPIO7 (EPWM4B) */
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1; /* Configure GPIO6 as EPWM4A */
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1; /* Configure GPIO7 as EPWM4B */

    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 0);                 //CJS
    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_SYNC);       //CJS
    GPIO_WritePin(13, 0);                                   //CJS
	EALLOW;											// PWM 모듈에 공급되는 시스템 클럭 분주
	ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;		// 0: 1분주, 1: 1/2분주
	EDIS;
//============================================================================================


//============================================================================================
// Step 3. 인터럽트 초기화:
//--------------------------------------------------------------------------------------------
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;


	// Vector Remapping
	EALLOW;
	PieVectTable.ADCB1_INT = &adcb1_isr;
	EDIS;

	// 외부 인터터트 포합된 백터 활성화
	PieCtrlRegs.PIEIER1.bit.INTx2 = 1;	// PIE 인터럽트(ADCA1INT) 활성화
	IER |= M_INT1;						// CPU 인터럽트(INT1)  활성화

    InitPieVectTable();
//============================================================================================


//============================================================================================
// Step 4. ADC 초기화
//--------------------------------------------------------------------------------------------

	EALLOW;

	//  5.1 Interrupt Service routine re-mapping and Interrupt vector enable
	EALLOW;
	PieVectTable.EPWM4_INT = &EPwm4Isr; // Interrupt Service Routine Re-mapping
	EDIS;

	PieCtrlRegs.PIEIER3.bit.INTx4 = 1;  // Enable PIE group 3 interrupt 4 for EPWM4_INT

    IER |= M_INT3;  // Enable CPU INT3 for EPWM4_INT

	// ADC-B 모드 설정 및 Power-up
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; 			// ADCCLK = SYSCLK / 4, SYSCLK = 200MHz
	AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;		// ADC Interrupt Pulse Position: 변환종료 후 발생
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;			// ADC 시동(Power-up)
	DELAY_US(1000);								// ADC가 시동되는 동안 1ms 지연

	// ADC-D 모드 설정 및 Power-up
	AdcdRegs.ADCCTL2.bit.PRESCALE = 6; 			// ADCCLK = SYSCLK / 4, SYSCLK = 200MHz
	AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;			// ADC 시동(Power-up)
	DELAY_US(1000);								// ADC가 시동되는 동안 1ms 지연

	EDIS;


	EALLOW;
	// ADC-B SOC(채널, S/H시간, 트리거소스) 및 인터럽트 설정
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;			// SOC0 : ADCINB0 채널 변환
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;			// (S/H 시간) 설정 75ns = (ACQPS+1)/SYSCLK
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 7;		// SOC0: ePWM2 SOCA/C로 트리거
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;		// EOC0가 ADCINT1를 발생
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;		// ADCINT1 활성화
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		// ADCINT1 flag 클리어 확인

	// ADC-D SOC(채널, S/H시간, 트리거소스) 설정
	AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;			// SOC0 : ADCIND0 채널 변환
	AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14;			// (S/H 시간) 설정 75ns = (ACQPS+1)/SYSCLK
	AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 7;		// SOC0: ePWM2 SOCA/C로 트리거
	EDIS;

	//ADC SOC 트리거를 위한 ePWM2 설정
	EALLOW;
	EPwm2Regs.ETSEL.bit.SOCAEN	= 1;	        // SOCA 이벤트 트리거 Enable
	EPwm2Regs.ETSEL.bit.SOCASEL	= 2;	        // SCCA 트리거 조건 : 카운터 주기 일치 시
	EPwm2Regs.ETPS.bit.SOCAPRD = 1;		        // SOCA 이벤트 분주 설정 : 트리거 조건 한번 마다
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;	// count up and start
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;			// TBCLK = [SYSCLKOUT / ((HSPCLKDIV*2) * 2^(CLKDIV))]
	EPwm2Regs.TBCTL.bit.CLKDIV = 0;				// TBCLK = [200MHz / (1*1)] = 200MHz
	EPwm2Regs.TBPRD = 10000 - 1;				// TBCLK/(TBPRD+1) = 200MHz/10,000 = 20KHz
	EPwm2Regs.TBCTR = 0x0000;					// TB 카운터 초기화
	EDIS;

//============================================================================================


//============================================================================================

//6.1 Initialize Peripherals for User Application
	    EALLOW;
	    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	    EDIS;

	    EALLOW;
	    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1; // 0: EPWMCLK = SYSCLKOUT = 200MHz
	                                                // 1: EPWMCLK = SYSCLKOUT/2 = 100MHz (Default)
	    EDIS;

	    InitEPwm4Module();  // Initialize EPWM4 Module

	    EALLOW;
	    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	    EDIS;

// Step 6. 변수 초기화
//-------------------------------------------------------------------------------------------- 
	ADC_value01 = 0;
	ADC_value02 = 0;
	Loop_cnt = 0;
    BackTicker = 0;
    EPwm4IsrTicker = 0;

    PwmCarrierFrequency = PWM_CARRIER;
    PwmDutyRatioA = PWM_DUTY_RATIO_A;
    FallingEdgeDelay = (1.0 / TBCLK) * EPwm4Regs.DBFED.bit.DBFED;
    RisingEdgeDelay = (1.0 / TBCLK) * EPwm4Regs.DBRED.bit.DBRED;
//============================================================================================


//============================================================================================ 
// 전역 인터럽트 활성화 및 and 리얼타임 디버깅 이벤트 활성화
//--------------------------------------------------------------------------------------------
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
//============================================================================================


//============================================================================================
// IDLE loop. Just sit and loop forever :
//--------------------------------------------------------------------------------------------
	for(;;)
	{	
		Loop_cnt++;
        BackTicker++;

        FallingEdgeDelay = (1.0 / TBCLK) * EPwm4Regs.DBFED.bit.DBFED;
        RisingEdgeDelay = (1.0 / TBCLK) * EPwm4Regs.DBRED.bit.DBRED;
	}
//============================================================================================

}
//============================================================================================
//	메인 함수 - 끝 
//============================================================================================



//============================================================================================
// ADC 인터럽트 서비스 루틴
//--------------------------------------------------------------------------------------------
interrupt void adcb1_isr(void)
{
	ADC_value01= AdcbResultRegs.ADCRESULT0;
	ADC_value02= AdcdResultRegs.ADCRESULT0;

	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//============================================================================================

//  Step 10
//  10.1 Local Interrupt Service Routines & Functions
interrupt void EPwm4Isr(void)
{
    EPwm4IsrTicker++;

    EPwm4Regs.TBPRD = (TBCLK / PwmCarrierFrequency) - 1;
    EPwm4Regs.CMPA.bit.CMPA = (EPwm4Regs.TBPRD + 1) * PwmDutyRatioA;

    // Clear INT flag for this timer
    EPwm4Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.bit.ACK3 = 1;
}

void InitEPwm4Module(void)
{
    // Setup Counter Mode and Clock
    EPwm4Regs.TBCTL.bit.CTRMODE = 0;        // Count Up (Asymmetric)
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;      // TBCLK = SYSCLKOUT / EPWMCLKDIV / (HSPCLKDIV * CLKDIV) = 100MHz
    EPwm4Regs.TBCTL.bit.CLKDIV = 0;

    // Setup Phase
    EPwm4Regs.TBPHS.bit.TBPHS = 0;          // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0;          // Disable phase loading

    // Setup Period (Carrier Frequency)
    EPwm4Regs.TBPRD = (TBCLK/PWM_CARRIER)-1;    // Set Timer Period, (100MHz/100KHz)-1 = 999 (0x03E7)
    EPwm4Regs.TBCTR = 0;                        // Clear Counter

    // Set Compare Value
    EPwm4Regs.CMPA.bit.CMPA = (Uint16)((EPwm4Regs.TBPRD + 1) * PWM_DUTY_RATIO_A);   // Set Compare A Value to 50%

    // Setup shadowing
    EPwm4Regs.TBCTL.bit.PRDLD = 0;          // Period Register is loaded from its shadow when CNTR=Zero
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = 0;     // Compare A Register is loaded from its shadow when CNTR=Zero
    EPwm4Regs.CMPCTL.bit.LOADAMODE = 0;

    // Set actions
    EPwm4Regs.AQCTLA.bit.ZRO = 2;           // Set EPWM4A on CNTR=Zero
    EPwm4Regs.AQCTLA.bit.CAU = 1;           // Clear EPWM4A on CNTR=CMPA, Up-Count

    // Set Dead-time
    EPwm4Regs.DBCTL.bit.IN_MODE = 0;        // EPWMxA is the source for both falling-edge & rising-edge delay
    EPwm4Regs.DBCTL.bit.OUT_MODE = 3;       // Dead-band is fully enabled for both rising-edge delay on EPWMxA and falling-edge delay on EPWMxB
    EPwm4Regs.DBCTL.bit.POLSEL = 2;         // Active High Complementary (AHC). EPWMxB is inverted
    EPwm4Regs.DBFED.bit.DBFED = 30;         // 1usec, Falling Edge Delay
    EPwm4Regs.DBRED.bit.DBRED = 30;         // 1usec, Rising Edge Delay

    // Set Interrupts
    EPwm4Regs.ETSEL.bit.INTSEL = 1;         // Select INT on CNTR=Zero
    EPwm4Regs.ETPS.bit.INTPRD = 1;          // Generate INT on 1st event
    EPwm4Regs.ETSEL.bit.INTEN = 1;          // Enable INT
}


//  End of file.


