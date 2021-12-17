//============================================================================================
// 선행처리 지시
//--------------------------------------------------------------------------------------------
#include "F28x_Project.h"						// Device Headerfile and Examples Include File
//============================================================================================


//============================================================================================
//	함수 선언
//--------------------------------------------------------------------------------------------
interrupt void adcb1_isr(void);
//============================================================================================


//============================================================================================
// 시스템에서 사용할 전역 변수 선언
//--------------------------------------------------------------------------------------------
Uint16 ADC_value01, ADC_value02;
Uint16 Loop_cnt, ADC_cnt;
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
//============================================================================================


//============================================================================================
// Step 2. 시스템 클럭 초기화:
//--------------------------------------------------------------------------------------------
	InitSysCtrl();

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
	InitPieVectTable();

	// Vector Remapping
	EALLOW;
	PieVectTable.ADCB1_INT = &adcb1_isr;
	EDIS;

	// 외부 인터터트 포합된 백터 활성화
	PieCtrlRegs.PIEIER1.bit.INTx2 = 1;	// PIE 인터럽트(ADCA1INT) 활성화
	IER |= M_INT1;						// CPU 인터럽트(INT1)  활성화
//============================================================================================


//============================================================================================
// Step 4. ADC 초기화
//--------------------------------------------------------------------------------------------

	EALLOW;
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
// Step 6. 변수 초기화
//-------------------------------------------------------------------------------------------- 
	ADC_value01 = 0;
	ADC_value02 = 0;
	Loop_cnt = 0;
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


