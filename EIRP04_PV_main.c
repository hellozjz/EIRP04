/* ****************************************************************************************************
 * File name: main.c		//	for DC/DC converter: Battery and PV converter
 * Written by: Nguyen Xuan Bac
 * Last updated: Aug 10, 2017														  				  *
 *******************************************************************************************************/
#include "DSP28x_Project.h"
#include "IQmathlib.h"
#include "stdio.h"
#include "string.h"
#include "user_define.h"
#include "CanBus.h"

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);
interrupt void  adc_isr(void);
void InitEPwm_Interleaved(void);
void Scan_button(void);
void Turn_off_Converter(void);
void BAT_Send_Data_Canbus(int16 d0, int16 d1, int16 d2, int16 group_data_index);
void BAT_Send_Fault_Data_Canbus(int16 d0_fault, int16 d1_fault, int16 d2_fault, int16 group_data_index);
void PV_Send_Data_Canbus(int16 d0_PV, int16 d1_PV, int16 d2_PV, int16 group_data_index);
void PV_Send_Fault_Data_Canbus(int16 d0_PV_fault, int16 d1_PV_fault, int16 d2_PV_fault, int16 group_data_index);

void BAT_Send_To_BBB(void);
void PV_Send_To_BBB(void);
void Receive_Data_Canbus(void);
void Protection(void);
void Soft_Transition(void);
void BAT_Transition_Mode(void);
void PV1_Transition_Mode(void);
void PV2_Transition_Mode(void);
void PV3_Transition_Mode(void);
void ADC_Calculation(void);
void MPPT_PO(void);
void PI_Controller(void);
void PWM_Modulation(void);
float LowPassFilter(float32 PreOut, float32 Input, float32 CutFre);

//===========================================================================================================
// define constant
#define cycle_time 	100				// interrupt timer 0  100us
#define Ts 			0.0001			// Descretized period
#define ADC_MODCLK 	0x0003 			// HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
#define ADC_CKPS   	0x1   			// ADC module clock = HSPCLK/2*ADC_CKPS   = 25.0MHz/(1*2) = 12.5MHz
#define ADC_SHCLK  	0x6   			// S/H width in ADC module periods                        = 16 ADC clocks
#define deadtime	50
#define	period		375				//	375 <=>100kHz, 750 <=> 50kHz
#define phase_b		250
#define phase_c		250
//====================== ADC Calibration ==================================================================

////B2 => Package for PV conv
//#define offset_a0_pv 	2060//2054
//#define offset_a1_pv 	2054//2052//2060
//#define offset_a2_pv 	2058//2055
//#define offset_a3_pv 	2055//2054//2054
//#define offset_a4_pv 	2051//2049//2053//2051
//#define offset_a5_pv 	2061//2057//2062
//#define offset_b6_pv 	2059//2055
//#define offset_b7_pv 	2055//2052
//
//#define gain_a0_pv		-0.02945 	//0.0333	//Iout		//estimated
//#define gain_a1_pv	    -0.0300		//0.0333	//IL3
//#define gain_a2_pv	    0.2423		//VESS1    //estimated	ok
//#define gain_a3_pv	    0.2428		//0.2428	//0.242	VDC					ok
//#define gain_a4_pv	    -0.02914 	//0.0333	//IL2
//#define gain_a5_pv	    -0.02980 	//0.0300	//IL1		//			ok
//#define gain_b6_pv	    0.2439		//VESS3		//estimated
//#define gain_b7_pv	    0.2439		//VESS2		//estimated
//
//
//
//
////B3	MS student  => Package for Bat conv
//#define offset_a0 2057 //fate
//#define offset_a1 2068 //fate 2060
//#define offset_a2 2061// fate //2055
//#define offset_a3 2063// fate 2054
//#define offset_a4 2055 // fate 2051
//#define offset_a5 2074// fate //2062
//#define offset_b6 2064// fate 2055
//#define offset_b7 2063//fate 2052
//
//#define gain_a0    -0.0146 // 				fate 0.0333 //Iout //estimated
//#define gain_a1    -0.0145 //-0.011			fate 0.0333 //IL3
//#define gain_a2    0.2420 // 				fate 0.2423 //VESS1    //estimated ok
//#define gain_a3    0.2427 //				fate 0.2420 //VDC ok
//#define gain_a4    -0.0145// -0.011			fate 0.0333 //IL2
//#define gain_a5    -0.0147// -0.011			fate 0.0300 //IL1 // ok
//#define gain_b6    0.2430 // 				fate 0.2439 //VESS3 //estimated
//#define gain_b7    0.2429//					fate 0.2439 //VESS2 //estimated





//B2 + Controller: Machine solder 3rd  => inside the cage
//#define offset_a0 2049 //fate
//#define offset_a1 2052 //fate 2060
//#define offset_a2 2056// fate //2055
//#define offset_a3 2053// fate 2054
//#define offset_a4 2059 // fate 2051
//#define offset_a5 2050// fate //2062
//#define offset_b6 2056// fate 2055
//#define offset_b7 2058//fate 2052
//
//#define gain_a0    -0.045 // fate 0.0333 //Iout //estimated
//#define gain_a1    -0.029 //fate 0.0333 //IL3
//#define gain_a2    0.2205 // fate 0.2423 //VESS1    //estimated ok
//#define gain_a3    0.231 //fate 0.2420 //VDC ok
//#define gain_a4    -0.029// fate 0.0333 //IL2
//#define gain_a5    -0.029// fate 0.0300 //IL1 // ok
//#define gain_b6    0.2205 // fate 0.2439 //VESS3 //estimated
//#define gain_b7    0.2205//fate 0.2439 //VESS2 //estimated

//B5	FYP student
					//new //student
//#define offset_a0 	2056 //2055 //2615
//#define offset_a1 	2058 //2055 //2048
//#define offset_a2 	2060 //2056 //2085
//#define offset_a3 	2056 //2054 //2055
//#define offset_a4 	2061 //2060 //2062
//#define offset_a5 	2056 //2055 //2058
//#define offset_b6 	2060 //2059 //2060
//#define offset_b7 	2064 //2148 //2086
//
//#define gain_a0		-0.0517	//Iout		//estimated 0.033
//#define gain_a1	    -0.0349	//IL3       //0.029
//#define gain_a2	    0.2186  //0.2197	//VESS1    //estimated	ok 0.2423
//#define gain_a3	    0.2297  //0.2303	//VDC	0.2420				ok
//#define gain_a4	    -0.00535 //-0.0276	//IL2   0.0333
//#define gain_a5	    -0.0284	//IL1		//			ok 0.03
//#define gain_b6	    0.2195	//VESS3		//estimated 0.2439
//#define gain_b7	    0.2196  //0.2339	//VESS2		//estimated 0.22



// 2nd, 3rd for BAT
// 1st, 4,5,6th for PV

//===========================FIRST SET====================================PV01:OK
////BAT New set: 1st DC/DC 10 Jan 2017
//#define offset_a0 2046 //fate			//
//#define offset_a1 2061 //fate 2060
//#define offset_a2 2057// fate //2055
//#define offset_a3 2053// fate 2054
//#define offset_a4 2044 // fate 2051
//#define offset_a5 2061// fate //2062
//#define offset_b6 2058// fate 2055
//#define offset_b7 2054//fate 2052
//
//#define gain_a0    -0.01467 // fate 0.0333 //Iout //estimated
//#define gain_a1    -0.01527 //fate 0.0333 //IL3
//#define gain_a2    0.2420 // fate 0.2423 //VESS1    //estimated ok
//#define gain_a3    0.2427 //fate 0.2420 //VDC ok
//#define gain_a4    -0.01534// fate 0.0333 //IL2
//#define gain_a5    -0.0158// fate 0.0300 //IL1 // ok
//#define gain_b6    0.2430 // fate 0.2439 //VESS3 //estimated
//#define gain_b7    0.2429//fate 0.2439 //VESS2 //estimated
//
////PV New set: 1st DC/DC 10 Jan 2017
//#define offset_a0_pv 2046 //fate			//
//#define offset_a1_pv 2061 //fate 2060
//#define offset_a2_pv 2057// fate //2055
//#define offset_a3_pv 2053// fate 2054
//#define offset_a4_pv 2044 // fate 2051
//#define offset_a5_pv 2061// fate //2062
//#define offset_b6_pv 2058// fate 2055
//#define offset_b7_pv 2054//fate 2052
//
//#define gain_a0_pv    -0.01467 // fate 0.0333 //Iout //estimated
//#define gain_a1_pv    -0.01527 //fate 0.0333 //IL3
//#define gain_a2_pv    0.2420 // fate 0.2423 //VESS1    //estimated ok
//#define gain_a3_pv    0.2427 //fate 0.2420 //VDC ok
//#define gain_a4_pv    -0.01534// fate 0.0333 //IL2
//#define gain_a5_pv    -0.0158// fate 0.0300 //IL1 // ok
//#define gain_b6_pv    0.2430 // fate 0.2439 //VESS3 //estimated
//#define gain_b7_pv    0.2429//fate 0.2439 //VESS2 //estimated

// L = 1.4mH


//===========================SECOND SET====================================BAT02:OK
////BAT New set: 2nd DC/DC 12 Jan 2017
//#define offset_a0 2059 //fate			//
//#define offset_a1 2055 //fate 2060
//#define offset_a2 2058// fate //2055
//#define offset_a3 2063// fate 2054
//#define offset_a4 2057 // fate 2051
//#define offset_a5 2064// fate //2062
//#define offset_b6 2066// fate 2055
//#define offset_b7 2069//fate 2052
//
//#define gain_a0    -0.01455 // fate 0.0333 //Iout //estimated
//#define gain_a1    -0.01527 //fate 0.0333 //IL3
//#define gain_a2    0.2440 // fate 0.2423 //VESS1    //estimated ok
//#define gain_a3    0.2435 //fate 0.2420 //VDC ok
//#define gain_a4    -0.0155// fate 0.0333 //IL2
//#define gain_a5    -0.0162// fate 0.0300 //IL1 // ok
//#define gain_b6    0.2450 // fate 0.2439 //VESS3 //estimated
//#define gain_b7    0.245//fate 0.2439 //VESS2 //estimated


////===========================THIRD SET====================================BAT01:OK
//BAT New set: 3rd DC/DC 13 Jan 2017
#define offset_a0 2062 //fate			//
#define offset_a1 2060 //fate 2060
#define offset_a2 2052// fate //2055
#define offset_a3 2062// fate 2054
#define offset_a4 2052 // fate 2051
#define offset_a5 2070// fate //2062
#define offset_b6 2056// fate 2055
#define offset_b7 2057//fate 2052

#define gain_a0    -0.01455 // fate 0.0333 //Iout //estimated
#define gain_a1    -0.01459 //fate 0.0333 //IL3
#define gain_a2    0.2430 // fate 0.2423 //VESS1    //estimated ok
#define gain_a3    0.2431 //fate 0.2420 //VDC ok
#define gain_a4    -0.01504// fate 0.0333 //IL2
#define gain_a5    -0.01486// fate 0.0300 //IL1 // ok
#define gain_b6    0.2431 // fate 0.2439 //VESS3 //estimated
#define gain_b7    0.2431//fate 0.2439 //VESS2 //estimated
//

//===========================FORTH SET====================================PV03: MD1.2
//BAT New set: 4th DC/DC 22 Mar 2017
//#define offset_a0 2051 //fate			//
//#define offset_a1 2056 //fate 2060
//#define offset_a2 2050// fate //2055
//#define offset_a3 2052// fate 2054
//#define offset_a4 2043 // fate 2051
//#define offset_a5 2064// fate //2062
//#define offset_b6 2056// fate 2055
//#define offset_b7 2050//fate 2052
//
//#define gain_a0    -0.01455 // fate 0.0333 //Iout //estimated
//#define gain_a1    -0.01459 //fate 0.0333 //IL3
//#define gain_a2    0.2470 // fate 0.2423 //VESS1    //estimated ok
//#define gain_a3    0.2471 //fate 0.2420 //VDC ok
//#define gain_a4    -0.01504// fate 0.0333 //IL2
//#define gain_a5    -0.01486// fate 0.0300 //IL1 // ok
//#define gain_b6    0.2471 // fate 0.2439 //VESS3 //estimated
//#define gain_b7    0.2471//fate 0.2439 //VESS2 //estimated

//PV New set: 4th DC/DC 22 Mar 2017
#define offset_a0_pv 2051 //fate			//
#define offset_a1_pv 2056 //fate 2060
#define offset_a2_pv 2050// fate //2055
#define offset_a3_pv 2052// fate 2054
#define offset_a4_pv 2043 // fate 2051
#define offset_a5_pv 2064// fate //2062
#define offset_b6_pv 2056// fate 2055
#define offset_b7_pv 2050//fate 2052

#define gain_a0_pv    -0.01455 // fate 0.0333 //Iout //estimated
#define gain_a1_pv    -0.01459 //fate 0.0333 //IL3
#define gain_a2_pv    0.2470 // fate 0.2423 //VESS1    //estimated ok
#define gain_a3_pv    0.2471 //fate 0.2420 //VDC ok
#define gain_a4_pv    -0.01504// fate 0.0333 //IL2
#define gain_a5_pv    -0.01486// fate 0.0300 //IL1 // ok
#define gain_b6_pv    0.2471 // fate 0.2439 //VESS3 //estimated
#define gain_b7_pv    0.2471//fate 0.2439 //VESS2 //estimated



//===========================FIFTH SET====================================PV02:OK
////BAT New set: 5th DC/DC 17 Mar 2017
//#define offset_a0 2052 //fate			//
//#define offset_a1 2044 //fate 2060
//#define offset_a2 2062// fate //2055
//#define offset_a3 2062// fate 2054
//#define offset_a4 2070 // fate 2051
//#define offset_a5 2049// fate //2062
//#define offset_b6 2064// fate 2055
//#define offset_b7 2058//fate 2052
//
//#define gain_a0    -0.01455 // fate 0.0333 //Iout //estimated
//#define gain_a1    -0.01459 //fate 0.0333 //IL3
//#define gain_a2    0.2430 // fate 0.2423 //VESS1    //estimated ok
//#define gain_a3    0.2431 //fate 0.2420 //VDC ok
//#define gain_a4    -0.01504// fate 0.0333 //IL2
//#define gain_a5    -0.01486// fate 0.0300 //IL1 // ok
//#define gain_b6    0.2431 // fate 0.2439 //VESS3 //estimated
//#define gain_b7    0.2431//fate 0.2439 //VESS2 //estimated
//
//////PV New set: 5th DC/DC 17 Mar 2017
//#define offset_a0_pv 2052 //fate			//
//#define offset_a1_pv 2044 //fate 2060
//#define offset_a2_pv 2062// fate //2055
//#define offset_a3_pv 2062// fate 2054
//#define offset_a4_pv 2070 // fate 2051
//#define offset_a5_pv 2049// fate //2062
//#define offset_b6_pv 2064// fate 2055
//#define offset_b7_pv 2058//fate 2052
//
//#define gain_a0_pv    -0.01455 // fate 0.0333 //Iout //estimated
//#define gain_a1_pv    -0.01459 //fate 0.0333 //IL3
//#define gain_a2_pv    0.2430 // fate 0.2423 //VESS1    //estimated ok
//#define gain_a3_pv    0.2431 //fate 0.2420 //VDC ok
//#define gain_a4_pv    -0.01504// fate 0.0333 //IL2
//#define gain_a5_pv    -0.01486// fate 0.0300 //IL1 // ok
//#define gain_b6_pv    0.2431 // fate 0.2439 //VESS3 //estimated
//#define gain_b7_pv    0.2431//fate 0.2439 //VESS2 //estimated





//===========================SIXTH SET====================================PV04:OK  MD1.1
////BAT New set: 6th DC/DC 20 Mar 2017
//#define offset_a0 2045 //fate			//
//#define offset_a1 2053 //fate 2060
//#define offset_a2 2045// fate //2055
//#define offset_a3 2060// fate 2054
//#define offset_a4 2042 // fate 2051
//#define offset_a5 2038// fate //2062
//#define offset_b6 2060// fate 2055
//#define offset_b7 2050//fate 2052
//
//#define gain_a0    -0.01455 // fate 0.0333 //Iout //estimated
//#define gain_a1    -0.01459 //fate 0.0333 //IL3
//#define gain_a2    0.2430 // fate 0.2423 //VESS1    //estimated ok
//#define gain_a3    0.2431 //fate 0.2420 //VDC ok
//#define gain_a4    -0.01504// fate 0.0333 //IL2
//#define gain_a5    -0.01486// fate 0.0300 //IL1 // ok
//#define gain_b6    0.2431 // fate 0.2439 //VESS3 //estimated
//#define gain_b7    0.2431//fate 0.2439 //VESS2 //estimated

////PV New set: 6th DC/DC 20 Mar 2017
//#define offset_a0_pv 2045 //fate			//
//#define offset_a1_pv 2053 //fate 2060
//#define offset_a2_pv 2045// fate //2055
//#define offset_a3_pv 2060// fate 2054
//#define offset_a4_pv 2042 // fate 2051
//#define offset_a5_pv 2038// fate //2062
//#define offset_b6_pv 2060// fate 2055
//#define offset_b7_pv 2050//fate 2052
//
//#define gain_a0_pv    -0.01455 // fate 0.0333 //Iout //estimated
//#define gain_a1_pv    -0.01459 //fate 0.0333 //IL3
//#define gain_a2_pv    0.2430 // fate 0.2423 //VESS1    //estimated ok
//#define gain_a3_pv    0.2431 //fate 0.2420 //VDC ok
//#define gain_a4_pv    -0.01504// fate 0.0333 //IL2
//#define gain_a5_pv    -0.01486// fate 0.0300 //IL1 // ok
//#define gain_b6_pv    0.2431 // fate 0.2439 //VESS3 //estimated
//#define gain_b7_pv    0.2431//fate 0.2439 //VESS2 //estimated





#define	epsilon			0.3					//delta_vref
#define MPPT_ADC_COUNT	300

#define vPV_min 		100
#define vPV_max 		260
#define MIN_PV_STARTUP_VOL	150

#define	BAT_CONVERTER 	1
#define	PV_CONVERTER 	2


#define VRM				0
#define	PRM				1
#define	MPPT			2
#define	IDLE			3
#define	ISOLATED		4


//// protection rating
#define		IMMEDIATE_STOP_DC_HI_VOL	550	//	450
#define		OFF_THRESHOLD_OVER_DC_VOL	500	//	420		//BAT shutdown
#define		OFF_THRESHOLD_OVER_DC_VOL1	480	//	420		//BAT shutdown
#define		OFF_THRESHOLD_OVER_DC_VOL2	450	//	420		//BAT shutdown

#define		PV1_THRESHOLD_OVER_DC_VOL	410	//	400		//PV1 shutdown
#define		PV2_THRESHOLD_OVER_DC_VOL	420	//	405		//PV2 shutdown
#define		PV3_THRESHOLD_OVER_DC_VOL	430	//	410		//PV3 shutdown

#define		BAT_THRESHOLD_OVER_DC_VOL	400	//  390			//BAT auto transition
#define		BAT_THRESHOLD_LOW_DC_VOL	360	//	370			//BAT auto transition

#define		OFF_THRESHOLD_LOW_DC_VOL2	340	//	340				//PV and BAT shutdown
#define		OFF_THRESHOLD_LOW_DC_VOL1	320	//	340				//PV and BAT shutdown
#define		OFF_THRESHOLD_LOW_DC_VOL	300	//	340				//PV and BAT shutdown
#define		IMMEDIATE_STOP_DC_LOW_VOL	250	//	320

#define		RETURN_BACK_HIGH_VOL		400	//	390
#define		RETURN_BACK_LOW_VOL			360	//	370


#define i_sat 						12				// maximum input current of inner controller
#define MAX_DUTY					375
#define MIN_DUTY					0
#define MIN_DUTY_MPPT				100


#define	INPUT_PROTECT_COUNT			100				//10ms
#define	INPUT_PROTECT_COUNT1		50				//2ms
#define	INPUT_PROTECT_COUNT2		10				//2ms

#define	BAT_UNSCHEDULED_COUNT		20				//2ms		t1		voltage
#define	PV_PROTECT_COUNT			40				//4ms		t2 // and unscheduled				voltage

#define	TRIP_COUNT					5				//500us		t5		high voltage
#define	TRIP_COUNT1					50				//5ms		t4		high voltage
#define	TRIP_COUNT2					100				//10ms		t3		high voltage

#define	TRIP_COUNT0_LOW					10				//500us		t5		low voltage
#define	TRIP_COUNT_LOW					50				//500us		t5		low voltage
#define	TRIP_COUNT1_LOW					500				//5ms		t4	low	voltage
#define	TRIP_COUNT2_LOW					1000				//10ms		t3	low	voltage


#define OVER_CURRENT_COUNT			1000			//100ms				current
#define	OVER_POWER_COUNT			5000			//500ms				current

//======================================= Variables ===========================================================================

// rated values
#define MIN_INPUT_VOL 				150				//	150
#define MIN_INPUT_VOL1 				130				//	150
#define MIN_INPUT_VOL2 				100				//	150

#define MAX_INPUT_VOL				330				//	330
#define MAX_INPUT_VOL1				350				//	330
#define MAX_INPUT_VOL2				370				//	330

#define MAX_INPUT_CUR				15				//	9kW
#define MAX_OUTPUT_CUR				18.5			//	7kW
#define MAX_POWER_RATING			7  				//



Uint16	converter = BAT_CONVERTER;	// BAT converter
Uint16	PV = 1;

// Controller
//PI parameters
float 	kpv_A = 0.5;
float 	kiv_A = 150;
float 	kpi_A = 1;
float 	kii_A = 100;

float 	kpv_A1 = 0.1;
float 	kiv_A1 = 10;
float 	kpi_A1 = 0.1;
float 	kii_A1 = 10;


float 	kpv_B = 0.5;
float 	kiv_B = 150;
float 	kpi_B = 1;
float 	kii_B = 100;

float 	kpv_B1 = 0.1;
float 	kiv_B1 = 10;
float 	kpi_B1 = 0.1;
float 	kii_B1 = 10;

float 	kpv_C = 0.5;
float 	kiv_C = 150;
float 	kpi_C = 1;
float 	kii_C = 100;

float 	kpv_C1 = 0.1;
float 	kiv_C1 = 10;
float 	kpi_C1 = 0.1;
float 	kii_C1 = 10;

float 	duty_max = period, duty_max_A = period, duty_max_B = period, duty_max_C = period; 						//normal mode
float 	duty_min = 0, duty_min_A = 0, duty_min_B = 0, duty_min_C = 0;
float 	duty_max_reset = 0, duty_max_reset_A = 0, duty_max_reset_B = 0, duty_max_reset_C = 0;
float 	duty_min_set = 0, duty_min_set_A = 0, duty_min_set_B = 0, duty_min_set_C = 0;
float	duty_step_plus = 0.005;
float	duty_step_minus	= 0.005;

float 	vref_A = 0, vref_B = 0, vref_C = 0;									//reference output voltage
//float 	vref_A_pre = 0, vref_B_pre = 0, vref_C_pre = 0;
float 	i_set = 0, i_set_A = 0, i_set_B = 0, i_set_C = 0;

//float 	ei = 0, ei_pre = 0;
float 	ei_A = 0, ei_B = 0, ei_C = 0, ei_A_pre = 0, ei_B_pre = 0, ei_C_pre = 0;
//float 	ev = 0, ev_pre = 0;
float 	ev_A = 0, ev_B = 0, ev_C = 0, ev_A_pre = 0, ev_B_pre = 0, ev_C_pre = 0;

//float 	iref = 0, iref_pre = 0;					// inner controller variables
float 	iref_A = 0, iref_B = 0, iref_C = 0;
float 	iref_A_pre = 0, iref_B_pre = 0, iref_C_pre = 0;						// inner controller variables

//float 	duty = 0, duty_pre = 0;
float 	duty_A = 0, duty_B = 0, duty_C = 0, duty_A_pre = 0, duty_B_pre = 0, duty_C_pre = 0;

float 	duty_HA = 0, duty_LA = period, duty_HB = 0, duty_LB = period, duty_HC = 0, duty_LC = period;

// ADC variables
long 	adc_a0, adc_a1, adc_a2, adc_a3, adc_a4, adc_a5, adc_b6, adc_b7;

float 		iiA = 0, iiA_sum = 0, iiA_ave = 0;												// Dong dien ngo vao
float 		iiB = 0, iiB_sum = 0, iiB_ave = 0;
float 		iiC = 0, iiC_sum = 0, iiC_ave = 0;

float 		viA = 0, viA_sum = 0, viA_ave = 0;												// Dien ap ngo vao
float 		viB = 0, viB_sum = 0, viB_ave = 0;
float 		viC = 0, viC_sum = 0, viC_ave = 0;

float 		vo = 0, vo_sum  = 0, vo_ave  = 0;												// Dien ap ngo ra
float 		io = 0, io_sum  = 0, io_ave  = 0;												// Dong dien ngo ra
float 		i_in_ave = 0;

Uint16  	ConversionCount = 0;
float	 	ConversionCount1;


//MPPT variables


float	 	PowerPV_A = 0, PowerPV_B = 0, PowerPV_C = 0;
float	 	PowerPV_A_pre = 0, PowerPV_B_pre = 0, PowerPV_C_pre = 0;
float		viA_ave_pre = 0, viB_ave_pre = 0, viC_ave_pre = 0;
float		vref_A_pre = 0, vref_B_pre = 0, vref_C_pre = 0;
float		ave_viA = 0, ave_viB = 0, ave_viC = 0;
float		ave_iiA = 0, ave_iiB = 0, ave_iiC = 0;
float		sum_viA = 0, sum_viB = 0, sum_viC = 0;
float		sum_iiA = 0, sum_iiB = 0, sum_iiC = 0;
float		ave_viA_pre = 0, ave_viB_pre = 0, ave_viC_pre = 0;
float		ave_iiA_pre = 0, ave_iiB_pre = 0, ave_iiC_pre = 0;
float		epsilon_A = 0.2, epsilon_B = 0.2, epsilon_C = 0.2;
float		deltaP = 0.1;
Uint16		k = 0, k1 = 0, k2 = 0;
Uint16		update_pv = 0;

// Operating variables
int		mode = ISOLATED, mode_A = ISOLATED, mode_B = ISOLATED, mode_C = ISOLATED;
int		new_mode = ISOLATED, new_mode_A = ISOLATED, new_mode_B = ISOLATED, new_mode_C = ISOLATED;									// command from BBB
Uint16	mode_change = 0, mode_change_counter = 0;
//Uint16	mode_change_pv1 = 0, mode_change_counter_pv1 = 0;
//Uint16	mode_change_pv2 = 0, mode_change_counter_pv2 = 0;
//Uint16	mode_change_pv3 = 0, mode_change_counter_pv3 = 0;
//Uint16	droop_control = 0, droop_control_A = 0, droop_control_B = 0, droop_control_C = 0;
float	reference_vol = 380, reference_vol_A = 380, reference_vol_B = 380, reference_vol_C = 380;
float	reference_vol_new = 380, reference_vol_new_A = 380, reference_vol_new_B = 380, reference_vol_new_C = 380;
float	droop_coef = 0.4, droop_coef_A = 0.4, droop_coef_B = 0.4, droop_coef_C = 0.4;
//Uint16	power_dispatch, power_dispatch_A, power_dispatch_B, power_dispatch_C;
float	reference_power = 0, reference_power_A = 0, reference_power_B = 0, reference_power_C = 0;
float	reference_power_new = 0, reference_power_new_A = 0, reference_power_new_B = 0, reference_power_new_C = 0;
//Uint16	bat_isolate = 1, isolate_A = 1, isolate_B = 1, isolate_C = 1;
//Uint16	mppt_A = 0, mppt_B = 0, mppt_C = 0;

//==================================================================================================================

// Ecan variables
Uint16	LED_BLINK_count = 0;
Uint16	ecan_count = 0;
Uint16 	ecan_receive_done = 0, ecan_request = 0, ecan_send_done = 1, ecan_error = 0;
Uint16  ecan_send_done_PV = 0;
int16	d_0 = 0, d_1 = 0, d_2 = 0;
int16	d_0_PV = 0, d_1_PV = 0, d_2_PV = 0;
Uint16	message_7 = 0, message_16 = 0;
int16	ecan_sending = 0;
int16	ecan_send_counter = 0;
int16	ecan_send_error_flag = 0;

Uint16 	CONVERTER_STATUS = 0;
Uint16	CH1_ONOFF_STATUS = 0, CH2_ONOFF_STATUS = 0, CH3_ONOFF_STATUS = 0;
Uint16	INPUT_RELAY_STATUS = 0, OUTPUT_RELAY_STATUS = 0;
Uint16	PV1_PRIOR = 1, PV2_PRIOR = 1, PV3_PRIOR = 1;

Uint16 	fault_message1 = 0, fault_message2 = 0;

Uint16	unscheduled_BAT_message = 0, unscheduled_PV_message_A = 0, unscheduled_PV_message_B = 0, unscheduled_PV_message_C = 0;
Uint16  unscheduled_power_flag = 0, unscheduled_voltage_flag = 0;
Uint16	exceed_discharging_power_flag = 0, exceed_charging_power_flag = 0;
Uint16	exceed_high_voltage_flag = 0, exceed_low_voltage_flag = 0;
Uint16 	unscheduled_PV1_flag = 0;
Uint16 	unscheduled_PV2_flag = 0;
Uint16 	unscheduled_PV3_flag = 0;
Uint16  pow_unscheduled_indication_message = 0, vol_unscheduled_indication_message = 0;
Uint16	start_up_flag = 0;

Uint16	send_fault_flag = 0;
float 	heatsink_temp = 0;
Uint16	heatsink_temp_int;
int16	iiA_int = 0, iiB_int = 0, iiC_int, io_int = 0;
int16	viA_int = 0, viB_int = 0, viC_int = 0, vo_int = 0;
int16	bat_vol_int = 0, dc_bus_vol_int = 0;

Uint16	ref_vol_confirm_int = 0, ref_vol_confirm_A_int = 0, ref_vol_confirm_B_int = 0, ref_vol_confirm_C_int = 0;
Uint16	droop_coef_confirm_int = 0, droop_coef_confirm_A_int = 0, droop_coef_confirm_B_int = 0, droop_coef_confirm_C_int = 0;
int16	ref_power_confirm_int = 0, ref_power_confirm_A_int = 0, ref_power_confirm_B_int = 0, ref_power_confirm_C_int = 0;

Uint16	reserve = 0;
Uint32	ecan_count_check = 0;

// Protection variables

Uint16 	START_BUTTON = 1;
Uint16 	STOP_BUTTON = 1;
Uint16  STOP_BUTTON1 = 1;
Uint16 	stop = 0, stop_A = 0, stop_B = 0, stop_C = 0;
Uint16 	st = 0;

Uint16 	short_circuit = 0;
Uint16 	short_sw = 0;

Uint16	over_vol_DC = 0, over_vol_DC_counter = 0;					//420
Uint16	PV1_over_vol_DC = 0, PV1_over_vol_DC_counter = 0;				//400
Uint16	PV2_over_vol_DC = 0, PV2_over_vol_DC_counter = 0;				//405
Uint16	PV3_over_vol_DC = 0, PV3_over_vol_DC_counter = 0;				//410


Uint16	BAT_over_vol_DC = 0, BAT_over_vol_DC_counter = 0;			//395
Uint16	BAT_low_vol_DC	= 0, BAT_low_vol_DC_counter  = 0;			//365

Uint16	low_vol_DC_counter0	= 0;
Uint16	low_vol_DC_counter1	= 0;
Uint16	low_vol_DC_counter2	= 0;

Uint16	low_vol_DC	= 0, low_vol_DC_counter	= 0;					//340

Uint16	over_vol_A = 0, over_vol_A_counter = 0;						// input channel A: MAX_INPUT_VOL
Uint16	over_vol_B = 0, over_vol_B_counter = 0;
Uint16	over_vol_C = 0, over_vol_C_counter = 0;

Uint16	low_vol_A = 0, low_vol_A_counter = 0;						// input channel A: MIN_INPUT_VOL
Uint16	low_vol_B = 0, low_vol_B_counter = 0;
Uint16	low_vol_C = 0, low_vol_C_counter = 0;

int16	over_cur_A	= 0, over_cur_A_counter = 0;					// |iiA_ave| > MAX_INPUT_CUR
int16	over_cur_B	= 0, over_cur_B_counter = 0;
int16	over_cur_C	= 0, over_cur_C_counter = 0;
int16	over_cur_DC = 0, over_cur_DC_counter = 0;					// |io_ave| > MAX_OUTPUT_CUR

int16	precharging_counter = 0;
int16	precharging_counter_pv = 0;

// Testing
Uint16 	time2change = 0;
Uint16	test = 0, test_duty = 0, test1 = 0;
float32	testing = 0;

double test2 = 0, sum_test2 = 0;

float32 test_powerPVA = 0, test_viA = 0;
float32 test_powerPVB = 0, test_viB = 0;
float32 test_powerPVC = 0, test_viC = 0;

double SOC, SOC_new, SOC_new1;
float BAT_capacitive = 75; //Ah
double initial_SOC = 50, new_initial_SOC = 50;

extern Uint32 device_id;
extern int variable_index;

// flash setup
// These are defined by the linker (see F28335.cmd)
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;

//********************************************* MAIN Function *******************************************************
main() {

 	InitSysCtrl();// Initialize System Control: PLL, WatchDog, enable Peripheral Clocks

// Specific clock setting for ADC:
	EALLOW;
	SysCtrlRegs.HISPCP.all = ADC_MODCLK;	// HSPCLK = SYSCLKOUT/ADC_MODCLK
	EDIS;

//flash setup
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32) &RamfuncsLoadSize);
	InitFlash();
	InitGpio();								// Initialize I/O port

//Clear all interrupts and initialize PIE vector table
	DINT;
	// Disable CPU interrupts
	InitPieCtrl();// Initialize the PIE control registers to their default state
	IER = 0x0000;							// Disable CPU interrupts
	IFR = 0x0000;							// Clear all CPU interrupt flags
	InitPieVectTable();	// Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).

	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;

	InitCpuTimers();						// Initialize the CPU timer
	InitEPwm_Interleaved();

    EALLOW;  // This is needed to write to EALLOW protected register
    PieVectTable.ADCINT = &adc_isr;																								//===========
    EDIS;    // This is needed to disable write to EALLOW protected registers

    InitAdc();								// Initialize ADC module

// Nap gia tri cho cac timers (don vi us)
	ConfigCpuTimer(&CpuTimer0, 150, cycle_time);

// Start timer 0
	CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
	IER |= M_INT1;									// Enable interrupt timer 0
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1; 				// Enable ADCINT in PIE														//============
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;// Enable TINT0 in the PIE: Group 1 interrupt 7
	EINT;
	// Enable Global interrupt INTM
	ERTM;

	configureEcanB();

	Turn_off_Converter();

//  testing
    converter = BAT_CONVERTER;
//  converter = PV_CONVERTER;

//  PV =	1;
  PV =	2;

// One time message to BBB
	if (converter == BAT_CONVERTER)
	{
	    BAT_Send_Data_Canbus(MAX_INPUT_CUR, MAX_OUTPUT_CUR, BAT_THRESHOLD_OVER_DC_VOL, BAT_MESSAGE_101_INDEX);
	    BAT_Send_Data_Canbus(MIN_INPUT_VOL, OFF_THRESHOLD_OVER_DC_VOL, OFF_THRESHOLD_LOW_DC_VOL, BAT_MESSAGE_102_INDEX);
	}
	else
	{
	    ;
	}



// forever loop
for (;;) {

//	Keep sending message to BBB 10ms once

		if (ecan_send_error_flag == 1)

		{
			configureEcanB();
		}

		if (LED_BLINK_count >= 5000)
			{
			LED_BLINK_count = 0;
			BLINK_LED();
			if (fault_message1 != 0)	BLINK_LED1();
			}


		if (ecan_count >= 10000)
		{
			ecan_count 	= 0;
			if (converter == BAT_CONVERTER)
			{
				BAT_Send_To_BBB();
			}
			else // converter == PV_CONVERTER
			{
				PV_Send_To_BBB();
				ecan_count_check++;
			}
		}   // end if ecan_count


}		// end for
}		// end main function
//*******************************************************************************************************************


// Interrupt Timer 0: Main processing *******************************************************************************
interrupt void cpu_timer0_isr(void) {
//	GpioDataRegs.GPASET.bit.GPIO24 = 1; // check total time for this interrupt routine on LED RUN (GREEN)

	ecan_count++;								// period to send ecan: 100 x 100us = 10ms
	LED_BLINK_count++;
	if (ecan_sending == 1)	ecan_send_counter ++;
	if (mode_change == 1)		mode_change_counter++;

	ADC_Calculation();
	Receive_Data_Canbus();

	//Scan_button();
	Protection();

	testing = LowPassFilter(testing, reference_power, 0.5);

	if (converter == BAT_CONVERTER)
	{
	    if ((stop == 1) || (new_mode == ISOLATED))  Turn_off_Converter();
	}
	else
	{
	    if ((stop_A == 1) && (stop_B == 1) && (stop_C == 1))
	    {
	    	stop = 1;
	    	Turn_off_Converter();
	    }
	    if ((new_mode_A == ISOLATED) || (new_mode_B == ISOLATED) || (new_mode_C == ISOLATED))  Turn_off_Converter();
	}

	Soft_Transition();
	if (converter == PV_CONVERTER)
	{
		MPPT_PO();
	}
	PI_Controller();
	PWM_Modulation();

/* Update duty cycle */
	EPwm1Regs.CMPA.half.CMPA 	= duty_HA; 			//
	EPwm1Regs.CMPB 				= duty_LA;			//
	EPwm4Regs.CMPA.half.CMPA 	= duty_HB;
	EPwm4Regs.CMPB 				= duty_LB;
	EPwm5Regs.CMPA.half.CMPA 	= duty_HC;
	EPwm5Regs.CMPB 				= duty_LC;
	PieCtrlRegs.PIEACK.all 		= PIEACK_GROUP1;

//	GpioDataRegs.GPACLEAR.bit.GPIO24 = 1; 			// check total time for this interrupt routine
}
// End Timer0 Interrupt *********************************************************************************************

//===================================================================================================================
void InitEPwm_Interleaved() {
	//=====================================================================
	// Config
	// Initialization Time
	//========================
	// EPWM Module 1 config
	EPwm1Regs.TBPRD = period; // Period = 1500 TBCLK counts = 1500*(1/150Mhz) = 10us => 100kHz
	EPwm1Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Master module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM1A
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;

//  Upper Mosfet is independent to Lower Mosfet (can set one mosfet to be permanently off): more safe, high loss
	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;	// set actions for EPWM1B
	EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;

	/*
	 //  Deadtime mode (both Upper and Lower Mosfets are switched)  >> reduce loss
	 EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	 EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC; // Active Lo complementary
	 EPwm1Regs.DBFED = deadtime; // FED = 75 TBCLKs = 75/150Mhz = 0.5us
	 EPwm1Regs.DBRED = deadtime;
	 */

// EPWM Module 2 config
	EPwm4Regs.TBPRD = period; // Period = 1500 TBCLK counts
	EPwm4Regs.TBPHS.half.TBPHS = phase_b; // Phase = 500/1500 * 360 = 120 deg
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN; // Count DOWN on sync (=120 deg)
	EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm4Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM2A
	EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;

	EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;	// set actions for EPWM1B
	EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;
	//	EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	//	EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC; // Active Hi complementary
	//	EPwm4Regs.DBFED = deadtime; // FED = 75 TBCLKs = 75/150Mhz = 0.5us
	//	EPwm4Regs.DBRED = deadtime;

	// EPWM Module 3 config
	EPwm5Regs.TBPRD = period; // Period = 900 TBCLK counts
	EPwm5Regs.TBPHS.half.TBPHS = phase_c; // Phase = 500/1500 * 360 = 120 deg
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	EPwm5Regs.TBCTL.bit.PHSDIR = TB_UP; // Count UP on sync (=240 deg)
	EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm5Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM3Ai
	EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;

	EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;	// set actions for EPWM1B
	EPwm5Regs.AQCTLB.bit.CBD = AQ_SET;
	//	EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	//	EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC; // Active Hi complementary
	//	EPwm5Regs.DBFED = deadtime; // FED = 75 TBCLKs = 75/150Mhz = 0.5us
	//	EPwm5Regs.DBRED = deadtime;

}
//===================================================================================================================

//===================================================================================================================
void Scan_button() {
	START_BUTTON = GpioDataRegs.GPBDAT.bit.GPIO61;// Scan Start button, press => START_BUTTON = 0  => Precharge
	STOP_BUTTON = GpioDataRegs.GPBDAT.bit.GPIO60;	// Scan Stop button
	STOP_BUTTON1 = GpioDataRegs.GPADAT.bit.GPIO25;
		if ((START_BUTTON == 0) && (STOP_BUTTON == 1) && (STOP_BUTTON1 == 1))
		{
			stop = 0;
			stop_A = 0;
			stop_B = 0;
			stop_C = 0;
			fault_message1 = 0;
//			fault_message1 = 0;
		}
		else if ((STOP_BUTTON == 0) || (STOP_BUTTON1 == 0))
			{
			stop_A = 1;
			stop_B = 1;
			stop_C = 1;
			stop = 1;
			Turn_off_Converter();
			}
}
//===================================================================================================================

//===================================================================================================================
void Turn_off_Converter() {
	DISABLE_ALL_PWM();
	OPEN_RELAY_DC();
	OPEN_RELAY_BAT();
	CONVERTER_STATUS = 0;

	INPUT_RELAY_STATUS = 0;
	OUTPUT_RELAY_STATUS = 0;

	LED_RUN_OFF();
	LED_STOP_ON();

	LED_RUN_OUTSIDE_OFF();
	LED_STOP_OUTSIDE_ON();
//	LED_STOP_OUTSIDE_TOGGLE();
	if (fault_message1 == 0)	LED_IDLE_OUTSIDE_OFF();

//	BLINK_LED();

	CH1_ONOFF_STATUS = 0;
	CH2_ONOFF_STATUS = 0;
	CH3_ONOFF_STATUS = 0;
	CONVERTER_STATUS = 0;

	duty_max_reset = 0;
	duty_min_set = 0;
	duty_max = MAX_DUTY;
	duty_min = MIN_DUTY;
	mode = ISOLATED;


	short_sw = 0;
	over_vol_DC_counter = 0;
	PV1_over_vol_DC_counter = 0;
	PV2_over_vol_DC_counter = 0;
	PV3_over_vol_DC_counter = 0;

	PV1_over_vol_DC = 0;
	PV2_over_vol_DC = 0;
	PV3_over_vol_DC = 0;

	BAT_over_vol_DC_counter = 0;
	BAT_low_vol_DC_counter  = 0;
	low_vol_DC_counter	= 0;

	over_vol_A_counter = 0;
	over_vol_B_counter = 0;
	over_vol_C_counter = 0;

	low_vol_A_counter = 0;
	low_vol_B_counter = 0;
	low_vol_C_counter = 0;

	over_cur_A_counter = 0;
	over_cur_B_counter = 0;
	over_cur_C_counter = 0;
	over_cur_DC_counter = 0;

	mode   = ISOLATED;
	mode_A = ISOLATED;
	mode_B = ISOLATED;
	mode_C = ISOLATED;

	sum_viA = 0;
	sum_iiA = 0;
	sum_viB = 0;
	sum_iiB = 0;
	sum_viC = 0;
	sum_iiC = 0;
	k = 0;

}
//===================================================================================================================
//send start
void BAT_Send_Data_Canbus(int16 d0, int16 d1, int16 d2, int16 group_data_index)
{
	struct CAN_DATA can_data_to_send_1;
	ecan_send_done = 0;
	can_data_to_send_1.data0 = d0;
	can_data_to_send_1.data1 = d1;
	can_data_to_send_1.data2 = d2;

	can_data_to_send_1.index = group_data_index;

	send_data(BAT_ID_INDEX, can_data_to_send_1);

	ecan_send_done = 1;
}
//send end
//===================================================================================================================


//===================================================================================================================
//send start
void BAT_Send_Fault_Data_Canbus(int16 d0_fault, int16 d1_fault, int16 d2_fault, int16 group_data_index)
{
	struct CAN_DATA can_data_to_send_1;
	ecan_send_done = 0;
	can_data_to_send_1.data0 = d0_fault;
	can_data_to_send_1.data1 = d1_fault;
	can_data_to_send_1.data2 = d2_fault;

	can_data_to_send_1.index = group_data_index;

	send_data(BAT_FAULT_ID_INDEX, can_data_to_send_1);

	ecan_send_done = 1;
}
//send end
//===================================================================================================================


//===================================================================================================================
//send start
void PV_Send_Data_Canbus(int16 d0_PV, int16 d1_PV, int16 d2_PV, int16 group_data_index)
{
	struct CAN_DATA can_data_to_send_1;
	ecan_send_done_PV = 0;
	can_data_to_send_1.data0 = d0_PV;
	can_data_to_send_1.data1 = d1_PV;
	can_data_to_send_1.data2 = d2_PV;

	can_data_to_send_1.index = group_data_index;

	if (PV == 1)	send_data(PV_ID_INDEX, can_data_to_send_1);
	if (PV == 2)	send_data(WIN_ID_INDEX, can_data_to_send_1);
	ecan_send_done_PV = 1;
}
//send end
//===================================================================================================================


//===================================================================================================================
//send start
void PV_Send_Fault_Data_Canbus(int16 d0_PV_fault, int16 d1_PV_fault, int16 d2_PV_fault, int16 group_data_index)
{
	struct CAN_DATA can_data_to_send_1;
	ecan_send_done_PV = 0;
	can_data_to_send_1.data0 = d0_PV_fault;
	can_data_to_send_1.data1 = d1_PV_fault;
	can_data_to_send_1.data2 = d2_PV_fault;

	can_data_to_send_1.index = group_data_index;

	if (PV == 1)	send_data(PV_FAULT_ID_INDEX, can_data_to_send_1);
	if (PV == 2)	send_data(WIN_FAULT_ID_INDEX, can_data_to_send_1);
	ecan_send_done_PV = 1;
}
//send end
//===================================================================================================================


//===================================================================================================================
// Periodical data BAT send to BBB
void BAT_Send_To_BBB(){

//Group Message 1
			iiA_int		= (int16) (iiA_ave*10);
			iiB_int		= (int16) (iiB_ave*10);
			iiC_int		= (int16) (iiC_ave*10);
			BAT_Send_Data_Canbus(iiA_int, iiB_int, iiC_int, BAT_MESSAGE_1_INDEX);
//Group Message 2
			io_int					= (int16) (io_ave*10);
			ref_vol_confirm_int		= (Uint16) (reference_vol*10);
			droop_coef_confirm_int	= (Uint16) (droop_coef*10);
			BAT_Send_Data_Canbus(io_int, ref_vol_confirm_int, droop_coef_confirm_int, BAT_MESSAGE_2_INDEX);
//Group Message 3
// form the data
			if (CH1_ONOFF_STATUS != 0)		message_7 |= 0x0001;
			else 							message_7 &= 0xFFFE;
			if (CH2_ONOFF_STATUS != 0)		message_7 |= 0x0002;
			else 							message_7 &= 0xFFFD;
			if (CH3_ONOFF_STATUS != 0)		message_7 |= 0x0004;
			else 							message_7 &= 0xFFFB;
			if (INPUT_RELAY_STATUS != 0)	message_7 |= 0x0008;
			else 							message_7 &= 0xFFF7;
			if (OUTPUT_RELAY_STATUS != 0)	message_7 |= 0x0010;
			else 							message_7 &= 0xFFEF;

			switch (mode)
			{
			case VRM:
			{
				message_7 &= 0b1111111000111111;
				break;
			}
			case PRM:
			{
				message_7 |= 0b0000000001000000;
				message_7 &= 0b1111111001111111;
				break;
			}
			case MPPT:
			{
				message_7 |= 0b0000000010000000;
				message_7 &= 0b1111111010111111;
				break;
			}
			case IDLE:
			{
				message_7 |= 0b0000000011000000;
                message_7 &= 0b1111111011111111;
				break;
			}
			case ISOLATED:
			{
                message_7 |= 0b0000000100000000;
                message_7 &= 0b1111111100111111;

                break;
			}
			}

			heatsink_temp_int = (Uint16) (heatsink_temp*10);
			SOC_new1 = SOC_new*10;
			BAT_Send_Data_Canbus(message_7, SOC_new1, heatsink_temp_int, BAT_MESSAGE_3_INDEX);

//Group Message 4
//			ref_power_confirm			= reference_power;
			ref_power_confirm_int		= (int16) (reference_power*10);
			bat_vol_int					= (int16) (viA_ave*10);
			dc_bus_vol_int				= (int16) (vo_ave*10);
			BAT_Send_Data_Canbus(ref_power_confirm_int, bat_vol_int, dc_bus_vol_int, BAT_MESSAGE_4_INDEX);
}
// End send
//===================================================================================================================

//===================================================================================================================
// Periodical data PV send to BBB
void PV_Send_To_BBB(){
//Group Message 1
			iiA_int		= (int16) (iiA_ave*10);
			iiB_int		= (int16) (iiB_ave*10);
			iiC_int		= (int16) (iiC_ave*10);
			PV_Send_Data_Canbus(iiA_int, iiB_int, iiC_int, PV_MESSAGE_1_INDEX);

//Group Message 2
			viA_int		= (int16) (viA_ave*10);
			viB_int		= (int16) (viB_ave*10);
			viC_int		= (int16) (viC_ave*10);
			PV_Send_Data_Canbus(viA_int, viB_int, viC_int, PV_MESSAGE_2_INDEX);

//Group Message 3
			ref_vol_confirm_A_int		= (int16) (reference_vol_A*10);
			ref_power_confirm_A_int		= (int16) (reference_power_A*10);
			droop_coef_confirm_A_int	= (Uint16) (droop_coef_A*10);
			if (mode_A == VRM)			ref_power_confirm_A_int = droop_coef_confirm_A_int;
			PV_Send_Data_Canbus(mode_A, ref_vol_confirm_A_int, ref_power_confirm_A_int, PV_MESSAGE_3_INDEX);

//Group Message 4
			ref_vol_confirm_B_int		= (int16) (reference_vol_B*10);
			ref_power_confirm_B_int		= (int16) (reference_power_B*10);
			droop_coef_confirm_B_int	= (Uint16) (droop_coef_B*10);
			if (mode_B == VRM)			ref_power_confirm_B_int = droop_coef_confirm_B_int;
			PV_Send_Data_Canbus(mode_B, ref_vol_confirm_B_int, ref_power_confirm_B_int, PV_MESSAGE_4_INDEX);

//Group Message 5
			ref_vol_confirm_C_int		= (int16) (reference_vol_C*10);
			ref_power_confirm_C_int		= (int16) (reference_power_C*10);
			droop_coef_confirm_C_int	= (Uint16) (droop_coef_C*10);
			if (mode_C == VRM)			ref_power_confirm_C_int = droop_coef_confirm_C_int;
			PV_Send_Data_Canbus(mode_C, ref_vol_confirm_C_int, ref_power_confirm_C_int, PV_MESSAGE_5_INDEX);


//Group Message 6
// form the data
			if (CH1_ONOFF_STATUS != 0)		message_16 |= 0b0000000000000001;
			else 							message_16 &= 0b1111111111111110;
			if (CH2_ONOFF_STATUS != 0)		message_16 |= 0b0000000000000010;
			else 							message_16 &= 0b1111111111111101;
			if (CH3_ONOFF_STATUS != 0)		message_16 |= 0b0000000000000100;
			else 							message_16 &= 0b1111111111111011;
			if (INPUT_RELAY_STATUS != 0)	message_16 |= 0b0000000000001000;
			else 							message_16 &= 0b1111111111110111;
			if (OUTPUT_RELAY_STATUS != 0)	message_16 |= 0b0000000000010000;
			else 							message_16 &= 0b1111111111101111;

		switch (PV1_PRIOR)
		{
			case 1:
			{
				message_16 |= 0b0000000000100000;
				message_16 &= 0b1111111110111111;
				break;
			}
			case 2:
			{
				message_16 |= 0b0000000001000000;
				message_16 &= 0b1111111111011111;
				break;
			}
			case 3:
			{
				message_16 |= 0b0000000001100000;
				break;
			}
		}

		switch (PV2_PRIOR)
		{
			case 1:
			{
				message_16 |= 0b0000000010000000;
				message_16 &= 0b1111111011111111;
				break;
			}
			case 2:
			{
				message_16 |= 0b0000000100000000;
				message_16 &= 0b1111111101111111;
				break;
			}
			case 3:
			{
				message_16 |= 0b0000000110000000;
				break;
			}
		}

		switch (PV3_PRIOR)
		{
			case 1:
			{
				message_16 |= 0b0000001000000000;
				message_16 &= 0b1111101111111111;
				break;
			}
			case 2:
			{
				message_16 |= 0b0000010000000000;
				message_16 &= 0b1111110111111111;
				break;
			}
			case 3:
			{
				message_16 |= 0b0000011000000000;
				break;
			}
		}

			io_int = (int16) (io_ave*10);
			vo_int = (int16) (vo_ave*10);
			PV_Send_Data_Canbus(message_16, io_int, vo_int, PV_MESSAGE_6_INDEX);

//Group Message 7
			heatsink_temp_int				= (int16) (heatsink_temp*10);
			PV_Send_Data_Canbus(heatsink_temp_int, reserve, reserve, PV_MESSAGE_7_INDEX);

//Group Message 201
			PV_Send_Data_Canbus(fault_message1, fault_message2, reserve, PV_MESSAGE_201_INDEX);
}
// End send
//===================================================================================================================

//===================================================================================================================
//start receive
void Receive_Data_Canbus()
{
	if (new_data)  					// check if new data come, receive new_mode
	{
		ecan_receive_done = 0;
		if (can_data.id == BAT_ID)		//device_id
		{
			switch (can_data.index)										//variable_index
			{

			case BBB_MESSAGE_1_INDEX:									//droop control
			{
				ecan_send_error_flag = 0;
				//ecan_send_counter = 0;
				fault_message1 = 0;										//reset fault message
				unscheduled_BAT_message = 0;
				unscheduled_power_flag = 0;
				unscheduled_voltage_flag = 0;
				exceed_discharging_power_flag = 0;
				exceed_charging_power_flag = 0;
				exceed_high_voltage_flag = 0;
				exceed_low_voltage_flag = 0;


				new_mode = can_data.data0;

                reference_vol_new = (float) (can_data.data1);
                reference_vol_new = reference_vol_new * 0.1;

                if (new_mode == VRM)
                {
                    droop_coef = (float) (can_data.data2);
                    droop_coef = 0.4;//droop_coef * 0.1;
                }
                else if (new_mode == PRM)
                {
                    reference_power_new = (float)(can_data.data2);
                    reference_power_new = reference_power_new*0.1;
                }
                else
                {
                    reference_power_new = 0;
                }

                break;
			}

			case BBB_MESSAGE_2_INDEX:
			{
				ecan_send_error_flag = 0;
				fault_message1 = 0;										//reset fault message
				//ecan_send_counter = 0;
//				fault_message1 = 0;										//reset fault message


                if (can_data.data0 == 1)
                {
                    new_mode = ISOLATED;
                }
                else if (can_data.data0 == 0)
                {
                    new_mode = IDLE;
                    stop = 0;
                }
					break;
			}

			case BBB_MESSAGE_100_INDEX:		//one time message
			{

					break;
			}

						case BBB_MESSAGE_103_INDEX:		//one time message
						{
								new_initial_SOC = (float) (can_data.data0);
								new_initial_SOC = new_initial_SOC;
								BAT_capacitive  = (float) (can_data.data1);
								break;
						}

			}
		}	//End if BAT_ID

//===============================================================================
	if (PV == 1)
	{
		if (can_data.id == PV_ID)		//device_id
		{
			switch (can_data.index)
			{
			    case BBB_MESSAGE_1_INDEX:
			    {
					ecan_send_error_flag = 0;
					//ecan_send_counter = 0;
					fault_message1 = 0;										//reset fault message
					unscheduled_PV_message_A = 0;
					unscheduled_PV_message_B = 0;
					unscheduled_PV_message_C = 0;

					new_mode_A = can_data.data0;

					reference_vol_new_A = (float) (can_data.data1);
					reference_vol_new_A = reference_vol_new_A * 0.1;

					if (new_mode_A == VRM)
					{
						droop_coef_A = (float) (can_data.data2);
						droop_coef_A = droop_coef_A * 0.1;
					}
					else if (new_mode_A == PRM)
					{
						reference_power_new_A = (float)(can_data.data2);
						reference_power_new_A = reference_power_new_A*0.1;
					}
	                else
	                {
	                    reference_power_new_A = 0;
	                }

					if ((mode_A == ISOLATED) && (new_mode_A != ISOLATED))
					{
						new_mode_B = IDLE;
						new_mode_C = IDLE;
					}
					break;
			    }

			    case BBB_MESSAGE_2_INDEX:
			    {
					ecan_send_error_flag = 0;
					fault_message1 = 0;										//reset fault message
					//ecan_send_counter = 0;
					fault_message1 = 0;										//reset fault message


                    new_mode_B = can_data.data0;

                    reference_vol_new_B = (float) (can_data.data1);
                    reference_vol_new_B = reference_vol_new_B * 0.1;

                    if (new_mode_B == VRM)
                    {
                        droop_coef_B = (float) (can_data.data2);
                        droop_coef_B = droop_coef_B * 0.1;
                    }
                    else if (new_mode_B == PRM)
                    {
                        reference_power_new_B = (float)(can_data.data2);
                        reference_power_new_B = reference_power_new_B*0.1;
                    }
                    else
                    {
                        reference_power_new_B = 0;
                    }

					if ((mode_B == ISOLATED) && (new_mode_B != ISOLATED))
					{
						new_mode_A = IDLE;
						new_mode_C = IDLE;
					}
                    break;
				}

			    case BBB_MESSAGE_3_INDEX:
			    {
					ecan_send_error_flag = 0;
					fault_message1 = 0;										//reset fault message
					//ecan_send_counter = 0;
					fault_message1 = 0;										//reset fault message


                    new_mode_C = can_data.data0;

                    reference_vol_new_C = (float) (can_data.data1);
                    reference_vol_new_C = reference_vol_new_C * 0.1;

                    if (new_mode_C == VRM)
                    {
                        droop_coef_C = (float) (can_data.data2);
                        droop_coef_C = droop_coef_C * 0.1;
                    }
                    else if (new_mode_C == PRM)
                    {
                        reference_power_new_C = (float)(can_data.data2);
                        reference_power_new_C = reference_power_new_C*0.1;
                    }
                    else
                    {
                        reference_power_new_C = 0;
                    }

					if ((mode_C == ISOLATED) && (new_mode_C != ISOLATED))
					{
						new_mode_A = IDLE;
						new_mode_B = IDLE;
					}
                    break;
				}

			    case BBB_MESSAGE_4_INDEX:
			    {
					ecan_send_error_flag = 0;
					fault_message1 = 0;										//reset fault message
					//ecan_send_counter = 0;
					fault_message1 = 0;										//reset fault message


			        if (can_data.data0 == 1)
			        {
			            new_mode_A = ISOLATED;
			            new_mode_B = ISOLATED;
			            new_mode_C = ISOLATED;
			        }
			        else if (can_data.data0 == 0)
                    {
                        new_mode_A = IDLE;
                        new_mode_B = IDLE;
                        new_mode_C = IDLE;
                        stop = 0;
                        stop_A = 0;
                        stop_B = 0;
                        stop_C = 0;
                    }

			        break;
			    }


			    case BBB_MESSAGE_102_INDEX:
			    {
			        break;
			    }

                case BBB_MESSAGE_103_INDEX:
                {
                    break;
                }

			}
		}	//End if PV_ID
	}
//=========================================================================================
	if (PV == 2)
	{
		if (can_data.id == WIN_ID)		//device_id
		{
			switch (can_data.index)
			{
			    case BBB_MESSAGE_1_INDEX:
			    {
					ecan_send_error_flag = 0;
					//ecan_send_counter = 0;
					fault_message1 = 0;										//reset fault message
					unscheduled_PV_message_A = 0;
					unscheduled_PV_message_B = 0;
					unscheduled_PV_message_C = 0;

					new_mode_A = can_data.data0;

					reference_vol_new_A = (float) (can_data.data1);
					reference_vol_new_A = reference_vol_new_A * 0.1;

					if (new_mode_A == VRM)
					{
						droop_coef_A = (float) (can_data.data2);
						droop_coef_A = droop_coef_A * 0.1;
					}
					else if (new_mode_A == PRM)
					{
						reference_power_new_A = (float)(can_data.data2);
						reference_power_new_A = reference_power_new_A*0.1;
					}
	                else
	                {
	                    reference_power_new_A = 0;
	                }

					if ((mode_A == ISOLATED) && (new_mode_A != ISOLATED))
					{
						new_mode_B = IDLE;
						new_mode_C = IDLE;
					}
					break;
			    }

			    case BBB_MESSAGE_2_INDEX:
			    {
					ecan_send_error_flag = 0;
					fault_message1 = 0;										//reset fault message
					//ecan_send_counter = 0;
					fault_message1 = 0;										//reset fault message


                    new_mode_B = can_data.data0;

                    reference_vol_new_B = (float) (can_data.data1);
                    reference_vol_new_B = reference_vol_new_B * 0.1;

                    if (new_mode_B == VRM)
                    {
                        droop_coef_B = (float) (can_data.data2);
                        droop_coef_B = droop_coef_B * 0.1;
                    }
                    else if (new_mode_B == PRM)
                    {
                        reference_power_new_B = (float)(can_data.data2);
                        reference_power_new_B = reference_power_new_B*0.1;
                    }
                    else
                    {
                        reference_power_new_B = 0;
                    }

					if ((mode_B == ISOLATED) && (new_mode_B != ISOLATED))
					{
						new_mode_A = IDLE;
						new_mode_C = IDLE;
					}
                    break;
				}

			    case BBB_MESSAGE_3_INDEX:
			    {
					ecan_send_error_flag = 0;
					fault_message1 = 0;										//reset fault message
					//ecan_send_counter = 0;
					fault_message1 = 0;										//reset fault message


                    new_mode_C = can_data.data0;

                    reference_vol_new_C = (float) (can_data.data1);
                    reference_vol_new_C = reference_vol_new_C * 0.1;

                    if (new_mode_C == VRM)
                    {
                        droop_coef_C = (float) (can_data.data2);
                        droop_coef_C = droop_coef_C * 0.1;
                    }
                    else if (new_mode_C == PRM)
                    {
                        reference_power_new_C = (float)(can_data.data2);
                        reference_power_new_C = reference_power_new_C*0.1;
                    }
                    else
                    {
                        reference_power_new_C = 0;
                    }

					if ((mode_C == ISOLATED) && (new_mode_C != ISOLATED))
					{
						new_mode_A = IDLE;
						new_mode_B = IDLE;
					}
                    break;
				}

			    case BBB_MESSAGE_4_INDEX:
			    {
					ecan_send_error_flag = 0;
					fault_message1 = 0;										//reset fault message
					//ecan_send_counter = 0;
					fault_message1 = 0;										//reset fault message


			        if (can_data.data0 == 1)
			        {
			            new_mode_A = ISOLATED;
			            new_mode_B = ISOLATED;
			            new_mode_C = ISOLATED;
			        }
			        else if (can_data.data0 == 0)
                    {
                        new_mode_A = IDLE;
                        new_mode_B = IDLE;
                        new_mode_C = IDLE;
                        stop = 0;
                        stop_A = 0;
                        stop_B = 0;
                        stop_C = 0;
                    }

			        break;
			    }


			    case BBB_MESSAGE_102_INDEX:
			    {
			        break;
			    }

                case BBB_MESSAGE_103_INDEX:
                {
                    break;
                }

			}
		}	//End if WIN_ID
	}

//===============================================================================
		new_data = FALSE;
	}	// End if new_data

	ecan_receive_done = 1;
}
//end receive
//===================================================================================================================

//===================================================================================================================
void Protection() {

//=====================================================
// 0.Short circuit Protection							//=
	short_circuit = GpioDataRegs.GPADAT.bit.GPIO16;	//=
	if (short_circuit == 1)							//=
	{												//=
		short_sw++;									//=
		if (short_sw >= TRIP_COUNT)					//=
		{
			fault_message1 |= 0b0000000000000001;	//=
			stop = 1;
			Turn_off_Converter();					//=
		}
	}												//=
	else
		{
			short_sw = 0;							//=
		//	fault_message1 &= 0b1111111111111110;	//=
		}
//=====================================================

// 1. Loss CAN Communication
	if (ecan_send_error_flag == 1)	fault_message1 |= 0b0000000000000010;	//=
//=====================================================

// 2. DC Bus Extremely high voltage (>550V): Immediate stop
	if (vo_ave > IMMEDIATE_STOP_DC_HI_VOL)
	{
		stop = 1;
		Turn_off_Converter();
		fault_message1 |= 0b0000000000000100;	// DC bus EXTREMELY HIGH voltage
		over_vol_DC = 1;
	}
//===============================================================================

// 2'. DC Bus Extremely high voltage (>500V): Detect multiple times
	if (vo_ave > OFF_THRESHOLD_OVER_DC_VOL)
	{
		over_vol_DC_counter++;
		if(over_vol_DC_counter >= TRIP_COUNT)		//500us
		{
			stop = 1;
			Turn_off_Converter();
			fault_message1 |= 0b0000000000000100;	// DC bus EXTREMELY HIGH voltage
			over_vol_DC = 1;
			// Send_data_CanBus();
		}
	}
	else
	{
		over_vol_DC_counter = 0;
		over_vol_DC = 0;
	}

//===============================================================================

// 2''. DC Bus Extremely high voltage (>480V): Detect multiple times
	if ((vo_ave > OFF_THRESHOLD_OVER_DC_VOL1) && (vo_ave < OFF_THRESHOLD_OVER_DC_VOL))
	{
		over_vol_DC_counter++;
		if(over_vol_DC_counter >= TRIP_COUNT1)		//500us
		{
			stop = 1;
			Turn_off_Converter();
			fault_message1 |= 0b0000000000000100;	// DC bus EXTREMELY HIGH voltage
			over_vol_DC = 1;
			// Send_data_CanBus();
		}
	}
	else
	{
		over_vol_DC_counter = 0;
		over_vol_DC = 0;
	}

//===============================================================================

// 2'''. DC Bus Extremely high voltage (>450V): Detect multiple times
	if ((vo_ave > OFF_THRESHOLD_OVER_DC_VOL2) && (vo_ave < OFF_THRESHOLD_OVER_DC_VOL1))
	{
		over_vol_DC_counter++;
		if(over_vol_DC_counter >= TRIP_COUNT2)		//500us
		{
			stop = 1;
			Turn_off_Converter();
			fault_message1 |= 0b0000000000000100;	// DC bus EXTREMELY HIGH voltage
			over_vol_DC = 1;
			// Send_data_CanBus();
		}
	}
	else
	{
		over_vol_DC_counter = 0;
		over_vol_DC = 0;
	}

//===============================================================================


if (((stop == 0) && (converter == BAT_CONVERTER)) || (converter == PV_CONVERTER))
{
// 3.  DC Bus Extremely low voltage (<250V): Immediate stop
	if (converter == BAT_CONVERTER)
	{
		if ((vo_ave < IMMEDIATE_STOP_DC_LOW_VOL) && (start_up_flag == 0) && (CONVERTER_STATUS == 1))
		{
			stop = 1;
			Turn_off_Converter();
			fault_message1 |= 0b0000000000001000;	// DC bus EXTREMELY HIGH voltage
			low_vol_DC = 1;
		}
	}
	else //pv
	{
		if ((vo_ave < IMMEDIATE_STOP_DC_LOW_VOL) && (CONVERTER_STATUS == 1))
		{
			low_vol_DC_counter0++;
			if(low_vol_DC_counter0 >= TRIP_COUNT0_LOW)
			{
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000000001000;	// DC bus EXTREMELY HIGH voltage
				low_vol_DC = 1;
			}
		}
		else
		{
			low_vol_DC_counter0 = 0;
			low_vol_DC	= 0;
		}

	}

// 3'. DC Bus Extremely low voltage (<300V): Detect multiple times
//===============================================================================
	if (converter == BAT_CONVERTER)
	{
		if ((vo_ave < OFF_THRESHOLD_LOW_DC_VOL) && (start_up_flag == 0) && (CONVERTER_STATUS == 1))
		{
			low_vol_DC_counter++;
			if(low_vol_DC_counter >= TRIP_COUNT_LOW)
			{
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000000001000;
				low_vol_DC = 1;
				// Send_data_CanBus();
			}
		}
		else
		{
			low_vol_DC_counter = 0;
			low_vol_DC	= 0;
		}
	}
	else
	{
		if ((vo_ave < OFF_THRESHOLD_LOW_DC_VOL) && (CONVERTER_STATUS == 1))
		{
			low_vol_DC_counter++;
			if(low_vol_DC_counter >= TRIP_COUNT_LOW)
			{
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000000001000;
				low_vol_DC = 1;
				// Send_data_CanBus();
			}
		}
		else
		{
			low_vol_DC_counter = 0;
			low_vol_DC	= 0;
		}

	}

// 3''. DC Bus Extremely low voltage (<320V): Detect multiple times
//===============================================================================
	if (converter == BAT_CONVERTER)
	{
		if ((vo_ave > OFF_THRESHOLD_LOW_DC_VOL) && (vo_ave < OFF_THRESHOLD_LOW_DC_VOL1) && (start_up_flag == 0) && (CONVERTER_STATUS == 1))
		{
			low_vol_DC_counter1++;
			if(low_vol_DC_counter1 >= TRIP_COUNT1_LOW)
			{
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000000001000;
				low_vol_DC = 1;
				// Send_data_CanBus();
			}
		}
		else
		{
			low_vol_DC_counter1 = 0;
			low_vol_DC	= 0;
		}
	}
	else
	{
		if ((vo_ave > OFF_THRESHOLD_LOW_DC_VOL) && (vo_ave < OFF_THRESHOLD_LOW_DC_VOL1) && (CONVERTER_STATUS == 1))
		{
			low_vol_DC_counter1++;
			if(low_vol_DC_counter1 >= TRIP_COUNT1_LOW)
			{
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000000001000;
				low_vol_DC = 1;
				// Send_data_CanBus();
			}
		}
		else
		{
			low_vol_DC_counter1 = 0;
			low_vol_DC	= 0;
		}

	}


// 3'''. DC Bus Extremely low voltage (<320V): Detect multiple times
//===============================================================================
	if (converter == BAT_CONVERTER)
	{
		if ((vo_ave > OFF_THRESHOLD_LOW_DC_VOL1) && (vo_ave < OFF_THRESHOLD_LOW_DC_VOL2) && (start_up_flag == 0) && (CONVERTER_STATUS == 1))
		{
			low_vol_DC_counter2++;
			if(low_vol_DC_counter2 >= TRIP_COUNT2_LOW)
			{
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000000001000;
				low_vol_DC = 1;
				// Send_data_CanBus();
			}
		}
		else
		{
			low_vol_DC_counter2 = 0;
			low_vol_DC	= 0;
		}
	}
	else
	{
		if ((vo_ave > OFF_THRESHOLD_LOW_DC_VOL1) && (vo_ave < OFF_THRESHOLD_LOW_DC_VOL2) && (CONVERTER_STATUS == 1))
		{
			low_vol_DC_counter2++;
			if(low_vol_DC_counter2 >= TRIP_COUNT2_LOW)
			{
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000000001000;
				low_vol_DC = 1;
				// Send_data_CanBus();
			}
		}
		else
		{
			low_vol_DC_counter2 = 0;
			low_vol_DC	= 0;
		}

	}

// 4.  Extremely high DC output current (>18A)

// High current output: High power => unscheduled transition from VRM --> PRM if mode=VRM.
//============================================================================================================================
if (converter == BAT_CONVERTER)
{
	if ((io_ave > MAX_OUTPUT_CUR) || (io_ave < - MAX_OUTPUT_CUR))
	{
		over_cur_DC_counter ++;
		if (over_cur_DC_counter >= OVER_POWER_COUNT)			// over range for 10s (50000x200us)
		{
			if ((mode == PRM) && (unscheduled_power_flag == 0))
			{
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000000010000;
				over_cur_DC = 1;
				// Send_data_CanBus();
			}
			if (mode == VRM)		//unscheduled transition
			{
				new_mode = PRM;
				unscheduled_power_flag = 1;
				if (io_ave > MAX_OUTPUT_CUR)
				{
					reference_power_new = MAX_POWER_RATING; 	// subjected to be MAX_POWER_RATING
					exceed_discharging_power_flag = 1;
					pow_unscheduled_indication_message = 1;
				}
				if (io_ave < - MAX_OUTPUT_CUR)
				{
					reference_power_new = - MAX_POWER_RATING;
					exceed_charging_power_flag = 1;
					pow_unscheduled_indication_message = 2;
				}

				unscheduled_BAT_message = (new_mode << 8) | mode;

				BAT_Send_Fault_Data_Canbus(unscheduled_BAT_message, pow_unscheduled_indication_message, reserve, BAT_UNSCHEDULED_MESSAGE_INDEX);

			}
		}
	}
	else
	{
		over_cur_DC = 0;
		over_cur_DC_counter = 0;
		if (unscheduled_power_flag == 1)
		{
			if ((exceed_discharging_power_flag == 1) && (vo_ave > RETURN_BACK_HIGH_VOL) && (vo_ave < BAT_THRESHOLD_OVER_DC_VOL))
			{
				new_mode = VRM;
				unscheduled_power_flag = 0;
				exceed_discharging_power_flag = 0;

				unscheduled_BAT_message = (new_mode << 8) | mode;
				BAT_Send_Fault_Data_Canbus(unscheduled_BAT_message, reserve, reserve, BAT_UNSCHEDULED_MESSAGE_INDEX);

			}

			if ((exceed_charging_power_flag == 1) && (vo_ave > BAT_THRESHOLD_LOW_DC_VOL) && (vo_ave < RETURN_BACK_LOW_VOL))
			{
				new_mode = VRM;
				unscheduled_power_flag = 0;
				exceed_charging_power_flag = 0;

				unscheduled_BAT_message = (new_mode << 8) | mode;
				pow_unscheduled_indication_message = 1;
				BAT_Send_Fault_Data_Canbus(unscheduled_BAT_message, pow_unscheduled_indication_message, reserve, BAT_UNSCHEDULED_MESSAGE_INDEX);
			}
		}
	}
}
//====================================================================================================================================

// 5.  Extremely low input voltage (<100V for PV)
//===============================================================================
	if (converter == PV_CONVERTER)
	{

		if ((viA_ave < vPV_min) && (mode_A == MPPT))
		{
			low_vol_A_counter++;
			if (low_vol_A_counter >= INPUT_PROTECT_COUNT)
			{
				new_mode_A = IDLE;
				low_vol_A = 1;
				stop_A = 1;
				fault_message1 |= 0b0000000000100000;
			}
		}
		else if (viA_ave >= vPV_min)
		{
			low_vol_A = 0;
			low_vol_A_counter = 0;
			stop_A = 0;
			fault_message1 &= 0b1111111111011111;
		}



		if ((viB_ave < vPV_min) && (mode_B == MPPT))
		{
			low_vol_B_counter++;
			if (low_vol_B_counter >= INPUT_PROTECT_COUNT)
			{
				new_mode_B = IDLE;
				low_vol_B = 1;
				stop_B = 1;
				fault_message1 |= 0b0000000001000000;
			}
		}
		else if (viB_ave >= vPV_min)
		{
			low_vol_B = 0;
			low_vol_B_counter = 0;
			stop_B = 0;
			fault_message1 &= 0b1111111110111111;
		}

		if ((viC_ave < vPV_min) && (mode_C == MPPT))
		{
			low_vol_C_counter++;
			if (low_vol_C_counter >= INPUT_PROTECT_COUNT)
			{
				new_mode_C = IDLE;
				low_vol_C = 1;
				stop_C = 1;
				fault_message1 |= 0b0000000010000000;
			}
		}
		else if (viC_ave >= vPV_min)
		{
			low_vol_C = 0;
			low_vol_C_counter = 0;
			stop_C = 0;
			fault_message1 &= 0b1111111101111111;
		}

		if ((low_vol_A == 1) && (low_vol_B == 1) && (low_vol_C == 1))
		{
			Turn_off_Converter();
		}

	}



//===============================================================================
	if ((CONVERTER_STATUS == 1) && (converter == BAT_CONVERTER))
	{
// 5.  Extremely low input voltage (<150V for BAT)
		if ((viA_ave < MIN_INPUT_VOL))
		{
			low_vol_A_counter++;
			if (low_vol_A_counter >= INPUT_PROTECT_COUNT)
			{
				stop_A = 1;
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000011100000;		// vi <<
				low_vol_A = 1;
				// Send_data_CanBus();
			}
		}
		else
			{
				stop_A = 0;
				low_vol_A = 0;
				low_vol_A_counter = 0;
			}

// 6.  Extremely low input voltage channel 2 (<150V for BAT, <100V for PV)

		if ((viB_ave < MIN_INPUT_VOL) && (CONVERTER_STATUS == 1) && (converter == BAT_CONVERTER))
		{
			low_vol_B_counter++;
			if (low_vol_B_counter >= INPUT_PROTECT_COUNT)
			{
				stop_B = 1;
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000000111000000;	// vi <<
				low_vol_B = 1;
				// Send_data_CanBus();
			}
		}
		else
			{
			    stop_B = 0;
				low_vol_B = 0;
				low_vol_B_counter = 0;
			}

// 7.  Extremely low input voltage channel 3 (<150V for BAT, <100V for PV)

		if ((viC_ave < MIN_INPUT_VOL) && (CONVERTER_STATUS == 1) && (converter == BAT_CONVERTER))
		{
			low_vol_C_counter++;
			if (low_vol_C_counter >= INPUT_PROTECT_COUNT)
			{
				stop_C = 1;
				stop = 1;
				Turn_off_Converter();
				fault_message1 |= 0b0000001110000000;	// vi <<
				low_vol_C = 1;
				// Send_data_CanBus();
			}
		}
		else
			{
				stop_C = 0;
				low_vol_C = 0;
				low_vol_C_counter = 0;
			}

	}

// 8.  Extremely high input voltage channel 1 (>230V for BAT, >300V for PV)
//===============================================================================
	if (viA_ave > MAX_INPUT_VOL)
	{
		over_vol_A_counter++;
		if (over_vol_A_counter >= INPUT_PROTECT_COUNT)
		{
			stop_A = 1;
			over_vol_A = 1;
			Turn_off_Converter();
			if (converter == BAT_CONVERTER)
			{
				fault_message1 |= 0b0000011100000000;
			}
			else
			{
				fault_message1 |= 0b0000000100000000;
			}

		}
	}
	else
		{
			stop_A = 0;
			over_vol_A = 0;
			over_vol_A_counter = 0;
		}


// 9.  Extremely high input voltage channel 2 (>230V for BAT, >300V for PV)
//===============================================================================
	if (viB_ave > MAX_INPUT_VOL)
	{
		over_vol_B_counter++;
		if (over_vol_B_counter >= INPUT_PROTECT_COUNT)
		{
			stop_B = 1;
			over_vol_B = 1;			//stop = 1;
			Turn_off_Converter();
			if (converter == BAT_CONVERTER)
			{
				fault_message1 |= 0b0000011100000000;
			}
			else
			{
				fault_message1 |= 0b0000001000000000;
			}

		}
	}
	else
		{
			stop_B = 0;
			over_vol_B = 0;
			over_vol_B_counter = 0;
		}


// 10. Extremely high input voltage channel 3 (>230V for BAT, >300V for PV)
//===============================================================================
	if (viC_ave > MAX_INPUT_VOL)
	{
		over_vol_C_counter++;
		if (over_vol_C_counter >= INPUT_PROTECT_COUNT)
		{
			stop_C = 1;
			over_vol_C = 1;			//stop = 1;
			Turn_off_Converter();
			if (converter == BAT_CONVERTER)
			{
				fault_message1 |= 0b0000011100000000;
			}
			else
			{
				fault_message1 |= 0b0000010000000000;
			}
			// Send_data_CanBus();
		}
	}
	else
		{
			stop_C = 0;
			over_vol_C = 0;
			over_vol_C_counter = 0;
		}


// 11. Extremely high input current channel 1 (>12A)
//===============================================================================
	if ((iiA_ave > MAX_INPUT_CUR) || (iiA_ave < -MAX_INPUT_CUR))
	{
		over_cur_A_counter ++;
		if (over_cur_A_counter >= OVER_CURRENT_COUNT)			// over range for 10s (50000x200us)
		{
			stop_A = 1;
			//stop = 1;
			Turn_off_Converter();
			fault_message1 |= 0b0000100000000000;
			over_cur_A = 1;
			// Send_data_CanBus();
		}
	}
	else
		{
			stop_A = 0;
			over_cur_A = 0;
			over_cur_A_counter = 0;
		}


// 12. Extremely high input current channel 2 (>12A)
//================================================================================
	if ((iiB_ave > MAX_INPUT_CUR) || (iiB_ave < -MAX_INPUT_CUR))
	{
		over_cur_B_counter ++;
		if (over_cur_B_counter >= OVER_CURRENT_COUNT)			// over range for 10s (50000x200us)
		{
			stop_B = 1;
			//stop = 1;
			Turn_off_Converter();
			fault_message1 |= 0b0001000000000000;
			over_cur_B = 1;
			// Send_data_CanBus();
		}
	}
	else
		{
			stop_B = 0;
			over_cur_B = 0;
			over_cur_B_counter = 0;
		}


// 13. Extremely high input current channel 3 (>12A)
//================================================================================
	if ((iiC_ave > MAX_INPUT_CUR) || (iiC_ave < -MAX_INPUT_CUR))
	{
		over_cur_C_counter ++;
		if (over_cur_C_counter >= OVER_CURRENT_COUNT)			// over range for 10s (50000x200us)
		{
			stop_C = 1;
			//stop = 1;
			Turn_off_Converter();
			fault_message1 |= 0b0010000000000000;
			over_cur_C = 1;

		}
	}
	else
		{
			stop_C = 0;
			over_cur_C = 0;
			over_cur_C_counter = 0;
		}




// 14. High temperature (>80*C)



// UNSCHEDULED TRANSITION //=====================================================
// Protect PV converter: PV unscheduled transition
if (converter == PV_CONVERTER)
{

	//PV3=============================================================================================
	if ((vo_ave > PV3_THRESHOLD_OVER_DC_VOL) && (vo_ave < OFF_THRESHOLD_OVER_DC_VOL))
	{
		PV3_over_vol_DC_counter++;
		if(PV3_over_vol_DC_counter >= PV_PROTECT_COUNT)
		{
			//	fault_message1 |= 0b0000000000001000;	// DC bus EXTREMELY HIGH voltage
			PV3_over_vol_DC = 1;
			unscheduled_PV1_flag = 1;
			unscheduled_PV2_flag = 1;
			unscheduled_PV3_flag = 1;

			if ((mode_A == MPPT) || (mode_A == PRM) || (mode_A == IDLE))	new_mode_A = IDLE;
			if ((mode_B == MPPT) || (mode_B == PRM) || (mode_B == IDLE))	new_mode_B = IDLE;
			if ((mode_C == MPPT) || (mode_C == PRM) || (mode_C == IDLE))	new_mode_C = IDLE;

			unscheduled_PV_message_A = (new_mode_A << 8) | mode_A;
			unscheduled_PV_message_B = (new_mode_B << 8) | mode_B;
			unscheduled_PV_message_C = (new_mode_C << 8) | mode_C;
			if ((mode_A != IDLE) || (mode_B != IDLE) || (mode_C != IDLE))
			{
				PV_Send_Fault_Data_Canbus(unscheduled_PV_message_A, unscheduled_PV_message_B, unscheduled_PV_message_C, PV_UNSCHEDULED_MESSAGE_INDEX);
			}

		}
	}
	else
	{
		PV3_over_vol_DC_counter = 0;
		PV3_over_vol_DC = 0;
	}
	//=================================================================================================


	//PV2=============================================================================================
	if ((vo_ave > PV2_THRESHOLD_OVER_DC_VOL) && (vo_ave < PV3_THRESHOLD_OVER_DC_VOL))
	{
		PV2_over_vol_DC_counter++;
		if(PV2_over_vol_DC_counter >= PV_PROTECT_COUNT)
		{
			//	fault_message1 |= 0b0000000000001000;	// DC bus EXTREMELY HIGH voltage
			PV2_over_vol_DC = 1;
			unscheduled_PV1_flag = 1;
			unscheduled_PV2_flag = 1;
			if ((mode_A == MPPT) || (mode_A == PRM) || (mode_A == IDLE))	new_mode_A = IDLE;
			if ((mode_B == MPPT) || (mode_B == PRM) || (mode_B == IDLE))	new_mode_B = IDLE;

			unscheduled_PV_message_A = (new_mode_A << 8) | mode_A;
			unscheduled_PV_message_B = (new_mode_B << 8) | mode_B;
			if ((mode_A != IDLE) || (mode_B != IDLE))
			{
				PV_Send_Fault_Data_Canbus(unscheduled_PV_message_A, unscheduled_PV_message_B, unscheduled_PV_message_C, PV_UNSCHEDULED_MESSAGE_INDEX);
			}

		}
	}
	else
	{
		PV2_over_vol_DC_counter = 0;
		PV2_over_vol_DC = 0;
	}

	//PV1=============================================================================================
	if ((vo_ave > PV1_THRESHOLD_OVER_DC_VOL) && (vo_ave < PV2_THRESHOLD_OVER_DC_VOL))
	{
		PV1_over_vol_DC_counter++;
		if(PV1_over_vol_DC_counter >= PV_PROTECT_COUNT)
		{
//			fault_message1 |= 0b0000000000001000;	// DC bus EXTREMELY HIGH voltage
			PV1_over_vol_DC = 1;
			unscheduled_PV1_flag = 1;
			if ((mode_A == MPPT) || (mode_A == PRM) || (mode_A == IDLE))	new_mode_A = IDLE;
//			new_mode_B = VRM;
//			new_mode_C = VRM;
			unscheduled_PV_message_A = (new_mode_A << 8) | mode_A;
			if (mode_A != IDLE)	PV_Send_Fault_Data_Canbus(unscheduled_PV_message_A, unscheduled_PV_message_B, unscheduled_PV_message_C, PV_UNSCHEDULED_MESSAGE_INDEX);
		}
	}
	else
	{
		PV1_over_vol_DC_counter = 0;
		PV1_over_vol_DC = 0;
	}


}


//===============================================================================
// Protect BAT converter: BAT unscheduled transition
// High vol DC
if (converter == BAT_CONVERTER)
{
	if ((vo_ave > BAT_THRESHOLD_OVER_DC_VOL) && (exceed_charging_power_flag == 0))
	{
		BAT_over_vol_DC_counter++;
		if(BAT_over_vol_DC_counter >= BAT_UNSCHEDULED_COUNT)
		{
			BAT_over_vol_DC = 1;
			if (mode == PRM)
			{
				new_mode = VRM;
				reference_vol_new = 380;
				droop_coef = 0.4;
				unscheduled_voltage_flag = 1;
				exceed_high_voltage_flag = 1;
				unscheduled_BAT_message = (new_mode << 8) | mode;
				vol_unscheduled_indication_message = 2;
				BAT_Send_Fault_Data_Canbus(unscheduled_BAT_message, vol_unscheduled_indication_message, reserve, BAT_UNSCHEDULED_MESSAGE_INDEX);
			}
		}
	}
	else
	{
		BAT_over_vol_DC_counter = 0;
		BAT_over_vol_DC = 0;
	}


//Low vol DC
//===============================================================================
	if ((vo_ave < BAT_THRESHOLD_LOW_DC_VOL) && (exceed_discharging_power_flag == 0) && (duty_max_reset == 0) && (duty_min_set == 0) && (CONVERTER_STATUS == 1))  //k can ktra 2dk giua
	{
		BAT_low_vol_DC_counter++;
		if(BAT_low_vol_DC_counter >= BAT_UNSCHEDULED_COUNT)
		{
			BAT_low_vol_DC = 1;
			if (mode == PRM)
			{
				new_mode = VRM;
				reference_vol_new = 380;
				droop_coef = 0.4;
				unscheduled_voltage_flag = 1;
				exceed_low_voltage_flag = 1;
				unscheduled_BAT_message = (new_mode << 8) | mode;
				vol_unscheduled_indication_message = 1;
				BAT_Send_Fault_Data_Canbus(unscheduled_BAT_message, vol_unscheduled_indication_message, reserve, BAT_UNSCHEDULED_MESSAGE_INDEX);
			}
		}
	}

	else
		{
		BAT_low_vol_DC_counter = 0;
		BAT_low_vol_DC = 0;
		}
}

if (fault_message1 == 0)	send_fault_flag = 0;

if ((fault_message1 != 0) && (fault_message1 != 2))
{
	if (send_fault_flag == 0)
	{
		if (converter == BAT_CONVERTER) BAT_Send_Fault_Data_Canbus(fault_message1, fault_message2, reserve, BAT_FAULT_MESSAGE_INDEX);
		if (converter == PV_CONVERTER)  PV_Send_Fault_Data_Canbus(fault_message1, fault_message2, reserve, PV_FAULT_MESSAGE_INDEX);
		send_fault_flag = 1;
	}
}


} // end if stop = 0
//================================================================================

}



void Soft_Transition() {

if (converter == BAT_CONVERTER)
{
	BAT_Transition_Mode();
}

//=======================================================
else	//PV converter
{
	if (((mode_A == ISOLATED) && (new_mode_A != mode_A)) || ((mode_B == ISOLATED) && (new_mode_B != mode_B)) || ((mode_C == ISOLATED) && (new_mode_C != mode_C)))
	{
		precharging_counter_pv ++;
	}

	PV1_Transition_Mode();
	PV2_Transition_Mode();
	PV3_Transition_Mode();

	if ((mode_A == MPPT) || (mode_A == VRM) || (mode_B == MPPT) ||(mode_B == VRM) ||(mode_C == MPPT) ||(mode_C == VRM))
	{
		LED_RUN_OUTSIDE_ON();
		LED_STOP_OUTSIDE_OFF();
		if (fault_message1 == 0)	LED_IDLE_OUTSIDE_OFF();
	}

	if ((mode_A == IDLE) && (mode_B == IDLE) && (mode_C == IDLE))
	{
		CONVERTER_STATUS = 0;
		LED_RUN_OUTSIDE_OFF();
		LED_STOP_OUTSIDE_OFF();
		if (fault_message1 == 0)	LED_IDLE_OUTSIDE_ON();
	}

	if ((mode_A == ISOLATED) || (mode_B == ISOLATED) || (mode_C == ISOLATED))
	{
		LED_RUN_OUTSIDE_OFF();
		LED_STOP_OUTSIDE_ON();
		if (fault_message1 == 0)	LED_IDLE_OUTSIDE_OFF();
	}

}
//=======================================================


//******************************************************
switch (converter)
{
case BAT_CONVERTER:
{
	if ((duty_max_reset_A == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
//			test_duty++;
		duty_max_A = duty_max_A + duty_step_plus;
		if (duty_max_A >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_A = MAX_DUTY;
			duty_max_reset_A = 0;
		}
	}
	if ((duty_min_set_A == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
		duty_min_A = duty_min_A - duty_step_minus;
		if (duty_min_A <= MIN_DUTY)
		{
			duty_min_A = MIN_DUTY;
			duty_min_set_A = 0;
		}
	}

//******************************************************
	if ((duty_max_reset_B == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
//			test_duty++;
		duty_max_B = duty_max_B + duty_step_plus;
		if (duty_max_B >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_B = MAX_DUTY;
			duty_max_reset_B = 0;
		}
	}
	if ((duty_min_set_B == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
		duty_min_B = duty_min_B - duty_step_minus;
		if (duty_min_B <= MIN_DUTY)
		{
			duty_min_B = MIN_DUTY;
			duty_min_set_B = 0;
		}
	}

//******************************************************
	if ((duty_max_reset_C == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
//			test_duty++;
		duty_max_C = duty_max_C + duty_step_plus;
		if (duty_max_C >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_C = MAX_DUTY;
			duty_max_reset_C = 0;
		}
	}
	if ((duty_min_set_C == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
		duty_min_C = duty_min_C - duty_step_minus;
		if (duty_min_C <= MIN_DUTY)
		{
			duty_min_C = MIN_DUTY;
			duty_min_set_C = 0;
		}
	}
	break;
}

//================================================================================================
case PV_CONVERTER:
{
	if ((duty_max_reset_A == 1) && (mode_A != IDLE) && (mode_A != ISOLATED))
	{
//			test_duty++;
		duty_max_A = duty_max_A + duty_step_plus;
		if (duty_max_A >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_A = MAX_DUTY;
			duty_max_reset_A = 0;
		}
	}
	if ((duty_min_set_A == 1) && (mode_A != IDLE) && (mode_A != ISOLATED))
	{
		duty_min_A = duty_min_A - 0.02;
		if (duty_min_A <= MIN_DUTY_MPPT)
		{
			duty_min_A = MIN_DUTY_MPPT;
			duty_min_set_A = 0;
		}
	}

//******************************************************
	if ((duty_max_reset_B == 1) && (mode_B != IDLE) && (mode_B != ISOLATED))
	{
//			test_duty++;
		duty_max_B = duty_max_B + duty_step_plus;
		if (duty_max_B >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_B = MAX_DUTY;
			duty_max_reset_B = 0;
		}
	}
	if ((duty_min_set_B == 1) && (mode_B != IDLE) && (mode_B != ISOLATED))
	{
		duty_min_B = duty_min_B - 0.02;
		if (duty_min_B <= MIN_DUTY)
		{
			duty_min_B = MIN_DUTY;
			duty_min_set_B = 0;
		}
	}

//******************************************************
	if ((duty_max_reset_C == 1) && (mode_C != IDLE) && (mode_C != ISOLATED))
	{
//			test_duty++;
		duty_max_C = duty_max_C + duty_step_plus;
		if (duty_max_C >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_C = MAX_DUTY;
			duty_max_reset_C = 0;
		}
	}
	if ((duty_min_set_C == 1) && (mode_C != IDLE) && (mode_C != ISOLATED))
	{
		duty_min_C = duty_min_C - 0.02;
		if (duty_min_C <= MIN_DUTY)
		{
			duty_min_C = MIN_DUTY;
			duty_min_set_C = 0;
		}
	}
	break;
}
}

//******************************************************

}		// End Soft Transition
////=================================================================================================================

// BAT_Transition mode===============================================================================================
void BAT_Transition_Mode() {

if (stop == 0)
{
		if ((mode == ISOLATED) && (new_mode != mode))
		{
			mode = IDLE;
			duty_min_set_A = 0;
			duty_max_reset_A = 0;

			duty_min_set_B = 0;
			duty_max_reset_B = 0;

			duty_min_set_C = 0;
			duty_max_reset_C = 0;


			ENABLE_ALL_PWM();
			CLOSE_RELAY_DC();
			CLOSE_RELAY_BAT();
			INPUT_RELAY_STATUS = 1;
			OUTPUT_RELAY_STATUS = 1;
			LED_RUN_OFF();
			LED_STOP_ON();

			LED_RUN_OUTSIDE_OFF();
			LED_STOP_OUTSIDE_OFF();

			CONVERTER_STATUS = 0;
			CH1_ONOFF_STATUS = 0;
			CH2_ONOFF_STATUS = 0;
			CH3_ONOFF_STATUS = 0;



			if (fault_message1 == 0)	LED_IDLE_OUTSIDE_ON();
		}

// New
//---------------------------------------------------------------------------------------------------------------------------------------------------

if (INPUT_RELAY_STATUS == 1)
{
switch (new_mode)
{
	case VRM:
	{
		if (mode == IDLE)
		{
			if (vo_ave < 300)			//cold start
			{
				duty_max_A = 0;
				duty_min_A = 0;
				duty_max_reset_A = 1;

				duty_max_B = 0;
				duty_min_B = 0;
				duty_max_reset_B = 1;

				duty_max_C = 0;
				duty_min_C = 0;
				duty_max_reset_C = 1;

				mode = new_mode;
				reference_vol = reference_vol_new;
				CONVERTER_STATUS = 1;
				CH1_ONOFF_STATUS = 1;
				CH2_ONOFF_STATUS = 1;
				CH3_ONOFF_STATUS = 1;
				LED_RUN_ON();
				LED_STOP_OFF();

				LED_RUN_OUTSIDE_ON();
				LED_STOP_OUTSIDE_OFF();
				LED_IDLE_OUTSIDE_OFF();

				start_up_flag = 1;
			}
			else						//hot start
			{
				duty_max_A = 380 - viA_ave + 5;
				duty_min_A = 380 - viA_ave - 10;
				duty_max_reset_A = 1;
				duty_min_set_A = 1;

				duty_max_B = 380 - viA_ave + 5;
				duty_min_B = 380 - viA_ave - 10;
				duty_max_reset_B = 1;
				duty_min_set_B = 1;

				duty_max_C = 380 - viA_ave + 5;
				duty_min_C = 380 - viA_ave - 10;
				duty_max_reset_C = 1;
				duty_min_set_C = 1;

				mode = new_mode;
				reference_vol = reference_vol_new;
				CONVERTER_STATUS = 1;
				CH1_ONOFF_STATUS = 1;
				CH2_ONOFF_STATUS = 1;
				CH3_ONOFF_STATUS = 1;
				LED_RUN_ON();
				LED_STOP_OFF();

				LED_RUN_OUTSIDE_ON();
				LED_STOP_OUTSIDE_OFF();
				LED_IDLE_OUTSIDE_OFF();
			}

		}

		else if ((mode == PRM) || ((mode == VRM) && (reference_vol != reference_vol_new)))
		{
			mode = new_mode;
			reference_vol = reference_vol_new;
		}

		break;
	}

	case PRM:
	{
		if ((mode == VRM) || ((mode == PRM) && (reference_power_new != reference_power)))
		{
			mode = new_mode;
			reference_power = reference_power_new;
		}
		else if (mode == IDLE)
		{

			duty_max_A = 380 - viA_ave + 5;
			duty_min_A = 380 - viA_ave - 10;
			duty_max_reset_A = 1;
			duty_min_set_A = 1;

			duty_max_B = 380 - viA_ave + 5;
			duty_min_B = 380 - viA_ave - 10;
			duty_max_reset_B = 1;
			duty_min_set_B = 1;

			duty_max_C = 380 - viA_ave + 5;
			duty_min_C = 380 - viA_ave - 10;
			duty_max_reset_C = 1;
			duty_min_set_C = 1;


			mode = new_mode;
			reference_power = reference_power_new;

			CONVERTER_STATUS = 1;
			CH1_ONOFF_STATUS = 1;
			CH2_ONOFF_STATUS = 1;
			CH3_ONOFF_STATUS = 1;
			LED_RUN_ON();
			LED_STOP_OFF();

			LED_RUN_OUTSIDE_ON();
			LED_STOP_OUTSIDE_OFF();
			LED_IDLE_OUTSIDE_OFF();
		}
			break;
	}

	case IDLE:
	{
		mode = new_mode;
		CONVERTER_STATUS = 0;
		CH1_ONOFF_STATUS = 0;
		CH2_ONOFF_STATUS = 0;
		CH3_ONOFF_STATUS = 0;
		LED_RUN_OFF();
		LED_STOP_ON();

		LED_RUN_OUTSIDE_OFF();
		LED_STOP_OUTSIDE_OFF();
		if (fault_message1 == 0)	LED_IDLE_OUTSIDE_ON();
		break;
	}

	case ISOLATED:
	{
		mode = new_mode;
		Turn_off_Converter();
		break;
	}

} 	// End switch

}
}
}


// PV1 Transition mode ==============================================================================================
void PV1_Transition_Mode() {

		if ((mode_A == ISOLATED) && (new_mode_A != mode_A))
		{
			if (viA_ave >= MIN_PV_STARTUP_VOL)
			{

				duty_min_set_A = 0;
				duty_max_reset_A = 0;

//				if (precharging_counter_pv >= 10000)
//				{
				precharging_counter_pv = 0;
				ENABLE_ALL_PWM();
				CLOSE_RELAY_DC();
				CLOSE_RELAY_BAT();
				INPUT_RELAY_STATUS = 1;
				OUTPUT_RELAY_STATUS = 1;

				mode_A = IDLE;
				mode_B = IDLE;
				mode_C = IDLE;


				CONVERTER_STATUS = 0;
				CH1_ONOFF_STATUS = 0;
//				}

			}
			else
			{
				fault_message1 |= 0b0000000000100000;	// low input vol
			}
		}

if (INPUT_RELAY_STATUS == 1)
{
switch (new_mode_A)
{
	case VRM:
	{
		if ((mode_A == IDLE) || (mode_A == MPPT) || (mode_A == PRM))																				// TH1
		{
			duty_max_A = 0;
			duty_min_A = 0;
			duty_max_reset_A = 1;
			mode_A = new_mode_A;
			reference_vol_A = 380;
			CONVERTER_STATUS = 1;
			CH1_ONOFF_STATUS = 1;
		}
		break;
	}

	case PRM:
	{
		if ((mode_A == MPPT) || (mode_A == VRM) || (mode_A == IDLE))
		{
			duty_min_A = 0;
			duty_max_A = 0;
			duty_max_reset_A = 1;
			mode_A = new_mode_A;
			reference_power_A = reference_power_new_A;
			CONVERTER_STATUS = 1;
			CH1_ONOFF_STATUS = 1;

		}
		else if ((mode_A == PRM) && (reference_power_new_A != reference_power_A))
		{
			reference_power_A = reference_power_new_A;
		}
			break;
	}

	case MPPT:
	{
		if ((mode_A != MPPT) && (mode_A != ISOLATED) && (viA_ave >= MIN_PV_STARTUP_VOL))
		{
			vref_A = vPV_max - 50;
			duty_max_A = period;
			duty_min_A = period;
			duty_min_set_A = 1;
			mode_A = MPPT;
			CONVERTER_STATUS = 1;
			CH1_ONOFF_STATUS = 1;

			sum_viA = 0;
			sum_iiA = 0;
			ave_viA = viA_ave;
			ave_iiA = iiA_ave;
			k = 0;
		}
		break;
	}

	case IDLE:
	{
	if (mode_A != ISOLATED)
	{
		mode_A = IDLE;
		CH1_ONOFF_STATUS = 0;
	}
		break;
	}

	case ISOLATED:
	{
		mode_B = ISOLATED;
		mode_A = ISOLATED;
		mode_C = ISOLATED;
		Turn_off_Converter();
		break;
	}

} 	// End switch

}
}


// PV2 Transition mode ==============================================================================================
void PV2_Transition_Mode() {


		if ((mode_B == ISOLATED) && (new_mode_B != mode_B))
		{
			if (viB_ave >= MIN_PV_STARTUP_VOL)
			{
				duty_min_set_B = 0;
				duty_max_reset_B = 0;

//				if (precharging_counter_pv >= 10000)
//				{
				precharging_counter_pv = 0;

				ENABLE_ALL_PWM();
				CLOSE_RELAY_DC();
				CLOSE_RELAY_BAT();
				INPUT_RELAY_STATUS = 1;
				OUTPUT_RELAY_STATUS = 1;

				mode_B = IDLE;
				mode_A = IDLE;
				mode_C = IDLE;



				CONVERTER_STATUS = 0;
				CH2_ONOFF_STATUS = 0;
//				}
			}
			else
			{
				fault_message1 |= 0b0000000001000000;
			}

		}

if (INPUT_RELAY_STATUS == 1)
{
switch (new_mode_B)
{
	case VRM:
	{
		if ((mode_B == IDLE) || (mode_B == MPPT) || (mode_B == PRM))																				// TH1
		{
			duty_max_B = 0;
			duty_min_B = 0;
			duty_max_reset_B = 1;
			mode_B = new_mode_B;
			reference_vol_B = 380;
			CONVERTER_STATUS = 1;
			CH2_ONOFF_STATUS = 1;

		}
		break;
	}

	case PRM:
	{
		if ((mode_B == MPPT) || (mode_B == VRM) || (mode_B == IDLE))
		{
			duty_min_B = 0;
			duty_max_B = 0;
			duty_max_reset_B = 1;
			mode_B = new_mode_B;
			reference_power_B = reference_power_new_B;
			CONVERTER_STATUS = 1;
			CH2_ONOFF_STATUS = 1;

		}
		else if ((mode_B == PRM) && (reference_power_new_B != reference_power_B))
		{
			reference_power_B = reference_power_new_B;
		}
			break;
	}

	case MPPT:
	{
		if ((mode_B != MPPT) && (mode_B != ISOLATED) && (viB_ave >= MIN_PV_STARTUP_VOL))
		{
			vref_B = vPV_max - 50;
			duty_max_B = period;
			duty_min_B = period;
			duty_min_set_B = 1;
			mode_B = MPPT;
			CONVERTER_STATUS = 1;
			CH2_ONOFF_STATUS = 1;


			sum_viB = 0;
			sum_iiB = 0;
			ave_viB = viB_ave;
			ave_iiB = iiB_ave;
			k = 0;
		}
		break;
	}

	case IDLE:
	{
	if (mode_B != ISOLATED)
	{
		mode_B = IDLE;
		CH2_ONOFF_STATUS = 0;

	}
		break;
	}

	case ISOLATED:
	{
		mode_B = ISOLATED;
		mode_A = ISOLATED;
		mode_C = ISOLATED;
		Turn_off_Converter();
		break;
	}

} 	// End switch

}
}

// PV3 Transition mode ==============================================================================================
void PV3_Transition_Mode() {

			if ((mode_C == ISOLATED) && (new_mode_C != mode_C))
			{
				if (viC_ave >= MIN_PV_STARTUP_VOL)
				{
					duty_min_set_C = 0;
					duty_max_reset_C = 0;

//					if (precharging_counter_pv >= 10000)
//					{
					precharging_counter_pv = 0;

					ENABLE_ALL_PWM();
					CLOSE_RELAY_DC();
					CLOSE_RELAY_BAT();
					INPUT_RELAY_STATUS = 1;
					OUTPUT_RELAY_STATUS = 1;

					mode_B = IDLE;
					mode_A = IDLE;
					mode_C = IDLE;

					CONVERTER_STATUS = 0;
					CH3_ONOFF_STATUS = 0;
//					}
				}
				else
				{
					fault_message1 |= 0b0000000010000000;
				}

			}

if (INPUT_RELAY_STATUS == 1)
{
	switch (new_mode_C)
	{
		case VRM:
		{
			if ((mode_C == IDLE) || (mode_C == MPPT) || (mode_C == PRM))																				// TH1
			{
				duty_max_C = 0;
				duty_min_C = 0;
				duty_max_reset_C = 1;
				mode_C = new_mode_C;
				reference_vol_C = 380;
				CONVERTER_STATUS = 1;
				CH3_ONOFF_STATUS = 1;

			}
			break;
		}

		case PRM:
		{
			if ((mode_C == MPPT) || (mode_C == VRM) || (mode_C == IDLE))
			{
				duty_min_C = 0;
				duty_max_C = 0;
				duty_max_reset_C = 1;
				mode_C = new_mode_C;
				reference_power_C = reference_power_new_C;
				CONVERTER_STATUS = 1;
				CH3_ONOFF_STATUS = 1;

			}
			else if ((mode_C == PRM) && (reference_power_new_C != reference_power_C))
			{
				reference_power_C = reference_power_new_C;
			}
				break;
		}

		case MPPT:
		{
			if ((mode_C != MPPT) && (mode_C != ISOLATED) && (viC_ave >= MIN_PV_STARTUP_VOL))
			{
				vref_C = vPV_max - 50;
				duty_max_C = period;
				duty_min_C = period;
				duty_min_set_C = 1;
				mode_C = MPPT;
				CONVERTER_STATUS = 1;
				CH3_ONOFF_STATUS = 1;

				sum_viC = 0;
				sum_iiC = 0;
				ave_viC = viC_ave;
				ave_iiC = iiC_ave;
				k = 0;
			}
			break;
		}

		case IDLE:
		{
		if (mode_C != ISOLATED)
		{
			mode_C = IDLE;
			CH3_ONOFF_STATUS = 0;

		}
			break;
		}

		case ISOLATED:
		{
			mode_B = ISOLATED;
			mode_A = ISOLATED;
			mode_C = ISOLATED;
			Turn_off_Converter();
			break;
		}

} 	// End switch

}
}
interrupt void  adc_isr(void)
{

	adc_a0 = AdcRegs.ADCRESULT0 >> 4;  //Iout     offset  2055
	adc_a1 = AdcRegs.ADCRESULT1 >> 4;  //IL3              2055
	adc_a2 = AdcRegs.ADCRESULT2 >> 4;  //VESS1            2057
	adc_a3 = AdcRegs.ADCRESULT3 >> 4;  //VDC+             2058
	adc_a4 = AdcRegs.ADCRESULT4 >> 4;  //IL2              2054
	adc_a5 = AdcRegs.ADCRESULT5 >> 4;  //IL1              2058
	adc_b6 = AdcRegs.ADCRESULT6 >> 4;  //VESS3
	adc_b7 = AdcRegs.ADCRESULT7 >> 4;  //VESS2

if (converter == BAT_CONVERTER)
{
	viA = (float) ((adc_a2 - offset_a2) * gain_a2);
	viB = (float) ((adc_b7 - offset_b7) * gain_b7);
	viC = (float) ((adc_b6 - offset_b6) * gain_b6);
	iiA = (float) ((adc_a5 - offset_a5) * gain_a5);
	iiB = (float) ((adc_a4 - offset_a4) * gain_a4);
	iiC = (float) ((adc_a1 - offset_a1) * gain_a1);


	vo = (float) ((adc_a3 - offset_a3) * gain_a3);
	io = (float) ((adc_a0 - offset_a0) * gain_a0);


}
else
{
	viA = (float) ((adc_a2 - offset_a2_pv) * gain_a2_pv);
	viB = (float) ((adc_b7 - offset_b7_pv) * gain_b7_pv);
	viC = (float) ((adc_b6 - offset_b6_pv) * gain_b6_pv);
	iiA = (float) ((adc_a5 - offset_a5_pv) * gain_a5_pv);
	iiB = (float) ((adc_a4 - offset_a4_pv) * gain_a4_pv);
	iiC = (float) ((adc_a1 - offset_a1_pv) * gain_a1_pv);


	vo = (float) ((adc_a3 - offset_a3_pv) * gain_a3_pv);
	io = (float) ((adc_a0 - offset_a0_pv) * gain_a0_pv);

}
//=================

	ConversionCount++;

	viA_sum = viA_sum + viA;
	viB_sum = viB_sum + viB;
	viC_sum = viC_sum + viC;
	vo_sum  = vo_sum  + vo;

	iiA_sum = iiA_sum + iiA;
	iiB_sum = iiB_sum + iiB;
	iiC_sum = iiC_sum + iiC;
	io_sum  = io_sum  + io;
//=================


//=======================================

  // Reinitialize for next ADC sequence
  AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
  AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

// End ADC interrupt
//===================================================================================================================

//===================================================================================================================
void ADC_Calculation(){
// ADC calculation
	GpioDataRegs.GPCSET.bit.GPIO84 = 1; // check total time for this interrupt routine

	if (ConversionCount != 0)	ConversionCount1 = (float) (1.0 / ConversionCount);

	viA_ave = viA_sum*ConversionCount1;
	viB_ave = viB_sum*ConversionCount1;
	viC_ave = viC_sum*ConversionCount1;
	vo_ave  = vo_sum*ConversionCount1;

	iiA_ave = iiA_sum*ConversionCount1;
	iiB_ave = iiB_sum*ConversionCount1;
	iiC_ave = iiC_sum*ConversionCount1;
	io_ave  = io_sum*ConversionCount1;

	i_in_ave = (iiA_ave + iiB_ave + iiC_ave)*0.333;

	viA_sum = 0;
	viB_sum = 0;
	viC_sum = 0;
	vo_sum  = 0;
	iiA_sum = 0;
	iiB_sum = 0;
	iiC_sum = 0;
	io_sum  = 0;

	ConversionCount = 0;

	if ((start_up_flag == 1) && (vo_ave >= 360))	start_up_flag = 0;

/////////////////////////////////////
	if (initial_SOC != new_initial_SOC)
		{
		initial_SOC = new_initial_SOC;
		SOC = initial_SOC/10;
		if (SOC <= 0)	SOC = 0;
		if (SOC >= 100)	SOC = 100;
		}

		test2 = (iiA_ave + iiB_ave + iiC_ave)*0.01 / 3600.0 / BAT_capacitive;
		if (test2 >= 0.000001) test2 = 0.000001;
		if (test2 <= -0.000001) test2 = -0.000001;

		sum_test2 = sum_test2 + test2;

		if ((sum_test2 >= 0.001) || (sum_test2 <= -0.001))
		{
			SOC_new		= SOC - sum_test2;
			sum_test2 	= 0;
			if (SOC_new <= 0)	SOC_new = 0;
			if (SOC_new >= 100)	SOC_new = 100;
			SOC = SOC_new;
		}

	GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1; // check total time for this interrupt routine
}

//==========  2 closed loop PI controller ===========================================================================//
void PI_Controller() {

if (converter == BAT_CONVERTER)
{
	if ((mode == VRM) && (ecan_receive_done == 1))
		{
		vref_A = reference_vol - droop_coef *io_ave;
		vref_B = vref_A;
		vref_C = vref_A;
		ev_A = vref_A - vo_ave;
		ev_B = ev_A;
		ev_C = ev_A;

		reference_power = viA_ave*i_in_ave*0.003;
		}

		iref_A = iref_A_pre + (kpv_A + kiv_A * Ts / 2) * ev_A + (-kpv_A + kiv_A * Ts / 2) * ev_A_pre;
		iref_B = iref_B_pre + (kpv_B + kiv_B * Ts / 2) * ev_B + (-kpv_B + kiv_B * Ts / 2) * ev_B_pre;
		iref_C = iref_C_pre + (kpv_C + kiv_C * Ts / 2) * ev_C + (-kpv_C + kiv_C * Ts / 2) * ev_C_pre;

	//saturation
		if (iref_A < -i_sat)	iref_A = -i_sat;
		if (iref_B < -i_sat)	iref_B = -i_sat;
		if (iref_C < -i_sat)	iref_C = -i_sat;

//		if (iref > i_sat)	iref = i_sat;
		if (iref_A > i_sat)	iref_A = i_sat;
		if (iref_B > i_sat)	iref_B = i_sat;
		if (iref_C > i_sat)	iref_C = i_sat;

	// Power Dispatch
		if (mode == PRM)
		{
			if (viA_ave != 0)
			{
				i_set = testing*333.3/viA_ave;
			}
			else
			{
				i_set = 0;
			}

			if (i_set >=  0.7*i_sat)	i_set =  0.7*i_sat;
			if (i_set <= -0.7*i_sat)	i_set = -0.7*i_sat;
			iref_A = i_set;
			iref_B = i_set;
			iref_C = i_set;
		}

		ei_A = iref_A - iiA_ave;
		duty_A = duty_A_pre + (kpi_A + kii_A * Ts / 2) * ei_A + (-kpi_A + kii_A * Ts / 2) * ei_A_pre;

		ei_B = iref_B - iiB_ave;
		duty_B = duty_B_pre + (kpi_B + kii_B * Ts / 2) * ei_B + (-kpi_B + kii_B * Ts / 2) * ei_B_pre;

		ei_C = iref_C - iiC_ave;
		duty_C = duty_C_pre + (kpi_C + kii_C * Ts / 2) * ei_C + (-kpi_C + kii_C * Ts / 2) * ei_C_pre;

		if (duty_A >= duty_max_A) 		duty_A = duty_max_A;
		if (duty_A <= duty_min_A)		duty_A = duty_min_A;

		if (duty_B >= duty_max_B) 		duty_B = duty_max_B;
		if (duty_B <= duty_min_B)		duty_B = duty_min_B;

		if (duty_C >= duty_max_C) 		duty_C = duty_max_C;
		if (duty_C <= duty_min_C)		duty_C = duty_min_C;

		viA_ave_pre = viA_ave;
		vref_A_pre = vref_A;
		ev_A_pre = ev_A;
		ei_A_pre = ei_A;
		iref_A_pre = iref_A;
		duty_A_pre = duty_A;

		viB_ave_pre = viB_ave;
		vref_B_pre = vref_B;
		ev_B_pre = ev_B;
		ei_B_pre = ei_B;
		iref_B_pre = iref_B;
		duty_B_pre = duty_B;

		viC_ave_pre = viC_ave;
		vref_C_pre = vref_C;
		ev_C_pre = ev_C;
		ei_C_pre = ei_C;
		iref_C_pre = iref_C;
		duty_C_pre = duty_C;



}
else	// converter == PV converter
{

//== PV1 Controller==========================================================================================================
	if ((mode_A == VRM) && (ecan_receive_done == 1))
	{
		vref_A = reference_vol_A - droop_coef_A *io_ave;
		ev_A = vref_A - vo_ave;
	}

	if (mode_A == MPPT)
	{
		ev_A = vref_A - viA_ave;
	}

	iref_A = iref_A_pre + (kpv_A1 + kiv_A1 * Ts / 2) * ev_A + (-kpv_A1 + kiv_A1 * Ts / 2) * ev_A_pre;

	//saturation
		if (iref_A < -i_sat)	iref_A = -i_sat;
		if (iref_A > i_sat)		iref_A = i_sat;

	// Power Dispatch
		if (mode_A == PRM)
			{
			i_set_A = reference_power_A*1000/viA_ave;
			if (i_set_A >=  0.7*i_sat)	i_set_A =  0.7*i_sat;
			if (i_set_A <= -0.7*i_sat)	i_set_A = -0.7*i_sat;
			iref_A = i_set_A;
			}

		ei_A = iref_A - iiA_ave;
		duty_A = duty_A_pre + (kpi_A1 + kii_A1 * Ts / 2) * ei_A + (-kpi_A1 + kii_A1 * Ts / 2) * ei_A_pre;

	//saturation
		if (duty_min_A <= MIN_DUTY_MPPT)	duty_min_A = MIN_DUTY_MPPT;
		if (duty_A >= duty_max_A) 		duty_A = duty_max_A;
		if (duty_A <= duty_min_A)		duty_A = duty_min_A;

	// update variables
		viA_ave_pre = viA_ave;
		vref_A_pre = vref_A;
		ev_A_pre = ev_A;
		ei_A_pre = ei_A;
		iref_A_pre = iref_A;
		duty_A_pre = duty_A;

		PowerPV_A_pre = PowerPV_A;
		ave_viA_pre	  = ave_viA;
		ave_iiA_pre	  = ave_iiA;



//== End PV1 controller==========================================================================================================


//==PV2 controller===============================================================================================================

	if ((mode_B == VRM) && (ecan_receive_done == 1))
	{
		vref_B = reference_vol_B - droop_coef_B *io_ave;
		ev_B = vref_B - vo_ave;
	}

	if (mode_B == MPPT)
	{
		ev_B = vref_B - viB_ave;
	}

	iref_B = iref_B_pre + (kpv_B1 + kiv_B1 * Ts / 2) * ev_B + (-kpv_B1 + kiv_B1 * Ts / 2) * ev_B_pre;

	//saturation
		if (iref_B < -i_sat)	iref_B = -i_sat;
		if (iref_B > i_sat)		iref_B = i_sat;

	// Power Dispatch
		if (mode_B == PRM)
			{
			i_set_B = reference_power_B*1000/viB_ave;      // =1000/3: 3 channel
			if (i_set_B >=  0.7*i_sat)	i_set_B =  0.7*i_sat;
			if (i_set_B <= -0.7*i_sat)	i_set_B = -0.7*i_sat;
			iref_B = i_set_B;
			}

	//inner loop: current

		ei_B = iref_B - iiB_ave;
		duty_B = duty_B_pre + (kpi_B1 + kii_B1 * Ts / 2) * ei_B + (-kpi_B1 + kii_B1 * Ts / 2) * ei_B_pre;

	//saturation
		if (duty_min_B <= MIN_DUTY_MPPT)	duty_min_B 	= MIN_DUTY_MPPT;
		if (duty_B >= duty_max_B) 			duty_B 		= duty_max_B;
		if (duty_B <= duty_min_B)			duty_B 		= duty_min_B;

	// update variables
		viB_ave_pre = viB_ave;
		vref_B_pre = vref_B;
		ev_B_pre = ev_B;
		ei_B_pre = ei_B;
		iref_B_pre = iref_B;
		duty_B_pre = duty_B;

		PowerPV_B_pre = PowerPV_B;
		ave_viB_pre	  = ave_viB;
		ave_iiB_pre	  = ave_iiB;

//== End PV2 controller==========================================================================================================


//== PV3 controller =============================================================================================================

	if ((mode_C == VRM) && (ecan_receive_done == 1))
	{
		vref_C = reference_vol_C - droop_coef_C *io_ave;
		ev_C = vref_C - vo_ave;
	}

	if (mode_C == MPPT)
	{
		ev_C = vref_C - viC_ave;
	}

	iref_C = iref_C_pre + (kpv_C1 + kiv_C1 * Ts / 2) * ev_C + (-kpv_C1 + kiv_C1 * Ts / 2) * ev_C_pre;

	//saturation
		if (iref_C < -i_sat)	iref_C = -i_sat;
		if (iref_C > i_sat)		iref_C = i_sat;

	// Power Dispatch
		if (mode_C == PRM)
			{
			i_set_C = reference_power_C*1000/viC_ave;      // =1000/3: 3 channel
			if (i_set_C >=  0.7*i_sat)	i_set_C =  0.7*i_sat;
			if (i_set_C <= -0.7*i_sat)	i_set_C = -0.7*i_sat;
			iref_C = i_set_C;
			}

	//inner loop: current

		ei_C = iref_C - iiC_ave;
		duty_C = duty_C_pre + (kpi_C1 + kii_C1 * Ts / 2) * ei_C + (-kpi_C1 + kii_C1 * Ts / 2) * ei_C_pre;

	//saturation
		if (duty_min_C <= MIN_DUTY_MPPT)	duty_min_C 	= MIN_DUTY_MPPT;
		if (duty_C >= duty_max_C) 			duty_C 		= duty_max_C;
		if (duty_C <= duty_min_C)			duty_C 		= duty_min_C;

	// update variables
		viC_ave_pre = viC_ave;
		vref_C_pre = vref_C;
		ev_C_pre = ev_C;
		ei_C_pre = ei_C;
		iref_C_pre = iref_C;
		duty_C_pre = duty_C;

		PowerPV_C_pre = PowerPV_C;
		ave_viC_pre	  = ave_viC;
		ave_iiC_pre	  = ave_iiC;


}

}

// PWM modulation ====================================================================================================
void PWM_Modulation () {

		duty_HA = period - duty_A - deadtime;				// 0 = off; 375 = ON max

		if (duty_HA <= 0) duty_HA = 0;

		if ((converter == PV_CONVERTER) && (mode_A == MPPT))
		{
			duty_LA = duty_A;
		}
		else
		{
			duty_LA = period - duty_A;
		}


		duty_HB = period - duty_B - deadtime;

		if (duty_HB <= 0) duty_HB = 0;

		if ((converter == PV_CONVERTER) && (mode_B == MPPT))
		{
			duty_LB = duty_B;
		}
		else
		{
			duty_LB = period - duty_B;
		}


		duty_HC = period - duty_C - deadtime;

		if (duty_HC <= 0) duty_HC = 0;

		if ((converter == PV_CONVERTER) && (mode_C == MPPT))
		{
			duty_LC = duty_C;
		}
		else
		{
			duty_LC = period - duty_C;
		}

switch (converter)
{
case BAT_CONVERTER:
{
	if ((stop == 1) || (mode == IDLE) || (mode == ISOLATED))
	{
		duty_HA = 0;								// 375 = off; 0 = ON max
		duty_HB = 0;
		duty_HC = 0;
		duty_LA = period;								// 375 = off; 0 = ON max
		duty_LB = period;
		duty_LC = period;
		CH1_ONOFF_STATUS = 0;
		CH2_ONOFF_STATUS = 0;
		CH3_ONOFF_STATUS = 0;
		CONVERTER_STATUS = 0;
	}
	break;
}
case PV_CONVERTER:
{
	if ((stop_A == 1) || (mode_A == IDLE) || (mode_A == ISOLATED))
	{
		duty_HA = 0;
		duty_LA = period;
		CH1_ONOFF_STATUS = 0;
	}

	if ((stop_B == 1) || (mode_B == IDLE) || (mode_B == ISOLATED))
	{
		duty_HB= 0;
		duty_LB = period;
		CH2_ONOFF_STATUS = 0;
	}

	if ((stop_C == 1) || (mode_C == IDLE) || (mode_C == ISOLATED))
	{
		duty_HC = 0;
		duty_LC = period;
		CH3_ONOFF_STATUS = 0;
	}

	if ((CH1_ONOFF_STATUS == 0) && (CH2_ONOFF_STATUS == 0) && (CH3_ONOFF_STATUS == 0))		CONVERTER_STATUS = 0;

	duty_HA = 0;
	duty_HB = 0;
	duty_HC = 0;
	break;
}
}
}

void MPPT_PO(){

// Channel A
	if (mode_A == MPPT)
	{
		sum_viA = sum_viA + viA_ave;
		sum_iiA = sum_iiA + iiA_ave;
		k++;
		if (k >= (MPPT_ADC_COUNT - 1))
		{
			k = 0;
			ave_viA = sum_viA / MPPT_ADC_COUNT;
			ave_iiA = sum_iiA / MPPT_ADC_COUNT;
			sum_viA = 0;
			sum_iiA = 0;

			PowerPV_A = ave_viA * ave_iiA;


			test_powerPVA = PowerPV_A - PowerPV_A_pre;
			test_viA 	  = ave_viA - ave_viA_pre;

			if ((PowerPV_A - PowerPV_A_pre) < -deltaP)
			{
				epsilon_A 	= -epsilon_A;
				vref_A		= vref_A_pre + epsilon_A;
			}
			else if ((PowerPV_A - PowerPV_A_pre) > deltaP)
			{
				vref_A		= vref_A_pre + epsilon_A;
			}
			else
			{
				vref_A		= vref_A_pre;
			}
			if (vref_A <= vPV_min)			vref_A = vPV_min;   //PV reference voltage saturation
			if (vref_A >= vPV_max)			vref_A = vPV_max;


		}

	}


if (mode_B == MPPT)
{
	sum_viB = sum_viB + viB_ave;
	sum_iiB = sum_iiB + iiB_ave;
	k1++;
	if (k1 >= (MPPT_ADC_COUNT - 1))
	{
		k1 = 0;
		ave_viB = sum_viB / MPPT_ADC_COUNT;
		ave_iiB = sum_iiB / MPPT_ADC_COUNT;
		sum_viB = 0;
		sum_iiB = 0;

		PowerPV_B = ave_viB * ave_iiB;


		test_powerPVB = PowerPV_B - PowerPV_B_pre;
		test_viB 	  = ave_viB - ave_viB_pre;


		if ((PowerPV_B - PowerPV_B_pre) < -deltaP)
		{
			epsilon_B = -epsilon_B;
			vref_B= vref_B_pre + epsilon_B;
		}
		else if ((PowerPV_B - PowerPV_B_pre) > deltaP)
		{
			vref_B= vref_B_pre + epsilon_B;
		}
		else
		{
			vref_B= vref_B_pre;
		}
		if (vref_B <= vPV_min)			vref_B = vPV_min;   //PV reference voltage saturation
		if (vref_B >= vPV_max)			vref_B = vPV_max;

	}
}

// Channel C

if (mode_C == MPPT)
{
	sum_viC = sum_viC + viC_ave;
	sum_iiC = sum_iiC + iiC_ave;
	k2++;
	if (k2 >= (MPPT_ADC_COUNT - 1))
	{
		k2 = 0;
		ave_viC = sum_viC / MPPT_ADC_COUNT;
		ave_iiC = sum_iiC / MPPT_ADC_COUNT;
		sum_viC = 0;
		sum_iiC = 0;

		PowerPV_C = ave_viC * ave_iiC;


		test_powerPVC = PowerPV_C - PowerPV_C_pre;
		test_viC 	  = ave_viC - ave_viC_pre;


		if ((PowerPV_C - PowerPV_C_pre) < -deltaP)
		{
			epsilon_C = -epsilon_C;
			vref_C= vref_C_pre + epsilon_C;
		}
		else if ((PowerPV_C - PowerPV_C_pre) > deltaP)
		{
			vref_C= vref_C_pre + epsilon_C;
		}
		else
		{
			vref_C= vref_C_pre;
		}
		if (vref_C <= vPV_min)			vref_C = vPV_min;   //PV reference voltage saturation
		if (vref_C >= vPV_max)			vref_C = vPV_max;

	}
}

}

// MPPT end
//====================================================================================================================

float LowPassFilter(float32 PreOut, float32 Input, float32 CutFre)  // Y(k)=[Y(k-1)+2*PI*fcut*Ts*X(k)]/(1+2*PI*fcut*Ts)
{
  float OutputFilterInc;
  float OutputFilter;

  OutputFilterInc = 2 * 3.14 * CutFre * Ts;
  OutputFilter = (Input * OutputFilterInc + PreOut) / (1.0 + OutputFilterInc);

  return OutputFilter;
}




