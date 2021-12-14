//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define FEETINONEMETER 3.28083989501312

// Only one of these can be uncommented at a time
//#define MATLAB
//#define SIMULINK

#ifdef MATLAB
void matlab_serialRX(serial_t *s, char data);
#endif
#ifdef SIMULINK
void simulink_serialRX(serial_t *s, char data);
#endif


#pragma DATA_SECTION(matlabLock, ".my_vars")
float matlabLock = 0;
float matlabLockshadow = 0;

#pragma DATA_SECTION(matlab_dist, ".my_arrs")
float matlab_dist[1000];

#pragma DATA_SECTION(matlab_angle, ".my_arrs")
float matlab_angle[1000];


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
// __interrupt void SPIB_isr(void);

void serialRXA(serial_t *s, char data);
void serialRXD(serial_t *s, char data);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM2A(float controleffort) ;
void setEPWM2B(float controleffort) ;

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
// uint16_t sampleSize = 0;


//--------------------------------------------------------Use Lidar to find obstacle --------------------------------------------------------------------------
//Lidar variables and structures-----------------------------------------------------------------------------------------------------------------------------------

/* the structure to store data points.
 * distance is the measured distance
 * timestamp is the time that the point is captured
 * rawAngle is directly calculated with the start and end angle
 * cor_angle is calculated based on the correction
 * correction formula can be found at https://www.ydlidar.com/Public/upload/files/2020-04-13/YDLIDAR%20X2%20Development%20Manual.pdf
 */
typedef struct data_point{
    double distance;
    uint32_t timestamp;
    double rawAngle;
    double cor_angle;
}data_pt;

/* the structure to store the distance and time stamp from Lidar
 * distance is the distance of the point in feet
 * timestamp is the time the point is detected, and is the number of CPUTimer 0 being called. CPUTimer 0 is called every 0.5s
 */
typedef struct dis_time{
    double distance;   //in feet
    uint32_t timestamp;
}dis_time;

//variables for reading Lidar data
int16_t state = 0; //the state for reading Lidar data
double end_angle; //the end angle from the packet
double start_angle; //the start angle from the packet
double cal_angle; //the calculated angle
uint16_t sampleSize = 0; //sample size for the receiving packet
int16_t startangleLSB = 0; //lsb of start angle
int16_t endLSB = 0; //lsb of end angle
int16_t position = 0;//position of the place into the data of the packet(to ignore the check code)
int16_t dis_state = 0;//state of distance(lsb or msb)
int16_t dis_count = 0; //count of distance number(should usually be 40)
uint16_t dLSB;//LSB of distance
uint16_t arrayIndex = 0; //index for filling the distance array
uint16_t anglearrayindex = 0; //index for filling the angle array
uint16_t sampleindex = 0;
float cor_end = 0; // make sure it is larger than the start angle
float delta_angle = 0; // cor_end - start

//pts is the array to store raw data points directly from Lidar
data_pt pts[600];

/* arrays to store obstacle points data
 * the index corresponds to the angle of that point
 */
dis_time pingpts[360]; //raw Lidar reading
dis_time pongpts[360]; //raw Lidar reading
double x_ori[360]; //x position as the view of robot, the angle is also in the view of the robot
double y_ori[360]; //y position as the view of robot, the angle is also in the view of the robot
dis_time x_f[360]; //global x position of the point seen by the Lidar
dis_time y_f[360]; //global y position of the point seen by the Lidar

//initial condition, will use motion tracking, no need to initialize
double pose_x; //current x position of the robot car
double pose_y; //current y position of the robot car
double pose_rad; //current angle of the robot car, in rad

//to check if it is a full revolution
float anglesum = 0;
float dist;
int16_t checkfullrevolution = 0;

//pingpong buffer to store data
int16_t pingpongflag = 1;
int16_t UseLIDARping = 0;
int16_t UseLIDARpong = 1;

uint16_t startLidar = 0; //only start Lidar when there's motion tracking data coming in
uint32_t numRXD = 0;

uint32_t stamp90 = 0;

float dist00 = 0;
float dist45 = 0;
float dist90 = 0;
float dist180 = 0;
float dist135 = 0;
float dist270 = 0;

float dist00_1 = 0.0;
float dist45_1 = 0.0;
float dist90_1 = 0.0;
float dist180_1 = 0.0;
float dist135_1 = 0.0;
float dist270_1 = 0.0;
//--------------------------------------------------------End of Lidar initialization-------------------------------------------------------


//Robot Car Motor Global
int16_t temp = 0;
int16_t dummy = 0 ;
int16_t xAccel = 0 ;
int16_t yAccel = 0 ;
int16_t zAccel = 0 ;
int16_t temperature = 0 ;
int16_t xGyro = 0 ;
int16_t yGyro = 0 ;
int16_t zGyro = 0 ;

int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
int16_t pwmCount1 = 0 ;
int16_t pwmCount2 = 1500 ;
int16_t diff1 = 10 ;
int16_t diff2 = 10 ;
float adcVal1 = 0.0 ;
float adcVal2 = 0.0 ;

float xAccelReading = 0 ;
float yAccelReading = 0 ;
float zAccelReading = 0 ;
float xGyroReading = 0 ;
float yGyroReading = 0 ;
float zGyroReading = 0 ;

float angLeft = 0.0 ;
float angRight = 0.0 ;
float disLeft = 0.0 ;
float disRight = 0.0 ;
float uLeft = 5.0;
float uRight = 5.0;
float posLeft_K_1 = 0.0 ;
float posLeft_K = 0.0 ;
float posRight_K_1 = 0.0 ;
float posRight_K = 0.0 ;
float vLeftK = 0.0 ;
float vRightK = 0.0 ;

//ex5
float WR = 0.62;        //Robot width
float Rwh = 0.197;        //Radius wheel
float phiR = 0.0 ;
float thetaAvg = 0.0 ;
float thetaDotAvg = 0.0 ;
float xRDotK = 0.0 ;
float yRDotK = 0.0 ;
float xRDotK_1 = 0.0 ;
float yRDotK_1 = 0.0 ;
float xR = 0.0 ;    //integral of x distance
float yR= 0.0 ;     //integral of y distance
float angLeft_1 = 0.0 ;
float angRight_1 = 0.0 ;
float thetaDotLeft = 0.0 ;
float thetaDotRight = 0.0 ;

//Controller gains
float Kp = 3.0 ;
float Ki = 25.0 ;
float Vref = 0 ;
float e_LeftK = 0.0 ;
float e_RightK = 0.0 ;
float I_LeftK = 0.0 ;
float I_RightK = 0.0 ;
float e_LeftK_1 = 0.0 ;
float e_RightK_1 = 0.0 ;
float I_LeftK_1 = 0.0 ;
float I_RightK_1 = 0.0 ;
float Kp_turn = 3.0 ;
float turn = 0.0 ;
float e_Turn = 0.0 ;

//Random traversal variables
int followRight = 1 ; //Flag to indicate whether or not right wall following should resume/start
int followLeft = 1 ; //Flag to indicate whether or not left wall following should resume/start
int wallRight = 0 ;
int wallLeft = 0 ;
float vWallforward = 0.2 ;
float vWallright = 0.2 ;
float rightRef = 0.95 ;
float thresh1 = 0.75 ;
float obstThresh = 1.0 ;
float selfThresh = 1.0 ;
float allClear = 3.0 ;
// float thresh2 = 1.5 ;
int traverseState = 0 ;
float Kp_frw = 0.1 ;
float Kp_right = -1.0 ;
float Kp_left = 1.0 ;


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LuanchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    //EPWM2A&B Pin Mux
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index

    //Lab 3 EPWM2A & EPWM2B
    EPwm2Regs.TBCTL.bit.CTRMODE = 0x0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 3; //1x = FREE RUN, therefore bits set to b'11 or 3
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; //00 = CLK/1, therefore bits set to b'00 or 0
    EPwm2Regs.TBCTL.bit.PHSEN = 0 ; //00 = No Loading, bits set to b'00 or 0
    EPwm2Regs.TBCTR = 0; //Ensures timer starts at 0
    EPwm2Regs.TBPRD = 0x9C4 ; //To achieve period of 20KHz, CTR needs to count up to 2500 or x9C4
    EPwm2Regs.CMPA.bit.CMPA = 0 ;//Ensures CMPA starts at duty cycle of 0%
    EPwm2Regs.CMPB.bit.CMPB = 0 ;//Ensures CMPA starts at duty cycle of 0%
    EPwm2Regs.AQCTLA.bit.ZRO = 2 ; //10 = Sets pi, therefore set to b'10 or 2
    EPwm2Regs.AQCTLA.bit.CAU = 1 ; //01 = Clears when CMPA is reached (Up Count)
    EPwm2Regs.AQCTLB.bit.ZRO = 2 ; //10 = Sets pi, therefore set to b'10 or 2
    EPwm2Regs.AQCTLB.bit.CBU = 1 ; //01 = Clears when CMPA is reached (Up Count)
    EPwm2Regs.TBPHS.bit.TBPHS = 0 ; //00 = Set phase to zero - to be safe

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    // PieVectTable.SPIB_RX_INT = &SPIB_isr ; // Lab 5 - Point to address of interrupt function for SPI

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
    ConfigCpuTimer(&CpuTimer1, 200, 4000); //Changed to 4ms
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;


#ifdef MATLAB
    init_serial(&SerialA,115200,matlab_serialRX);
#else
    #ifdef SIMULINK

        init_serial(&SerialA,115200,simulink_serialRX);
    #else
        init_serial(&SerialA,115200,serialRXA);
    #endif
#endif


//    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
    init_serial(&SerialD,115200,serialRXD);

// //----------------Exercise 4, call function for SPIB SETUP-------------------------
//     setupSpib();
//-------------------------------------------------------------------------------------

    //Calling initialization code for eQEPs for motor encoder
    init_eQEPs() ;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // // Enable SPIB in the PIE: Group 6 interrupt 3
    // PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UseLIDARping == 1) {
            UseLIDARping = 0;
            dist00 = pingpts[90].distance; //Left Distance
            dist45 = pingpts[45].distance; //Left-diag distance
            dist90 = pingpts[0].distance; //Center distance
            stamp90 = pingpts[0].timestamp;
            dist135 = pingpts[315].timestamp; //Right-diag distance
            dist180 = pingpts[270].distance; //Right distance
            UARTPrint = 1;
        } 
        if (UseLIDARpong == 1) {
            UseLIDARpong = 0;
            dist00 = pongpts[90].distance; //Left Distance
            dist45 = pongpts[45].distance;  //Left distance
            dist90 = pongpts[0].distance; //Center distance
            stamp90 = pongpts[0].timestamp;
            dist135 = pongpts[315].timestamp; //Right-diag distance
            dist180 = pongpts[270].distance; //Right distance
            UARTPrint = 1;
        }

        if (UARTPrint == 1 ) {
#ifndef MATLAB
#ifndef SIMULINK
            serial_printf(&SerialA,"TimeStamp %ld Dist90: %.3f Dist180: %.3f Dist00: %.3f \r\n",stamp90,dist90,dist180,dist00);
#endif
#endif

            UARTPrint = 0;
        }
    }
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;

    DINT;

}

// __interrupt void SPIB_isr(void){
//     GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//     dummy =  SpibRegs.SPIRXBUF; // Ignore first 16 bit value
//     xAccel = SpibRegs.SPIRXBUF; // Next 16 bit value off RX FIFO to get accel x axis
//     yAccel = SpibRegs.SPIRXBUF; //  Next 16 bit value off RX FIFO to get accel y axis
//     zAccel = SpibRegs.SPIRXBUF; //  Next 16 bit value off RX FIFO to get accel z axis
//     temperature = SpibRegs.SPIRXBUF; //  Next 16 bit value off RX FIFO to read temp to ignore
//     xGyro = SpibRegs.SPIRXBUF; // Next 16 bit value off RX FIFO to get gyro x axis
//     yGyro = SpibRegs.SPIRXBUF; //  Next 16 bit value off RX FIFO to get gyro y axis
//     zGyro = SpibRegs.SPIRXBUF; //  Next 16 bit value off RX FIFO to get gyro z axis

//     xAccelReading = (float)xAccel * (4.0/32767.0);
//     yAccelReading = (float)yAccel * (4.0/32767.0);
//     zAccelReading = (float)zAccel * (4.0/32767.0);

//     xGyroReading = (float)xGyro * (250.0/32767.0);
//     yGyroReading = (float)yGyro * (250.0/32767.0);
//     zGyroReading = (float)zGyro * (250.0/32767.0);

//     //Convert spivalue 2 and 3 (ignore 1 since it is garbage data) to ADC values
// //    adcVal1 = (float)spivalue2 * (3.3 / 4095.0);
// //    adcVal2 = (float)spivalue3 * (3.3 / 4095.0);

//     //GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO 9 high to end Slave Select. Now to Scope. Later to deselect DAN28027
//     // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
//     SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
//     SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
//     PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
// }

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    //Get distances
    if (UseLIDARping == 1) {
        UseLIDARping = 0;
        dist00 = pingpts[90].distance; //Left Distance
        dist45 = pingpts[45].distance; //Left-diag distance
        dist90 = pingpts[0].distance; //Center distance
        stamp90 = pingpts[0].timestamp;
        dist135 = pingpts[315].timestamp; //Right-diag distance
        dist180 = pingpts[270].distance; //Right distance
        UARTPrint = 1;
    }
    if (UseLIDARpong == 1) {
        UseLIDARpong = 0;
        dist00 = pongpts[90].distance; //Left Distance
        dist45 = pongpts[45].distance;  //Left distance
        dist90 = pongpts[0].distance; //Center distance
        stamp90 = pongpts[0].timestamp;
        dist135 = pongpts[315].timestamp; //Right-diag distance
        dist180 = pongpts[270].distance; //Right distance
        UARTPrint = 1;
    }

    //eliminate error in readings - prevents random spikes in data
    if (dist90 == 0.0){
        dist90 = dist90_1;
    }
    if (dist180 == 0.0){
        dist180 = dist180_1;
    }
    if (dist45 == 0.0){
        dist45 = dist45_1;
    }
    if (dist135 == 0.0){
        dist135 = dist135_1;
    }
    if (dist00 == 0.0){
        dist00 = dist00_1;
    }

    //Go forward
    // If front obstacle, turn right
    // If can't turn right, turn left,
    // If can't left or right, then go back - reverse gears

    //Traverse like a Roomba
    if(traverseState == 0){
        Vref = vWallforward ;
        turn = 0.0 ;
        //Detect right wall
        if (dist180 <= thresh1){
            wallRight = 1;
            turn = Kp_frw * (thresh1 - dist180) ; //self-correct to stay within certain distance of right wall
        }
        else{
            wallRight = 0;
        }
        //Detect left wall
        if (dist00 < thresh1){
            wallLeft = 1;
            turn = -Kp_frw * (thresh1 - dist00) ; //self-correct to stay within certain distance of left wall
        }
        else{
            wallLeft = 0;
        }
        //Detect front obstacle
        // if ((dist45 < obstThresh) || (dist90 < obstThresh) || (dist180 < obstThresh)){
        if (dist90 < obstThresh){
            traverseState = 1;
        }
    }
    else if(traverseState == 1){
        if (!wallRight){
            //turn right
            Vref = vWallright ;
            turn = Kp_right * (allClear - dist90);
        }
        else if (!wallLeft){
            //turn left
            Vref = vWallright ;
            turn = Kp_left * (allClear - dist90);
        }
        else{
            //reverse
            Vref = -0.1 ;
            turn = 0.0 ;
        }

        if (dist90 > allClear){
            traverseState = 0 ;
        }
    }

    //Right wall following
    // if (followRight == 1){
    //     turn = Kp_right * (rightRef - dist180) ;
    //     Vref = vWallright;
    //     if (dist90 < thresh1){
    //         followRight = 0 ;
    //     }
    // }
    // else{
    //     turn = Kp_front * (thresh2 - dist90) ;
    //     Vref = vWallfront;
    //     if (dist90 > thresh2){
    //         followRight = 1 ;
    //     }
    // }

    //Saturate turn command?

    //Wheel Encoder code
    //Read left and right angle of each wheel in radians
    angLeft = readEncLeft() ;
    angRight = readEncRight() ;

    //Exercise 5 - Position of robot calculation
    phiR = Rwh*((angRight-angLeft)/WR);

    //Ang Velocity calculations
    thetaDotLeft = (angLeft - angLeft_1) / 0.004 ;
    thetaDotRight = (angRight - angRight_1) / 0.004 ; 

    //Avg angle and velocity calculation
    thetaAvg = (angRight + angLeft)*0.5;
    thetaDotAvg = (thetaDotRight + thetaDotLeft)*0.5 ;

    xRDotK = Rwh*thetaDotAvg*cos(phiR);
    yRDotK = Rwh*thetaDotAvg*sin(phiR);

    xR += ((xRDotK + xRDotK_1)*0.5)*0.004;  // Integrate Trapezoidal Rule
    yR += ((yRDotK + yRDotK_1)*0.5)*0.004;
    //Save previous angle states
    angLeft_1 = angLeft;
    angRight_1 = angRight;

    // Save previous x,y velocity
    xRDotK_1 = xRDotK;
    yRDotK_1 = yRDotK;

    //Calculate total distance traversed by each wheel by calculating arc length (angle*radius of wheel)
    disLeft = (angLeft)*(0.19685);
    disRight = (angRight)*(0.19685);

    //Calculate left and right velocities
    posLeft_K_1 = posLeft_K ;
    posLeft_K = disLeft ;
   

    posRight_K_1 = posRight_K ;
    posRight_K = disRight ;

    vLeftK = (posLeft_K - posLeft_K_1) / 0.004 ;
    vRightK = (posRight_K - posRight_K_1) / 0.004 ; 

    //Calculate error between reference velocity and current velocity for each motor
    e_Turn = turn + (vLeftK - vRightK);
    e_LeftK = Vref - vLeftK - (Kp_turn*e_Turn);
    e_RightK = Vref - vRightK + (Kp_turn*e_Turn);


    //Calculate integral error 
    I_LeftK = I_LeftK_1 + ((0.004) * ((e_LeftK + e_LeftK_1)/2.0)) ;
    I_RightK = I_RightK_1 + ((0.004) * ((e_RightK + e_RightK_1)/2.0)) ;

    //Anti-windup controller - if Ik saturates
    if((uLeft >= 10.0) || (uLeft <= -10.0)){
        I_LeftK = I_LeftK_1 ;
    }
    if((uRight >= 10.0) || (uRight <= -10.0)){
        I_RightK = I_RightK_1 ;
    }

    //Calculate new velocity needed to reach reference velocity
    uLeft = (Kp*e_LeftK) + (Ki*I_LeftK) ;
    uRight = (Kp*e_RightK) + (Ki*I_RightK) ;

    //Calling EPWM2A & EPWM2B to spin motors
    setEPWM2A(uRight) ;
    setEPWM2B(-uLeft) ;

    //Save previous states
    e_LeftK_1 = e_LeftK ;
    e_RightK_1 = e_RightK ;
    I_LeftK_1 = I_LeftK ;
    I_RightK_1 = I_RightK ;
    dist00_1 = dist00 ;
    dist45_1 = dist45 ;
    dist90_1 = dist90 ;
    dist135_1 = dist135 ;
    dist180_1 = dist180 ;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 50) == 0) {
        UARTPrint = 1;
    }
}


// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;

}
//----------------------next step: calculate the correction data---------------------------//
// int16_t lidar_data[200];
// int16_t lidar_count = 0;
// int16_t state = 0; //read data
// int16_t sample_size[100]; //sample size array
// float start_angle1[100]; // start angle array
// float start_angle2[100]; // for debug
// float start_angle3[100];  //for debug
// float end_angle1[100]; //end angle array
// float angle_diff[100]; //array that keep track of angle diff

// double end_angle;
// double start_angle;
// double cal_angle;

// float angles[15][40];//record all the angles by calculating with the start/end angle
// uint16_t angle_index = 0;
// float anglescor[15][40]; //the corrected angles

// float distance[15][40]; //distance array
// //int16_t packetcount=-1; // count packet, since increase in the first time entering the loop, should set to -1
// int16_t startangleLSB = 0; //lsb of start angle
// int16_t endLSB = 0; //lsb of end angle
// int16_t position = 0;//position of the
// int16_t dis_state = 0;//state of distance(lsb or msb)
// int16_t dis_count = 0; //count of distance number(should usually be 40)
// uint16_t dLSB;//LSB of distance
// uint16_t arrayIndex = 0; //index for filling the distance array
// uint16_t anglearrayindex = 0; //index for filling the angle array
// uint16_t sampleindex = 0;
// float cor_end = 0; // make sure it is larger than the start angle
// float delta_angle = 0; // cor_end - start

void serialRXD(serial_t *s, char data) {
    numRXD++;
    if(state == 0) { //check 0xaa
        if(data == 0xAA) {
            state = 1;
        } else {
            state = 0;
        }
    } else if (state == 1) {//check 0x55
        if(data == 0x55) {
            state = 2;
        } else {
            state = 0;
        }
    } else if (state == 2) {//check 0x0
        if (data == 0x0) {
            state = 3;
        } else {
            state = 0;
        }
    } else if (state == 3){ //read sample size
        sampleSize = data;
        state = 4;
    } else if (state == 4) {//read starting angle lsb
        startangleLSB = data;
        state = 5;
    } else if (state == 5) {//record starting angle
        start_angle = ((((data<<8)| startangleLSB)>>1)&0x7FFF)/64.0; //get the starting angle
        state = 6;
    } else if (state == 6) { //read end angle
        endLSB = data;
        state = 7;
    } else if (state == 7) {//record end angle
        end_angle = ((((data<<8)| endLSB)>>1)&0x7FFF)/64.0;  //get the ending angle
        //make sure the end angle is greater than the staring angle
        if (end_angle < start_angle) {
            cor_end = end_angle + 360;
        } else {
            cor_end = end_angle;
        }
        //calculate the difference between the start and end angle
        delta_angle = cor_end-start_angle;
        state = 8;
    } else if (state == 8) {//record samples and ignore the check code
        //position is 0 when entering
        if(position > 1) {
            if (dis_state == 0){
                dLSB = data;
                dis_state = 1;
            } else if (dis_state == 1){
                float dist = (((uint16_t)data<<8 | dLSB))/4.0; //calculate the distance
                pts[arrayIndex].distance = dist;
                pts[arrayIndex].timestamp = numTimer0calls;

                //calculate the raw angle
                float raw_angle = (delta_angle)/(sampleSize - 1) * (sampleindex) +start_angle;
                sampleindex = sampleindex+1;
                if(sampleindex == sampleSize) {
                    sampleindex = 0;
                }
                pts[arrayIndex].rawAngle = raw_angle;
                //calculate the calibrated angle
                if(dist == 0){
                    cal_angle = raw_angle;
                } else {
                    //use the correction angle formula from the development manual. Need to minus 90 because the 0 degree of the Lidar is actually at y axis of the robot
                    cal_angle = raw_angle + atan(21.8*(155.3-dist)/(155.3*dist))*57.296-90;
                }
                pts[arrayIndex].cor_angle = cal_angle; //need to consider the 0 degree of the lidar is not the same as the x direction of the robot car
                //use a pingpong buffer to keep the data
                if (pingpongflag == 1) {
                    //Since the Lidar is rotating CW, need to use 360 - angle to get make the angle CCW.
                    //MAKE SURE THE ARRAY INDEX IS NOT OUT OF BOUND BY MOD 360
                    pingpts[(360-((int16_t)(cal_angle + 360.5))%360)%360].distance = dist*FEETINONEMETER*0.001; //in feet
                    pingpts[(360-((int16_t)(cal_angle + 360.5))%360)%360].timestamp = numTimer0calls;
                } else {
                    pongpts[(360-((int16_t)(cal_angle + 360.5))%360)%360].distance = dist*FEETINONEMETER*0.001; // in feet
                    pongpts[(360-((int16_t)(cal_angle + 360.5))%360)%360].timestamp = numTimer0calls;

                }
                dis_count += 1;
                dis_state = 0;
                if (arrayIndex < 599) {
                    arrayIndex += 1;
                }
            }
        }
        position += 1;
        //when done all the readings
        if(position >= sampleSize*2+2) { //the total number of bytes for check code(2) and sample data

            if (checkfullrevolution == 0) {
                checkfullrevolution = 1;
                anglesum = delta_angle;
            } else {
                anglesum += delta_angle;
                if (anglesum > 360) { //only updating data when it goes more than a 360 degrees
                    checkfullrevolution = 0;
                    if (pingpongflag == 1){
                        UseLIDARping = 1;
                        pingpongflag = 0;
                    } else {
                        UseLIDARpong = 1;
                        pingpongflag = 1;
                    }
                }
            }
            position = 0;
            state = 0;//enter state 0 to read new packet
            GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------
//-----------------------Exercise 4 Communicating with the MPU-9250--------------------------------------------------------------------
// void setupSpib(void){ //Call this function in main() somewhere after the DINT; line of code.
//     //Step 1.
//      // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
//     //between each transfer to 0. Also don�t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
//     //66 which are also a part of the SPIB setup.

//      GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
//      GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
//      GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
//      GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
//      GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
//      GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 0xF); //Set GPIO63 pin to SPISIMOB
//      GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 0xF); //Set GPIO64 pin to SPISOMIB
//      GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 0xF);//Set GPIO65 pin to SPICLKB

//      EALLOW;
//      GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
//      GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
//      GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
//      GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
//      GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
//      GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
//      EDIS;

//      SpibRegs.SPICCR.bit.SPISWRESET = 0x0; // Put SPI in Reset
//      SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
//      SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
//      SpibRegs.SPICTL.bit.MASTER_SLAVE = 0x1; // Set to SPI Master
//      SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
//      SpibRegs.SPICTL.bit.TALK = 0x1; // Enable transmission
//      SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
//      SpibRegs.SPICTL.bit.SPIINTENA = 0x0; // Disables the SPI interrupt
//      SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 0x31; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is - Set to decimal 49 or hex 31
//       // 50MHZ. And this setting divides that base clock to create SCLK�s period
//      SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
//      SpibRegs.SPIFFTX.bit.SPIRST = 0x1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
//      SpibRegs.SPIFFTX.bit.SPIFFENA = 0x1; // Enable SPI FIFO enhancements
//      SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
//      SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
//      SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
//      SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
//      SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
//      SpibRegs.SPIFFRX.bit.RXFFIENA = 0x1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
//      SpibRegs.SPIFFCT.bit.TXDLY = 0x00; //Set delay between transmits to 0 spi clocks. Needed by MPU chip
//      SpibRegs.SPICCR.bit.SPISWRESET = 0x1; // Pull the SPI out of reset
//      SpibRegs.SPIFFTX.bit.TXFIFO = 0x1; // Release transmit FIFO from reset.
//      SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
//      SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don�t think this is needed. Need to Test
//      SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below

//      //-----------------------------------------------------------------------------------------------------------------
//      // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
//      // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
//      // some code is given, most you have to fill you yourself.
//      GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
//     // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
//      // To address 00x13 write 0x00
//      SpibRegs.SPITXBUF = 0x1300 ;
//      // To address 00x14 write 0x00
//      // To address 00x15 write 0x00
//      SpibRegs.SPITXBUF = (0x0000 | 0x0000) ;
//      // To address 00x16 write 0x00
//      // To address 00x17 write 0x00
//      SpibRegs.SPITXBUF = (0x0000 | 0x0000) ;
//      // To address 00x18 write 0x00
//      // To address 00x19 write 0x13
//      SpibRegs.SPITXBUF = (0x0000 | 0x0013) ;
//      // To address 00x1A write 0x02 - to configure the gyroscope and temperature sensor's bandwidth and delay
//      // To address 00x1B write 0x00 - to set FChoice_b = b'11 to also configure gyroscope and temperature sensor
//      SpibRegs.SPITXBUF = (0x0200 | 0x0000) ;
//      // To address 00x1C write 0x08 - set full scale at 4g
//      // To address 00x1D write 0x06 - set Accelerometer low pass filter
//      SpibRegs.SPITXBUF = (0x0800 | 0x0006) ;
//      // To address 00x1E write 0x00
//      // To address 00x1F write 0x00
//      SpibRegs.SPITXBUF = (0x0000 | 0x0000) ;
//      // wait for the correct number of 16 bit values to be received into the RX FIFO
//      while(SpibRegs.SPIFFRX.bit.RXFFST != 0x7);
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      //Read additional number of garbage receive values off the RX FIFO to clear out the RX FIFO - 14 total
//      DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

//      // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
//      // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
//      // some code is given, most you have to fill you yourself.
//      GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
//      // Perform the number of needed writes to SPITXBUF to write to all 7 registers
//      // To address 00x23 write 0x00
//      SpibRegs.SPITXBUF = (0x2300 | 0x0000) ;
//      // To address 00x24 write 0x40
//      // To address 00x25 write 0x8C
//      SpibRegs.SPITXBUF = (0x4000 | 0x008C) ;
//      // To address 00x26 write 0x02
//      // To address 00x27 write 0x88
//      SpibRegs.SPITXBUF = (0x0200 | 0x0088) ;
//      // To address 00x28 write 0x0C
//      // To address 00x29 write 0x0A
//      SpibRegs.SPITXBUF = (0x0C00 | 0x000A) ;
//      // wait for the correct number of 16 bit values to be received into the RX FIFO
//      while(SpibRegs.SPIFFRX.bit.RXFFST != 0x4);
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      temp = SpibRegs.SPIRXBUF;
//      //Read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO - 7 Total
//      DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

//      //----------------------------------------------------------------------------------------------------

//    // -------------------------------------------------------------------------------------------------------





//      // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
//     GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//      // Write to address 0x2A the value 0x81
//     SpibRegs.SPITXBUF = (0x2A00 | 0x0081) ;
//      // wait for one byte to be received
//      while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//      temp = SpibRegs.SPIRXBUF;
//      DELAY_US(10);
//      // The Remainder of this code is given to you and you do not need to make any changes.
//      GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//      SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
//      while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//      temp = SpibRegs.SPIRXBUF;
//      DELAY_US(10);
//      GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//      SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
//      while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//      temp = SpibRegs.SPIRXBUF;
//      DELAY_US(10);
//      GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//      SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
//      while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//      temp = SpibRegs.SPIRXBUF;
//      DELAY_US(10);
//      GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//      SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
//      while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//      temp = SpibRegs.SPIRXBUF;
//      DELAY_US(10);
//      GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//      SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
//      while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//      temp = SpibRegs.SPIRXBUF;
//      DELAY_US(10);
//      GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//      SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
//      while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//      GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//      temp = SpibRegs.SPIRXBUF;
//      DELAY_US(10);
//      GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//      SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
//      while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//       GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//       temp = SpibRegs.SPIRXBUF;
//       DELAY_US(10);
//       GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//       SpibRegs.SPITXBUF = (0x7700 | 0x00E9); // 0x7700
//       while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//       GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//       temp = SpibRegs.SPIRXBUF;
//       DELAY_US(10);
//       GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//       SpibRegs.SPITXBUF = (0x7800 | 0x004E); // 0x7800
//       while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//       GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//       temp = SpibRegs.SPIRXBUF;
//       DELAY_US(10);
//       GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//       SpibRegs.SPITXBUF = (0x7A00 | 0x00EC); // 0x7A00
//       while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//       GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//       temp = SpibRegs.SPIRXBUF;
//       DELAY_US(10);
//       GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//       SpibRegs.SPITXBUF = (0x7B00 | 0x00E4); // 0x7B00
//       while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//       GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//       temp = SpibRegs.SPIRXBUF;
//       DELAY_US(10);
//       GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//       SpibRegs.SPITXBUF = (0x7D00 | 0x001A); // 0x7D00
//       while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//       GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//       temp = SpibRegs.SPIRXBUF;
//       DELAY_US(10);
//       GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//       SpibRegs.SPITXBUF = (0x7E00 | 0x0056); // 0x7E00
//       while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
//       GpioDataRegs.GPCSET.bit.GPIO66 = 1;
//       temp = SpibRegs.SPIRXBUF;
//       DELAY_US(50);
//       // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
//       SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
//       SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
//       PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
// }


void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);

    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;

    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B

    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

float readEncLeft(void) {
    int32_t raw = 0;
    float angleLeft = 0.0 ;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    angleLeft = -((float)raw/600.0)*(2*PI);

    return angleLeft;
}

float readEncRight(void) {
    int32_t raw = 0;
    float angleRight = 0.0 ;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    angleRight = ((float)raw/600.0)*(2*PI);
    return angleRight;
}

void setEPWM2A(float controleffort){
    float controlsat = 0.0;
    float dutycycle = 0.0 ;

    if (controleffort > 10){
        controlsat = 10 ;
    }
    else if (controleffort < -10){
        controlsat = -10 ;
    }
    else{
        controlsat = controleffort ;
    }

    dutycycle = (controlsat+10.0)/20.0  ;
    EPwm2Regs.CMPA.bit.CMPA = ((float)EPwm2Regs.TBPRD)*dutycycle;

}

void setEPWM2B(float controleffort){
    //Set two local variables
    float controlsat2 = 0.0;
    float dutycycle2 = 0.0 ;

    //Saturate to keep input within -10/10 range
    if (controleffort > 10){
        controlsat2 = 10 ;
    }
    else if (controleffort < -10){
        controlsat2 = -10 ;
    }
    else{
        controlsat2 = controleffort ;
    }

    //calculate linear duty cycle
    dutycycle2 = (controlsat2+10.0)/20.0  ;
    EPwm2Regs.CMPB.bit.CMPB = ((float)EPwm2Regs.TBPRD)*dutycycle2; //set new servo angle by changing CMPA bit
}

#ifdef MATLAB
#define MAX_VAR_NUM 10

int UARTsensordatatimeouterror = 0; // Initialize timeout error count
int UARTtransmissionerror = 0;          // Initialize transmission error count

int UARTbeginnewdata = 0;
int UARTdatacollect = 0;
char UARTMessageArray[101];
int UARTreceivelength = 0;

char Main_sendingarray = 0;  // Flag to Stop terminal prints when using matlab commands
//  Only way to clear this flag

union mem_add {
    float f;
    long i;
    char c[2];
}memloc;

union ptrmem_add {
    float* f;
    long* i;
    char c[2];
}ptrmemloc;

long* Main_address[MAX_VAR_NUM];
float Main_value[MAX_VAR_NUM];
char Main_SendArray[128];
char Main_SendArray2[128];
float Main_tempf=0;
int Main_i = 0;
int Main_j = 0;
int Main_memcount = 0;

//the code below is used to transmit data to MATLAB
void EchoSerialData(int memcount,char *buffer) {


    char sendmsg[256];
    int i;

    sendmsg[0] = 0x2A; // *
    sendmsg[2] = '0'; // 0
    for (i=3;i<(memcount*8+3);i++) {
        sendmsg[i] = buffer[i-3];
    }
    sendmsg[1] = i;
    serial_send(&SerialA, sendmsg, i);


}

void matlab_serialRX(serial_t *s, char data) {
    if (!UARTbeginnewdata) {// Only TRUE if have not yet begun a message
        if (42 == (unsigned char)data) {// Check for start char

            UARTdatacollect = 0;        // amount of data collected in message set to 0
            UARTbeginnewdata = 1;       // flag to indicate we are collecting a message
            Main_memcount = 0;
            Main_i = 0;
        }
    } else {    // Filling data
        if (0 == UARTdatacollect){
            UARTreceivelength = ((int)data)-1; // set receive length to value of char after start char
            UARTdatacollect++;
        }else if (UARTdatacollect < UARTreceivelength){
            UARTMessageArray[UARTdatacollect-1] = (char) data;
            // If sending out float value(s), save input memory locations and values at those addresses
            if (('0' == UARTMessageArray[0]) &&  (UARTdatacollect > 1)){

                if (Main_i == 0) {
                    ptrmemloc.c[1] = ((UARTMessageArray[UARTdatacollect-1] & 0xFF) << 8);
                }
                if (Main_i == 1) {
                    ptrmemloc.c[1] |= (UARTMessageArray[UARTdatacollect-1] & 0xFF);
                }
                if (Main_i == 2) {
                    ptrmemloc.c[0] = ((UARTMessageArray[UARTdatacollect-1] & 0xFF) << 8);
                }
                if (3 == Main_i){
                    ptrmemloc.c[0] |= (UARTMessageArray[UARTdatacollect-1] & 0xFF);

                    Main_address[Main_memcount]=ptrmemloc.i;
                    Main_value[Main_memcount]=*ptrmemloc.f;

                    Main_i = 0;
                    Main_memcount++;
                }else{
                    Main_i++;
                }
            }
            UARTdatacollect++;
        }
        if (UARTdatacollect == UARTreceivelength){  // If input receive length is reached
            UARTbeginnewdata = 0;   // Reset the flag
            UARTdatacollect = 0;    // Reset the number of chars collected

            // Case '0' : Sending data in endian format (big-endian address, big-endian value)
            if ('0' == UARTMessageArray[0]){
                for (Main_i = 0;Main_i<Main_memcount;Main_i++){
                    ptrmemloc.i=Main_address[Main_i];
                    Main_SendArray[0+8*Main_i]=((ptrmemloc.c[1]>>8)&0xFF);
                    Main_SendArray[1+8*Main_i]=ptrmemloc.c[1]&0xFF;
                    Main_SendArray[2+8*Main_i]=((ptrmemloc.c[0]>>8)&0xFF);
                    Main_SendArray[3+8*Main_i]=ptrmemloc.c[0]&0xFF;
                    memloc.f=Main_value[Main_i];
                    Main_SendArray[4+8*Main_i]=((memloc.c[1]>>8)&0xFF);
                    Main_SendArray[5+8*Main_i]=memloc.c[1]&0xFF;
                    Main_SendArray[6+8*Main_i]=((memloc.c[0]>>8)&0xFF);
                    Main_SendArray[7+8*Main_i]=memloc.c[0]&0xFF;
                }
                EchoSerialData(Main_memcount,Main_SendArray);   // Append header information to send data and transmit
                // Case '1' : Writing float value to memory address (big-endian received address / value)
            }else if ('1' == UARTMessageArray[0]){
                for (Main_i = 0; Main_i < (UARTreceivelength - 2)/8;Main_i++){

                    ptrmemloc.c[1] = ((UARTMessageArray[1+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[1] |= (UARTMessageArray[2+8*Main_i]&0xFF);
                    ptrmemloc.c[0] = ((UARTMessageArray[3+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[0] |= (UARTMessageArray[4+8*Main_i]&0xFF);

                    memloc.c[1] = ((UARTMessageArray[5+8*Main_i]&0xFF)<<8);
                    memloc.c[1] |= (UARTMessageArray[6+8*Main_i]&0xFF);
                    memloc.c[0] = ((UARTMessageArray[7+8*Main_i]&0xFF)<<8);
                    memloc.c[0] |= (UARTMessageArray[8+8*Main_i]&0xFF);

                    *ptrmemloc.i = memloc.i;


                }

                matlabLockshadow = matlabLock;
                // Case '2' : Sending array data in following format [char 1,char2,char3,...]
                // [*,3+input length of array,3 (code for array receiving in Matlab),...
                //      array(0) chars in little-endian, ... , array(memcount) chars in little-endian]
            }else if ('2' == UARTMessageArray[0]){
                Main_sendingarray = 1;
                matlabLock = 1.0;
                matlabLockshadow = matlabLock;
                memloc.c[1] = NULL;
                memloc.c[0] = ((UARTMessageArray[5]&0xFF)<<8);
                memloc.c[0] |= (UARTMessageArray[6]&0xFF);
                Main_memcount = memloc.i;
                ptrmemloc.c[1] = ((UARTMessageArray[1]&0xFF)<<8);
                ptrmemloc.c[1] |= (UARTMessageArray[2]&0xFF);
                ptrmemloc.c[0] = ((UARTMessageArray[3]&0xFF)<<8);
                ptrmemloc.c[0] |= (UARTMessageArray[4]&0xFF);
                Main_SendArray[0]='*';
                Main_SendArray[1]=3+Main_memcount;
                Main_SendArray[2]='3';

                serial_send(&SerialA, Main_SendArray, 3);

                for (Main_i = 0; Main_i < Main_memcount;Main_i++){
                    Main_tempf = *ptrmemloc.f;
                    memloc.f = Main_tempf;
                    Main_SendArray2[0+Main_j*4] =  (memloc.c[0]&0xFF);
                    Main_SendArray2[1+Main_j*4] =  ((memloc.c[0]>>8)&0xFF);
                    Main_SendArray2[2+Main_j*4] =  (memloc.c[1]&0xFF);
                    Main_SendArray2[3+Main_j*4] =  ((memloc.c[1]>>8)&0xFF);
                    memloc.c[1] = ptrmemloc.c[1];
                    memloc.c[0] = ptrmemloc.c[0];
                    memloc.i+=2;  // was plus 4
                    ptrmemloc.c[1]=memloc.c[1];
                    ptrmemloc.c[0]=memloc.c[0];
                    Main_j++;
                    if (32 == Main_j){
                        memcpy(Main_SendArray,Main_SendArray2,128);
                        serial_send(&SerialA, Main_SendArray, 128);
                        Main_j = 0;
                    }
                }
                if (Main_j != 0){
                    serial_send(&SerialA, Main_SendArray2, (Main_memcount%32)*4);
                    Main_j = 0;
                }
                Main_sendingarray = 0;
                // Case '3' : Write float value to memory address (big-endian received address,
                //      little-endian received value)
            }else if ('3' == UARTMessageArray[0]){
                for (Main_i = 0; Main_i < (UARTreceivelength - 2)/8;Main_i++){

                    ptrmemloc.c[1] = ((UARTMessageArray[1+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[1] |= (UARTMessageArray[2+8*Main_i]&0xFF);
                    ptrmemloc.c[0] = ((UARTMessageArray[3+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[0] |= (UARTMessageArray[4+8*Main_i]&0xFF);

                    memloc.c[1] = ((UARTMessageArray[8+8*Main_i]&0xFF)<<8);
                    memloc.c[1] |= (UARTMessageArray[7+8*Main_i]&0xFF);
                    memloc.c[0] = ((UARTMessageArray[6+8*Main_i]&0xFF)<<8);
                    memloc.c[0] |= (UARTMessageArray[5+8*Main_i]&0xFF);

                    *ptrmemloc.i = memloc.i;

                }

                matlabLockshadow = matlabLock;
            }
        }
    }
}
#endif

#ifdef SIMULINK
char SIMU_databyte1 = 0;
int SIMU_Var1_fromSIMU_16bit = 0;
int SIMU_Var2_fromSIMU_16bit = 0;
int SIMU_Var3_fromSIMU_16bit = 0;
int SIMU_Var4_fromSIMU_16bit = 0;
int SIMU_Var5_fromSIMU_16bit = 0;
int SIMU_Var6_fromSIMU_16bit = 0;
int SIMU_Var7_fromSIMU_16bit = 0;

char SIMU_TXrawbytes[12];

long SIMU_Var1_toSIMU_32bit = 0;
long SIMU_Var2_toSIMU_32bit = 0;
long SIMU_Var3_toSIMU_32bit = 0;
long SIMU_Var4_toSIMU_32bit = 0;
long SIMU_Var5_toSIMU_32bit = 0;
long SIMU_Var6_toSIMU_32bit = 0;
long SIMU_Var7_toSIMU_32bit = 0;

long SIMU_Var1_toSIMU_16bit = 0;
long SIMU_Var2_toSIMU_16bit = 0;
long SIMU_Var3_toSIMU_16bit = 0;
long SIMU_Var4_toSIMU_16bit = 0;
long SIMU_Var5_toSIMU_16bit = 0;
long SIMU_Var6_toSIMU_16bit = 0;
long SIMU_Var7_toSIMU_16bit = 0;

int SIMU_beginnewdata = 0;
int SIMU_datacollect = 0;
int SIMU_Tranaction_Type = 0;
int SIMU_checkfirstcommandbyte = 0;

void simulink_serialRX(serial_t *s, char data) {
    if (!SIMU_beginnewdata) {// Only true if have not yet begun a message
        if (SIMU_checkfirstcommandbyte == 1) {
            if (0xFF == (unsigned char)data) {// Check for start 2 bytes command = 32767 becuase assuming command will stay between -10000 and 10000
                SIMU_checkfirstcommandbyte = 0;
            }
        } else {
            SIMU_checkfirstcommandbyte = 1;
            if (0x7F == (unsigned char)data) {// Check for start char

                SIMU_datacollect = 0;       // amount of data collected in message set to 0
                SIMU_beginnewdata = 1;      // flag to indicate we are collecting a message

                SIMU_Tranaction_Type = 2;

                // Coould Start ADC and then ADC interrupt will read ENCs also and then send
                // but that is for Simulink control
                // For Simulink data collection just send most current ADC and ENCs
                // Simulink Sample rate needs to be at least 500HZ but 200Hz probably better

//                SIMU_Var1_toSIMU_16bit = 1000*tilt_value;
//                SIMU_Var2_toSIMU_16bit = 1000*gyro_value;
//                SIMU_Var3_toSIMU_16bit = 1000*thetaR;
//                SIMU_Var4_toSIMU_16bit = 1000*vel_orig;
//                SIMU_Var5_toSIMU_16bit = 1000*u;
//                SIMU_Var6_toSIMU_16bit = 0;//1000*th_value;
//
//                SIMU_TXrawbytes[3] = (char)((SIMU_Var2_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[2] = (char)(SIMU_Var2_toSIMU_16bit & 0xFF);
//                SIMU_TXrawbytes[1] = (char)((SIMU_Var1_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[0] = (char)(SIMU_Var1_toSIMU_16bit & 0xFF);
//
//                SIMU_TXrawbytes[7] = (char)((SIMU_Var4_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[6] = (char)(SIMU_Var4_toSIMU_16bit & 0xFF);
//                SIMU_TXrawbytes[5] = (char)((SIMU_Var3_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[4] = (char)(SIMU_Var3_toSIMU_16bit & 0xFF);
//
//                SIMU_TXrawbytes[11] = (char)((SIMU_Var6_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[10] = (char)(SIMU_Var6_toSIMU_16bit & 0xFF);
//                SIMU_TXrawbytes[9] = (char)((SIMU_Var5_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[8] = (char)(SIMU_Var5_toSIMU_16bit & 0xFF);


                SIMU_Var1_toSIMU_32bit = 10000*readEnc1();
                SIMU_Var2_toSIMU_32bit = 10000*readEnc2();
                SIMU_Var1_toSIMU_16bit = 0;//1000*tilt_value;
                SIMU_Var2_toSIMU_16bit = 0;//1000*gyro_value;
                SIMU_TXrawbytes[3] = (char)((SIMU_Var1_toSIMU_32bit >> 24) & 0xFF);
                SIMU_TXrawbytes[2] = (char)((SIMU_Var1_toSIMU_32bit >> 16) & 0xFF);
                SIMU_TXrawbytes[1] = (char)((SIMU_Var1_toSIMU_32bit >> 8) & 0xFF);
                SIMU_TXrawbytes[0] = (char)((SIMU_Var1_toSIMU_32bit) & 0xFF);

                SIMU_TXrawbytes[7] = (char)((SIMU_Var2_toSIMU_32bit >> 24) & 0xFF);
                SIMU_TXrawbytes[6] = (char)((SIMU_Var2_toSIMU_32bit >> 16) & 0xFF);
                SIMU_TXrawbytes[5] = (char)((SIMU_Var2_toSIMU_32bit >> 8) & 0xFF);
                SIMU_TXrawbytes[4] = (char)((SIMU_Var2_toSIMU_32bit) & 0xFF);

                SIMU_TXrawbytes[8] = (char)(SIMU_Var1_toSIMU_16bit & 0xFF);
                SIMU_TXrawbytes[9] = (char)((SIMU_Var1_toSIMU_16bit >> 8) & 0xFF);
                SIMU_TXrawbytes[10] = (char)(SIMU_Var2_toSIMU_16bit & 0xFF);
                SIMU_TXrawbytes[11] = (char)((SIMU_Var2_toSIMU_16bit >> 8) & 0xFF);

                serial_send(&SerialA,SIMU_TXrawbytes,12);

            }
        }
    } else {    // Filling data
        if (SIMU_Tranaction_Type == 2) {
            if (SIMU_datacollect == 0){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 1){

                SIMU_Var1_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;

                SIMU_datacollect++;
            } else if (SIMU_datacollect == 2){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 3){

                SIMU_Var2_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;

                SIMU_datacollect++;
            } else if (SIMU_datacollect == 4){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 5){

                SIMU_Var3_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 6){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 7){

                SIMU_Var4_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 8){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 9){

                SIMU_Var5_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 10) {
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 11) {
                SIMU_Var6_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 12) {
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 13) {
                SIMU_Var7_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;

                SIMU_beginnewdata = 0;  // Reset the flag
                SIMU_datacollect = 0;   // Reset the number of chars collected
                SIMU_Tranaction_Type = 0;
            }

        }
    }
}
#endif
