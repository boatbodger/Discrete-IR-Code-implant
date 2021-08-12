/* YouView controller implant */
// PIC16F15313 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
/*
Device pin usage
pin	port	usage
1	Vdd		+5V
2	RA5		(not used)
3 	RA4		Infra Red in/out
4	nMCLR	Programming
5 	RA2		Video detect in (active low)
6	RA1		Prog Clock
7	RA0		Prog Data
8	Vss		GND

Program Function:
Listen for I/R codes on the incoming line
If we receive one of our home-defined codes, act accordingly

Timer setup:






*/

#include <xc.h>
//Typedefs to allow modern integer styles to be used with explicit bit lengths to save my poor old brain
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef __uint24 uint24_t;

/* Project #defines */
#define nVideoActive PORTAbits.RA2
#define TRIS_nVideo TRISAbits.TRISA2
#define TRIS_IR TRISAbits.TRISA4
#define IRInput PORTAbits.RA4
#define IROutput LATAbits.LATA4
#define IROUT_ON   TRIS_IR = 0
#define IROUT_OFF  TRIS_IR = 1

//Next two are the non-inverted byte of the command.  
#define POWERTOGGLE 0x00
#define POWEROFF 0x32
#define POWERON 0x22
#define YOUVIEW 0x70

//Humax device address
#define HUMAXADDHI 0x00
#define HUMAXADDLO 0x08

//I/R Timing values in timer ticks which are units of 38kHz/4
#define ttStart	(0x156/4)
#define ttStartMin ((ttStart*9)/10)
#define ttStartMax ((ttStart*11)/10)

#define ttStOnce (0xAA/4)
#define ttStOnceMin ((ttStOnce*9)/10)
#define ttStOnceMax ((ttStOnce*11)/10)

#define ttStRept (0x55/4)
#define ttStReptMin ((ttStRept*9)/10)
#define ttStReptMax ((ttStRept*11)/10)

#define ttBit 	(0x16/4)
#define ttBitMin	(ttBit-2)
#define ttBitMax	(ttBit+2)

#define ttOne 	(0x40/4)
#define ttOneMin	((ttOne*9)/10)
#define ttOneMax	((ttOne*11)/10)

#define ttZero	ttBit
#define ttZeroMin	ttBitMin
#define ttZeroMax ttBitMax

#define ttThresh ((ttZero+ttOne)/2)

#define ttGuardOnce (0x3e7/4)
#define ttGuardRep (0xe38/4)
/* The error guard timer is set to a quarter second */
#define ttGuardErr (9500/4)	
/* Delay from receiving a discrete power command to sending our translation */
#define ttTxDelay (9500/2)  /*0.5 sec*/

#define typeONCE 1
#define typeREPT 2

/* Local prototypes */
void initIRRx();
void sendIRCmd(uint8_t command);
void enableIOC();
void disableIOC();

/* Variables */
//Transmit
uint8_t TxBuf[4];
uint8_t TxIndex;
uint8_t TxMask;
uint16_t TxTtCount;
//Receive
uint8_t RxBuf[4];
uint8_t adHi,adLo,cmd,invcmd;
uint8_t rxComplete;             //Flag to say that a new 'receive' is complete
uint8_t rxType;                 //either typeONCE or typeREPT
uint16_t rxGuard;              //If non-zero, we're waiting for the guard period after a receive
uint16_t rxTicks;
uint16_t tickCount;			//free-running Count of timer interrupts for diagnostic porpoises

uint8_t lastErr;

enum {
    rxsIDLE,				//0 Waiting for AGC burst to start
    rxsSTART,				//1 Waiting for end of AGC burst
    rxsTYPE,				//2 Waiting for data burst to find out if it is a 'once' or 'repeat' code
    rxsBIT,					//3 Waiting for data pulse to end at one bit time
    rxsDATA,				//4 Timing the silence either go back to BIT or wrap up, set guard timer and go to guard
    rxsGUARD,				//5 Waiting out the guard time (no edges expected, but if we get any, ignore them.
}rxState;

enum {
	txsIDLE,
	txsSTART,
	txsSTART_ON,
	txsIR_ON,
	txsIR_OFF,
    txsIR_LAST_ON,
	txsGUARD
}txState;


void main(){
    //Set up input pins for video detection and initialise I/R pin as an input
    ANSELA = 0x00;
    TRIS_nVideo = 1;
    TRIS_IR = 1;   
    //Prepare the I/R pin for output as follows
    ODCONAbits.ODCA2 = 1;   //Don't risk 'fighting' with the target device's I/R receiver
    LATAbits.LATA2 = 0;     //Cue up a '0' to be let out by enabling TRIS_IR when sending
    WPUAbits.WPUA4 = 1;     //Enable weak pull-up on the I/R line so that we can test timing on the bench
    

    /* Timer Set-up
    Initialise interrupt timer at quarter of 38kHz to simplify time counts
    Processor clock is 32MHz
	Our desired timer 'tick' is 9500Hz
	This requires a divisor of 3368
	Timer 0 is 16 bit, so we can manage without the pre-scaler, but will have to preset the timver value on interrupt
	Alternatively, we can pre-scale by 16, and use an 8-bit timer
	Thus Fout = 32000000/16/210 = 9524 Hz, an error of 0.2% which is acceptable
	*/
	T0CON1bits.T0CS = 0b011;    //HFINTOSC, i.e. 32MHz input clock
    T0CON1bits.T0CKPS = 0b0100; //1:16 prescaler
    TMR0H = 210;                //Factor to give about 9.5kHz interrupt rate
    T0CON0 = 0x80;              //Enable timer in 8-bit mode with no post-scaler

    //Set up Interrupt-on-change for the I/R input, noting that we will disable it when sending
    IOCAPbits.IOCAP4 = 1;
    IOCANbits.IOCAN4 = 1;
    //See also enableIOC() and disableIOC())
                
    //Finally, enable all necessary interrupts
    PIE0bits.TMR0IE = 1;    //Timer0 is already running, so interrupts can now start to occur
    //PIE0bits.IOCIE = 1;     //Note - only enable this when we want to start monitoring edges
    INTCONbits.PEIE = 1;    //Enable interupts from peripherals
    INTCONbits.GIE = 1;     //Kerpow!
    //Loop, waiting for an I/R code to become available
    initIRRx();
    while(1){
        if (rxComplete){
            adHi = RxBuf[0]; adLo=RxBuf[1];cmd=RxBuf[2]; invcmd=RxBuf[3];
        	rxComplete = 0;
            if ((adHi == HUMAXADDHI) && (adLo == HUMAXADDLO) && (cmd == (invcmd ^ 0xFF))) { //Then it is for us, and it is a syntactically correct command
                adHi += adLo;   //Dummy statement to allow a breakpoint
                switch(cmd){
                    case POWEROFF:
                        adHi -= adLo;
                    	if (nVideoActive == 0){
                    		sendIRCmd(POWERTOGGLE);
                    	}
                        
                        break;
                    case YOUVIEW:    
                    case POWERON:
                        adLo -= adHi;
                        if (nVideoActive == 1){
                        	sendIRCmd(POWERTOGGLE);
                        }                   
                        break;
                    default:
                        //Allow all other commands to pass un-noticed
                        ;
                }               
            }
        }
    }   
}

//Enable IOC must clear the flags, and then enable the interrupt
void enableIOC(){
    IOCAF = 0;
    PIR0bits.IOCIF = 1;
}

void disableIOC(){
    PIR0bits.IOCIF = 0;
}

void initIRRx(){
    adHi=adLo=cmd=invcmd=0;
    
}

void sendIRCmd(uint8_t txCmd){
	TxBuf[0] = HUMAXADDHI;
	TxBuf[1] = HUMAXADDLO;
	TxBuf[2] = txCmd;
	TxBuf[3] = txCmd ^ 0xFF;
	TxIndex = 0;	//Send MSByte of address first
	TxMask = 0x80;	//Send MSBit first
	while(rxGuard)	//Decremented by timer interrupt if it was running
		;			//Wait until back end of incoming char has finished.
	disableIOC();
    TxTtCount = ttTxDelay;
	txState = txsSTART;


}

/*
 Interrupt Service Routine:
This ISR handles both timer interrupts and Interrupt-on-change (IOC) on the infra red.
Transmit:



 
 */
    static uint8_t rxAcc;     //Accumulator for incoming bits
    static uint8_t rxBits;    //Number of bits received
    static uint8_t rxBytes;   //Number of bytes received
void __interrupt() isr(void){

    //Was there a timer interrupt?
    if (PIR0bits.TMR0IF){       //Timeout
        PIR0bits.TMR0IF = 0;
        ++tickCount;            //Free-running counter of interrupt ticks received which is mainly for debugging
        ++rxTicks;              //Receive timer, gets reset at each event
        if (rxGuard){
        	if (--rxGuard == 0){
                rxState = rxsIDLE;
            }	//Timer since last 'bleat' on the line
        }
        //Process transmit side
        switch(txState){
case txsIDLE:
			break;
case txsSTART:
            if (--TxTtCount == 0){
                IROUT_ON;
                TxTtCount = ttStart;
                txState = txsSTART_ON;        
            }
			break;
case txsSTART_ON:
			if (--TxTtCount == 0){
				IROUT_OFF;
				TxTtCount = ttStOnce;
				txState = txsIR_OFF;
			}

			break;
case txsIR_OFF:						//IR is off.  If the timeout has expired, send a one bit-width pulse
			if (--TxTtCount == 0){
				IROUT_ON;
				TxTtCount = ttBit;
				txState = (TxIndex < 4)?txsIR_ON:txsIR_LAST_ON;				
			}

			break;			
case txsIR_ON:
			if (--TxTtCount == 0){
				IROUT_OFF;
				TxTtCount = (TxBuf[TxIndex] & TxMask? ttOne: ttZero);
				TxMask = TxMask >> 1;
				txState = txsIR_OFF;		
				if (TxMask == 0){
					++TxIndex;          //When this reaches 4, the tsxIR_OFF will handle the end condition
					TxMask = 0x80;	    //Start on the next byte
				}
			}
			break;
            
case txsIR_LAST_ON:
            if (--TxTtCount == 0){
                IROUT_OFF;
                TxTtCount = ttGuardOnce;
                txState = txsGUARD;             
            }

            break;  
case txsGUARD:
default:    if (TxTtCount == 0)
                break;
			if (--TxTtCount == 0){
				txState = txsIDLE;
				enableIOC();
			}
            break;
        }
        
    }	//End of timer interrupt
    /*
    Receive processing mainly driven by IOC

	The sequence of events is as follows, starting from idle
	0 Interrupt on arrival of I/R - start counter
	1 Interrupt on loss of I/R - check counter - it needs to be 9ms - abort or clear counter
	2 Interrupt on arrival of I/R - check counter - it needs to be 4.5ms for a "once" or 2.25ms for a "repeat" - abort or clear counter. Cue repeat signal, guard timer to 108-12-6 = 90ms, jump to 5
	3 Interrupt on loss - should be one bit time  - start timer
	4 Interrupt on arrival - time should one or three bit times.  Store the bit.  If it is not the last bit go back to 3, otherwise signal RxComplete, set guard timer to 35msproceed to 5
	5 wait for loss - should be one bit time - signal repeat or result, start guard time 
	6 wait for guard timer to expire.
*/
    
    if (PIR0bits.IOCIF){
    	//Clear the stimulus flags
    	IOCAF = 0;
    	switch(rxState){
            case rxsIDLE:	//IR input has changed when idle - first check that IR is actually on.  If not, we need to error guard
            	if (IRInput != 0){	//Something has gone horribly wrong - go to guard
                    lastErr = 1;
            		rxGuard = ttGuardErr;
            		rxState = rxsGUARD;
            	}
            	else {
            		rxTicks = 0;
            		rxState = rxsSTART;
            	}
            	break;

            case rxsSTART:	//IR was active and has just gone.  Check rxTicks to confirm length
            	if (IRInput != 1){		//Something has gone wrong
                    lastErr = 2;
            		rxGuard = ttGuardErr;
            		rxState = rxsGUARD;
            	}
            	else {
            		if ((rxTicks > ttStartMin) && (rxTicks < ttStartMax)){
            			rxTicks = 0;
            			rxState = rxsTYPE;
            		}
            		else {
                        lastErr = 3;
	            		rxGuard = ttGuardErr;
	            		rxState = rxsGUARD;
            		}

            	}
            	break;

            case rxsTYPE:	//IR was inactive and has just re-started with the pulse for the first data bit.  Check timing to see if this is Once or ttStRept
            	if (IRInput != 0){		//Something has gone wrong
                    lastErr = 4;
            		rxGuard = ttGuardErr;
            		rxState = rxsGUARD;
            	}
            	else {
            		if ((rxTicks > ttStOnceMin) && (rxTicks < ttStOnceMax)){
            			rxType = typeONCE;
            			rxTicks = 0;
            			rxBits = 0;
            			rxAcc = 0;
            			rxBytes = 0;
            			rxState = rxsBIT;
            		}
            		else if ((rxTicks > ttStReptMin) && (rxTicks < ttStReptMax)){
                        lastErr = 5;
	            		rxType = typeREPT;
                        rxGuard = ttGuardErr;
                        rxState = rxsGUARD;
            		}
            		else {	//Unknown timing - probably an alien code
                        lastErr = 6;
	            		rxGuard = ttGuardErr;
	            		rxState = rxsGUARD;
            		}
            	}
            	break;

            case rxsBIT:	//IR was on and has just gone off at the end of the bit pulse
            	if (IRInput != 1){		//Something has gone wrong
                    lastErr = 7;
            		rxGuard = ttGuardErr;
            		rxState = rxsGUARD;
            	}
            	else {
            		if ((rxTicks >= ttBitMin) && (rxTicks <= ttBitMax)){
            			rxTicks = 0;
            			rxState = rxsDATA;
            		}
            		else {
                        lastErr = 8;
	            		rxGuard = ttGuardErr;
	            		rxState = rxsGUARD;
            		}

            	}
            	break;

            case rxsDATA:	//IR was off and has just restarted at the end of the bit time, so we need to find out what the bit value is and advance the TxIndex
            	if (IRInput != 0){		//Something has gone horribly wrong. Just when it was going so well...
                    lastErr = 9;
            		rxGuard = ttGuardErr;
            		rxState = rxsGUARD;
            	}
            	else {
            		if ((rxTicks >= ttZeroMin) && (rxTicks <= ttOneMax)){
            			rxAcc = (rxAcc << 1) + (rxTicks > ttThresh?1:0);
            			if (++rxBits == 8){
                            rxBits = 0;
            				RxBuf[rxBytes] = rxAcc;
            				if (++rxBytes == 4){
                                rxComplete = typeONCE;
                                rxGuard = ttGuardOnce;
                                rxState = rxsGUARD;
            				}
            			}
            			rxType = typeONCE;
            			rxTicks = 0;
            			rxState = rxsBIT;
            		}
            		else if ((rxTicks >= ttStReptMin) && (rxTicks <= ttStReptMax)){
                        lastErr = 10;
	            		rxGuard = ttGuardErr;
	            		rxState = rxsGUARD;
            		}
            		else {	//Unknown timing - probably an alien code
                        lastErr = 11;
	            		rxGuard = ttGuardErr;
	            		rxState = rxsGUARD;
            		}
            	}
            	break;            

            case rxsGUARD:	//There should be no transitions on the I/R line during this time.  If we get any, add 50ms ish to the Guard timer

            	rxGuard += 500;
            default:
                break;

    	}
    }
    
}	/* end of ISR */

/* End of youview.c */
