/******************************************************************************
            USB Custom Demo, Host

This file provides the main entry point to the Microchip USB Custom
Host demo.  This demo shows how a PIC24F system could be used to
act as the host, controlling a USB device running the Microchip Custom
Device demo.

******************************************************************************/

/******************************************************************************
* Filename:        main.c
* Dependancies:    USB Host Driver with Generic Client Driver
* Processor:       PIC24F256GB1xx
* Hardware:        Explorer 16 with USB PICtail Plus
* Compiler:        C30 v2.01/C32 v0.00.18
* Company:         Microchip Technology, Inc.

Software License Agreement

The software supplied herewith by Microchip Technology Incorporated
(the ìCompanyÅE for its PICmicroÆ Microcontroller is intended and
supplied to you, the Companyís customer, for use solely and
exclusively on Microchip PICmicro Microcontroller products. The
software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws. All rights are reserved.
Any use in violation of the foregoing restrictions may subject the
user to criminal sanctions under applicable laws, as well as to
civil liability for the breach of the terms and conditions of this
license.

THIS SOFTWARE IS PROVIDED IN AN ìAS ISÅECONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.


*******************************************************************************/

#include <stdlib.h>
#define USE_AND_OR /* To enable AND_OR mask setting */
#include<ports.h>
#include<pps.h>
#include<timer.h>
#include<outcompare.h>

#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "usb_config.h"
#include "USB/usb.h"
#include "USB/usb_host_generic.h"
#include "ctimer.h"

#include "lwbt/phybusif.h"
#include "lwbt/lwbt_memp.h"
#include "lwbt/hci.h"
#include "lwbt/l2cap.h"
#include "lwbt/sdp.h"
#include "lwbt/rfcomm.h"
#include "lwip/memp.h"
#include "lwip/mem.h"
#include "lwip/sys.h"
#include "lwip/stats.h"

#define DEBUG_MODE

// *****************************************************************************
// *****************************************************************************
// Configuration Bits
// *****************************************************************************
// *****************************************************************************

#ifdef __C30__
    // Configuration Bit settings  for an Explorer 16 with USB PICtail Plus
    //      Primary Oscillator:             HS
    //      Internal USB 3.3v Regulator:    Disabled
    //      IOLOCK:                         Set Once
    //      Primary Oscillator Output:      Digital I/O
    //      Clock Switching and Monitor:    Both disabled
    //      Oscillator:                     Primary with PLL
    //      USB 96MHz PLL Prescale:         Divide by 2
    //      Internal/External Switch Over:  Enabled
    //      WDT Postscaler:                 1:32768
    //      WDT Prescaler:                  1:128
    //      WDT Window:                     Non-window Mode
    //      Comm Channel:                   EMUC2/EMUD2
    //      Clip on Emulation Mode:         Reset into Operation Mode
    //      Write Protect:                  Disabled
    //      Code Protect:                   Disabled
    //      JTAG Port Enable:               Disabled

    #if defined(__PIC24FJ256GB110__)
        _CONFIG2(FNOSC_PRIPLL & POSCMOD_HS & PLL_96MHZ_ON & PLLDIV_DIV2) // Primary HS OSC with PLL, USBPLL /2
        _CONFIG1(JTAGEN_OFF & FWDTEN_OFF & ICS_PGx2)   // JTAG off, watchdog timer off
    #elif defined(__PIC24FJ64GB004__)
        _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_ON)
        _CONFIG3(WPFP_WPFP0 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
        _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)
    #elif defined(__PIC24FJ64GB002__)
		 _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
		 _CONFIG2(POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_FRCPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
		 _CONFIG3(WPFP_WPFP0 & SOSCSEL_IO & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
		 _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)
    #elif defined(__PIC24FJ256GB106__)
        _CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx1) 
        _CONFIG2( PLL_96MHZ_ON & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV4 & IOL1WAY_ON)
    #elif defined(__PIC24FJ256DA210__) || defined(__PIC24FJ256GB210__)
        _CONFIG1(FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & IOL1WAY_ON & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
    #endif

#elif defined( __PIC32MX__ )

    #pragma config UPLLEN   = ON            // USB PLL Enabled
    #pragma config FPLLMUL  = MUL_15        // PLL Multiplier
    #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
    #pragma config FPLLIDIV = DIV_2         // PLL Input Divider
    #pragma config FPLLODIV = DIV_1         // PLL Output Divider
    #pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
    #pragma config FWDTEN   = OFF           // Watchdog Timer
    #pragma config WDTPS    = PS1           // Watchdog Timer Postscale
    #pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
    #pragma config OSCIOFNC = OFF           // CLKO Enable
    #pragma config POSCMOD  = HS            // Primary Oscillator
    #pragma config IESO     = OFF           // Internal/External Switch-over
    #pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
    #pragma config FNOSC    = PRIPLL        // Oscillator Selection
    #pragma config CP       = OFF           // Code Protect
    #pragma config BWP      = OFF           // Boot Flash Write Protect
    #pragma config PWP      = OFF           // Program Flash Write Protect
    #pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
    #pragma config DEBUG    = ON            // Background Debugger Enable

#else

    #error Cannot define configuration bits.

#endif

// Application States
typedef enum
{
    BT_INITIALIZE = 0,                // Initialize the app when a device is attached
    BT_STATE_IDLE,                    // Inactive State
	BT_STATE_PROCESS,

    BT_STATE_ERROR                    // An error has occured

} BT_STATE;

typedef struct
{
    BYTE		Initialized;			// Address of the device on the USB
	BT_STATE	State;
} BTCLIENTDATA;

u16_t tcount = 0;
BTCLIENTDATA btClientData;
BYTE deviceAddress;

#define DATA_PACKET_LENGTH  64

// *****************************************************************************
// *****************************************************************************
// Macros
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Global Variables
// *****************************************************************************
// *****************************************************************************

unsigned char buf1[64];
unsigned char buf2[64];

struct phybusif_cb *cb;

extern void bt_spp_start();
extern void bt_spp_tmr();
//******************************************************************************
//******************************************************************************
// Local Routines
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        InitializeSystem
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Returns:         TRUE if successful, FALSE if not.
 *
 * Side Effects:    See below
 *
 * Overview:        This routine initializes the processor and peripheral,
 *                  setting clock speeds and enabling any required
 *                  features.
 *************************************************************************/

BOOL InitializeSystem ( void )
{
    #if defined( __PIC24FJ256GB110__ )
        // Configure U2RX - put on pin 49 (RP10)
        RPINR19bits.U2RXR = 10;

        // Configure U2TX - put on pin 50 (RP17)
        RPOR8bits.RP17R = 5;

        OSCCON = 0x3302;    // Enable secondary oscillator
        CLKDIV = 0x0000;    // Set PLL prescaler (1:1)

        TRISA = 0x0000;
        TRISD = 0x00C0;

   #elif defined( __PIC24FJ256GB106__ )
		// Configure U2RX - put on pin 17 (RP8)
		RPINR19bits.U2RXR = 8;
		// Configure U2TX - put on pin 16 (RP7)
		RPOR3bits.RP7R = 5;

//        OSCCON = 0x3302;    // Enable secondary oscillator
        CLKDIV = 0x0000;    // Set PLL prescaler (1:1)

   #elif defined(__PIC24FJ64GB004__)
	//On the PIC24FJ64GB004 Family of USB microcontrollers, the PLL will not power up and be enabled
	//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
	//This allows the device to power up at a lower initial operating frequency, which can be
	//advantageous when powered from a source which is not gauranteed to be adequate for 32MHz
	//operation.  On these devices, user firmware needs to manually set the CLKDIV<PLLEN> bit to
	//power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        CLKDIVbits.PLLEN = 1;
        while(pll_startup_counter--);
    }
   #elif defined(__PIC24FJ64GB002__)
		//On the PIC24FJ64GB004 Family of USB microcontrollers, the PLL will not power up and be enabled
		//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
		//This allows the device to power up at a lower initial operating frequency, which can be
		//advantageous when powered from a source which is not gauranteed to be adequate for 32MHz
		//operation.  On these devices, user firmware needs to manually set the CLKDIV<PLLEN> bit to
		//power up the PLL.
	    {
	        unsigned int pll_startup_counter = 600;
	        CLKDIVbits.PLLEN = 1;
	        while(pll_startup_counter--);
	    }

		AD1PCFG = 0xffff;
	    CLKDIV = 0x0000;    // Set PLL prescaler (1:1)

		// Configure U2RX - put on pin 17 (RP8)
		RPINR19bits.U2RXR = 8;
		// Configure U2TX - put on pin 16 (RP7)
		RPOR3bits.RP7R = 5;

    #elif defined(__PIC32MX__)
        {
            int  value;
    
            value = SYSTEMConfigWaitStatesAndPB( GetSystemClock() );
    
            // Enable the cache for the best performance
            CheKseg0CacheOn();
    
            INTEnableSystemMultiVectoredInt();
    
            value = OSCCON;
            while (!(value & 0x00000020))
            {
                value = OSCCON;    // Wait for PLL lock to stabilize
            }
        }
    #endif

    // Init UART
    UART2Init();

    return TRUE;
} // InitializeSystem


/*************************************************************************
 * Function:        CheckForNewAttach
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          deviceAddress (global)
 *                  Updates the device address when an attach is found.
 *
 * Returns:         TRUE if a new device has been attached.  FALSE,
 *                  otherwise.
 *
 * Side Effects:    Prints attach message
 *
 * Overview:        This routine checks to see if a new device has been
 *                  attached.  If it has, it records the address.
 *************************************************************************/

BOOL CheckForNewAttach ( void )
{
    // Try to get the device address, if we don't have one.
    if (deviceAddress == 0)
    {
        GENERIC_DEVICE_ID DevID;

        DevID.vid   = 0x0A12;
        DevID.pid   = 0x0001;
        #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
            DevID.serialNumberLength = 0;
            DevID.serialNumber = NULL;
        #endif

        if (USBHostGenericGetDeviceAddress(&DevID))
        {
            deviceAddress = DevID.deviceAddress;
            UART2PrintString( "Generic demo device attached - polled, deviceAddress=" );
            UART2PutDec( deviceAddress );
            UART2PrintString( "\r\n" );
            return TRUE;
        }
    }

    return FALSE;

} // CheckForNewAttach
void bt_init ( void )
{
	sys_init();
	mem_init();
	memp_init();
	pbuf_init();
	UART2PrintString("mem mgmt initialized\r\n");
	lwbt_memp_init();

	phybusif_init("");

	cb = malloc(sizeof(struct phybusif_cb));
	phybusif_reset(cb);

	if(hci_init() != ERR_OK) {
		UART2PrintString("HCI initialization failed!\r\n");
		return;
	}
	l2cap_init();
	sdp_init();
	rfcomm_init();
	UART2PrintString("Bluetooth initialized.\r\n");

	bt_spp_start();
	UART2PrintString("Applications started.\r\n");
}

void BTClientTasks(void)
{
	BYTE deviceStatus;

	// Make sure weÅfre in an initialized state.
	if (btClientData.Initialized != TRUE)
		return;

	// Check device status.
	deviceStatus= USBHostDeviceStatus(deviceAddress);

	// Make sure our device hasnÅft been disconnected.
	if ( deviceStatus != USB_DEVICE_ATTACHED )
	{
		btClientData.Initialized = FALSE;
		btClientData.State = BT_STATE_IDLE;
		return;
	}

	// Perform state-specific tasks.
	switch (btClientData.State)
	{
		case BT_INITIALIZE:
			bt_init();
			tcount = 0;
			btClientData.State = BT_STATE_IDLE;
		break;
		case BT_STATE_IDLE:
	        if (!USBHostGenericRx1IsBusy(deviceAddress) )
			{
				USBHostGenericRead1(deviceAddress,buf1,DATA_PACKET_LENGTH);
			}
	        if (!USBHostGenericRx2IsBusy(deviceAddress) )
			{
				USBHostGenericRead2(deviceAddress,buf2,DATA_PACKET_LENGTH);
			}
		break;
		case BT_STATE_PROCESS:

		break;

		default:// invalid state!
		btClientData.State = BT_STATE_IDLE;
		break;
	}
	if(tcount > 1000)
	{
		l2cap_tmr();
		rfcomm_tmr();
		bt_spp_tmr();
		tcount = 0;
	}
	tcount++;
}

//******************************************************************************
//******************************************************************************
// USB Support Functions
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        USB_ApplicationEventHandler
 *
 * Preconditions:   The USB must be initialized.
 *
 * Input:           event       Identifies the bus event that occured
 *
 *                  data        Pointer to event-specific data
 *
 *                  size        Size of the event-specific data
 *
 * Output:          deviceAddress (global)
 *                  Updates device address when an attach or detach occurs.
 *
 *                  DemoState (global)
 *                  Updates the demo state as appropriate when events occur.
 *
 * Returns:         TRUE if the event was handled, FALSE if not
 *
 * Side Effects:    Event-specific actions have been taken.
 *
 * Overview:        This routine is called by the Host layer or client
 *                  driver to notify the application of events that occur.
 *                  If the event is recognized, it is handled and the
 *                  routine returns TRUE.  Otherwise, it is ignored (or
 *                  just "sniffed" and the routine returns FALSE.
 *************************************************************************/

BOOL USB_ApplicationEventHandler ( BYTE address, USB_EVENT event, void *data, DWORD size )
{
    #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
        BYTE i;
    #endif
	#ifdef DEBUG_MODE
		int data_num;
    #endif

    // Handle specific events.
    switch (event)
    {
        case EVENT_GENERIC_ATTACH:
            if (size == sizeof(GENERIC_DEVICE_ID))
            {
                deviceAddress   = ((GENERIC_DEVICE_ID *)data)->deviceAddress;
                UART2PrintString( "Generic demo device attached - event, deviceAddress=" );
                UART2PutDec( deviceAddress );
                UART2PrintString( "\r\n" );
                #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
                    for (i=1; i<((GENERIC_DEVICE_ID *)data)->serialNumberLength; i++)
                    {
                        UART2PutChar( ((GENERIC_DEVICE_ID *)data)->serialNumber[i] );
                    }
                #endif
                UART2PrintString( "\r\n" );

				btClientData.Initialized = TRUE;
				btClientData.State = BT_INITIALIZE;

                return TRUE;
            }
            break;

        case EVENT_GENERIC_DETACH:
            deviceAddress   = 0;
            UART2PrintString( "Generic demo device detached - event\r\n" );
            return TRUE;

        case EVENT_GENERIC_TX2_DONE:           // The main state machine will poll the driver.
//           UART2PrintString( "TX2_DONE\r\n" );
            return TRUE;
        case EVENT_GENERIC_RX2_DONE:
			if(*(DWORD*)data != 0)
			{
				#ifdef DEBUG_MODE
				UART2PrintString( "HCI:R2: " );
				for(data_num=0;data_num<*(DWORD*)data;data_num++)
	      	          {UART2PutHex(buf2[data_num]);UART2PutChar(' ');}
				UART2PrintString( "\r\n" );
				#endif

				phybusif_input_acl(cb,buf2,*(DWORD*)data);
			}

			btClientData.State = BT_STATE_IDLE;

            return TRUE;
        case EVENT_GENERIC_RX1_DONE:
			if(*(DWORD*)data != 0)
			{
				#ifdef DEBUG_MODE
				UART2PrintString( "HCI:R1: " );
				for(data_num=0;data_num<*(DWORD*)data;data_num++)
	      	          {UART2PutHex(buf1[data_num]);UART2PutChar(' ');}
				UART2PrintString( "\r\n" );
				#endif

				phybusif_input_event(cb,buf1,*(DWORD*)data);
			}

			btClientData.State = BT_STATE_IDLE;

            return TRUE;

        case EVENT_VBUS_REQUEST_POWER:
            // We'll let anything attach.
            return TRUE;

        case EVENT_VBUS_RELEASE_POWER:
            // We aren't keeping track of power.
            return TRUE;

        case EVENT_HUB_ATTACH:
            UART2PrintString( "\r\n***** USB Error - hubs are not supported *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSUPPORTED_DEVICE:
            UART2PrintString( "\r\n***** USB Error - device is not supported *****\r\n" );
            return TRUE;
            break;

        case EVENT_CANNOT_ENUMERATE:
            UART2PrintString( "\r\n***** USB Error - cannot enumerate device *****\r\n" );
            return TRUE;
            break;

        case EVENT_CLIENT_INIT_ERROR:
            UART2PrintString( "\r\n***** USB Error - client driver initialization error *****\r\n" );
            return TRUE;
            break;

        case EVENT_OUT_OF_MEMORY:
            UART2PrintString( "\r\n***** USB Error - out of heap memory *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSPECIFIED_ERROR:   // This should never be generated.
            UART2PrintString( "\r\n***** USB Error - unspecified *****\r\n" );
            return TRUE;
            break;

        case EVENT_SUSPEND:
        case EVENT_DETACH:
        case EVENT_RESUME:
        case EVENT_BUS_ERROR:
            return TRUE;
            break;

        default:
            break;
    }

    return FALSE;

} // USB_ApplicationEventHandler

//******************************************************************************
//******************************************************************************
// Main
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        main
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Returns:         Never exits
 *
 * Side Effects:    Runs the application
 *
 * Overview:        This is the USB Custom Demo Application's main entry
 *                  point.
 *************************************************************************/

int main ( void )
{
    // Initialize the processor and peripherals.
    if ( InitializeSystem() != TRUE )
    {
        UART2PrintString( "\r\n\r\nCould not initialize USB Custom Demo App - system.  Halting.\r\n\r\n" );
        while (1);
    }
    if ( USBHostInit(0) == TRUE )
    {
        UART2PrintString( "\r\n\r\n***** USB Custom Demo App Initialized *****\r\n\r\n" );
    }
    else
    {
        UART2PrintString( "\r\n\r\nCould not initialize USB Custom Demo App - USB.  Halting.\r\n\r\n" );
        while (1);
    }

	btClientData.State = BT_STATE_IDLE;
	btClientData.Initialized = FALSE;

	mPORTAOutputConfig(0x3);

	mPORTAWrite(0x0);

	mPORTBOutputConfig(0x10);

	// OC 1
	PPSOutput(PPS_RP4, PPS_OC1);

	//Enable Interrupt
	SetPriorityIntOC1(4);
	EnableIntOC1;

	OpenTimer2(T2_ON | T2_PS_1_8 ,0xffff); // 
	OpenOC1(OC_IDLE_CON | OC_TIMER2_SRC | OC_PWM_EDGE_ALIGN ,OC_SYNC_TRIG_IN_TMR2,0,0);

	SetDCOC1PWM(0xc00,0);

    // Main Processing Loop
    while (1)
    {
        BTClientTasks();

        // Maintain USB Host State
        USBHostTasks();

		DelayMs(1);
    }

    return 0;

} // main


/*************************************************************************
 * EOF main.c
 */

