/**********************************************************************
* 2010 Microchip Technology Inc.
*
* FileName:        ECAN.c
* Dependencies:    ECAN (.h) & other files if applicable, see below
* Processor:       PIC18F66K80 family
* Linker:          MPLINK 4.37+
* Compiler:        C18 3.36+
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all
* ownership and intellectual property rights in the code accompanying
* this message and in all derivatives hereto.  You may use this code,
* and any derivatives created by any person or entity by or on your
* behalf, exclusively with Microchip's proprietary products.  Your
* acceptance and/or use of this code constitutes agreement to the
* terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS". NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
* NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
* CODE, ITS INTERACTION WITH MICROCHIP'S PRODUCTS, COMBINATION WITH
* ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE
* LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR
* BREACH OF STATUTORY DUTY), STRICT LIABILITY, INDEMNITY,
* CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR
* EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER
* CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE
* DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
* CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP
* SPECIFICALLY TO HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify,
* test, certify, or support the code.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author        Date      	Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Manning C.    12/1/2010	First release of source file
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
* Code Tested on:
* PIC18 Explorer Demo Board with PIC18F46K80 (PIC18F66K80 family) controller + ECAN/LIN Daughterboard
*
*
* DESCRIPTION:
* In this example, CPU is initially configured to run from external
* secondary osc and then clock switching is initiated to run from
* Internal FRC.
*********************************************************************/




/*********************************************************************
*
*                            Includes
*
*********************************************************************/
#include <p18f2580.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>
#include "ECAN.h"


/*********************************************************************
*
*                             Defines
*
*********************************************************************/
// ECAN bitrate define, only can choose one rate
#define F_ECAN_100    0                 // 1 set ECAN module on 100Kbps
#define F_ECAN_125    0                 // 1 set ECAN module on 125Kbps
#define F_ECAN_500    0                 // 1 set ECAN module on 500Kbps
#define F_ECAN_1000   1                 // 1 set ECAN module on 1Mbps

/*********************************************************************
*
*                            Global Variables
*
*********************************************************************/
unsigned char temp_EIDH;
unsigned char temp_EIDL;
unsigned char temp_SIDH;
unsigned char temp_SIDL;
unsigned char temp_DLC;
unsigned char temp_D0;
unsigned char temp_D1;
unsigned char temp_D2;
unsigned char temp_D3;
unsigned char temp_D4;
unsigned char temp_D5;
unsigned char temp_D6;
unsigned char temp_D7;



/*********************************************************************
*
*                       Configure the CAN Module
*
*********************************************************************/
void InitECAN(void)
{
    CANCON = 0x00;
     // Enter CAN module into config mode
        //REQOP<2:0>=100
    CANCON = 0x80;    //REQOP<2:0>=100
    while(!((CANSTATbits.OPMODE0==0) && (CANSTATbits.OPMODE1==0) && (CANSTATbits.OPMODE2==1)));

     //Enter CAN module into Mode 0
     //ECANCON = 0x40;
    ECANCONbits.MDSEL0 = 0x00;
    ECANCONbits.MDSEL1 = 0x00;
   // BSEL0 = 0xF8;
//    PIE3bits.TXB0IE =1;
//    PIE3bits.TXB1IE =1;
//    PIE3bits.TXB2IE =1;
//
//    BIE0bits.B0IE=1;

    // Initialize CAN Timing
    if (F_ECAN_100==1)
    {
        //  100 Kbps @ 64MHz
        BRGCON1 = 0x93; //0001 1111     //SJW=3TQ     BRP  19
        BRGCON2 = 0xB8; //1011 1000     //SEG2PHTS 1    sampled once  PS1=8TQ  PropagationT 1TQ
        BRGCON3 = 0x05; //0000 0101     //PS2  6TQ
    }
    else if (F_ECAN_125==1)
    {
        //  125 Kbps @ 64MHz
        BRGCON1 = 0x8F; //0000 0111     //SJW=3TQ     BRP  15
        BRGCON2 = 0x90; //1011 1000     //SEG2PHTS 1    sampled once  PS1=8TQ  PropagationT 1TQ
        BRGCON3 = 0x05; //0000 0101     //PS2  6TQ
    }
    else if (F_ECAN_500==1)
    {
        //  500 Kbps @ 64MHz
        BRGCON1 = 0x83; //0000 0111     //SJW=3TQ     BRP  3
        BRGCON2 = 0xB8; //1011 1000     //SEG2PHTS 1    sampled once  PS1=8TQ  PropagationT 1TQ
        BRGCON3 = 0x05; //0000 0101     //PS2  6TQ
    }
    else if (F_ECAN_1000==1)
    {
        BRGCON1 = 0x81; //1000 0001     //SJW=3TQ     BRP  1
        BRGCON2 = 0x90;//0xB8; //1011 1000     //SEG2PHTS 1    sampled once  PS1=8TQ =>5  PropagationT 1TQ
        BRGCON3 = 0x00;//0x05; //0000 0101    //PS2  6TQ => 1
    }
    else
    {
        //  100 Kbps @ 64MHz
        BRGCON1 = 0x93; //0001 1111     //SJW=3TQ     BRP  31
        BRGCON2 = 0xB8; //1010 0000     //SEG2PHTS 1    sampled once  PS1=8TQ  PropagationT 1TQ
        BRGCON3 = 0x05; //0000 0010     //PS2  6TQ
    }

    // Initialize Receive Masks
    //  The first mask is used that accepts all SIDs and no EIDs
    RXM0EIDH = 0x00;    //
    RXM0EIDL = 0x00;
    RXM0SIDH = 0xFF;    // Standard ID FILTER
    RXM0SIDL = 0xE0;

    //  The second mask is used to ignore all SIDs and EIDs
    RXM1EIDH = 0x00;    // 0's for EID and SID
    RXM1EIDL = 0x00;
    RXM1SIDH = 0xFF;
    RXM1SIDL = 0xE0;

    // Enable Filters
    //  Only using two filters
    RXFCON0 = 0x00;     //Disable all
    RXFCON1 = 0x00;     //Disable all

    // Initialize Receive Filters
    //  Filter 0 = 0x196
    //  Filter 1 = 0x19E

    RXF0EIDH = 0x00;
    RXF0EIDL = 0x00;
    RXF0SIDH = 0x32;
    RXF0SIDL = 0xC0;

    RXF2EIDH = 0x00;
    RXF2EIDL = 0x00;
    RXF2SIDH = 0x33;
    RXF2SIDL = 0xC0;


    // Enter CAN module into normal mode
    //    CANCON  |= 0x40;
    //    CANCON &= 0x5F;
    CANCONbits.REQOP0 = 0;
    CANCONbits.REQOP1 = 0;
    CANCONbits.REQOP2 = 0;
    //while(!((CANSTAT & 0b11100000)==0x40));
    //CANCON = 0x40;

    while(!((CANSTATbits.OPMODE0==0) && (CANSTATbits.OPMODE1==0) && (CANSTATbits.OPMODE2==0) ) );

    // Set Receive Mode for buffers
    RXB0CON = 0x00;
    RXB1CON = 0x00;

}

/*********************************************************************
*
*                Check the buffers, if it have message
*
*********************************************************************/
unsigned char ECAN_Receive(void)
{
   // putsUSART(mensagem);
    unsigned char RXMsgFlag;

    RXMsgFlag = 0x00;

    if (RXB0CONbits.RXFUL) //CheckRXB0
    {
        temp_EIDH = RXB0EIDH;
        temp_EIDL = RXB0EIDL;
        temp_SIDH = RXB0SIDH;
        temp_SIDL = RXB0SIDL;
        temp_DLC = RXB0DLC;
        temp_D0 = RXB0D0;
        temp_D1 = RXB0D1;
        temp_D2 = RXB0D2;
        temp_D3 = RXB0D3;
        temp_D4 = RXB0D4;
        temp_D5 = RXB0D5;
        temp_D6 = RXB0D6;
        temp_D7 = RXB0D7;
        RXB0CONbits.RXFUL = 0;
        RXMsgFlag = 0x01;
    }
    else if (RXB1CONbits.RXFUL) //CheckRXB1
    {
        temp_EIDH = RXB1EIDH;
        temp_EIDL = RXB1EIDL;
        temp_SIDH = RXB1SIDH;
        temp_SIDL = RXB1SIDL;
        temp_DLC = RXB1DLC;
        temp_D0 = RXB1D0;
        temp_D1 = RXB1D1;
        temp_D2 = RXB1D2;
        temp_D3 = RXB1D3;
        temp_D4 = RXB1D4;
        temp_D5 = RXB1D5;
        temp_D6 = RXB1D6;
        temp_D7 = RXB1D7;
        RXB1CONbits.RXFUL = 0;
        //RXMsgFlag = 0x01;
    }
    else if (B0CONbits.RXFUL) //CheckB0
    {
        temp_EIDH = B0EIDH;
        temp_EIDL = B0EIDL;
        temp_SIDH = B0SIDH;
        temp_SIDL = B0SIDL;
        temp_DLC = B0DLC;
        temp_D0 = B0D0;
        temp_D1 = B0D1;
        temp_D2 = B0D2;
        temp_D3 = B0D3;
        temp_D4 = B0D4;
        temp_D5 = B0D5;
        temp_D6 = B0D6;
        temp_D7 = B0D7;

        B0CONbits.RXFUL = 0;
       // RXMsgFlag = 0x01;
    }

    if  (RXMsgFlag == 0x01)
    {
        RXMsgFlag = 0x00;
        PIR3bits.RXB1IF = 0; //A CAN Receive Buffer has received a new message
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}



/*********************************************************************
*
*                      Transmit Sample Mesage
*
*********************************************************************/
void ECAN_Transmit(double data, int dLc, int sIDh0, int sIDl0 )
//data to send; ID standard high zero, ID ...low...,length of data in bytes
{
    TXB0EIDH = 0x00;
    TXB0EIDL = 0x00;

    //0x35E    0110 1011 110
    TXB0SIDH = sIDh0;//0x32;
    TXB0SIDL = sIDl0;// 0xC0;

    TXB0DLC = dLc;

    TXB0D0 = 0;
    TXB0D1 = 0;//0xCC;
    TXB0D2 = 0;
    TXB0D3 = 0;
    TXB0D4 = 0;//0xCC;
    TXB0D5 = 0;
    TXB0D6 = 0;
    TXB0D7 = 0;//0xCC;


//
//    TXB0D0 = data & 0xFF;//data[0];//0xA0;
//    TXB0D1 = (data >> 8) & 0xFF;//data[1];//0xCC;
//    TXB0D2 = (data >> 16) & 0xFF;//data[2];
//    TXB0D3 = (data >> 24) & 0xFF;//data[3];
//    TXB0D4 = (data >> 4) & 0xFF;//data[4];
//    TXB0D5 = (data >> 5) & 0xFF;//data[5];
//    TXB0D6 = (data >> 6) & 0xFF;//data[6];
//    TXB0D7 = (data >> 7) & 0xFF;//data[7];
//TXB0D0 = 10;
//    TXB1EIDH = 0x00;
//    TXB1EIDL = 0x00;
//
//    //0x35E    0110 1011 110
//    TXB1SIDH = sIDh0;//0x32;
//    TXB1SIDL = sIDl0;// 0xC0;
//
//    TXB1DLC = dLc;
//    TXB1D0 = data[0];//0xA0;
//    TXB1D1 = data[1];//0xCC;
//    TXB1D2 = data[2];
//    TXB1D3 = data[3];
//    TXB1D4 = data[4];
//    TXB1D5 = data[5];
//    TXB1D6 = data[6];
//    TXB1D7 = data[7];
//
//    TXB2EIDH = 0x00;
//    TXB2EIDL = 0x00;
//
//    //0x35E    0110 1011 110
//    TXB2SIDH = sIDh0;//0x32;
//    TXB2SIDL = sIDl0;// 0xC0;
//
//    TXB2DLC = dLc;
//    TXB2D0 = data[0];//0xA0;
//    TXB2D1 = data[1];//0xCC;
//    TXB2D2 = data[2];
//    TXB2D3 = data[3];
//    TXB2D4 = data[4];
//    TXB2D5 = data[5];
//    TXB2D6 = data[6];
//    TXB2D7 = data[7];




    TXB0CONbits.TXREQ = 1; //Set the buffer to transmit

    while(TXB0CONbits.TXREQ);

}

