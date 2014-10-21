/*
 * File:   main.c
 * Author: Daniel Diegues
 * Descrição:
 * TCC PIC 18F2580
 *
 * Created on November 8, 2013, 12:48 PM
 */
/* TODO
 * 1.
 *
 */

//#define __18F2580   //evita bug de ficar c/ marcas vermelhas

#include <xc.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "ECAN.h"
#include "gpsclass.h"
//#include <delays.h>
//#include <plib/adc.h>
#include <plib/usart.h>
//#include <plib/pwm.h>
//#include <plib/timers.h>
//#include <plib/portb.h>
//#include <math.h>


//------------------------------------------------------------------------------
#pragma config OSC  = HS//IRCIO67
#pragma config PWRT = ON
#pragma config WDT  = OFF, WDTPS = 1
#pragma config LVP  = OFF
#pragma config DEBUG = OFF
#pragma config MCLRE = ON
#pragma config CP0  = OFF,CP1 = OFF,CP2 = OFF,CP3 = OFF,CPB = OFF,CPD = OFF
#pragma config WRT0 = OFF,WRT1 = OFF,WRT2 = OFF,WRT3 = OFF,WRTB = OFF,WRTC = OFF,WRTD = OFF
#pragma config IESO = ON

//------------------------------------------------------------------------------
// Definicoes
//------------------------------------------------------------------------------

#define _XTAL_FREQ 20000000
#define BAUDRATE 9600
#define BRG_VAL ((_XTAL_FREQ/BAUDRATE)/64)-1 // for high baud use 16 instaed 64

#define linha1() envia_byte_lcd(0,0x80);
#define linha2() envia_byte_lcd(0,0xC0);
#define basetempo 0xFFEE //0xFEDA//83us //0xFF25 //0xFFFF menor tempo possível 20us 0xFF25 para aprox.100us
#define basetempo3 0xFFF0
#define UPPER

#define STRLEN 60

//#define AUSART_V1
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//GPS Receiving and Parse Variables
//------------------------------------------------------------------------------

volatile unsigned char temp;            //character RX
//volatile unsigned char indexRx;
//volatile unsigned char indexStd = 0;
//volatile unsigned char bufferGGA[75];
//volatile unsigned char bufferGSA[75];
//volatile unsigned char bufferGSV[75];
//volatile unsigned char bufferRMC[75];
//volatile unsigned char bufferVTG[75];
//volatile unsigned char bufferRxTemp[75];
//volatile unsigned char bufferRxTempStd[6];

struct flag_Bits
{
    int flagRxNew : 1;
    int flagRxIni : 1;

}flagBits;


//DEBUG UNION
//union parse_CANTXRX
//{
//    double frame[2];
//    unsigned char dadosCAN[8];
//
//}TxCANRxParse;
//
//union parse_CANTXRX1
//{
//    double frame[2];
//    unsigned char dadosCAN[8];
//
//}TxCANRxParse1;
//
////typedef	struct reg_GPS{
////				volatile unsigned char bufferGGA[70];
////				volatile unsigned char bufferGSA[70];
////				volatile unsigned char bufferGSV[70];
////				volatile unsigned char bufferRMC[70];
////				volatile unsigned char bufferVTG[70];
////
////				volatile unsigned char bufferRxTemp[70];
////
////				unsigned int index;
////
////		}GPSregister;
////
////
////
////
////	GPSregister gpsreg;
////
////
////
////        int get_message(char charRx,GPSregister *gpsReg)
////	{
////		    if(temp!='*')         				//not final of the message
////            {
////                gpsReg->bufferRxTemp = charRx;  //save Rx data initial of string
////                indexRx++;
////                return 0;
////            }
////            else
////            {
////                bufferRxTemp[indexRx] = temp;   //put terminator in the end of message
////
////                if(!strncmp(bufferRxTemp,"$GPGGA",6))
////                   strncpy(bufferGGA, bufferRxTemp,(indexRx+1));
////                if(!strncmp(bufferRxTemp,"$GPGSA",6))
////                   strncpy(bufferGSA, bufferRxTemp,(indexRx+1));
////                if(!strncmp(bufferRxTemp,"$GPGSV",6))
////                   strncpy(bufferGSV, bufferRxTemp,(indexRx+1));
////                if(!strncmp(bufferRxTemp,"$GPRMC",6))
////                   strncpy(bufferRMC, bufferRxTemp,(indexRx+1));
////                if(!strncmp(bufferRxTemp,"$GPVTG",6))
////                   strncpy(bufferVTG, bufferRxTemp,(indexRx+1));
////				indexRx = 0;
////                return 1;
////
////            }
////	}


unsigned int matchStd = 0;    
unsigned int ctrlData = 0;     //control which data is parsing
unsigned char flagNew  = 0;     //flag to new character received
unsigned char flagEnd  = 0;     //flag to end of correct gps mensage
unsigned char cntEnd   = 0;     //cont to finals char after*

unsigned int indexMatch = 0;    //control match with the standard choosen

unsigned int maxLen = 0 ;
unsigned int indexMatch1 = 0;




char standard1[7]   =   "RMC";

//unsigned int indexStd = 0;
char std[6];
char standard[7]   =   "GGA";
char UTC[12];
char latitude[12];
unsigned char coordNS[3];
char longitude[12] ;
char coordEW[]  =   "0";
char altitude[4];
char numSat[3];
char HDOP[3];

char course[5];
char hourUTC[2];
unsigned int hourUTCc[10];
unsigned int latitudeNum[8];
unsigned int longitudeNum[8];
unsigned int altitudeNum[8];
unsigned int numSatNum[8];
unsigned int HDOPNum[8];
unsigned int courseNum[8];

//char mensageGPS[65]= 0;


//  1.  GGA          Global Positioning System Fix Data
//  2.  123519       Fix taken at 12:35:19 UTC
//  3.  4807.038,N   Latitude 48 deg 07.038' N
//  4.  N or S
//  5.  01131.000,E  Longitude 11 deg 31.000' E
//  6.  E or W
//  7.  1            Fix quality: 0 = invalid,1 = GPS fix (SPS), 2 = DGPS fix, 3 = PPS fix, 4 = Real Time Kinematic
//			          5 = Float RTK, 6 = estimated (dead reckoning) (2.3 feature), 7 = Manual input mode, 8 = Simulation mode
//  8.  08           Number of satellites being tracked
//  9. 0.9          Horizontal dilution of position
// 10. 545.4,M      Altitude, Meters, above mean sea level
// 11. 46.9,M       Height of geoid (mean sea level) above WGS84 ellipsoid
// 12. (empty field) time in seconds since last DGPS update
// 13. (empty field) DGPS station ID number
// 14. *47          the checksum data, always begins with *

//------------------------------------------------------------------------------

unsigned char mensagem[30] = "OLA MUNDO\n\r";
char mensagem1[50];
char mensagem2[20] = "ok\n\r";
unsigned char mensagem3[5];


GPSregister gpsReg;
GPSutc gpsUTC;
GPSpos gpsPos;

//int cnt = 0;
int flag =0;
int cntflag =0;

int flagNovo = 0;
int cnt = 0;
// Protótipo da Funções (Cabeçalho)

void ConfigPIC(void);

void InitializeUSART(void);


void USART_putc(unsigned char c)
{
    while (!TXSTAbits.TRMT); // wait until transmit shift register is empty
    TXREG = c;               // write character to TXREG and start transmission
}

void USART_puts(unsigned char *s)
{
    while (*s)
    {
        USART_putc(*s);     // send character pointed to by s
        s++;                // increase pointer location to the next character
    }
}

void ParseGPS()
{
    //is necessary global ctrlVal, indexRx, temp and the destination data strings

//    switch (ctrlData)
//    {
//        case 0:
//            //standard[indexRx] = temp;
//            indexRx ++;
//        break;
//        case 1:
//            if(matchStd==1)
//                UTC[indexRx] = temp;
//            if(matchStd==2);
//
//            indexRx ++;
//        break;
//        case 2:
//            if(matchStd==1)
//                latitude[indexRx] = temp;
//            indexRx ++;
//        break;
//        case 3:
//            if(matchStd==1)
//                coordNS[indexRx] = temp;
//            indexRx ++;
//         break;
//         case 4:
//             if(matchStd==1)
//                longitude[indexRx] = temp;
//            indexRx ++;
//         break;
//         case 9:
//          if(matchStd==1)
//            altitude[indexRx] = temp;
//            indexRx ++;
//         break;
//         case 7:
//          if(matchStd==1)
//            numSat[indexRx] = temp;
//            indexRx ++;
//         break;
//         case 8:
//          if(matchStd==1)
//            HDOP[indexRx] = temp;
//          if(matchStd==2)
//              course[indexRx] = temp;
//
//            indexRx ++;
//         break;
//
//        default:
//            indexRx = 0;
//        break;
//
//    }
//    if(ctrlData == 1)
//    {
//        hourUTC[indexRx] = temp;
//        indexRx++;
//    }
}

unsigned char CompStd(unsigned char std, unsigned char lengthStd)
{
    int match;

}


void interrupt Interrupcao() {

    if (PIR1bits.RCIF)         // check if receive interrupt has fired by USART
    {
        temp = RCREG;          // read received character to buffer

        if(temp == '$')
            flagBits.flagRxIni = 1;

        if(flagBits.flagRxIni)
            flagBits.flagRxNew = getGPSmessage(temp,&gpsReg);

        if(flagBits.flagRxNew)
            flagBits.flagRxIni = 0;


//        //getsUSART(mensagem2,1);
//
//        if(matchStd)           // standard matched!!
//        {
//            if(temp!='*')      //not final of the mensage               // && temp!=',')
//            {
//                if(temp == ',') //similar to sscanf for formated string with ','
//                {               //as a separeted
//                  ctrlData++;   //next data from mensage
//                  indexRx = 0;  //reset the index for new string
//                }
//                else
//                  ParseGPS();
//
//            }
//            else               //final
//            {
//               matchStd = 0;   //reset the 'standard matched' to init a new std comparation
//               ctrlData = 0;   //reset for parse the first
//               flagNovo++;
//            }
//        }
//        else
//        {
//             if(temp == standard[indexMatch])//compare with std GPGGA
//                indexMatch++;                //#'s of matched char of the standard(std)
//             else
//                 indexMatch = 0;             //not match reset the compare position
//
//             if(indexMatch == 3)             //maximum matched chars indicate the correct std
//             {
//                indexMatch = 0;
//                 matchStd = 1;
//             }
//
//            if(temp == standard1[indexMatch1])//compare with std GPGGA
//                indexMatch1++;                //#'s of matched char of the standard(std)
//             else
//                 indexMatch1 = 0;             //not match reset the compare position
//
//             if(indexMatch1 == 3)             //maximum matched chars indicate the correct std
//             {
//                indexMatch1 = 0;
//                 matchStd = 2;
//             }
//
//        }

        
//        if(temp == standard[matchStd] && matchStd < 6)
//        {
//          // matchStd++;
//        }
//
//        if(matchStd == 6)
//        {
//            ParseGPS(ctrlData,temp);
//            if (temp == ',')
//            {
//                ctrlData++;
//                indexRx = 0;
//            }
//            if(temp == '*')
//            {
//                flagEnd = 1;
//                indexRx = 0;
//                matchStd = 0;
//                ctrlData = 0;
//            }
//        }



// flagNew = 1;
//
//        if(flagNew)
//        {
//            buffRx[indexRx] =  temp;
//            indexRx++;
//        }

//            flagNew = 0;
//            indexRx = 0;
//            flagEnd = 1;
//        }
//        flagNew = 1;
//        // check if received character is not new line character
//        // and that maximum string length has not been reached
//
//
//        if (temp == std[matchStd] && matchStd < 6) //choose the std on define de var.
//        {
//           matchStd++;
//        }
//
//        else if(!matchStd==6)//temp != std[matchStd])
//        {
//            ctrlData = 0;
//            indexRx  = 0;
//            matchStd = 0;
//            flagNew  = 0;
//        }
//
//        if (temp == ',' && matchStd == 6)
//        {
//            ctrlData ++;
//            indexRx  = 0;
//            flagNew  = 0;
//        }
//
//
//        if (temp == '*' && matchStd == 6)
//        {
//            ctrlData = 0;
//            indexRx  = 0;
//            matchStd = 0;
//            flagNew  = 0;
//            flagEnd  = 1;
//        }




//        if(flag)
//        {
//           USART_putc(t);
//           cntflag = 0;
//           maxLen++;
//        }
//        else
//        {
//            cntflag++;
//            if (cntflag < 6)
//            {
//                maxLen++;
//                USART_putc(t);
//            }
//        }

//        if (rcindex < STRLEN) //(t != '\n') && (rcindex < STRLEN) )
//        {
//            rcbuf[rcindex] = t; // append received character to string
//            rcindex++;          // increment string index
//        }
//        else
//        {
//           rcindex = 0;        // reset string index
//          // USART_puts(rcbuf);  // echo received string
//        }


       PIR1bits.RCIF = 0;      // reset receive interrupt flag
    }


    if(TXB0CONbits.TXBIF)
    {
//         cnt++;
//         if(cnt==1)
//              ECAN_Transmit(longitudeNum,5,0x20,0x80);
//         if(cnt==2)
//            ECAN_Transmit(altitudeNum,3,0x30,0x80); //#define GPS 0x80 #define UTC 0x40
//         if(cnt==3)
//            ECAN_Transmit(hourUTCc,4,0x40,0x80); //#define GPS 0x32 #define UTC 0x40
//         if(cnt==4)
//         {
//            ECAN_Transmit(hourUTCc,4,0x50,0x80); //#define GPS 0x32 #define UTC 0x40
//            cnt=0;
//         }



        TXB0CONbits.TXBIF = 0;
    }


    if(TXB1CONbits.TXBIF)
    {
        TXB1CONbits.TXBIF = 0;
    }


    if(TXB2CONbits.TXBIF)
    {
        TXB2CONbits.TXBIF = 0;
    }

    if (INTCONbits.INT0IF)
    {
        INTCONbits.INT0IF = 0;
    }


    if (INTCONbits.TMR0IF == 1)
    {
        INTCONbits.TMR0IF = 0;
    }

    else if (INTCONbits.TMR0IF || INTCONbits.INT0IF)

    {
        INTCONbits.TMR0IF = 0; //Clear flag
        INTCONbits.INT0IF = 0;

        //...
    }

}



//------------------------------------------------------------------------------

void main() {


    ConfigPIC();
    //InitializeADC();
    //InitializePWM();
    //InitializeTIMER();


//DEBUG UNION
//    TxCANRxParse.frame[0] = -12.3456789;
//    TxCANRxParse.frame[1] = -98.7654321;
//    int i;
//    for(i=0;i < 8;i++)
//    {
//        TxCANRxParse1.dadosCAN[i] = TxCANRxParse.dadosCAN[i];
//    }
//







    //InitializeLCD();
    //inicializa_lcd();
    initGPS(&gpsReg);
    InitializeUSART();
    //InitializeSPI();
    InitECAN();
    sprintf(mensagem1,"Ola\n\r");
   // sprintf(mensagem2,"1234.5678");
    float val = 0 ;
 while (1)
    {



     if(flagBits.flagRxNew)
     {
         
                getGPSutc(&gpsReg,&gpsUTC,NULL);

                getGPSpos(&gpsReg,&gpsPos,NULL);

                sprintf(mensagem1,"\n\r");
                putsUSART(gpsReg.bufferGGA);
                    putsUSART(mensagem1);
                putsUSART(gpsReg.bufferGSA);
                    putsUSART(mensagem1);
                putsUSART(gpsReg.bufferGSV);
                    putsUSART(mensagem1);
                putsUSART(gpsReg.bufferRMC);
                    putsUSART(mensagem1);
                putsUSART(gpsReg.bufferVTG);
                    putsUSART(mensagem1);
                
                sprintf(mensagem1,"UTC:%f.\n\rLat:%x\n\rLong:%x\n\r",gpsUTC.utc_ts,gpsPos.pos_lat,gpsPos.pos_lon);
                putsUSART(mensagem1);
//                sprintf(mensagem1,"Saida1:%f\n\rSaida2:%f\n\r",TxCANRxParse1.frame[0],TxCANRxParse1.frame[1]);
//               putsUSART(mensagem1);

                ECAN_Transmit(gpsUTC.utc_ts,gpsPos.pos_lat,4,0x50,0x80);

                flagBits.flagRxNew = 0;


     }

     

//ECAN_Transmit();

//  val = atof(mensagem2);
//                sprintf(mensagem1,"HORA:%s\n\rLATITUDE:%s\n\r",hourUTC,latitude);
//                putsUSART(mensagem1);

//        if (flagEnd)
//        {
////             sprintf(buffRx,"%s;%s;%s;%s\n\r",standard,hourUTC,latitude,coordNS);
////             putsUSART(buffRx);
//                 if(flagNew)
//                  sprintf(mensagem1,"ok%s\n\r",hourUTC);
//                 else
//                   sprintf(mensagem1,"not%s\n\r",hourUTC);
//
//                putsUSART(mensagem1);
//                flagNew = 0;
//                flagEnd = 0;
//        }
//
               
                
               
                
        

        //sprintf(mensagem2,"%c\n\r",temp);



//    cnt++;
//    if(flagNew)
//       ParseGPS();
//
//     if(flagEnd == 1)
//     {
//         //sprintf(mensagem1,"%s\n\r",standard);
//         //putsUSART(standard);
//        // putsUSART(mensagem);
//         flagEnd = 0;
//     }
//     putsUSART(buffRx);
//     if(flagEnd)
//     {
//            //sprintf(buffRx,"%s\n\r",buffRx);
//
//            flagEnd = 0;
//            //buffRx  = '\0';
//     }
    //putsUSART(mensagem1);

 
     if(flagNovo==2)
     {
        
         //if(cnt==1)
         
         if(!cnt)
         {
    //graus latitude em um byte
            hourUTC[0] = latitude[0];
            hourUTC[1] = latitude[1];
            latitudeNum[3] =  strtod(hourUTC,NULL);
    //minutos latitude em um byte
            hourUTC[0] = latitude[2];
            hourUTC[1] = latitude[3];
            latitudeNum[2] =  strtod(hourUTC,NULL);
    //0.11 minutos latitude em um byte
            hourUTC[0] = latitude[5];
            hourUTC[1] = latitude[6];
            latitudeNum[1] =  strtod(hourUTC,NULL);
    //0.0011 minutos latitude em um byte
            hourUTC[0] = latitude[7];
            hourUTC[1] = latitude[8];
            latitudeNum[0] =  strtod(hourUTC,NULL);

            

    //graus Longitude em um byte
            hourUTC[0] = longitude[0];
            hourUTC[1] = longitude[1];
            longitudeNum[4] =  strtod(hourUTC,NULL);
    //grau Longitude em um byte
            hourUTC[0] = longitude[2];
            hourUTC[1] = 0;
            longitudeNum[3] =  strtod(hourUTC,NULL);
    //minutos Longitude em um byte
            hourUTC[0] = longitude[3];
            hourUTC[1] = longitude[4];
            longitudeNum[2] =  strtod(hourUTC,NULL);
    //0.11 minutos Longitude em um byte
            hourUTC[0] = longitude[6];
            hourUTC[1] = longitude[7];
            longitudeNum[1] =  strtod(hourUTC,NULL);
    //0.0011 minutos Longitude em um byte
            hourUTC[0] = longitude[8];
            hourUTC[1] = longitude[9];
            longitudeNum[0] =  strtod(hourUTC,NULL);

          //  ECAN_Transmit(longitudeNum,5,0x20,0x80); //#define GPS 0x80 #define UTC 0x40

    //altitude em um byte
            hourUTC[0] = altitude[0];
            hourUTC[1] = altitude[1];
            altitudeNum[2] =  strtod(hourUTC,NULL);
    //altitude  em um byte
            hourUTC[0] = altitude[2];
            hourUTC[1] = altitude[3];
            altitudeNum[1] =  strtod(hourUTC,NULL);
    //altitude  em um byte
            hourUTC[0] = altitude[5];
            hourUTC[1] = 0;
            altitudeNum[0] =  strtod(hourUTC,NULL);

     //0.01 segundos em um byte
            hourUTC[0] = UTC[7];
            hourUTC[1] = UTC[8];
            hourUTCc[0] =  strtod(hourUTC,NULL);
     //segundos em um byte
            hourUTC[0] = UTC[4];
            hourUTC[1] = UTC[5];
            hourUTCc[1] = strtod(hourUTC,NULL);
     //minutos em um byte
            hourUTC[0] = UTC[2];
            hourUTC[1] = UTC[3];
            hourUTCc[2] = strtod(hourUTC,NULL);
    //horas em um byte
            hourUTC[0] = UTC[0];
            hourUTC[1] = UTC[1];
            hourUTCc[3] =  strtod(hourUTC,NULL);

    //numero de satelites
            hourUTC[0] = numSat[0];
            hourUTC[1] = numSat[1];
            numSatNum[0] =  strtod(hourUTC,NULL);
    //HDOP 0.1
            hourUTC[0] = HDOP[0];
            hourUTC[1] = 0;
            HDOPNum[1] =  strtod(hourUTC,NULL);
    //HDOP 1.0
            hourUTC[0] = HDOP[2];
            hourUTC[1] = 0;
            HDOPNum[0] =  strtod(hourUTC,NULL);
    //coruse 110.0
            hourUTC[0] = course[0];
            hourUTC[1] = course[1];
            courseNum[2] =  strtod(hourUTC,NULL);
    //coruse 001.0
            hourUTC[0] = course[2];
            hourUTC[1] = 0;
            courseNum[1] =  strtod(hourUTC,NULL);
    //coruse 000.1
            hourUTC[0] = course[4];
            hourUTC[1] = 0;
            courseNum[0] =  strtod(hourUTC,NULL);
         }

             //#define GPS 0x32 #define UTC 0x40
#define soma 450
         cnt++;
         {
//         if(cnt==1)
//            ECAN_Transmit(latitudeNum,8,0x10,0x80); //#define GPS 0x80 #define UTC 0x40
//         if(cnt==soma)
//            ECAN_Transmit(longitudeNum,8,0x20,0x80);
//         if(cnt==soma+soma)
//            ECAN_Transmit(altitudeNum,8,0x30,0x80); //#define GPS 0x80 #define UTC 0x40
//         if(cnt==soma+soma+soma)
//            ECAN_Transmit(hourUTCc,8,0x40,0x80); //#define GPS 0x32 #define UTC 0x40
//         if(cnt==soma+soma+soma+soma)
//            ECAN_Transmit(courseNum,8,0x50,0x80); //#define GPS 0x32 #define UTC 0x40
//         if(cnt==soma+soma+soma+soma+soma)
//            ECAN_Transmit(numSatNum,8,0x60,0x80); //#define GPS 0x32 #define UTC 0x40
//         if(cnt==soma+soma+soma+soma+soma+soma+soma)
//         {
//            ECAN_Transmit(HDOPNum,8,0x70,0x80); //#define GPS 0x32 #define UTC 0x40
//            cnt=0;
//            flagNovo = 0;
         }
     }
          //        if (1)
//        {
//                ECAN_Transmit(hourUTC[5],hourUTC[4]);
//                //sprintf(mensagem1,"\n\r");
//                //putsUSART(mensagem1);
//        }
//
//
//        if(ECAN_Receive())
//        {
//            sprintf(mensagem1,"foi:%u;%s\n\r",temp_D0,hourUTC);
//            //putsUSART(mensagem1);
//
//        }
//        else
//        {
//            sprintf(mensagem1,"not\n\r");
//            //putsUSART(mensagem1);
//        }

       // Heartbeat();

        // Delay for one millisecond to debounce pushbutton
       // Delay(ONE_MS);
    }

}
//----------------------------------------------------------------------------
//------------------------------------------------------------------------------

void InitializeTIMER(void) {



    OpenTimer0(TIMER_INT_ON & // Habilita interrupcao
            T0_16BIT & // Operacao de 16 bits
            T0_SOURCE_INT & //Seleciona a fonte de clock interna
            T0_EDGE_RISE &
            T0_PS_1_16); // Preescaler 1:256
    WriteTimer0(basetempo); //(0xFFC5);       // Escreve o valor no TIMER


    OpenTimer2(TIMER_INT_OFF & // Desabilita a interrupção do Timer 2
            T2_PS_1_1); // Prescaler igual a 16 (1:16)






    ////    PORTDbits.RD0 = 0;
    //    OpenTimer3 (TIMER_INT_OFF & // Desabilita interrupcao
    //        T3_16BIT_RW &           // Operacao de 16bits
    //        T3_SOURCE_EXT &         // Seleciona a fonte de clock externa
    //        T3_PS_1_1 &             // Preescaler 1:1
    //        T3_SYNC_EXT_OFF) ;      // Desabilita sincronismo de clock externa
    //    n_med = ReadTimer3();       // Retorna o valor do TIMER
    //    WriteTimer3 (0x000);        // Escreve o valor no TIMER

}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void InitializeLCD(void) {
    DelayPORXLCD();
    OpenXLCD(FOUR_BIT & LINES_5X7);
    DelayPORXLCD();
    WriteCmdXLCD(DON & CURSOR_OFF & BLINK_OFF);
    while (BusyXLCD());
}
//------------------------------------------------------------------------------7

void DelayFor18TCY(void) {
    __delay_us(20); //10

}
//------------------------------------------------------------------------------

void DelayPORXLCD(void) // minimum 15ms
{
    //    __delay_ms(20);//20


}
//------------------------------------------------------------------------------

void DelayXLCD(void) // minimum 5ms
{
    //    __delay_ms(20);//10

}
//------------------------------------------------------------------------------

void InitializeADC(void) {
    TRISAbits.TRISA0 = 1;

    OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_4_TAD,
            ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
            0b1100);



}
//------------------------------------------------------------------------------

int getADC(char chsel) {
    int adc;
    SetChanADC(chsel);
    ConvertADC(); // Start conversion
    while (BusyADC()); // Wait for ADC conversion
    adc = ReadADC(); // Read result and put in temp
    //CloseADC(); // Disable A/D converter
    return adc;
}
//----------------------------------------------------------------------------

void InitializeUSART(void) {

    OpenUSART(USART_TX_INT_OFF &
            USART_RX_INT_ON &
            USART_ASYNCH_MODE &
            USART_EIGHT_BIT &
            USART_CONT_RX &
            USART_BRGH_LOW, BRG_VAL);

}
//----------------------------------------------------------------------------

void InitializePWM(void) {


}
//------------------------------------------------------------------------------

void ConfigPIC(void) {

    INTCONbits.TMR0IE = 1; // Habilita interrupcao por estouro do contador do TIMER 0
    INTCONbits.INT0E = 1; //Habilita interrupcao EXTERNA

    INTCON2bits.TMR0IP = 1; // 1 - Alta prioridade , 0   Baixa prioridade
    INTCONbits.TMR0IF = 0; // Registro TMR0   1 se estourou , 0 nao estourou (Deve ser limpo por software )
    INTCON2bits.INTEDG0 = 0;
    INTCON2bits.RBIP = 1;


    INTCONbits.GIEH = 1; // Habilita as interrupcoes de alta prioridade
    INTCONbits.PEIE = 1; // enable peripheral interrupts.
    INTCONbits.GIE = 1; // enable interrupts

    TRISCbits.RC6 = 0; //TX pin set as output
    TRISCbits.RC7 = 1; //RX pin set as input
    // Set the internal oscillator to 32MHz
//    OSCCON = 0x72;


    OSCCONbits.IRCF = 0b111;
    OSCTUNEbits.PLLEN = 0;

    //OSCTUNE = 0x2F;
    // Initialize global variables to 0
  

    // Initialize I/O to be digital, with PORTD (LEDs) as outputs and PORTB as inputs (pushbutton)
    ADCON0 = ADCON1 = 0x00; //PARA USAR PORTB COMO I/O PABDEN = 1
    //LATC = 0x00;
    TRISCbits.RC1 = 0x1;
    TRISB = 0x00;
    TRISBbits.RB3 = 1;
    // Initialize CAN module

    //InitECAN();
    TRISCbits.RC6 = 0; //TX pin set as output
    TRISCbits.RC7 = 1; //RX pin set as input
    flagBits.flagRxNew = 0;
   

}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


