/*TODO
 * Ler dinamicamente as mensagens do GPGSV pois variam de 0 a 3 dependendo
 * da qtd de satelites
  */

#include <p18f2580.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>
#include "gpsclass.h"
/**
 *
 * @param gpsReg variavel do tipo GPSregister deve ser criada antes de utilizar a biblioteca
 */
void initGPS(GPSregister *gpsReg)
{
    gpsReg->index = 0;
}


/**
 * @brief Funcao que grava cada character no buffer do padrao especifico.
 * Grava em um buffer temporario .
 * Atualiza todos os buffers permanentes ao mesmo instante, quando recebe o GPVTG.  (!! MUDAR O RETURN !!)
 * @param charRx caracter recebido pela serial
 * @param gpsReg variavel do tipo GPSregister deve ser criada antes de utilizar a biblioteca
 * contem os buffers com os padroes GGA, GSA, GSV, RMC e VTG alem de controles internos
 * @return retorna '1' a cada nova mensagem recebida e gravada no buffer correto cada vez que recebe '*'.
 */

int getGPSmessage(char charRx,GPSregister *gpsReg)
{
    if(charRx!='*')         				//not final of the message
    {
        gpsReg->bufferRxTemp[gpsReg->index] = charRx;  //save Rx data initial of string
        gpsReg->index++;
        return 0;
    }
    else
    {
        gpsReg->bufferRxTemp[gpsReg->index] = charRx;   //put terminator in the end of message

        if(!strncmp(gpsReg->bufferRxTemp,"$GPGGA",6))
           strncpy(gpsReg->bufferGGATemp, gpsReg->bufferRxTemp,(gpsReg->index+1));

        if(!strncmp(gpsReg->bufferRxTemp,"$GPGSA",6))
           strncpy(gpsReg->bufferGSATemp, gpsReg->bufferRxTemp,(gpsReg->index+1));

        if(!strncmp(gpsReg->bufferRxTemp,"$GPGSV",6))
           strncpy(gpsReg->bufferGSVTemp, gpsReg->bufferRxTemp,(gpsReg->index+1));

        if(!strncmp(gpsReg->bufferRxTemp,"$GPRMC",6))
           strncpy(gpsReg->bufferRMCTemp, gpsReg->bufferRxTemp,(gpsReg->index+1));

        if(!strncmp(gpsReg->bufferRxTemp,"$GPVTG",6))
        {
           strncpy(gpsReg->bufferVTG, gpsReg->bufferRxTemp,(gpsReg->index+1));
           strncpy(gpsReg->bufferGGA, gpsReg->bufferGGATemp,72);
           strncpy(gpsReg->bufferGSA, gpsReg->bufferGSATemp,70);
           strncpy(gpsReg->bufferGSV, gpsReg->bufferGSVTemp,70);
           strncpy(gpsReg->bufferRMC, gpsReg->bufferRMCTemp,70);
        }
        gpsReg->index = 0;
        return 1;

    }
}

/**
 * @brief Funcao que extrai os dados de tempo (UTC) do padrao solicitado.
 * @param gpsReg variavel do tipo GPSregister os valores sao carregados pela funcao
 * getGPSmessage
 * @param gpsUTC variavel do tipo GPSutc onde serao carregados os valores de saida da funcao
 * @param std parametro com padrao de sentenca de onde deseja ser extraido o UTC
 * (!! Atualmente so o GGA !!)
 * @return   0 - success
 *	    -1 - parsing error
 *	    -2 - sentence marked invalid
 */
int getGPSutc(GPSregister *gpsReg,GPSutc *gpsUTC,char std)
{
    int i;
    char temp_str[3];
    char *sentence;

    sentence = gpsReg->bufferGGA;
     if(strncmp(sentence,"$GPGGA", 6) == 0)//std, 6) == 0) //?? strncmp(gpsReg->bufferGGA,*std, 6)
     {

        for(i = 0;i < 1;i++)
        {
            sentence = strchr(sentence, ',');// pointer to the separator ','
            if(sentence == NULL)
                    return -1;
            sentence++;			//first character in field

            //pull out data
            if(i == 0) //UTC
            {
                temp_str[2] = 0;
                strncpy(temp_str, sentence, 2); // hour
                gpsUTC->utc_hour = atoi(temp_str);

                strncpy(temp_str, sentence + 2, 2);//minutes
                gpsUTC->utc_min  = atoi(temp_str);

                strncpy(temp_str, sentence + 4, 2);//seconds
                gpsUTC->utc_sec  = atoi(temp_str);

                temp_str[5] = 0;
                strncpy(temp_str, sentence + 4, 6);//seconds and decimal
                gpsUTC->utc_ts   = ((gpsUTC->utc_min)*60) + atof(temp_str);//
                return 0;
            }
        }
    }
     else
         return -2;
}
/**
 * @brief Funcao de auxilio para converter a latitude e longitude em graus
 * @param coord entrada de dados da coordenada para ser convertida
 * @param degrees saida de dados com a coordenada convertida em graus
 * @return   0 - success
 *	    -1 - parsing error
 *	    -2 - sentence marked invalid
 */

int GPStodegree(char *coord, double *degrees)
{
	char *decimal_point;
	char temp[2];
	char dummy[2];
        double tempdegrees = 0;
	decimal_point = strchr(coord, '.');

	if(decimal_point == NULL)
		return -1;

        temp[2] = 0;
	strncpy(temp, decimal_point - 4, 2);
	*degrees = atof(temp);
        strncpy(temp, decimal_point - 2, 2);
        *degrees += atof(temp)/60;
        strncpy(temp, decimal_point + 1, 4);
        tempdegrees = atof(temp)/100;
        *degrees += tempdegrees/6000;

	return 0;
}



/**
 * @brief Funcao que extrai os dados de posicao Latitude e Longitude em Graus
 *  e Altitude em metros.
 * @param gpsReg variavel do tipo GPSregister os valores sao carregados pela funcao
 * getGPSmessage
 * @param gpsPos  variavel do tipo GPSpos onde serao carregados os valores de saida da funcao
 * @param std parametro com padrao de sentenca de onde deseja extrair a Pos
 * (!! Atualmente so o GGA !!)
 * @return  0 - success
 *	    -1 - parsing error
 *	    -2 - sentence marked invalid
 */
int getGPSpos(GPSregister *gpsReg,GPSpos *gpsPos,char std)
{
        int i;
    char temp_str[3];
    char *sentence;

    sentence = gpsReg->bufferGGA;
     if(strncmp(sentence,"$GPGGA", 6) == 0)//std, 6) == 0) //?? strncmp(gpsReg->bufferGGA,*std, 6)
     {

        for(i = 0; i < 11; i++)
        {
            sentence = strchr(sentence, ',');// pointer to the separator ','
            if(sentence == NULL)
                    return -1;
            sentence++;			//first character in field

            //pull out data
            if(i == 1) //latitude
            {
                if(GPStodegree(sentence,&gpsPos->pos_lat))
                    return -1;
            }
            if(i == 2) //latitude direction
            {
                if(*sentence == 'S')
                    gpsPos->pos_lat = -gpsPos->pos_lat;
            }if(i == 3) //longitude
            {
                if(GPStodegree(sentence,&gpsPos->pos_lon))
                    return -1;
            }
            if(i == 4) //longitude
            {
                if(*sentence == 'W')
                    gpsPos->pos_lon = -gpsPos->pos_lon;
            }
        }
    }
     else
         return -2;
}
