/**
 * @file   gpsclass.h
 * @brief Header da classe GPS para TCC
 * @author Daniel Diegues
 *
 * @date  07 de Outubro de 2014
 */
//#ifndef GPSCLASS_H
//#define	GPSCLASS_H

/**
 * @brief Registrador das mensagens NMEA do GPS com os campos de buffer temporarios
 * para manter os dados sincronizados
 *
 * Definicao do tipo GPSregister
 *
 */

typedef	struct reg_GPS{
    /**Campo para armazenar os dados temporarios de GPGGA*/
    volatile unsigned char bufferGGATemp[72];
    /**Campo para armazenar os dados de temporarios GPGSA*/
    volatile unsigned char bufferGSATemp[70];
    /**Campo para armazenar os dados de temporarios GPGSV*/
    volatile unsigned char bufferGSVTemp[70];
    /**Campo para armazenar os dados de temporarios GPRMC*/
    volatile unsigned char bufferRMCTemp[70];
    /**Campo para armazenar os dados de temporarios GPVTG*/
    volatile unsigned char bufferVTGTemp[70];
    /**Campo para armazenar os dados de GPGGA*/
    volatile unsigned char bufferGGA[72];
    /**Campo para armazenar os dados de GPGSA*/
    volatile unsigned char bufferGSA[70];
    /**Campo para armazenar os dados de GPGSV*/
    volatile unsigned char bufferGSV[70];
    /**Campo para armazenar os dados de GPRMC*/
    volatile unsigned char bufferRMC[70];
    /**Campo para armazenar os dados de GPVTG*/
    volatile unsigned char bufferVTG[70];
    /**Campo para armazenar os dados incompletos temporarios durante o processo
     *  de leitura da serial. Nao deve ser considerado dado valido*/
    volatile unsigned char bufferRxTemp[70];
    /**Campo para armazenar o index de cada um dos caracteres recebidos da
     * serial*/
    unsigned int index;
    /**Campo para indicar que recebeu uma mensagem de GPS inteira nova*/


}GPSregister;

/**
 * @brief Estrutura com os dados UTC (hora, minuto, segundo e tempo em segundos)
 * da funcao getUTC
 * Definicao do tipo GPSutc
 */
typedef	struct GPS_utc{
    /**Campo para armazenar a hora*/
    volatile unsigned int utc_hour;
	/**Campo para armazenar os minutos*/
    volatile unsigned int utc_min;
	/**Campo para armazenar os segundos (sem decimal)*/
    volatile unsigned int utc_sec;
	/**Campo para armazenar tempo em segundos com decimal e minutos convertidos*/
    volatile double utc_ts;
}GPSutc;

/**
 * @brief Estrutura com os dados de posicao
 * Latitude, Longitude e altitude.
 * da funcao getGPSpos
 * Definicao do tipo GPSpos
 */
typedef	struct GPS_pos{
    /**Campo para armazenar a latitude em graus*/
    volatile double pos_lat;
   /**Campo para armazenar a longitude em graus*/
    volatile double pos_lon;
   /**Campo para armazenar altitude em metros*/
    volatile double pos_alt;

}GPSpos;


/*
 * @brief Funcao de Inicializacao das variavies do GPS
 */
void initGPS(GPSregister *gpsReg);
/*
 * @brief Funcao que grava cada character no buffer do padrao especifico.
 * Grava em um buffer temporario
 * Atualiza todos os buffers permanentes ao mesmo instante, quando recebe o GPVTG
 */
int getGPSmessage(char charRx,GPSregister *gpsReg);

/*
 * @brief Funcao para obter os dados de tempo (UTC) do padrao solicitado
 */
int getGPSutc(GPSregister *gpsReg,GPSutc *gpsUTC,char std);


/*
 * @brief Funcao para obter os dados de posicao.
 * Sendo latitude, longitude e altitude.
 */
int getGPSpos(GPSregister *gpsReg,GPSpos *gpsPos,char std);



