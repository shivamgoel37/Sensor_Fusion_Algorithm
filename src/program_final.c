//#include <string.h>
//#include <stdio.h>
//#include <math.h>
//#include <gsl/gsl_math.h>
//#include <gsl/gsl_eigen.h>
//#define SENSOR_COUNT 3

#include "../include/main.h"

double mtx_d[SENSOR_COUNT][SENSOR_COUNT];

double sensor_values[SENSOR_COUNT];
double prev_sensor_values[SENSOR_COUNT];
char tokens[3][1024];
double e = 2.718281828459;

void clearTokens(char tokens[][1024])
{
    int i;
    for(i=0;i<3;i++)
    {        
        tokens[i][0]='\0';
    }
}

void copy_to_prev()
{
    int i;
    for(i=0;i<SENSOR_COUNT;i++)
    {        
        
        prev_sensor_values[i]=sensor_values[i];        
    }
}
void takeTokensValue()
{
    //sens1,10 -> sensor_value[0]=10
    char sensorname[100];
    int i;
    int j=0;
    int sensorno;
    double sensorvalue;

    for(i=4;i<strlen(tokens[1]);i++)
        sensorname[j++]=tokens[1][i];
    sensorname[j]='\0';

    sensorno=atoi(sensorname);
    sensorvalue = atof(tokens[2]);
    sensor_values[sensorno-1]=sensorvalue;    

    
}
double myAbs(double f)
{
    return (f>0)?f:-f;
}

