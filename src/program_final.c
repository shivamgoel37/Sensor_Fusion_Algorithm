//#include <stdio.h>
//#include <string.h>
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
void fusedvalue(FILE* fp, char* prevtime)
{
    
    //STEP 1-calculate D
    int i, j, k; 
    double data[SENSOR_COUNT*SENSOR_COUNT];


    
    fprintf(fp,"%s,",prevtime);
    
    

    for(i=0;i<SENSOR_COUNT;i++)
    {
        for(j=0;j<SENSOR_COUNT;j++)
        {    
            mtx_d[i][j]= pow(e,-1*abs(sensor_values[i]-sensor_values[j]));
            
            data[i*SENSOR_COUNT+j]=pow(e,-1*abs(sensor_values[i]-sensor_values[j]));       
        }
        
    }

    //STEP 2- gsl eigenvector calculate
    gsl_matrix_view m = gsl_matrix_view_array (data, SENSOR_COUNT, SENSOR_COUNT);
    gsl_vector *eval = gsl_vector_alloc (SENSOR_COUNT);
    gsl_matrix *evec = gsl_matrix_alloc (SENSOR_COUNT, SENSOR_COUNT);
    gsl_eigen_symmv_workspace * ew = gsl_eigen_symmv_alloc (SENSOR_COUNT);
       
    gsl_eigen_symmv(&m.matrix, eval, evec, ew);
     
    gsl_eigen_symmv_free (ew);
     
    gsl_eigen_symmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    
    double evals[SENSOR_COUNT];
    double evectors[SENSOR_COUNT][SENSOR_COUNT];

    
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        evals[i] = gsl_vector_get (eval, i);
        gsl_vector_view evetc= gsl_matrix_column (evec, i);
        gsl_vector v = evetc.vector;


        for(j=0;j<SENSOR_COUNT;j++)
        {
            evectors[i][j]=gsl_vector_get(&v,j);
        }     
       
    }
    
    //STEP 3
    double y[SENSOR_COUNT];

    for(i=0;i<SENSOR_COUNT;i++)
    {
        y[i]=0;
        for(j=0;j<SENSOR_COUNT;j++)
        {
            for(k=0;k<SENSOR_COUNT;k++)
            {
                y[i]+=evectors[i][j]*mtx_d[k][j];
            }
        }
    }
    
    
    
