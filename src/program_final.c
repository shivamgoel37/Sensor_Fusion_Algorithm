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

void init() {
    int i,j;
    for(i=0;i<SENSOR_COUNT;i++)
    {
        for(j=0;j<SENSOR_COUNT;j++)
        {
            mtx_d[i][j]=0;
            
        }
    }
    
    for(i=0;i<SENSOR_COUNT;i++)
    {        
        
        sensor_values[i]=0;        
    }
}

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
    
    
    //STEP 4;
    double alpha[SENSOR_COUNT];
    double totalEval=0;
    for(i=0;i<SENSOR_COUNT;i++)
    {
        totalEval += evals[i];
    }

    for(i=0;i<SENSOR_COUNT;i++)
    {
        alpha[i]=evals[i]/totalEval;
    }

    //STEP 5;
    double fi[SENSOR_COUNT];
    for(i=0;i<SENSOR_COUNT;i++)
    {
        fi[i]=0;
        for(j=0;j<=i;j++)
        {
            fi[i]+=alpha[j];
        }
        
    }

    //STEP 6:
    double Z[SENSOR_COUNT];
    double totalZ=0;

    
    for(i=0;i<SENSOR_COUNT;i++)
    {
        Z[i]=0;
        for(j=i;j<SENSOR_COUNT;j++)
        {
            Z[i]+=y[j]*alpha[j];
        }
        
        totalZ+=Z[i];
    }
    
    double prescribedrange=myAbs(totalZ/SENSOR_COUNT*1.0)*0.7;
    


    //Calculate condition
    double condZ[SENSOR_COUNT];
    double totalCondZ=0;
    
    for(i=0;i<SENSOR_COUNT;i++)
    {
        if(myAbs(Z[i])<prescribedrange)
        {
            
            condZ[i]=0;
        }
        else
        {
            condZ[i]=Z[i];
        }

        totalCondZ+=condZ[i];
    }

    //STEP 7
    double w[SENSOR_COUNT];

    
    for(i=0;i<SENSOR_COUNT;i++)
    {
        w[i] = condZ[i]/totalCondZ;
        
    }

    //STEP 8

    double fused_value=0;
    for(i=0;i<SENSOR_COUNT;i++)
    {
        fused_value+=w[i]*sensor_values[i];
       
    }

    
    //return fused_value
    fprintf(fp,"%lf,%lf,",fused_value,prescribedrange);
    //out range sensors
    for(i=0;i<SENSOR_COUNT;i++)
    {
        if(condZ[i]==0)
        {
            fprintf(fp,"sensor%d ",i+1);
        }
    }
    
    fprintf(fp,",");
    //stuck sensors
    for(i=0;i<SENSOR_COUNT;i++)
    {
        if(sensor_values[i]==prev_sensor_values[i])
        {
            fprintf(fp,"sensor%d ",i+1);
        }
    }
    fprintf(fp,"\n");
    return ;
}


int main(void) {

    char buf[1024];   

    int row_count = 0;
       

    FILE *fp = fopen("../data/input.csv", "r");

    if (!fp) {
        printf("Can't open file\n");
        return 0;
    }

    FILE *fout = fopen("../data/output.csv","w");
    if(!fout){
        printf("Can't Open output file\n");
        return 0;
    }
    fprintf(fout,"Time(24h clock),fused value,prescribed range, outside sensors, stuck sensors\n");
    
    
    char prevtime[20]="";   
    
    init();
    copy_to_prev();
    while (fgets(buf, 1024, fp)) {
        
        row_count++;

        if (row_count == 1) {
            continue;
        }

        clearTokens(tokens);
        int tokenIndex=0;

        //Get Tokens in row        
        char* token;
        token=strtok(buf,",");
        while(token!=NULL)
        {
            strcpy(tokens[tokenIndex++],token);
            token=strtok(NULL,",");
        }
              
        
        
        if(strcmp(prevtime,"")==0)
        {
            strcpy(prevtime,tokens[0]); 
            
            
                       
        }
        else
        {
            
            if(strcmp(prevtime,tokens[0])!=0)
            {           
                //when time changed, calculate previous fused_value 
                
                fusedvalue(fout,prevtime);
                copy_to_prev();
                //start new time
                strcpy(prevtime,tokens[0]);
                init();
            }
            
        }
        takeTokensValue();             
        
                
    }

    //save last time
    //fprintf(fout,"%s,%f\n",prevtime, fusedvalue());
    fusedvalue(fout,prevtime);
    
    
    fclose(fp);

    return 0;
}
