/**
 * \brief This file contains the function definitions to implement the Sensor Fusion Algorithm
 * \author Rishabh Shrivastav
 * \       Shivam Goel
 * \       Kunal Markan
 * \       Prabhjot Singh    
 * \date 2019-12-05
 */

#include "../include/main.h"

double mtx_d[SENSOR_COUNT][SENSOR_COUNT];
double sensor_values[SENSOR_COUNT];
double prev_sensor_values[SENSOR_COUNT];
char tokens[3][1024];
double e = 2.718281828459;

void copy_to_prev() {
    int i;
    for(i=0;i<SENSOR_COUNT;i++) {
        prev_sensor_values[i]=sensor_values[i];
    }
}

void clearTokens(char tokens[][1024]) {
    int i;
    for(i=0;i<3;i++) {
        tokens[i][0]='\0';
    }
}

void takeTokensValue() {
    /**
     *sens1,10 -> sensor_value[0]=10
     */
    char sensorname[100];
    int i;
    int j=0;
    int sensorno;
    double sensorvalue;

    for(i=4;i<strlen(tokens[1]);i++) {
        sensorname[j++]=tokens[1][i];
    }
    sensorname[j]='\0';

    sensorno=atoi(sensorname);
    sensorvalue = atof(tokens[2]);
    sensor_values[sensorno-1]=sensorvalue;
}

double myAbs(double f) {
    return (f>0)?f:-f;
}

/**************************************************/
/****** The SENSOR FUSION ALGORITHM ***************/
/**************************************************/

/**
 * \brief Function to compute the fused value as mentioned in the paper 
 * \param[in] file pointer fp
 * \param[in] previous time
 */
void fusedvalue(FILE* fp, char* prevtime) {

    /**
     * \brief Computing the support degree matrix
 	 * \details For the given number of sensors, support degree matrix is generated
     */
    int i, j, k; 
    double data[SENSOR_COUNT*SENSOR_COUNT];

    fprintf(fp,"%s,",prevtime);

    for(i=0;i<SENSOR_COUNT;i++) {
        for(j=0;j<SENSOR_COUNT;j++) {
            mtx_d[i][j]= pow(e,-1*abs(sensor_values[i]-sensor_values[j]));
            data[i*SENSOR_COUNT+j]=pow(e,-1*abs(sensor_values[i]-sensor_values[j]));       
        }
    }

    /**
 	 * \brief Calculating eigen values and corresponding eigen vector using GSL
 	 * \details For a given support degree matrix of
 	 * sensors the corresponding eigen values and vectors are produced
 	 */
    gsl_matrix_view m = gsl_matrix_view_array (data, SENSOR_COUNT, SENSOR_COUNT);
    gsl_vector *eval = gsl_vector_alloc (SENSOR_COUNT);
    gsl_matrix *evec = gsl_matrix_alloc (SENSOR_COUNT, SENSOR_COUNT);
    gsl_eigen_symmv_workspace * ew = gsl_eigen_symmv_alloc (SENSOR_COUNT);
       
    gsl_eigen_symmv(&m.matrix, eval, evec, ew);
     
    gsl_eigen_symmv_free (ew);
     
    gsl_eigen_symmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    
    double evals[SENSOR_COUNT];
    double evectors[SENSOR_COUNT][SENSOR_COUNT];

    
    for (i = 0; i < SENSOR_COUNT; i++) {
        evals[i] = gsl_vector_get (eval, i);
        gsl_vector_view evetc= gsl_matrix_column (evec, i);
        gsl_vector v = evetc.vector;

        for(j=0;j<SENSOR_COUNT;j++) {
            evectors[i][j]=gsl_vector_get(&v,j);
        }
    }
    
    /**
 	 * \brief Compute principal component of the degree matrix
 	 * \details Computed principal component for each of eigen vector
 	 */
    double y[SENSOR_COUNT];

    for(i=0;i<SENSOR_COUNT;i++) {
        y[i]=0;
        for(j=0;j<SENSOR_COUNT;j++) {
            for(k=0;k<SENSOR_COUNT;k++) {
                y[i]+=evectors[i][j]*mtx_d[k][j];
            }
        }
    }
    
    /**
 	 * \brief Calculate the contribution rate of principal component "alpha"
 	 * \details Compute the contribution rate of kth and mth principal components as per the paper
	 */
    double alpha[SENSOR_COUNT];
    double totalEval=0;
    for(i=0;i<SENSOR_COUNT;i++) {
        totalEval += evals[i];
    }

    for(i=0;i<SENSOR_COUNT;i++) {
        alpha[i]=evals[i]/totalEval;
    }

    double fi[SENSOR_COUNT];
    for(i=0;i<SENSOR_COUNT;i++) {
        
        fi[i]=0;
        for(j=0;j<=i;j++) {
            fi[i]+=alpha[j];
        }
    }

    /**
 	 * \brief Function to compute integrated support degree
 	 * \details Computed support degree matrix for each of the sensors
	 */
    double Z[SENSOR_COUNT];
    double totalZ=0;

    
    for(i=0;i<SENSOR_COUNT;i++) {
        Z[i]=0;
        for(j=i;j<SENSOR_COUNT;j++) {
            Z[i]+=y[j]*alpha[j];
        }
        
        totalZ+=Z[i];
    }
    
    double prescribedrange=myAbs(totalZ/SENSOR_COUNT*1.0)*0.7;

    /**
	 * Calculate condition
     */
    double condZ[SENSOR_COUNT];
    double totalCondZ=0;
    
    for(i=0;i<SENSOR_COUNT;i++) {
        if(myAbs(Z[i])<prescribedrange) {
            condZ[i]=0;
        }
        else {
            condZ[i]=Z[i];
        }

        totalCondZ+=condZ[i];
    }

    /**
	 * \brief Eliminating invalid sensor readings
	 * \details Discard the faulty sensor readings from the support degree matrix
	 * \details Compute weighted coefficients for each sensors from the support degree matrix
	 */
    double w[SENSOR_COUNT];

    for(i=0;i<SENSOR_COUNT;i++) {
        w[i] = condZ[i]/totalCondZ;
    }

	/**
	 * \brief Compute the weight coefficient for each sensor
	 * \details Compute the aggregated value of sensors from the weighted coefficients
	 */
    double fused_value=0;
    for(i=0;i<SENSOR_COUNT;i++) {
        fused_value+=w[i]*sensor_values[i];
    }
    
    /**
     * Return the fused_value
     */
    fprintf(fp,"%lf,%lf,",fused_value,prescribedrange);
    
    //out range sensors
    for(i=0;i<SENSOR_COUNT;i++) {
        if(condZ[i]==0) {
            fprintf(fp,"sensor%d ",i+1);
        }
    }
    
    fprintf(fp,",");
    
    /**
     * List of sensors that are stuck (i.e. sensors whose value does not change after a time “t”
    */
    for(i=0;i<SENSOR_COUNT;i++) {
        if(sensor_values[i]==prev_sensor_values[i]) {
            fprintf(fp,"sensor%d ",i+1);
        }
    }
    fprintf(fp,"\n");
    return ;
}

/********************************************/
/********************************************/

int main(void) {
    char buf[1024];
    int row_count = 0;

/** 
 * File handling operatoins are done below
 * Input is read from .csv file from the path and opened in "r" or read mode
 * appropriate exception handling is also done
 * Output is generated at runtime in a .csv file format and saved at the path
 */
       
    FILE *fp = fopen("../data/input.csv", "r");

    if (!fp) {
        printf("Can't open file\n");
        return 0;
    }

    FILE *fout = fopen("../data/output.csv","w");
    if(!fout) {
        printf("Can't Open output file\n");
        return 0;
    }
    
    fprintf(fout,"Time(24h clock),fused value,prescribed range, outside sensors, stuck sensors\n");
    
    
    char prevtime[20]="";   

/**
 * \brief Function call to initialise the reading process of each sensor
 * \details The definition of the following two functoin calls is defined in "main.h" header file 
 * Each reading value is parsed from the input file in the loop
 * The first row from the input file is ignored by "row_count == 1"
 */    
    init();

    copy_to_prev();

    while (fgets(buf, 1024, fp)) {
        row_count++;

        if (row_count == 1) {
            continue;
        }

        clearTokens(tokens);
        int tokenIndex=0;

        /**
         * Fetching Tokens in row and splitting the string by ","
         */

        char* token;
        token=strtok(buf,",");
        while(token!=NULL) {
            strcpy(tokens[tokenIndex++],token);
            token=strtok(NULL,",");
        }
        
        if(strcmp(prevtime,"")==0) {
            strcpy(prevtime,tokens[0]);
        }
        else {
            if(strcmp(prevtime,tokens[0])!=0){
                /**
                 * When time is changed, calculate the previous fused_value 
                 */
                
                fusedvalue(fout,prevtime);
                copy_to_prev();

                /**
                 Start new time
                 */
                strcpy(prevtime,tokens[0]);
                init();
            }
        }
        takeTokensValue();
    }

    /**
     *fprintf(fout,"%s,%f\n",prevtime, fusedvalue());
     */
    fusedvalue(fout,prevtime);
    
    fclose(fp);

    return 0;
    
}
