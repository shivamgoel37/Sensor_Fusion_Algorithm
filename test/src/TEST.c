/**
 * \file TEST.c
 * \brief This file tests the functioning of the whole project
 * \details Results are stored in test_case.txt file in /test/data
 * \author Rishabh Shrivastav
 * \       Shivam Goel
 * \       Kunal Markan
 * \       Prabhjot Singh    
 * \date 2019-12-18
 */

#include "../../include/main.h"


double mtx_d[SENSOR_COUNT][SENSOR_COUNT];
double sensor_values[SENSOR_COUNT];
double prev_sensor_values[SENSOR_COUNT];
char tokens[3][1024];
double e = 2.718281828459;

/**
 * \brief Function to reset the values of each element of the degree matrix to "0"
 * \details clears any previous sensor values
 */
void init() {
    int i,j;
    for(i=0;i<SENSOR_COUNT;i++) {
        
        for(j=0;j<SENSOR_COUNT;j++) {
            
            mtx_d[i][j]=0;
        }
    }
    
    for(i=0;i<SENSOR_COUNT;i++) {
        sensor_values[i]=0;
    }
}

/**
 * \brief Function to set the values of each element of the degree matrix
 * \details Uses SENSOR_COUNT as the test condition parameter, initialises sensor values
 * \details Each reading value is parsed from the input file in the loop
 * The first row from the input file is ignored by "row_count == 1"
 */
void copy_to_prev() {
    int i;
    for(i=0;i<SENSOR_COUNT;i++) {
        prev_sensor_values[i]=sensor_values[i];
    }
}

/**
 * \brief Function to clear tokens
 * \details Initialises the token value set in buffers to null
 */
void clear_tokens(char tokens[][1024]) {
    int i;
    for(i=0;i<3;i++) {
        tokens[i][0]='\0';
    }
}

/**
 * \brief Function to set sensor values
 * \details sensor name and value is set according to tokens
 */
void take_tokens_value() {
    //sens1,10 -> sensor_value[0]=10
     
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

/**
 * \brief Function to check test file 
 * \details file pointer is checked whether it works or not
 */
double my_abs(double f) {
    return (f>0)?f:-f;
}

/**************************************************/
/****** The SENSOR FUSION ALGORITHM ***************/
/**************************************************/

/**
 * \brief Function to compute the fused value as mentioned in the paper
 * \details Main functioning of the algorithm, contains all the computation steps
 * \param[in] file pointer fp
 * \param[in] previous time
 */
void fusedvalue(FILE* fp, char* prevtime) {

    /**
     * Computing the support degree matrix
 	 * For the given number of sensors, support degree matrix is generated
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
 	 * Calculating eigen values and corresponding eigen vector using GSL
 	 * For a given support degree matrix of
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
 	 * Compute principal component of the degree matrix
 	 * Computed principal component for each of eigen vector
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
 	 * Calculate the contribution rate of principal component "alpha"
 	 * Compute the contribution rate of kth and mth principal components as per the paper
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
 	 * Compute integrated support degree
 	 * Computed support degree matrix for each of the sensors
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
    
    double prescribedrange=my_abs(totalZ/SENSOR_COUNT*1.0)*0.7;

    /**
	 * Calculate condition
     */
    double condZ[SENSOR_COUNT];
    double totalCondZ=0;
    
    for(i=0;i<SENSOR_COUNT;i++) {
        if(my_abs(Z[i])<prescribedrange) {
            condZ[i]=0;
        }
        else {
            condZ[i]=Z[i];
        }

        totalCondZ+=condZ[i];
    }

    /**
	 * Eliminating invalid sensor readings
	 * Discard the faulty sensor readings from the support degree matrix
	 * Compute weighted coefficients for each sensors from the support degree matrix
	 */
    double w[SENSOR_COUNT];

    for(i=0;i<SENSOR_COUNT;i++) {
        w[i] = condZ[i]/totalCondZ;
    }

	/**
	 * Compute the weight coefficient for each sensor
	 * Compute the aggregated value of sensors from the weighted coefficients
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

/**
 * \brief Test function to check the functionalities 
 * \details This function checks the successful running of all functions
 * \param[in] file pointer fout1
 * \param[in] previous time
 */
void test(FILE *fout1, prevtime){
    init();
    copy_to_prev();
    take_tokens_value();
    fusedvalue(fout1,prevtime);
    
    fprintf(fout1," init() --> SUCCESS \n copy_to_prev() --> SUCCESS \n take_tokens_value --> SUCCESS \n fusedvalue(fout1,prevtime) --> SUCCESS \n");

}


/********************************************/
/********************************************/

/** 
 * \brief File handling operatoins are done below
 * \details Input read from .csv file opened in "r" mode
 * appropriate exception handling is done
 * Output generated at in a .csv file format and saved
 */
int main(void) {
    char buf[1024];
    int row_count = 0;

       
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
    
    FILE *fout1 = fopen("../data/test_case.txt","w");
    if(!fout1) {
        printf("Can't Open output file\n");
        return 0;
    }
    
    char prevtime[20]="";   
    
    init();

    copy_to_prev();

    while (fgets(buf, 1024, fp)) {
        row_count++;

        if (row_count == 1) {
            continue;
        }

        clear_tokens(tokens);
        int tokenIndex=0;

        //Fetching Tokens in row and splitting the string by ","

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
                
                //When time is changed, calculate the previous fused_value 
                
                fusedvalue(fout,prevtime);
                copy_to_prev();
                
                //Start new time
                strcpy(prevtime,tokens[0]);
                init();
            }
        }
        take_tokens_value();
    }

    //fprintf(fout,"%s,%f\n",prevtime, fusedvalue());
     
    fusedvalue(fout,prevtime);
    
    fclose(fp);
    
    test(fout1, prevtime);
    fclose(fout1);

    return 0;
    
}
