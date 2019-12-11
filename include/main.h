#ifndef __MAIN__H__
#define __MAIN__H__

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>

#define SENSOR_COUNT 3

/**
 * \brief Function to set the values of each element of the degree matrix to "0"
 * Takes SENSOR_COUNT as the test condition parameter
 * Clears any previous sensor values
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

#endif
