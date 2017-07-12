#include <stdint.h>
#include <math.h>
#include <platform.h>

#include "common/axis.h"
#include "kalman.h"

//the noise in the system
#define  qKALMAN  0.00005
#define  RKALMAN  2 //uncertainty of GPS -> say 2m
#define  RFACTOR  10000

// no G, necessary (0)

static float pkalmanx[2][2];
static float xkalmanx[2];
static float pkalmany[2][2];
static float xkalmany[2];
static float pkalmanz[2][2];
static float xkalmanz[2];


void kalmanInit(float Z, int axis, bool measDone, float dt) {
    float *x;
    float *p;

    switch(axis){
        case X:
            x = xkalmanx;
            p = pkalmanx;
        case Y:
            x = xkalmany;
            p = pkalmany;
        case Z:
            x = xkalmanz;
            p = pkalmanz;
    }
//initialise with first two measurements
    if(measDone){ //already has a measurement
    //initialise velocity by velocity calc.
        *(x+1) = (Z - *x)/dt;
        *x = Z;
    //initialise certainty using Q matrix (given in terms of timesteps)
        *p = RFACTOR * powf(dt,3)/3;
        *(p+1) = RFACTOR * dt * dt * 0.5;
        *(p+2) = RFACTOR * dt * dt * 0.5;
        *(p+3) = RFACTOR * dt
    } else {
        *x = Z;
    }
}


 struct kalmanUpdate(float Z, int i, float dt) {
    //Predict Step
    float p_pred[2][2] ={{0}};
    float x_pred[2] = {0};
    //CV process noise matrix
    float Q[2][2]= {{qKALMAN * powf(dt,3)/3, qKALMAN * dt * dt * 0.5},
                    {qKALMAN * dt * dt * 0.5, qKALMAN * dt}};
    //CV State transition matrix                
    float F[2][2]= {{1,dt},{0,1}};
    float Fdash[2][2] = {{1,0},{dt,1}};
    float sKalmanInverse = 0;
    float kkalman[2][2] = {{0}};

    float *x;
    float *p;

    switch(axis){
        case X:
            x = xkalmanx;
            p = pkalmanx;
        case Y:
            x = xkalmany;
            p = pkalmany;
        case Z:
            x = xkalmanz;
            p = pkalmanz;
    }
    //Predict step
    //calculate xpred
    matrixMultiply(2,2,1,&F,x,&x_pred);
    //calculate p_pred 
    //transpose of F required
    matrixMultiply(2,2,2,&F,p,&x_pred);
    matrixMultiply(2,2,2,&x_pred,&Fdash,&x_pred);
    sqMatrixAdd(2,2,&x_pred,&Q,&x_pred);    
    
    //Calculate the Kalman gain
    sKalmanInverse = (1.0/(p_pred[0][0] + RKALMAN));
    kkalman[0] = sKalmanInverse * p_pred[0][0];
    kkalman[1] = sKalmanInverse * p_pred[1][0];
    
    //Update Step
    //state estimate
    scalarMultiplyMatrix(2,1,(Z - x_pred[0]),&kkalman,x);
    matrixAdd(2,1,&x_pred,x,x)
    //covariance estimate
     
    pkalman[i] = (1- kkalman) * (1-kkalman) * p_pred + kkalman*RKALMAN*kkalman;       
    return xkalman[i];
}

/* 
    if mtrx_a is (m x n) and mtrx_b is (n x p), 
    the product is an (m x p) matrix
*/
void matrixMultiply(int m, int n, int p, float* mtrx_a, float* mtrx_b, float* mtrx_r)
{

    for (int i = 0; i < m; i++){

        for (int j = 0; j < p; j++){

            for (int k = 0; k < n; k++){

                mtrx_r [i][j] += mtrx_a [i][k] * mtrx_b [k][j];
            }
        }
    }
}

void matrixAdd(int m, int n, float* mtrx_a float* mtrx_b, float* mtrx_r){

    for (int i = 0; i < m; i++){

        for(int j = 0; j < n; j++){

            mtrx_r[i][j] = mtrx_a[i][j] + mtrx_b[i][j];

        }

    }
}

void scalarMultiplysqMatrix(int m, int n, float scalar, float* mtrx_a, float* mtrx_r){
    for (int i = 0; i < m; i++){

        for(int j = 0; j < n; j++){

            mtrx_r[i][j] = scalar * mtrx_a[i][j];

        }

    }

}