// Header
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "MPU9250.h"
#include <sys/time.h>
#include <unistd.h>
#include <MS5611.h>
#include "LSM9DS1.h"


#define ADGAIN_THR_LO (0.1)     // lower error value threshold (for gain factor one)
#define ADGAIN_THR_HI (0.2)     // upper error value threshold (for gain factor zero)
#define GYRO_GAIN (0.99)                            // percentage of gyro data in attitude estimation
#define MAGNETIC_DECLINATION (-0.1745)    // local magnetic declination [rad] -0.1745
#define PI 3.1416
typedef struct {
    double Q_ACC[4];        // acceleration-dependent quaternion
    double Q_MAG[4];        // magnetic field-dependent quaternion
    double Q_ACCMAG[4];     // acc&mag-fused quaternion
    double Q_GYRO[4];       // angular speed-dependent quaternion
    double Q_EST[4];        // finally estimated attitude
}att_t;

typedef struct {
    const short ACID = 1;           // aircraft id
    double ANGSPD[3], ANGSPDF[3];   // angular velocities (unfiltered/filtered) of body-fixed axes [rad/s]
    double ATTITUDE[3];             // euler angles describing estimated attitude [rad]
    double ACC[3], ACCF[3];         // linear accelerations (unfiltered/filtered) of body-fixed axes [g]
    double VERTSPD;                 // vertical speed [m/s]
    double ADACC;                   // aerodynamic acceleration [m/s^2]
    double ADSPD, ADSPDF;           // aerodynamic speed (unfiltered/filtered) [m/s]
    double BAROALT, BAROALTF;       // barometric altitude (unfiltered/filtered) [m]
    double MAG[3], MAGF[3];         // magnetic field components (unfiltered/filtered) of body-fixed axes [gauss]
    double MAGHDG;                  // magnetic heading [rad]
    int RCDATA[7];                  // local copy of rc data and status [1]
}imdata_t;

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
float segundos_ant = 0;
static unsigned long previoustime, currenttime;
float dt_ant = 0;

struct timeval tv2;
float dt2,segundos2=0,contador2=0;
float segundos_ant2 = 0;
static unsigned long previoustime2, currenttime2;

//MPU9250
MPU9250 mpu1;
float axmpu,aympu,azmpu;
float gxmpu,gympu,gzmpu;
float mxmpu,mympu,mzmpu;

//LSM9DS1
LSM9DS1 lsm1;
float axlsm,aylsm,azlsm;
float gxlsm,gylsm,gzlsm;
float mxlsm,mylsm,mzlsm;

//MS5611
MS5611 barometer;
float temperatura_baro            = 0;
float altitude_baro               = 0;
float altitude_baro_anterior      = 0;
float velocidade_vertical         = 0;
float velocidade_vertical_ant     = 0;
float delta_altitude_pressao      = 0;
float velocidade_vertical_fil     = 0;
float velocidade_vertical_fil_ant = 0;


// Function prototypes
void process_attitude(double DT, imdata_t *IMDATA, att_t *ATT);
void q_accmag(double ACC[], double MAG[], att_t *ATT);
void q_gyro(double DT, double ANGSPD[], att_t *ATT);
void qnorm(double Q[]);
void qprod(double P[], double Q[], double PQ[]);
double dotprod(double Q[], double P[]);
double func_adaptive_gain(double ERR);
void process_convertEA(double Q[], double EA[], double *MAGHDG);

float f_passa_baixa(float x_antes,float delta_t, float leitura, float freq_corte)
{
    //float freq_corte = 5;//hz
    float w = freq_corte*2*3.1415;
    return (x_antes+(delta_t*w*(leitura-x_antes)));
}

//thread 1
void* f_thread1(void* data)
{
	float p0       = 101325;
	
	while(1)
	{
		//TEMPO THREAD 1
		gettimeofday(&tv2,NULL);
        previoustime2 = currenttime2;
        currenttime2 = 1000000 * tv2.tv_sec + tv2.tv_usec;
        dt2 = (currenttime2 - previoustime2) / 1000000.0;
        //IGNORA LEITURA INICIAL ALTA.
        if (dt2>100)
        {
            dt2 = 0.01;
        }
        segundos2 = segundos2 + dt2;
		// BAROMETRO
        barometer.refreshPressure();
        usleep(10000); 
        barometer.readPressure();
        barometer.refreshTemperature();
        usleep(10000); 
        barometer.readTemperature();
        barometer.calculatePressureAndTemperature();
        altitude_baro     = -8430.125*log((barometer.getPressure()*100)/p0); //milibar
        temperatura_baro = barometer.getTemperature(); //ºC

        delta_altitude_pressao = altitude_baro - altitude_baro_anterior;
        velocidade_vertical = delta_altitude_pressao/dt2;

        //filtro
        velocidade_vertical_fil 	= f_passa_baixa( velocidade_vertical_fil_ant, dt2,  velocidade_vertical,  0.1);
        velocidade_vertical_fil_ant = velocidade_vertical_fil;
        velocidade_vertical_ant 	= velocidade_vertical;
        
        altitude_baro_anterior = altitude_baro;
        // printf("vert_spd_f = %10.4f vert_spd = %10.4f dt = %10.8f\n",velocidade_vertical_fil,velocidade_vertical, dt2);
	}
}

int main()
{
    double DT = 0.01;

    att_t ATT;
    imdata_t IMDATA;

    mpu1.initialize();
    lsm1.initialize();

    memset(IMDATA.ANGSPD, 0, sizeof(IMDATA.ANGSPD));
    memset(IMDATA.ANGSPDF, 0, sizeof(IMDATA.ANGSPDF));
    memset(IMDATA.ATTITUDE, 0, sizeof(IMDATA.ATTITUDE));
    memset(IMDATA.ACC, 0, sizeof(IMDATA.ACC));
    memset(IMDATA.ACCF, 0, sizeof(IMDATA.ACCF));
    IMDATA.VERTSPD = 0;
    IMDATA.ADACC = 0;
    IMDATA.ADSPD = 0;
    IMDATA.ADSPDF = 0;
    IMDATA.BAROALT = 0;
    IMDATA.BAROALTF = 0;
    memset(IMDATA.MAG, 0, sizeof(IMDATA.MAG));
    memset(IMDATA.MAGF, 0, sizeof(IMDATA.MAGF));
    // IMDATA.MAGHDG = 0.0;
    memset(IMDATA.RCDATA, 0, sizeof(IMDATA.RCDATA));

    memset(ATT.Q_ACC, 0, sizeof(ATT.Q_ACC));
    memset(ATT.Q_MAG, 0, sizeof(ATT.Q_MAG));
    memset(ATT.Q_ACCMAG, 0, sizeof(ATT.Q_ACCMAG));
    memset(ATT.Q_GYRO , 0, sizeof(ATT.Q_GYRO ));
    memset(ATT.Q_EST  , 0, sizeof(ATT.Q_EST  ));
    ATT.Q_GYRO[0] = 1;
    ATT.Q_GYRO[1] = 1;
    ATT.Q_GYRO[2] = 1;
    ATT.Q_GYRO[3] = 1;

    ATT.Q_EST[0] = 0.5;
    ATT.Q_EST[1] = 0.5;
    ATT.Q_EST[2] = 0.5;
    ATT.Q_EST[3] = 0.5;
 	
 	//BARÔMETRO MS5611
    barometer.initialize();

    //threads
    pthread_t thread1;
    pthread_create(&thread1,NULL,f_thread1,NULL);


     while(1)
     {
        //TEMPO
        gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        //IGNORA LEITURA INICIAL ALTA.
        if (dt>100)
        {
            dt = 0.01;
        }
        segundos = segundos + dt;

        //MPU9250
        //LEITURA DE DADOS DE ACELERAÇÃO, GIRO E MAGNÉTICOS
        mpu1.update();
        mpu1.read_accelerometer(&axmpu, &aympu, &azmpu);
        mpu1.read_gyroscope(&gxmpu, &gympu, &gzmpu);
        mpu1.read_magnetometer(&mxmpu, &mympu, &mzmpu);

        lsm1.update();
        lsm1.read_accelerometer(&axlsm, &aylsm, &azlsm);
        lsm1.read_gyroscope(&gxlsm, &gylsm, &gzlsm);
        lsm1.read_magnetometer(&mxlsm, &mylsm, &mzlsm);

        IMDATA.ACCF[0] = double(aylsm/9.81);
        IMDATA.ACCF[1] = double(axlsm/9.81);
        IMDATA.ACCF[2] = double(-azlsm/9.81);

        IMDATA.ANGSPDF[0] = double(gylsm);
        IMDATA.ANGSPDF[1] = double(gxlsm);
        IMDATA.ANGSPDF[2] = double(-gzlsm);

        // IMDATA.ANGSPD[0] = double(gxmpu);
        // IMDATA.ANGSPD[1] = double(gympu);
        // IMDATA.ANGSPD[2] = double(gzmpu);

        IMDATA.MAGF[0] = double(mylsm);
        IMDATA.MAGF[1] = double(mxlsm);
        IMDATA.MAGF[2] = double(-mzlsm);

        // IMDATA.BAROALTF = altitude_baro;
        // IMDATA.VERTSPD = velocidade_vertical_fil;

        // velocidade_vertical = delta_altitude_pressao/dt;

        process_attitude(double(dt), &IMDATA, &ATT);

        // printf("acc0:%5.2f acc1:%5.2f acc2:%5.2f acc3:%5.2f| gyro0:%5.2f gyro1:%5.2f gyro2:%5.2f gyro3:%5.2f| mag0:%5.2f mag1:%5.2f mag2:%5.2f mag3:%5.2f| r:%5.2f p:%5.2f y:%5.2f\n",axmpu,aympu,azmpu,ATT.Q_ACC[3],ATT.Q_GYRO[0],ATT.Q_GYRO[1],ATT.Q_GYRO[2],ATT.Q_GYRO[3],ATT.Q_MAG[0],ATT.Q_MAG[1],ATT.Q_MAG[2],ATT.Q_MAG[3],IMDATA.ATTITUDE[0]*180/PI,IMDATA.ATTITUDE[1]*180/PI,IMDATA.ATTITUDE[2]*180/PI);
        printf("r:%7.2f p:%7.2f y:%7.2f\n",IMDATA.ATTITUDE[0]*180/PI,IMDATA.ATTITUDE[1]*180/PI,IMDATA.ATTITUDE[2]*180/PI);

        // printf("mxmpu = %16.6f mympu = %16.6f mzmpu = %16.6f mxlsm = %16.6f mylsm = %16.6f mzlsm = %16.6f\n",mxmpu,mympu,mzmpu,mxlsm,mylsm,mzlsm);

        // usleep(100000);
     }

    
    return 0;
}

void process_attitude (double DT, imdata_t *IMDATA, att_t *ATT) {

    double ANGSPD[3], ACC[3], MAG[3];   // local copies of data
    double ERR_ACC, AGAIN;              // error-function value & adaptive gain
    double ABS;                         // temporary values

    /* 1. DATA SOURCES */
    memcpy(ANGSPD, &IMDATA->ANGSPDF, sizeof(IMDATA->ANGSPDF));
    memcpy(ACC, &IMDATA->ACCF, sizeof(IMDATA->ACCF));
    memcpy(MAG, &IMDATA->MAGF, sizeof(IMDATA->MAGF));

    // printf("angspd0:%5.2f angspd1:%5.2f angspd2:%5.2f\n",ANGSPD[0],ANGSPD[1],ANGSPD[2]);

    // normalize magnetic field
    ABS = sqrt(MAG[0]*MAG[0] + MAG[1]*MAG[1] + MAG[2]*MAG[2]);
    MAG[0] = MAG[0]/ABS;
    MAG[1] = MAG[1]/ABS;
    MAG[2] = MAG[2]/ABS;

    // error function for adaptive gain & acceleration normalization
    ABS = sqrt(ACC[0]*ACC[0] + ACC[1]*ACC[1] + ACC[2]*ACC[2]);
    ERR_ACC = fabs(ABS - 1);
    ACC[0] = ACC[0]/ABS;
    ACC[1] = ACC[1]/ABS;
    ACC[2] = ACC[2]/ABS;


    /* 2. ACCMAG & 3. GYRO QUATERNION */
    q_accmag(ACC, MAG, ATT);
    // printf("QNORM_GY1: gyro0:%5.2f gyro1:%5.2f gyro2:%5.2f gyro3:%5.2f\n",ATT->Q_GYRO[0],ATT->Q_GYRO[1],ATT->Q_GYRO[2],ATT->Q_GYRO[3]);
    q_gyro(DT, ANGSPD, ATT);
    // printf("QNORM_GY2: gyro0:%5.2f gyro1:%5.2f gyro2:%5.2f gyro3:%5.2f\n",ATT->Q_GYRO[0],ATT->Q_GYRO[1],ATT->Q_GYRO[2],ATT->Q_GYRO[3]);


    /* 4. LINEAR INTERPOLATION */
    // check sign of q_accmag (to ensure shortest rotation for interpolatin)
    if(dotprod(ATT->Q_ACCMAG, ATT->Q_GYRO) < 0) {
        // switch signs if dotproduct is less than zero
        ATT->Q_ACCMAG[0] = (-1) * ATT->Q_ACCMAG[0];
        ATT->Q_ACCMAG[1] = (-1) * ATT->Q_ACCMAG[1];
        ATT->Q_ACCMAG[2] = (-1) * ATT->Q_ACCMAG[2];
        ATT->Q_ACCMAG[3] = (-1) * ATT->Q_ACCMAG[3];
    }

    // adaptive gain (for Q_ACCMAG) considering acceleration error
    AGAIN = func_adaptive_gain(ERR_ACC);
    // printf("again %5.2f\n",AGAIN);

    // interpolation
    ATT->Q_EST[0] = AGAIN * ATT->Q_ACCMAG[0] + (1-AGAIN) * ATT->Q_GYRO[0];
    ATT->Q_EST[1] = AGAIN * ATT->Q_ACCMAG[1] + (1-AGAIN) * ATT->Q_GYRO[1];
    ATT->Q_EST[2] = AGAIN * ATT->Q_ACCMAG[2] + (1-AGAIN) * ATT->Q_GYRO[2];
    ATT->Q_EST[3] = AGAIN * ATT->Q_ACCMAG[3] + (1-AGAIN) * ATT->Q_GYRO[3];

    //printf("PROD0:%5.2f PROD1:%5.2f PROD2:%5.2f PROD3:%5.2f| P0:%5.2f P1:%5.2f P2:%5.2f P3:%5.2f| Q0:%5.2f Q1:%5.2f Q2:%5.2f Q3:%5.2f\n",ATT->Q_EST[0],ATT->Q_EST[1],ATT->Q_EST[2],ATT->Q_EST[3],ATT->Q_ACCMAG[0],ATT->Q_ACCMAG[1],ATT->Q_ACCMAG[2],ATT->Q_ACCMAG[3],ATT->Q_GYRO[0],ATT->Q_GYRO[1],ATT->Q_GYRO[2],ATT->Q_GYRO[3]);

    // printf("EST: ");
    qnorm(ATT->Q_EST);


    /* 5. CONVERT TO EULER ANGLES */
    process_convertEA(ATT->Q_EST, IMDATA->ATTITUDE, &IMDATA->MAGHDG);

    return;

}

void q_accmag(double ACC[3], double MAG[3], att_t *ATT) {

    double MAG_Q[4], L;
    double Q_ACCINV[4];


    /* 1. ACCELERATION QUATERNION */
    if(ACC[2] < 0) {
        ATT->Q_ACC[0] = sqrt( (1-ACC[2])/2 );
        ATT->Q_ACC[1] = ACC[1] / sqrt( 2*(1-ACC[2]) );
        ATT->Q_ACC[2] = (-1)*ACC[0] / sqrt( 2*(1-ACC[2]) );
        ATT->Q_ACC[3] = 0;
    }
    else {
        ATT->Q_ACC[0] = ACC[1] / sqrt( 2*(1+ACC[2]) );
        ATT->Q_ACC[1] = sqrt( (1+ACC[2])/2 );
        ATT->Q_ACC[2] = 0;
        ATT->Q_ACC[3] = (-1)*ACC[0] / sqrt( 2*(1+ACC[2]) );
    }

    // normalize
    // printf("ACC:");
    qnorm(ATT->Q_ACC);

    // inverse acceleration quaternion
    Q_ACCINV[0] = ATT->Q_ACC[0];
    Q_ACCINV[1] = (-1)*ATT->Q_ACC[1];
    Q_ACCINV[2] = (-1)*ATT->Q_ACC[2];
    Q_ACCINV[3] = (-1)*ATT->Q_ACC[3];


    /* 2. TURN MAGNETIC VECTOR */
    // store mag data as quaternion
    MAG_Q[0] = 0;
    memcpy(&MAG_Q[1], MAG, 3*sizeof(double));

    // rotate magnetic data with acceleration quaternion
    // printf("mag1: ");
    qprod(MAG_Q, ATT->Q_ACC, MAG_Q);
    // printf("mag2: ");
    qprod(Q_ACCINV, MAG_Q, MAG_Q);

    // store result
    memcpy(MAG, &MAG_Q[1], 3*sizeof(double));


    /* 3. MAGNETIC QUATERNION */
    L = sqrt(MAG[0]*MAG[0] + MAG[1]*MAG[1]);
    if (MAG[0] < 0) {
        ATT->Q_MAG[0] = MAG[1] / sqrt( 2*L*(L-MAG[0]) );
        ATT->Q_MAG[1] = 0;
        ATT->Q_MAG[2] = 0;
        ATT->Q_MAG[3] = sqrt( (L-MAG[0]) / (2*L) );
    }
    else if (MAG[0] == 0 && MAG[1] == 0) {
        ATT->Q_MAG[0] = 1;
        ATT->Q_MAG[1] = 0;
        ATT->Q_MAG[2] = 0;
        ATT->Q_MAG[3] = 0;
    }
    else {
        ATT->Q_MAG[0] = sqrt( (MAG[0]+L) / (2*L) );
        ATT->Q_MAG[1] = 0;
        ATT->Q_MAG[2] = 0;
        ATT->Q_MAG[3] = MAG[1] / sqrt( 2*L*(MAG[0]+L) );
    }

    // normalize
    // printf("MAG: ");
    qnorm(ATT->Q_MAG);


    /* 4. FUSED (ACCMAG) QUATERNION */
    // printf("acmg: ");
    qprod(ATT->Q_ACC, ATT->Q_MAG, ATT->Q_ACCMAG);
    // printf("AEM: ");
    qnorm(ATT->Q_ACCMAG);

    return;

}

void q_gyro(double DT, double ANGSPD[3], att_t *ATT) {

    double ANG, S, DQ[4];

    /* rotation angle */
    ANG = DT*sqrt(ANGSPD[0]*ANGSPD[0] + ANGSPD[1]*ANGSPD[1] + ANGSPD[2]*ANGSPD[2]);
    // printf("dt %6.2f angspd0 %6.2f angspd1 %6.2f angspd2 %6.2f ANG %10.6f\n",DT,ANGSPD[0],ANGSPD[1],ANGSPD[2],ANG);
    /* update quaternion (conjugated) */
    // real component
    DQ[0] = cos(ANG/2);

    // imaginary components

    S = (-1) * DT * sin(ANG/2) / ANG;
    DQ[1] = ANGSPD[0] * S;
    DQ[2] = ANGSPD[1] * S;
    DQ[3] = ANGSPD[2] * S;


    // printf("ang:%6.4f s:%6.4f dq0:%6.2f dq1:%6.2f dq2:%6.2f dq3:%6.2f ang1:%6.2f ang2:%6.2f ang3:%6.2f\n",ANG,S,DQ[0],DQ[1],DQ[2],DQ[3],ANGSPD[0],ANGSPD[1],ANGSPD[2]);
    // normalize
    // printf("DQ : ");
    // printf("QNADA   : gyro0:%5.2f gyro1:%5.2f gyro2:%5.2f gyro3:%5.2f\n",ATT->Q_GYRO[0],ATT->Q_GYRO[1],ATT->Q_GYRO[2],ATT->Q_GYRO[3]);
    qnorm(DQ);
    // printf("QNORM_DQ: gyro0:%5.2f gyro1:%5.2f gyro2:%5.2f gyro3:%5.2f\n",ATT->Q_GYRO[0],ATT->Q_GYRO[1],ATT->Q_GYRO[2],ATT->Q_GYRO[3]);
    /* calculate gyro quaternion */
    // printf("gyro: ");
    
    qprod(DQ, ATT->Q_EST, ATT->Q_GYRO);
    // printf("QPROD   : gyro0:%5.2f gyro1:%5.2f gyro2:%5.2f gyro3:%5.2f\n",ATT->Q_GYRO[0],ATT->Q_GYRO[1],ATT->Q_GYRO[2],ATT->Q_GYRO[3]);
    // normalize
    // printf("GYR: ");
   
    qnorm(ATT->Q_GYRO);

    // printf("QNORM_GY: gyro0:%5.2f gyro1:%5.2f gyro2:%5.2f gyro3:%5.2f\n",ATT->Q_GYRO[0],ATT->Q_GYRO[1],ATT->Q_GYRO[2],ATT->Q_GYRO[3]);

    // printf("ang:%6.4f s:%6.4f dq0:%6.2f dq1:%6.2f dq2:%6.2f dq3:%6.2f ang1:%6.2f ang2:%6.2f ang3:%6.2f\n",ANG,S,DQ[0],DQ[1],DQ[2],DQ[3],ANGSPD[0],ANGSPD[1],ANGSPD[2]);
    

}

void qnorm(double Q[4]) {

    double P[4], ABS;

    ABS = sqrt(Q[0]*Q[0] + Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3]);

    P[0] = Q[0] / ABS;
    P[1] = Q[1] / ABS;
    P[2] = Q[2] / ABS;
    P[3] = Q[3] / ABS;

    memcpy(Q, P, sizeof(P));

    // printf("q0:%5.2f q1:%5.2f q2:%5.2f q3:%5.2f| ABS:%5.2f| p0:%5.2f p1:%5.2f p2:%5.2f p3:%5.2f\n",Q[0],Q[1],Q[2],Q[3],ABS,P[0],P[1],P[2],P[3]);

    return;

}

void qprod(double P[4], double Q[4], double PQ[4]) {

    double PROD[4];

    PROD[0] = P[0]*Q[0] - P[1]*Q[1] - P[2]*Q[2] - P[3]*Q[3];
    PROD[1] = P[0]*Q[1] + P[1]*Q[0] + P[2]*Q[3] - P[3]*Q[2];
    PROD[2] = P[0]*Q[2] - P[1]*Q[3] + P[2]*Q[0] + P[3]*Q[1];
    PROD[3] = P[0]*Q[3] + P[1]*Q[2] - P[2]*Q[1] + P[3]*Q[0];

    memcpy(PQ, PROD, sizeof(PROD));

    // printf("PROD0:%5.2f PROD1:%5.2f PROD2:%5.2f PROD3:%5.2f| P0:%5.2f P1:%5.2f P2:%5.2f P3:%5.2f| Q0:%5.2f Q1:%5.2f Q2:%5.2f Q3:%5.2f\n",PROD[0],PROD[1],PROD[2],PROD[3],P[0],P[1],P[2],P[3],Q[0],Q[1],Q[2],Q[3]);
    // printf("pq0:%5.2f pq0:%5.2f pq0:%5.2f pq0:%5.2f\n",PQ[0],PQ[1],PQ[2],PQ[3]);
    return;

}

double dotprod(double Q[4], double P[4]) {

    return Q[0]*P[0] + Q[1]*P[1] + Q[2]*P[2] + Q[3]*P[3];

}

double func_adaptive_gain(double ERR) {

    double GAIN_FACTOR;

    if(ERR < ADGAIN_THR_LO)
        GAIN_FACTOR = 1;

    else if(ERR > ADGAIN_THR_HI)
        GAIN_FACTOR = 0;

    else
        GAIN_FACTOR = (double) (ADGAIN_THR_HI - ERR) / (ADGAIN_THR_HI - ADGAIN_THR_LO);

    return GAIN_FACTOR * (1-GYRO_GAIN);

}

void process_convertEA(double Q[4], double EA[3], double *MAGHDG) {

    double ARG;

    // protect argument of asin against unexact normalization
    ARG = (-2)*(Q[0]*Q[2]+Q[1]*Q[3]);
    ARG = fmin( fmax( ARG , -1.0 ) , 1.0 );

    // calculate euler angles (protect asin against unexact normalization)
    EA[0] = atan2( 2*(Q[2]*Q[3]-Q[0]*Q[1]) , 1-2*(Q[1]*Q[1]+Q[2]*Q[2]) );
    EA[1] = asin( ARG );
    *MAGHDG = atan2( 2*(Q[1]*Q[2]-Q[0]*Q[3]) , 1-2*(Q[2]*Q[2]+Q[3]*Q[3]) );

    // consider magnetic declination
    EA[2] = *MAGHDG + MAGNETIC_DECLINATION;
    if(EA[2] > M_PI) EA[2] -= 2*M_PI;

    return;

}