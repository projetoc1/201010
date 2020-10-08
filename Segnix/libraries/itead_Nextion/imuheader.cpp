#include "imuheader.h"



/* attitude/process_attitude.c ---------------------------------------
 *	Attitude estimation using angular speed, acceleration and magnetic
 *	field data as follows:
 *		1. Copy measurement data to local scope and normalize.
 *		2. Estimate quaternion based on acceleration & magnetic data.
 *		3. Estimate quaternion based on angular rate.
 *      4. Linear interpolation of both estimation quaternions.
 *      5. Conversion to euler angles.
 *	------------------------------------------------------------------
 *	DT: Timestep (seconds) used for integration of angular rates.
 *	IMDATA: Structure containing measurement data and where resulting
 *          euler angles are stored.
 *	ATT: Structure containing quaternions.
 *	--------------------------------------------------------------- */

imuheader::imuheader()
{
	


	
}

void imuheader::process_attitude (double DT, imdata_t *IMDATA, att_t *ATT) {

	double ANGSPD[3], ACC[3], MAG[3];	// local copies of data
	double ERR_ACC, AGAIN;				// error-function value & adaptive gain
	double ABS;							// temporary values

	/* 1. DATA SOURCES */
	memcpy(ANGSPD, &IMDATA->ANGSPDF, sizeof(IMDATA->ANGSPDF));
	memcpy(ACC, &IMDATA->ACCF, sizeof(IMDATA->ACCF));
	memcpy(MAG, &IMDATA->MAGF, sizeof(IMDATA->MAGF));

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
	q_gyro(DT, ANGSPD, ATT);


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

	// interpolation
	ATT->Q_EST[0] = AGAIN * ATT->Q_ACCMAG[0] + (1-AGAIN) * ATT->Q_GYRO[0];
	ATT->Q_EST[1] = AGAIN * ATT->Q_ACCMAG[1] + (1-AGAIN) * ATT->Q_GYRO[1];
	ATT->Q_EST[2] = AGAIN * ATT->Q_ACCMAG[2] + (1-AGAIN) * ATT->Q_GYRO[2];
	ATT->Q_EST[3] = AGAIN * ATT->Q_ACCMAG[3] + (1-AGAIN) * ATT->Q_GYRO[3];
	qnorm(ATT->Q_EST);


	/* 5. CONVERT TO EULER ANGLES */
	process_convertEA(ATT->Q_EST, IMDATA->ATTITUDE, &IMDATA->MAGHDG);

	return;

}

/* attitude/process_q_accmag.c ---------------------------------------
 *	Estimation of current attitude using acceleration and magnetic
 *	field data as follows:
 *		1. Calculate acceleration-quaternion describing roll & pitch.
 *		2. Rotate magnetic field data with acceleration-quaternion.
 *		3. Calculate magnetic-quaternion describing yaw.
 *		4. Fuse both quaternions to get resulting acc-mag-quaternion.
 *	------------------------------------------------------------------
 *	ACC: Measured acceleration data.
 *	MAG: Measured magnetic field data.
 *	ATT: Structure containing quaternions.
 *	--------------------------------------------------------------- */

void imuheader::q_accmag(double ACC[3], double MAG[3], att_t *ATT) {

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
	qprod(MAG_Q, ATT->Q_ACC, MAG_Q);
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
	qnorm(ATT->Q_MAG);


	/* 4. FUSED (ACCMAG) QUATERNION */
	qprod(ATT->Q_ACC, ATT->Q_MAG, ATT->Q_ACCMAG);
	qnorm(ATT->Q_ACCMAG);

	return;

}

/* attitude/process_q_gyro.c -----------------------------------------
 *	Estimation of current attitude using angular rates.
 *	------------------------------------------------------------------
 *	DT: Timestep (seconds) used for integration.
 *	ANGSPD: Measured angular rates.
 *	ATT: Structure containing quaternions.
 *	--------------------------------------------------------------- */

void imuheader::q_gyro(double DT, double ANGSPD[3], att_t *ATT) {

	double ANG, S, DQ[4];

	/* rotation angle */
	ANG = DT*sqrt(ANGSPD[0]*ANGSPD[0] + ANGSPD[1]*ANGSPD[1] + ANGSPD[2]*ANGSPD[2]);

	/* update quaternion (conjugated) */
	// real component
	DQ[0] = cos(ANG/2);

	// imaginary components
	S = (-1) * DT * sin(ANG/2) / ANG;
	DQ[1] = ANGSPD[0] * S;
	DQ[2] = ANGSPD[1] * S;
	DQ[3] = ANGSPD[2] * S;

	// normalize
	qnorm(DQ);

	/* calculate gyro quaternion */
	qprod(DQ, ATT->Q_EST, ATT->Q_GYRO);

	// normalize
	qnorm(ATT->Q_GYRO);

}


/* attitude/func_dotprod.c -------------------------------------------
 *	Returns dotproduct of two 4-element vectors (quaternions) Q and P
 *	having the form Q = Q[0] + Q[1]*i + Q[2]*j + Q[3]*k (P just like Q).
 *	--------------------------------------------------------------- */

double imuheader::dotprod(double Q[4], double P[4]) {

	return Q[0]*P[0] + Q[1]*P[1] + Q[2]*P[2] + Q[3]*P[3];

}

/* attitude/func_qnorm.c ---------------------------------------------
 *	Returns normalized 4-element vector (quaternion) Q having the form
 *	Q = Q[0] + Q[1]*i + Q[2]*j + Q[3]*k.
 *	--------------------------------------------------------------- */

void imuheader::qnorm(double Q[4]) {

	double P[4], ABS;

	ABS = sqrt(Q[0]*Q[0] + Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3]);

	P[0] = Q[0] / ABS;
	P[1] = Q[1] / ABS;
	P[2] = Q[2] / ABS;
	P[3] = Q[3] / ABS;

	memcpy(Q, P, sizeof(P));

	return;

}

/* attitude/func_qprod.c ---------------------------------------------
 *	Calculate quaternion product PQ = P*Q.
 *	------------------------------------------------------------------
 *	P,Q: Quaternions Q[0] + Q[1]*i + Q[2]*j + Q[3]*k (P just like Q)
 *       to be multiplied, where Q describes a rotation followed by
 *		 the rotation P.
 *	PQ: Resulting quaternion.
 *	--------------------------------------------------------------- */

void imuheader::qprod(double P[4], double Q[4], double PQ[4]) {

	double PROD[4];

	PROD[0] = P[0]*Q[0] - P[1]*Q[1] - P[2]*Q[2] - P[3]*Q[3];
	PROD[1] = P[0]*Q[1] + P[1]*Q[0] + P[2]*Q[3] - P[3]*Q[2];
	PROD[2] = P[0]*Q[2] - P[1]*Q[3] + P[2]*Q[0] + P[3]*Q[1];
	PROD[3] = P[0]*Q[3] + P[1]*Q[2] - P[2]*Q[1] + P[3]*Q[0];

	memcpy(PQ, PROD, sizeof(PROD));

	return;

}

/* attitude/func_adaptive_gain.c -------------------------------------
 *	Returns gain of acceleration&magnetic quaternion.
 *	------------------------------------------------------------------
 *	ERR: Gravity error. In unaccelerated flight, ERR should be zero.
 *	     In accelerated flight, ERR should be positive.
 *	return: Gain of acceleration&magnetic quaternion. Range between
 *          zero and (1-GYRO_GAIN).
 *	--------------------------------------------------------------- */

double imuheader::func_adaptive_gain(double ERR) {

	double GAIN_FACTOR;

	if(ERR < ADGAIN_THR_LO)
        GAIN_FACTOR = 1;

    else if(ERR > ADGAIN_THR_HI)
        GAIN_FACTOR = 0;

    else
        GAIN_FACTOR = (double) (ADGAIN_THR_HI - ERR) / (ADGAIN_THR_HI - ADGAIN_THR_LO);

	return GAIN_FACTOR * (1-GYRO_GAIN);

}

/* attitude/process_convertEA.c --------------------------------------
 *	Converts quaternion to 3-2-1 euler angles.
 *	------------------------------------------------------------------
 *	Q: Quaternion Q[0] + Q[1]*i + Q[2]*j + Q[3]*k describing attitude.
 *	EA: Resulting roll/pitch/yaw following the 3-2-1 convention.
 *	--------------------------------------------------------------- */

void imuheader::process_convertEA(double Q[4], double EA[3], double *MAGHDG) {

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