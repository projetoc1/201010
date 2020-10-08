#ifndef imuheader_h
#define imuheader_h
// Header
#include <stdio.h>
#include <string.h>
#include <math.h>
// Fix values


#define ADGAIN_THR_LO (0.1)		// lower error value threshold (for gain factor one)
#define ADGAIN_THR_HI (0.2)		// upper error value threshold (for gain factor zero)
#define GYRO_GAIN (0.99)							// percentage of gyro data in attitude estimation
#define MAGNETIC_DECLINATION (0.063704517697793)	// local magnetic declination [rad]

	typedef struct
	{
		double Q_ACC[4];		// acceleration-dependent quaternion
		double Q_MAG[4];		// magnetic field-dependent quaternion
		double Q_ACCMAG[4];		// acc&mag-fused quaternion
		double Q_GYRO[4];		// angular speed-dependent quaternion
		double Q_EST[4];		// finally estimated attitude
	}att_t;

	typedef struct
	{
		const short ACID;				// aircraft id
		double ANGSPD[3], ANGSPDF[3];	// angular velocities (unfiltered/filtered) of body-fixed axes [rad/s]
		double ATTITUDE[3];				// euler angles describing estimated attitude [rad]
		double ACC[3], ACCF[3];			// linear accelerations (unfiltered/filtered) of body-fixed axes [g]
		double VERTSPD;					// vertical speed [m/s]
		double ADACC;					// aerodynamic acceleration [m/s^2]
		double ADSPD, ADSPDF;			// aerodynamic speed (unfiltered/filtered) [m/s]
		double BAROALT, BAROALTF;		// barometric altitude (unfiltered/filtered) [m]
		double MAG[3], MAGF[3];			// magnetic field components (unfiltered/filtered) of body-fixed axes [gauss]
		double MAGHDG;					// magnetic heading [rad]
		int RCDATA[7];					// local copy of rc data and status [1]
	}imdata_t;

class imuheader
{
public:
	imuheader();
	
	void process_attitude(double DT, imdata_t *IMDATA, att_t *ATT);
	void q_accmag(double ACC[], double MAG[], att_t *ATT);
	void q_gyro(double DT, double ANGSPD[], att_t *ATT);
	void qnorm(double Q[]);
	void qprod(double P[], double Q[], double PQ[]);
	double dotprod(double Q[], double P[]);
	double func_adaptive_gain(double ERR);
	void process_convertEA(double Q[], double EA[], double *MAGHDG);

};
#endif 
// typedef struct {
// 	double Q_ACC[4];		// acceleration-dependent quaternion
// 	double Q_MAG[4];		// magnetic field-dependent quaternion
// 	double Q_ACCMAG[4];		// acc&mag-fused quaternion
// 	double Q_GYRO[4];		// angular speed-dependent quaternion
// 	double Q_EST[4];		// finally estimated attitude
// } att_t;

// typedef struct {
// 	const short ACID;				// aircraft id
// 	double ANGSPD[3], ANGSPDF[3];	// angular velocities (unfiltered/filtered) of body-fixed axes [rad/s]
// 	double ATTITUDE[3];				// euler angles describing estimated attitude [rad]
// 	double ACC[3], ACCF[3];			// linear accelerations (unfiltered/filtered) of body-fixed axes [g]
// 	double VERTSPD;					// vertical speed [m/s]
// 	double ADACC;					// aerodynamic acceleration [m/s^2]
// 	double ADSPD, ADSPDF;			// aerodynamic speed (unfiltered/filtered) [m/s]
// 	double BAROALT, BAROALTF;		// barometric altitude (unfiltered/filtered) [m]
// 	double MAG[3], MAGF[3];			// magnetic field components (unfiltered/filtered) of body-fixed axes [gauss]
// 	double MAGHDG;					// magnetic heading [rad]
// 	int RCDATA[7];					// local copy of rc data and status [1]
// } imdata_t;


// // Function prototypes
// void process_attitude(double DT, imdata_t *IMDATA, att_t *ATT);
// void q_accmag(double ACC[], double MAG[], att_t *ATT);
// void q_gyro(double DT, double ANGSPD[], att_t *ATT);
// void qnorm(double Q[]);
// void qprod(double P[], double Q[], double PQ[]);
// double dotprod(double Q[], double P[]);
// double func_adaptive_gain(double ERR);
// void process_convertEA(double Q[], double EA[], double *MAGHDG);
