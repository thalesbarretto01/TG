/*
 * COPYRIGHT (C) 2020 - Thales Antunes de Oliveira Barretto
 * All Rights Reserved.
 *
 * Unnoficial MPU6050 driver for Raspberry Pi 3B
 * 
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>

#include <gsl/gsl_fit.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/types.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#include "mpu6050.h"

/* constants */
#define MPU6050_ADDR	0x68
#define PI 		3.1415926535897932384
double r2d = 180.0 / PI ;
#define USAGE "Usage: mpu [sampling frequency]\n"
#define MPU_LOGFILE_MSG_SIZE 120

struct mpu_archive_header {
	char message[MPU_LOGFILE_MSG_SIZE];
	time_t time_start;
	time_t time_stop;
} logfile_header;

struct mpu_archive_data {
	long long sample_counter;
	int16_t raw[16];
	double data[16][2];

} logfile_data;

char mpu_ctl_log_filename[L_tmpnam] = "";
FILE * mpu_ctl_log_fp;

/* device file descriptor */
int mpu6050;

/* timing control */
struct timespec mpu_time_collect_start;
struct timespec mpu_time_collect_stop;
struct timespec mpu_time_loop_start;
struct timespec mpu_time_loop_run;
struct timespec mpu_time_loop_stop;
clock_t mpu_tick, mpu_tok;


unsigned long long counter_samples;
int16_t raw[16];
double scale[16];
double data[16][2];
double *Ax, *Ay, *Az;
double *Gx, *Gy, *Gz;
double *Thermo;

/* DATA - Calibration */
double calibration_gravity;
double calibration_offsets[16];
double calibration_gains[16];
double calibration_driftrate[16];
double calibration_ypr_accel[4];
double *Ax_offset, *Ay_offset, *Az_offset;
double *Gx_offset, *Gy_offset, *Gz_offset;
double *Thermo_offset;
double *Ax_gain, *Ay_gain, *Az_gain;
double *Gx_gain, *Gy_gain, *Gz_gain;
double *Thermo_gain;
double *Ax_driftrate, *Ay_driftrate, *Az_driftrate;
double *Gx_driftrate, *Gy_driftrate, *Gz_driftrate;
double *Thermo_driftrate;

double drifter[4][100];

/* DATA - Processing */
double squares[16];
double *Ax2, *Ay2, *Az2;
double *Gx2, *Gy2, *Gz2;
double *Axy2, *Axz2, *Ayz2;
double *Gxy2, *Gxz2, *Gyz2;
double A_magnitude, G_magnitude;
double ypr_compl[4], ypr_kalman[4], ypr_accel[4], ypr_gyro[4];
double rho;

/* DATA - Filtering */
double stats_mean[16];
double delta[16];
double stats_variance[16];
double stats_covariance[16][16];
double *Ax_mean, *Ay_mean, *Az_mean;
double *Gx_mean, *Gy_mean, *Gz_mean;
double *Thermo_mean;
double *Ax_variance, *Ay_variance, *Az_variance;
double *Gx_variance, *Gy_variance, *Gz_variance;
double *Thermo_variance;
double mpu_gsl_fit_linear_c0[16];
double mpu_gsl_fit_linear_c1[16];
double mpu_gsl_fit_linear_cov00[16];
double mpu_gsl_fit_linear_cov01[16];
double mpu_gsl_fit_linear_cov11[16];
double mpu_gsl_fit_linear_sumsq[16];

/* CONFIG  
 * 
 * TODO
 * use a device struct to hold configuration data
 */
double accel_fullrange, accel_lbs, accel_bandwidth, accel_delay;
double gyro_fullrange, gyro_lbs, gyro_bandwidth, gyro_delay;
double sampling_rate, sampling_time, samplerate_divisor;
uint8_t a_en, g_en, t_en;
uint16_t gyro_output_rate;
uint16_t fifo_max = 1023;
uint8_t dump[100][3];

/* CONFIG - default */
uint8_t mpu_cfg[10][2] = {
	{ 10,		   0},
	{ PWR_MGMT_1,  	0x01}, //1
	{ ACCEL_CONFIG,	0x00}, //2
	{ GYRO_CONFIG, 	0x00}, //3
	{ SMPLRT_DIV,	0x00}, //4
	{ CONFIG,      	0x00}, //5
	{ INT_PIN_CFG, 	0x00}, //6
	{ INT_ENABLE,  	0x00}, //7
	{ USER_CTRL,   	0x60}, //8
	{ FIFO_ENABLE, 	0x00}  //9
};

/* Device management functions */
void mpu_ctl_open(int *fd, int adapter_nr);
void mpu_ctl_start(int *fd);
void mpu_ctl_stop(int *fd);
void mpu_ctl_reset(int *fd);
void mpu_ctl_setclk(int *fd, uint8_t clksrc);
void mpu_ctl_sleep(int *fd);
void mpu_ctl_wake(int *fd);
void mpu_ctl_delay(void);
void mpu_ctl_ping(int *fd);
void mpu_ctl_flush_fifo(int *fd);
void mpu_ctl_siginterrupt_handler(int signum);

/* Device communication handlers */
void     mpu_comm_write_byte(int *fd, uint8_t reg, uint8_t val);
uint8_t  mpu_comm_read_byte(int *fd, uint8_t reg);
uint16_t mpu_comm_read_word(int *fd, uint8_t reg);
void     mpu_comm_read_burst(int *fd);
int 	 mpu_rnB(int *fd, uint8_t reg, uint8_t *val, uint8_t n);

/* Configuration handling functions */
/*
 *  TODO 
 *  void 	mpu_cfg_set_dlpf(int *fd, int dlf_config); 
 */
  void 	mpu_cfg_set_samplerate(int *fd, double sampling_rate);
  void 	mpu_cfg_set_accel(int *fd, uint16_t accel_range);
  void 	mpu_cfg_set_gyro(int *fd, uint16_t gyro_range);
  void 	mpu_cfg_set_thermo(int *fd);
  void 	mpu_cfg_get_scaling(int *fd);
  void 	mpu_cfg_get_table(int *fd);
  void 	mpu_cfg_set_table(int *fd, uint8_t reg, uint8_t val);
  void	mpu_cfg_write(int *fd);
  void	mpu_cfg_validate(int *fd);
uint8_t	mpu_cfg_read(int *fd, uint8_t reg);

/* Data handling functions */
void mpu_get_fifo(int *fd);
void mpu_get_calibration(int *fd, int seconds);
void mpu_set_calibration(int *fd);
void mpu_get_statistics(int *fd);
void mpu_get_angles(int *fd);

/* Data presenting functions */
void mpu_print_data(int *fd);
void mpu_print_calibration(int *fd);
void mpu_print_gravity(int *fd);
void mpu_print_angles(int *fd);
void mpu_print_statistics(int *fd);
void mpu_print_config(int *fd);
void mpu_ctl_atexit(void);
void mpu_print_gsl_lsf(int *fd);
void mpu_log_create(void);
void mpu_dumpreg(int *fd, char* filename);

int main( int argc, char **argv )
{

	struct mpu_flags {
	unsigned int sampling_rate = 0;
	unsigned int accel_range = 0;
	unsigned int gyro_range = 0;
	unsigned int sampling_flag = 0;
	unsigned int accel_flag = 0;
	unsigned int gyro_flag = 0;
	unsigned int thermo_flag = 0;
	unsigned int calibration_flag = 0;
	unsigned int calibration_time = 0;
	unsigned int dump_flag = 0;
	char arg_dump_filename[32];
	} ;
	
	/*
	 * Implemented options
	 *
	 * c	Calibrate
	 * d	Dump
	 * s	sampling frequency  TODO: min max values 
	 * a	accel full range
	 * g	gyro full range
	 * t	temperature
	 */
	int c;
	opterr=0;
	struct mpu_flags flag;
	while ( (c = getopt(argc, argv, "c:d:s:a:g:t")) != -1)
	{
		switch(c)
		{
		case 'c': /* Calibrate */
			flag.calibration_flag = 1;
			if ((sscanf(optarg, "%d", &flag.calibration_time)) == EOF) {
				fprintf(stderr,"error writing to flag.calibration_time");
				exit(1);
			}
			break;
		case 'd': /* dump */ 
			flag.dump_flag = 1;
			if ((sscanf(optarg, "%s", &flag.dump_filename)) == EOF) {
				fprintf(stderr,"error writing to flag.dump_filename");
				exit(1);
			}
		case 's': /* sampling frequency */
			flag.sampling_flag = 1;
			if ( (sscanf(optarg, "%d", &flag.sampling_rate)) == EOF) {
				fprintf(stderr,"error writing to flag.sampling_rate");
				exit(1);
			}
			/*
			switch(flag.sampling_rate)
			{
			case 50 : break;
			case 100 : break;
			case 200 : break;
			default :
				fprintf( stderr, "Invalid Sampling rate %s.\n", optarg);
				return 1;
			}
			*/
			break;
		case 'a': /* accel full range */
			flag.accel_flag = 1;
			if ((sscanf(optarg, "%d", &flag.accel_range)) == EOF) {
				fprintf(stderr, "error writing to flag.accel_range");
				exit(1);
			}
			switch (flag.accel_range)
			{
			case 2 : break;
			case 4 : break;
			case 8 : break;
			case 16 : break;
			default :
				fprintf( stderr, "Invalid accel range rate %s.\n", optarg);
				return 1;
			}
			break;
		case 'g': /* Gyro-range */
			flag.gyro_flag = 1;
			if ((sscanf(optarg, "%d", &flag.gyro_range)) == EOF) {
				fprintf(stderr, "Error writing to flag.gyro_range");
				exit(1);
			}
			switch (flag.gyro_range)
			{
			case 250 : break;
			case 500 : break;
			case 1000 : break;
			case 2000 : break;
			default :
				fprintf( stderr, "Invalid gyro range %s.\n", optarg);
				return 1;
			}
			break;
		case 't': /* temperature */
			flag.thermo_flag = 1;
			break;

		case '?':
			switch(optopt)
			{
			case 's': fprintf (stderr, "Option -%c requires an argument.\n", optopt);
			case 'a': fprintf (stderr, "Option -%c requires an argument.\n", optopt);
			case 'g': fprintf (stderr, "Option -%c requires an argument.\n", optopt);
			default :
				if (isprint(optopt)) {
					fprintf (stderr, "Unknown option -%c.\n", optopt);
				} else {
					fprintf (stderr, "Unknown option character '\\x%x'.\n", optopt);
					return 1;
				}
			}
		default: abort();
		}
	}


	printf( MSG_AXIS );
	printf( "Press a key to start" );
	atexit(mpu_ctl_atexit);
	getchar();

	mpu_log_create();

	mpu_setup:
	mpu_ctl_open(&mpu6050, 1);
	if( flag.sampling_flag == 1 ) {
		mpu_cfg_set_samplerate(&mpu6050,flag.sampling_rate);
	}
	if( flag.gyro_flag == 1 ) {
		mpu_cfg_set_gyro(&mpu6050,flag.gyro_range);
	}
	if( flag.accel_flag == 1 ) {
		mpu_cfg_set_accel(&mpu6050,flag.accel_range);
	}
	if( flag.thermo_flag ==1 ) {
		mpu_cfg_set_thermo(&mpu6050);
	}
	mpu_ctl_start(&mpu6050);
	if( flag.calibration_flag ==1 ) {
 		mpu_get_calibration(&mpu6050,flag.calibration_time);
	}
	mpu_print_config(&mpu6050);
	mpu_print_statistics(&mpu6050);
	mpu_print_calibration(&mpu6050);
	mpu_print_gsl_lsf(&mpu6050);
	mpu_ctl_flush_fifo(&mpu6050);
	if( flag.dump_flag ==1 ) {
 		mpu_dumpreg(&mpu6050,flag.dump_filename);
	}
	mpu_ctl_flush_fifo(&mpu6050);

	counter_samples = 0;
	timespec_get(&mpu_time_collect_start, TIME_UTC);
	mpu_loop:
	while(1)
	{
		timespec_get(&mpu_time_loop_start, TIME_UTC);
		mpu_get_fifo(&mpu6050);
		printf( "%-6llu ", counter_samples);
		mpu_set_calibration(&mpu6050);
		mpu_get_statistics(&mpu6050);
		mpu_get_angles(&mpu6050);
		mpu_print_data(&mpu6050);
		mpu_print_angles(&mpu6050);
		timespec_get(&mpu_time_loop_run, TIME_UTC);
		mpu_time_loop_run.tv_sec -= mpu_time_loop_start.tv_sec;
		mpu_time_loop_run.tv_nsec -= mpu_time_loop_start.tv_nsec;
//		printf( "loop time: %li seconds +  %li nanosec|",\
		mpu_time_loop_run.tv_sec, mpu_time_loop_run.tv_nsec);
		printf("\n");

	}
	return 0;
}

/* print running time */
void mpu_ctl_atexit(void)
{
	//time(&mpu_time_stop);
	//printf("Running time %.0f seconds\n", difftime(mpu_time_stop, mpu_time_loop_start) );
}


/*
 * Read data from buffer
 *
 * The only access to the FIFO buffer, keeps it in sync.
 *
 * After a sequece of data is read from the fifo buffer, the raw values are
 * scaled with respect to the range settings on each sensor and stored
 * into data[] which is used for further processing.
 *
 */
void mpu_get_fifo(int *fd)
{
//	printf("%s\n", __func__);
	uint16_t count;
	uint16_t many = 2*(raw[0]-1);
	uint16_t temp;
//	printf("Enabled: accel=%i gyro=%i temp=%i\n", a_en, g_en, t_en);
//	printf("Will read %i bytes - raw[0]=%i\n", many, raw[0]);
	do {
		temp = mpu_comm_read_word(fd, FIFO_COUNT_H);
/*
		count = temp >> 8;
		count |= (temp <<8);
*/
		count = (temp << 8) | (temp >> 8);
		if (count > 1000) {
			printf("BUFFER OVERFLOW -- empty buffer\n");
			mpu_ctl_flush_fifo(fd);
		}
	} while ( count < many  );
//	printf("read %i bytes\n", count);

	for (int i=1; i < raw[0]; i++){
//		printf("%i\n", i);
		raw[i] = mpu_comm_read_byte(fd, FIFO_R_W) <<8 ;
		raw[i] |= mpu_comm_read_byte(fd,FIFO_R_W);
	}

	// TODO: this is ugly, better use a rotation matrix.
	count = 1;
	/* fix accelerometer axis to match gyroscope */
	if ( a_en == 1 )	{
//		printf("Accel enabled - a_en = %i\n", a_en);
		raw[count] *= -1;
		count++;
		raw[count] *= -1;
		count++;
		raw[count] *= -1;
//		printf("Calculating |A|\n");
		}
	if ( t_en == 1)	{
//		printf("Thermo enabled - t_en = %i\n", t_en);
		count++;
	}
	if ( g_en == 1 ){
//		printf("Gyro enabled - g_en = %i\n", g_en);
		count++;
		count++;
		count++;
//		printf("Calculating |G|\n");
		G_magnitude = sqrt( (*Gx2) + (*Gy2) + (*Gz2) );
	}

	/* scale data */
	for (int i=1; i < raw[0]; i++){
		data[i][1] = data[i][0];
		data[i][0] = (raw[i]*scale[i]);
	}

	/* square data */
	if (a_en) {
		*Ax2 = *Ax * *Ax;
		*Ay2 = *Ay * *Ay;
		*Az2 = *Az * *Az;
		A_magnitude = sqrt( (*Ax2) + (*Ay2) + (*Az2) );
	}
	if (t_en){
		*Gx2 = *Gx * *Gx;
		*Gy2 = *Gy * *Gy;
		*Gz2 = *Gz * *Gz;
		G_magnitude = sqrt( (*Gx2) + (*Gy2) + (*Gz2) );

	}
//	printf("Incrementing sample\n");
	counter_samples++;
}

/*
 * Apply calibration gain and calibration_offsets on data
 */
void mpu_set_calibration(int *fd)
{
	for (int i=1; i < raw[0]; i++) {
		data[i][0] -= calibration_offsets[i];
		data[i][0] *= calibration_gains[i];
		}
}

/*
 * Get current Yaw, Pitch, Roll.
 */
void mpu_get_angles(int *fd)
{

	/* Integrate the average rate between two readings
	 * Take the drift rate into account
	 */
	if (g_en == 1) {
		ypr_gyro[1] += sampling_time * ( (*Gz + *(Gz+1))/2 - *Gz_driftrate);
		ypr_gyro[2] += sampling_time * ( (*Gy + *(Gy+1))/2 - *Gy_driftrate);
		ypr_gyro[3] += sampling_time * ( (*Gx + *(Gx+1))/2 - *Gx_driftrate);

	}
	/*
	 * Integrate the last rate (not the current)
	 *
	ypr_gyro[1] += sampling_time * ( *(Gz+1) - *Gz_driftrate);
	ypr_gyro[2] += sampling_time * ( *(Gy+1) - *Gy_driftrate);
	ypr_gyro[3] += sampling_time * ( *(Gx+1) - *Gx_driftrate);
	 *
	 */

	/*
	 * Angle Conventions
	 * [1]YAW   = (Gz) CCW rotation about z axis
	 * [2]PITCH = (Gy) CCW rotation about y axis
	 * [3]ROLL  = (Gx) CCW rotation about x axis
	 *
	** Those are meaningless without specifyng the order
	 * in wich they are applied. We show 2 useful forms
	 * (thatis, forms with 3 equations and 2 variables):
	 *
	 * FORM1 - ROT_xyz =	[-sin(theta),
	 *		  	cos(theta)sin(phi),
	 *			cos(theta)cos(phi)]
	 * Solves to:
	 * Pitch(theta) = atan2(-Ax/sqrt(Ay2+Az2);
	 * Roll(phi) 	= -atan2(Ay/Az);
	 */
	if (a_en == 1) {
		ypr_accel[2] =  r2d*(atan2( -(*Ax), sqrt((*Ay2)+(*Az2)) ));
		ypr_accel[3] = -r2d*(atan2(  (*Ay), -(*Az) )) ;
	}
	/* FORM2 - ROT_yxz = 	[ -sin(theta)cos(phi),
	 *			sin(phi)
	 *			cos(theta)cos(phi)]
	 *
	 * Solves to:
	 * Pitch(theta)_yzx = atan2(-Ax/Az);  <----SIMPLER
	 * Roll(phi)_yzx = atan2(Ay/sqrt(Ax2+Az2);
	 */
	/* ROT_YXZ form code
	ypr_accel[2] = r2d*(atan2(data[1][0], data[3][0]));
	ypr_accel[3] = r2d*(atan2(data[2][0],srqt(squares[1]+squares[3]));
	*/

	/* Remeber, algthough the cannonical form is
	 *
	 * FORM3 - ROT_zyx =	[cos(theta)sin(phi)sin(psi) - cos(psi)sin(theta),
	 *			cos(psi)cos(theta)sin(phi) + sin(theta)sin(psi),
	 *			cos(theta)cos(phi)]
	 *
	 * but... it includes the "yaw" (psi) angle wich
	 * accelerometers only can't provide
	 */

	/*
	* TILT ANGLE(rho) formula
	* At rest, |Gp| = 1g, so:
	* Gp = [Ax,Ay,Az];
	* |Gp| = sqrt(Ax2+Ay2+Az2);
	* rho = acos(Az/|Gp|);
	rho = r2d*acos(-data[3][0]/A_magnitude);
	*/
	/*
	* -
	* v. 2.
	* At rest: |Gp| = 1g,
	*
	* rho = atan2( sqrt( Ax2 + Ay2 ), Az );
	*/
	if (a_en == 1) {
	rho = r2d*atan2( sqrt( (*Ax2) + (*Ay2) ), -(*Az) );
	//rho = r2d*atan2( sqrt( squares[1] + squares[2] ), data[3][0] );
	}

/*	GYRO INTEGRATION
	ypr_accel[1] = ts*(data[6][0] - calibration_driftrate[3]);
	ypr_accel[2] = -ts*((data[5][0]) - calibration_driftrate[2]);
	ypr_accel[3] = ts*((data[4][0]) - calibration_driftrate[1]);
*/

}


/*
 * Build statistics from scaled data.
 */
void mpu_get_statistics(int *fd)
{
	double prev[8];

	for (int i=1; i < raw[0]; i++) {
		prev[i]  = delta[i];

		/* read last data*/
		stats_mean[i] = (data[i][0] + stats_mean[i])/2;
		delta[i] = data[i][0] - stats_mean[i];
		stats_variance[i]= (prev[i]*prev[i])+(delta[i]*delta[i]);
	}
}

/*
 * Get calibration data.
 */
void mpu_get_calibration(int *fd, int seconds)
{
	int times = seconds * sampling_rate;
	printf("Calibrating in %i seconds (%i samples) - Fs:%f Ts%f\n", seconds, times, sampling_rate, sampling_time);
	/* A HUGE amount of data */
	double caldata[16][times];
	double caldata_offsets[16];

	/* flush buffer */
	mpu_ctl_flush_fifo(fd);
	/* wait 200ms for stabilization */
	for( double t = 0.200 * sampling_rate; t > 0; t--) {
		mpu_get_fifo(fd);
	}

	printf("Gathering data...");
	/* read */
	for (int k = 0; k < times; k++) {
		caldata[0][k] = k*sampling_time;
		mpu_get_fifo(fd);
		for (int i = 1; i < raw[0]; i++) {
			caldata[i][k] = data[i][0];
			calibration_offsets[i] += data[i][0];
			calibration_driftrate[i] += data[i][0] - data[i][1];
			squares[i] = data[i][0]*data[i][0];
		}
		if (a_en == 1) {
			// printf("Accumulate G\n");
			calibration_gravity += sqrt( (*Ax2) + (*Ay2) + (*Az2) );
		}
	}
	printf("done\n");

	printf("Calibrating Linear least square fitting...");
	for (int i = 1; i < raw[0]; i++) {
		gsl_fit_linear(caldata[0], 1, \
			caldata[i], 1, \
			times,
			&mpu_gsl_fit_linear_c0[i], \
			&mpu_gsl_fit_linear_c1[i], \
			&mpu_gsl_fit_linear_cov00[i], \
			&mpu_gsl_fit_linear_cov01[i], \
			&mpu_gsl_fit_linear_cov11[i], \
			&mpu_gsl_fit_linear_sumsq[i]);
	}
	printf("done\n");


	/* normalize */
	for (int i = 1; i < raw[0]; i++) {
		calibration_offsets[i] /= times;
		calibration_driftrate[i] /= times;
		stats_mean[i] = calibration_offsets[i];
	}
	calibration_gravity /= times;
	printf("Galibration Gravity = %f \n", calibration_gravity);


	/* setup */
	/* Accelerometers won't bias, only scale */
	if ( a_en == 1 ) {
	*Ax_gain = 1/calibration_gravity;
	*Ay_gain = 1/calibration_gravity;
	*Az_gain = 1/calibration_gravity;
	*Ax_offset = 0;
	*Ay_offset = 0;
	*Az_offset = 0;
	*Ax_driftrate = 0;
	*Ay_driftrate = 0;
	*Az_driftrate = 0;
	}
	/* Thermo has fixed calibration */
	if ( t_en == 1) {
	*Thermo_gain = 1;
	*Thermo_offset = -36.5;
	*Thermo_driftrate = 0;
	}
	if ( g_en == 1 ) {
	/* Gyroscopes won't scale */
	*Gx_gain = 1;
	*Gy_gain = 1;
	*Gz_gain = 1;
	}


	printf("Getting calibration ypr...\n");
	for (int k= 0; k < times; k++) {
		mpu_get_fifo(fd);
		mpu_get_angles(fd);
		for (int i =1; i < 4; i++) {
			calibration_ypr_accel[i] += ypr_accel[i];
		}
	}
	for (int i = 1; i < 4; i++) {
		calibration_ypr_accel[i] /= times;
		ypr_gyro[i] = calibration_ypr_accel[i];
	}
	printf("done\n");

	printf( "Getting calibration statistics..." );
	/* Accumulate the square of the deviation */
	for (int k = 0; k < times; k++) {
		for (int i = 1; i < raw[0]; i++) {
			stats_variance[i] += (caldata[i][k] - stats_mean[i])*(caldata[i][k] - stats_mean[i]);
		}
	}
	/* divide by (n-1), take the square root */
	for (int i = 1; i < raw[0]; i++) {
		stats_variance[i] /= (double)(times -1);
		stats_variance[i] = sqrt(stats_variance[i]);
	}
	printf( "done\n" );
}

/*
 * PRINTING
 * ========
 */

/*
 * Print scaled data in tabular format
 */
void mpu_print_data(int *fd)
{
//	printf("|A|%+2.3f |G|%+2.3f", A_magnitude, G_magnitude);
	if (a_en == 1)
		printf(	" Ax%+06.10f Ay%+06.10f Az%+06.10f", *Ax, *Ay, *Az);
	if (t_en == 1)
		printf( " Tp: %+06.10f", *Thermo);
	if (g_en == 1)
		printf(	" Gx%+06.10f Gy%+06.10f Gz%+06.10f", *Gx, *Gy, *Gz);
}
/*
 * Print calibration in tabular format
 */
void mpu_print_calibration(int *fd)
{
	printf( "=================================================\n"  \
		"       Calibration results\n" \
		"-------------------------------------------------\n" \
		"SENSOR        %-13s\t%-13s\n", \
		"Offset", "Gain" );

	if (a_en == 1 ) {
		printf(	\
		"Ax            %+06.10f\t%+06.10f\n"  \
	        "Ay            %+06.10f\t%+06.10f\n"  \
		"Az            %+06.10f\t%+06.10f\n", \
		*Ax_offset, *Ax_gain, \
		*Ay_offset, *Ay_gain, \
		*Az_offset, *Az_gain);\
	}
	if (t_en == 1 ) {
		printf( \
		"Thermo        %+06.10f\t%+06.10f\n", \
		*Thermo_offset, *Thermo_gain);
	}
	if (g_en == 1) {
		printf( \
		"Gx            %+06.10f\t%+06.10f\n" \
	        "Gy            %+06.10f\t%+06.10f\n" \
	        "Gz            %+06.10f\t%+06.10f\n", \
		*Gx_offset, *Gx_gain, \
		*Gy_offset, *Gy_gain, \
		*Gz_offset, *Gz_gain );
	}
	printf(	"-------------------------------------------------\n" );

}

/*
 * Print total g force
 */
void mpu_print_gravity(int *fd)
{
	printf(" Total calibration_gravity: %+3.12f \n", calibration_gravity );
}

/*
 * print current Yaw, Pitch, Roll.
 */
void mpu_print_angles(int *fd)
{
	printf("A:y%+03.02fp%+03.02fr:%+03.02ft%+03.02f ", ypr_accel[1], ypr_accel[2], ypr_accel[3], rho);
	printf("G:Y%+03.02fP%+03.02fR:%+03.02f", ypr_gyro[1], ypr_gyro[2], ypr_gyro[3]);
}

/*
 * Build statistics from scaled data.
 */
void mpu_print_statistics(int *fd)
{
	/* 52 characters-wide message */
	printf( "=================================================\n"  \
		"       Statistics results\n" \
		"-------------------------------------------------\n" \
		"SENSOR        %-13s\t%-13s\n", \
		"mean", "variance" );

	if (a_en == 1) {
		printf(	\
		"Ax            %+06.10f\t%+06.10f\n"  \
	        "Ay            %+06.10f\t%+06.10f\n"  \
		"Az            %+06.10f\t%+06.10f\n", \
		*Ax_mean, *Ax_variance, \
		*Ay_mean, *Ay_variance, \
		*Az_mean, *Az_variance);\
	}
	if (t_en == 1) { printf( \
		"Thermo %+06.10f\t%+06.10f\n", \
		 *Thermo_mean, *Thermo_variance);
	}
	if (g_en == 1) {
		printf( \
		"Gx            %+06.10f\t%+06.10f\n" \
	        "Gy            %+06.10f\t%+06.10f\n" \
	        "Gz            %+06.10f\t%+06.10f\n", \
		*Gx_mean, *Gx_variance, \
		*Gy_mean, *Gy_variance, \
		*Gz_mean, *Gz_variance );
	}
	printf("-------------------------------------------------\n" );

}

/*
 * Build GSL Linear Least-square fit from scaled data.
 */
void mpu_print_gsl_lsf(int *fd)
{
	/* 52 characters-wide message */
	printf( "=================================================\n"  \
		"       GSL Least-square fit results\n" \
		"-------------------------------------------------\n" \
		"SENSOR        %-13s\t%-13s\n", \
		"Linear", "Angular" );
	int count =1;
	if (a_en == 1) {
		printf(	\
		"Ax            %+06.10f\t%+06.10f\n"  \
	        "Ay            %+06.10f\t%+06.10f\n"  \
		"Az            %+06.10f\t%+06.10f\n", \
		mpu_gsl_fit_linear_c0[count], mpu_gsl_fit_linear_c1[count], \
		mpu_gsl_fit_linear_c0[count+1], mpu_gsl_fit_linear_c1[count+1], \
		mpu_gsl_fit_linear_c0[count+2], mpu_gsl_fit_linear_c1[count+2]); \
		count += 3;
	}
	if (t_en == 1) { printf( \
		"Thermo %+06.10f\t%+06.10f\n", \
		 *Thermo_mean, *Thermo_variance);
		count++;
	}
	if (g_en == 1) {
		printf( \
		"Gx            %+06.10f\t%+06.10f\n" \
	        "Gy            %+06.10f\t%+06.10f\n" \
	        "Gz            %+06.10f\t%+06.10f\n", \
		mpu_gsl_fit_linear_c0[count], mpu_gsl_fit_linear_c1[count], \
		mpu_gsl_fit_linear_c0[count+1], mpu_gsl_fit_linear_c1[count+1], \
		mpu_gsl_fit_linear_c0[count+2], mpu_gsl_fit_linear_c1[count+2]); \
		count += 3;
	}
	printf("-------------------------------------------------\n" );

}


/*
 * READING AND WRITING
 * ====================
 */

/*
 * Write 1 Byte data to register
 */
void mpu_comm_write_byte(int *fd, uint8_t reg, uint8_t val)
{
	__s32 temp;
	temp = i2c_smbus_write_byte_data(*fd,reg,val);
//	printf("WRITE_BYTE %x %s - %x\n", reg, mpu_regnames[reg], val);
	if (temp != 0) {
		fprintf(stderr, "Error writing: %20s  %02x  code:%i\n", mpu_regnames[reg], val, temp);
		exit(1);
	}
}

/*
 * Read 1 Byte data from register
 */
 uint8_t mpu_comm_read_byte(int *fd, uint8_t reg)
{
	uint8_t val;
	__s32 retvalue = i2c_smbus_read_byte_data(*fd, reg);
//	printf("READ_BYTE %x %s - %x\n", reg, mpu_regnames[reg], retvalue);
	if( retvalue < 0 ) {
		fprintf(stderr,"Error reading %20s %02x code: %i\n", mpu_regnames[reg], reg, retvalue );
		exit(1);
	} else 	return val = retvalue;
}


/*
 * Read 2 Byte data (1 Word) from register
 */

 uint16_t mpu_comm_read_word(int *fd, uint8_t reg)
{
	uint8_t val;
	__s32 retvalue = i2c_smbus_read_word_data(*fd, reg);
//	printf("READ_WORD %x %s - %x\n", reg, mpu_regnames[reg], retvalue);
	if( retvalue < 0 ) {
		printf( "Error writing data - code: %i\n", retvalue );
		exit( 1 );
	} else 	return val = retvalue;
}

/*
 * TESTING
 * ========
 */

/*
 * Reads "n" Bytes from mpu
 * Read the contents of registers starting at "reg"
 * on the target device with "addr", and puts as many
 * bytes into "val" as the size of "val".
 *
 * Parameters
 *
 * file - file descriptor associated to an i2c bus master
 * 	which is returned by "open("/dev/i2c-1", O_RDWR)"
 *
 * addr - the address of the target slave device in the bus
 *
 * reg	- the address of the target register on target device
 *
 * val - receives the contents of the target register
 * 	 val[0] contains the number of bytes to be read
 *
 */

void mpu_comm_read_burst(int *fd)
{
	printf("Entering %s\n", __func__);

	/* Prepare to read Accel  */
	uint8_t val[7];
	mpu_cfg_validate(fd);
	printf("Calling mpu_rnB...\n");
	mpu_rnB(fd,ACCEL_XOUT_H, val, 0x06);

	printf("Press a key to exit...\n");
	getchar();
	close(*fd);
	exit(0);
}
int mpu_rnB(int *fd,
//	unsigned char addr,  USE MPU6050_ADDR
	uint8_t reg,
	uint8_t *val,
	uint8_t n)
{
	printf("Entering %s - arguments: fd:%i reg:%02x val:%p, n%02x \n", __func__, fd, reg, val, n);
	printf("creating packets\n");
	/*
	 * We need to pass this as argument to the ioctl.
	 *
	 * Now, however, we must have 2 messages, one to
	 * send the address of the target register, other to
	 * store the contents of the target register.
	 *
	 * SEE THE I2C SPECIFICATION - MAYBE INCORRECT
	 * This is due to structure of the i2c messaging.
	 * In order to read a register, we do:
	 * 1-START->WRITE:ADDR+WRITE->wait for ACK
	 * 2->WRITE->REGADDR->wait for ACK
	 * 3->STOP
	 * 4->START->WRITE:ADDR+READ->wait for ACK
	 * 5->READ:value->send ACK
	 * 6->READ:value->send ACK ...
	 * 	7-->if done,send NACK
	 *	8->STOP
	 */
	struct i2c_rdwr_ioctl_data packets;

	/*
	 * In order to read a register, we first do a "dummy write" by writing
	 * 0 bytes to the register we want to read from.  This is similar to
	 * the packet in set_i2c_register, except it's 1 byte rather than 2.
	 *
	 * The first message contains the "write" part
	 * The second message contains the "read" part
	 */
	struct i2c_msg messages[2];
	uint8_t outbuf	  = reg;
	messages[0].addr  = MPU6050_ADDR;
	messages[0].flags = 0;
	messages[0].len   = sizeof(outbuf);
	messages[0].buf   = &outbuf;
	uint8_t inbuf[n];
	messages[1].addr  = MPU6050_ADDR;
	messages[1].flags = I2C_M_RD /* | I2C_M_NOSTART */ ;
	messages[1].len   = sizeof(inbuf);
	messages[1].buf   = inbuf;

	/*
	 * Send the request to the kernel and get the result back
	 */
	printf("building packet\n");
	packets.msgs      = messages;
	packets.nmsgs     = 2;

	printf("sending ioctrl\n");

	/*
	 * ioctl(file, I2C_RDWR, struct i2c_rdwr_ioctl_data *msgset)
	 *	Do combined read/write transaction without stop in between.
	 *	Only valid if the adapter has I2C_FUNC_I2C.
	 *
	 * The argument is a pointer to a:
	 *
	 *   struct i2c_rdwr_ioctl_data {
	 *	struct i2c_msg *msgs;		// ptr to array of simple messages
      	 *	int nmsgs;			//number of messages to exchange
    	 *	}
	 *
	 */

	if(ioctl(*fd, I2C_RDWR, &packets) < 0) {
		perror("Unable to send data");
		getchar();
		return 1;
	}

	/*
	 * If everything went ok, copy the reading to "val"
	 */
	printf("This is the read data:\n");
	/* *val = inbuf; */
	for (int i=0; i<n; i++){
	printf("inbuf[i]=%#02x\n",inbuf[i]);
	}
	getchar();
	return 0;
}



/*
 * DEVICE CONTROL
 * ==============
*/


/*
 * Open a file associated with the bus driver, attach the slave device.
 * Test for communication.
 *
 */
void mpu_ctl_open(int *fd, int adapter_nr)
{
	char filename[20];
	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
	*fd = open("/dev/i2c-1", O_RDWR);
	ioctl(*fd, I2C_SLAVE, MPU6050_ADDR);
	signal(SIGINT,mpu_ctl_siginterrupt_handler);
	mpu_ctl_ping(fd);
	mpu_ctl_wake(fd);
	mpu_ctl_setclk(fd,0);
}

/* Perform a startup sequence
 * Wakeup
 * Reset
 * ---> WRITE CONFIG
 * Set Clock Source
 * Set Gyro range
 * Set Accel Range
 * Set Sample rate
 * Set Interrupts
 * Set FIFO
 * ---> VALIDATE CONFIG
 * Enable FIFO
 * Stabilize - flush buffer
 * device, write config
 */
void mpu_ctl_start(int *fd)
{
	mpu_ctl_reset(fd);
	printf("%s\n", __func__);
	mpu_cfg_get_table(fd);
	mpu_cfg_write(fd);
	mpu_ctl_ping(fd);
	mpu_cfg_validate(fd);
	mpu_ctl_flush_fifo(fd);
}

/*
 * Perform a device reset and wait for it to complete.
 */
void mpu_ctl_reset(int *fd)
{
	uint8_t value = 0x80 | mpu_comm_read_byte(fd, PWR_MGMT_1);
	mpu_comm_write_byte(fd, PWR_MGMT_1, value);
	for (int i = 0; i < 50000; i++){};
}


/*
 * Configure the clock source setting [0-7]
 * TODO write to configuration table
 */
void mpu_ctl_setclk(int *fd, uint8_t clksrc)
{
	clksrc &= 0x07;
	clksrc |= mpu_comm_read_byte(fd, PWR_MGMT_1);
	mpu_comm_write_byte(fd, PWR_MGMT_1, clksrc);
}

/*
 * Set SLEEP bit, wich puts device in sleep low power mode
 * TODO write to configuration table
 */
void mpu_ctl_sleep(int *fd)
{
	mpu_comm_write_byte(fd, PWR_MGMT_1, 0x40);
}

/*
 * Clear SLEEP bit, waking up the device
 * TODO write to configuration table
 */
void mpu_ctl_wake(int *fd)
{
	uint8_t value = (~0x40) & mpu_comm_read_byte(fd, PWR_MGMT_1);
	/* recover from sleep mode */
	mpu_comm_write_byte(fd, PWR_MGMT_1, value);
	/* recover sensors from standby */
	mpu_comm_write_byte(fd, PWR_MGMT_2, 0x00);

}

/*
 * Handles SIGINTERRUPT
 */
void mpu_ctl_siginterrupt_handler(int signum) {
	printf("\nReceived signal INTERRUPT, aborting..\n");
	close(mpu6050);
	printf("....Done\n");
	exit(signum);
}

/*
 * Verify that the device is responding
 */
void mpu_ctl_ping(int *fd)
{
	uint8_t whoami = mpu_comm_read_byte(fd,  WHO_AM_I);

	if ( whoami != MPU6050_ADDR )
	{
		printf("ERROR: WHO_AM_I | response: %02x expected:%02x\n", whoami, MPU6050_ADDR);
		getchar();
		exit(1);
	}
}

/*
 * empty buffer by dummy readingg
 */
void mpu_ctl_flush_fifo(int *fd)
{
	printf("%s\n", __func__);
	int16_t temp, count;
	while ( count > 0) {
		temp = mpu_comm_read_word(fd, FIFO_COUNT_H);
		count = temp >> 8;
		count |= (temp <<8);
		temp = mpu_comm_read_byte(fd, FIFO_R_W) <<8 ;
		temp |= mpu_comm_read_byte(fd,FIFO_R_W);
	}
}

/*
 * NOOP
 */
void mpu_ctl_delay(void)
{
	for (int i = 0; i < 1000; i++){};
}

/*
 * Use to set the desired sampling rate.
 *
 * NOTE: too high will cause overflow
 *  Recommended: sampling_rate < 250
 */
void mpu_cfg_set_samplerate(int *fd, double sampling_rate)
{
	printf("%s\n", __func__);
	mpu_cfg_get_table(fd);
	uint8_t smplrt_div = (gyro_output_rate / sampling_rate) -1;
	mpu_cfg_set_table(fd, SMPLRT_DIV, smplrt_div);
}

/*
 * Configure the gryoscope range and enables FIFO buffering for its readings
 *
 * Valid ardumunts:
 * +-250 deg/s use 250
 * +-500 deg/s use 500
 * +-1000 deg/s use 1000
 * +-2000 deg/s use 2000
 */
void mpu_cfg_set_gyro(int *fd, uint16_t range)
{
	printf("%s\n", __func__);
	uint8_t fifov = (0x70 | mpu_cfg_read(fd, FIFO_ENABLE));
	uint8_t regv;
	switch (range)	{
	case 250 :	regv = 0x00; break;
	case 500 : 	regv = 0x01; break;
	case 1000 :	regv = 0x02; break;
	case 2000 :	regv = 0x03; break;
	default : 	regv = 0x00;
			fifov &= ~0x70;	break;
	}
	mpu_cfg_set_table(fd, FIFO_ENABLE, fifov);
	mpu_cfg_set_table(fd, GYRO_CONFIG, regv << 3);
	mpu_cfg_get_table(fd);
}

/*
 * Configure the gryoscope range and enables FIFO buffering for its readings
 *
 * Valid ranges:
 * +-2  g use 2
 * +-4  g use 4
 * +-8  g use 8
 * +-16 g use 16
 */
void mpu_cfg_set_accel(int *fd, uint16_t range)
{
//	printf("%s\n", __func__);
	uint8_t  fifov = (0x08 | mpu_cfg_read(fd,FIFO_ENABLE));
	uint8_t regv;
//	printf("%s range=%i regv=%x\n", __func__, range, regv);
	switch (range) {
	case 2 :	regv = 0x00; break;
	case 4 : 	regv = 0x01; break;
	case 8 :	regv = 0x02; break;
	case 16 :	regv = 0x03; break;
	default : 	regv = 0X00;
			fifov &= ~0x08;
			break;
	}
//	printf("%s got  regv=%x\n", __func__, regv);
//	printf("%s calling set_table FIFO_ENABLE\n", __func__);
	mpu_cfg_set_table(fd, FIFO_ENABLE, fifov);

//	printf("%s calling set_table ACCEL_CONFIG - passing regv=%x\n", __func__, regv);
	mpu_cfg_set_table(fd, ACCEL_CONFIG, regv << 3);

	printf("%s calling get_table\n", __func__);
	mpu_cfg_get_table(fd);
}

/* Enable temperature reading*/
void mpu_cfg_set_thermo(int *fd)
{
//	printf("%s\n", __func__);
	uint8_t  fifov = (0x80 | mpu_cfg_read(fd,FIFO_ENABLE));
	mpu_cfg_set_table(fd, FIFO_ENABLE, fifov);
	mpu_cfg_get_table(fd);
}

/*
 * Handles the setup of "scale" array
 *
 * Each entry is detemined by the range
 * settings stored in the config table
 */
void mpu_cfg_get_scaling(int *fd)
{
	int count = 1;
	if( a_en == 1)
	{
		for (int i=0; i < 3; i++) {
			scale[count] =1/accel_lbs;
			count++;
		}
	}

	if( t_en == 1)
	{
		scale[count]=1/340;
		count++;
	}
	if( g_en == 1)
	{
		for (int i=0; i < 3; i++) {
		 	scale[count] =1/gyro_lbs;
			count++;
		}
	}
}


/*
 * Read one config from table, return value
 */
 uint8_t mpu_cfg_read(int *fd, uint8_t reg)
{
	for (int i =1; i < mpu_cfg[0][0] ; i++)	{
		if ( mpu_cfg[i][0] == reg ) {
		 	return  mpu_cfg[i][1];
		}
	}
}

/*
 * Write config table to registers
 * Use mpu_ctl_start() for proper handling.
 */
void mpu_cfg_write(int *fd)
{
	int temp;
	printf("\nWriting config registers\n");
	for (int i =1; i < mpu_cfg[0][0] ; i++)	{
		 mpu_comm_write_byte(fd, mpu_cfg[i][0], mpu_cfg[i][1]);
		/* wait some time */
		for (int i = 0; i < 50000; i++){};
	}
}

/*
 * Checks config table against the actual device registers
 */
void mpu_cfg_validate(int *fd)
{
	int failed = 0;
	uint8_t temp[16][3];
	temp[0][0] = mpu_cfg[0][0];
	for (int i = 1; i < mpu_cfg[0][0]; i++)	{
		temp[i][0] = mpu_cfg[i][0];
		temp[i][1] = mpu_cfg[i][1];
		temp[i][2] = mpu_comm_read_byte(fd, mpu_cfg[i][0]);
	}
	/* 4 columns
	 * 20 1 = Register name
	 * 6  2 = Expec.
	 * 6  3 = Actual
	 * 5  4 = PASS/FAIL
	 */
	printf( "=================================================\n"  \
		"       Config Validation\n" \
		"-------------------------------------------------\n");

	for (int i = 1; i < temp[0][0] ; i++) {
		printf(	"%20s %02x vs %02x  ",\
			mpu_regnames[temp[i][0]], \
			temp[i][1], \
			temp[i][2] );

		if ( failed = (temp[i][1] != temp[i][2]) )
			printf( "%-5s\n", "FAIL");
		else
			printf( "%-5s\n", "PASS");
	}
	if (failed) {
		printf( "\n   CONFIG VALIDATION FAIL \n" );
		getchar();
	} else {
		printf( "\n   CONFIG VALIDATION PASS \n" );
	}
	printf(	"-------------------------------------------------\n" );

}

/*
 * Print config table data to the screen
 */
void mpu_print_config(int *fd)
{
	printf(" CONFIG	\tVALUE\n");
	printf(" accel_fullrange \t%-8.8f\n", accel_fullrange);
	printf(" accel_lbs \t%-8.8f\n", accel_lbs);
	printf(" accel_bandwidth \t%-8.8f\n", accel_bandwidth);
	printf(" accel_delay \t%-8.8f\n", accel_delay);
	printf(" gyro_fullrange \t%-8.8f\n", gyro_fullrange);
	printf(" gyro_lbs \t%-8.8f\n", gyro_lbs);
	printf(" gyro_bandwidth \t%-8.8f\n", gyro_bandwidth);
	printf(" gyro_output_rate \t%-17u\n" ,gyro_output_rate);
	printf(" sampling_rate \t%-8.8f\n", sampling_rate);
	printf(" samplerate_divisor \t%-17u\n", samplerate_divisor);
}

/*
 * Set and update config table
 */
void mpu_cfg_set_table(int *fd, uint8_t reg, uint8_t val)
{
	printf("%s\n", __func__);
	for (int i =1; i < mpu_cfg[0][0] ; i++)	{
		if ( mpu_cfg[i][0] == reg ) {
			mpu_cfg[i][1] = val;
		}
	}
	printf("%s calling get_table\n", __func__);
	mpu_cfg_get_table(fd);
}

/*
 * Get attributes according to config
 *
 * SENSITITVITY
 * Readings are 16-bit signed variables
 * so their effective range is (65536-1)
 *
 * SAMPLING FREQUENCY
 * sampling_rate = gyro_out_rate / samplerate_divisor
 */
void mpu_cfg_get_table(int *fd)
{
	printf("%s\n", __func__);
	/* scan the config table for settings */
	for (int i =1; i < mpu_cfg[0][0]; i++) {
//		printf("%s %i searching %s\n", __func__, i, mpu_regnames[mpu_cfg[i][0]]);
		switch ( mpu_cfg[i][0] ) {
		case FIFO_ENABLE :
			/* CRUCIAL - raw[0] = active sensors + 1 */
			switch (mpu_cfg[i][1]) {
			case 0x08 : a_en = 1; g_en = 0; t_en = 0; raw[0] = 4; break;
			case 0x78 : a_en = 1; g_en = 1; t_en = 0; raw[0] = 7; break;
			case 0xF8 : a_en = 1; g_en = 1; t_en = 1; raw[0] = 8; break;
			case 0x88 : a_en = 1; g_en = 0; t_en = 1; raw[0] = 5; break;
			case 0x70 : a_en = 0; g_en = 1; t_en = 0; raw[0] = 4; break;
			case 0xF0 : a_en = 0; g_en = 1; t_en = 1; raw[0] = 5; break;
			case 0x80 : a_en = 0; g_en = 0; t_en = 1; raw[0] = 2; break;
			case 0x00 : a_en = 0; g_en = 0; t_en = 0; raw[0] = 1; break;
			default :
				printf("SHOULD NOT BE HERE found FIFO_ENABLE value: %x", mpu_cfg[i][1]);
				a_en = 0; g_en = 0; t_en = 0; raw[0] = 0;
				getchar();
				break;
			} break;
		case ACCEL_CONFIG :
			switch ( (mpu_cfg[i][1]) >> 3 ) {
			case 0x00 :
				accel_fullrange = 2.0;
				break;
			case 0x01 :
				accel_fullrange = 4.0;
				break;
			case 0x02 :
				accel_fullrange = 8.0;
				break;
			case 0x03 :
				accel_fullrange = 16.0;
				break;
			default :
				printf("SHOULD NOT BE HERE ACCEL_CONFIG value: %x", mpu_cfg[i][1]);
				getchar();
				break;
			} break;
		case GYRO_CONFIG :
			switch ( (mpu_cfg[i][1]) >> 3 ) {
			case 0x00 : gyro_fullrange = 250.0; break;
			case 0x01 : gyro_fullrange = 500.0; break;
			case 0x02 : gyro_fullrange = 1000.0; break;
			case 0x03 : gyro_fullrange = 2000.0; break;
			default :
				printf("SHOULD NOT BE HERE GYRO_CONFIGvalue: %x", mpu_cfg[i][1]);
				getchar();
				break;

			} break;
		case CONFIG :
			switch (mpu_cfg[i][1]) {
			case 0x00 :
				accel_bandwidth = 260.0;
				accel_delay = 0;
				gyro_bandwidth = 256.0;
				gyro_delay = 0.98;
				gyro_output_rate = 8000;
				break;
			case 0x01 :
				accel_bandwidth = 184.0;
				accel_delay = 2.0;
				gyro_bandwidth = 188.0;
				gyro_delay = 1.9;
				gyro_output_rate = 1000;
				break;
			case 0x02 :
				accel_bandwidth = 94.0;
				accel_delay = 3.0;
				gyro_bandwidth = 98;
				gyro_delay = 2.8;
				gyro_output_rate = 1000;
				break;
			case 0x03 :
				accel_bandwidth = 44.0;
				accel_delay = 4.9;
				gyro_bandwidth = 42.0;
				gyro_delay = 4.0;
				gyro_output_rate = 1000;
				break;
			case 0x04 :
				accel_bandwidth = 21.0;
				accel_delay = 8.5;
				gyro_bandwidth = 20.0;
				gyro_delay = 8.3;
				gyro_output_rate = 1000;
				break;
			case 0x05 :
				accel_bandwidth = 10.0;
				accel_delay = 13.8;
				gyro_bandwidth = 10.0;
				gyro_delay = 13.4;
				gyro_output_rate = 1000;
				break;
			case 0x06 :
				accel_bandwidth = 5.0;
				accel_delay = 19.0;
				gyro_bandwidth = 5.0;
				gyro_delay = 18.6;
				gyro_output_rate = 1000;
				break;
			case 0x07 :
				accel_bandwidth = 0;
				accel_delay = 0;
				gyro_bandwidth = 0;
				gyro_delay = 0;
				gyro_output_rate = 8000;
				break;
			default :
//				printf("SHOULD NOT BE HERE CONFIG value: %x", mpu_cfg[i][1]);
//				getchar();
				break;
			} break;
		default :
//			printf("Config not found\n");
//			getchar();
			break;
		}

	}

	/* THIS MUST BE DONE AFTER THE ABOVE STEPS
	 * Because the sampling frequency depends on TWO SETTINGS
	 * - the DLPF and the SMPLRT_DIV
	 *
	 * Thereforem sampling_rate and sampling_time are determined
	 * by a second order effect.
	 */
	for (int i =1; i < mpu_cfg[0][0] ; i++)	{
		if ( mpu_cfg[i][0] == SMPLRT_DIV ) {
			samplerate_divisor = (mpu_cfg[i][1] +1) ;
			sampling_rate = gyro_output_rate / samplerate_divisor ;
			sampling_time = 1 / sampling_rate;
		}
	}

	/*
	 * find LBS - Lowest Bit Lensitivity
	 *
	 * 16 bits signed data range
	 * max: 2⁽¹⁶⁻¹⁾   = +32768;
	 * min: 2⁽¹⁶⁻¹⁾-1 = -32767;
	 */
	accel_lbs = 32768.0/accel_fullrange;
	gyro_lbs = 32768.0/gyro_fullrange;

	mpu_cfg_get_scaling(fd);

	/* Associate data with meaningful names */
	int count = 1;
	if ( a_en == 1 ) {
		// 1
		Ax = &data[count][0];
		Ax2 = &squares[count];
		Ax_offset = &calibration_offsets[count];
		Ax_gain = &calibration_gains[count];
		Ax_driftrate = &calibration_driftrate[count];
		Ax_mean = &stats_mean[count];
		Ax_variance = &stats_variance[count];
		count++;
		// 2;
		Ay = &data[count][0];
		Ay2 = &squares[count];
		Ay_offset = &calibration_offsets[count];
		Ay_gain = &calibration_gains[count];
		Ay_driftrate = &calibration_driftrate[count];
		Ay_mean = &stats_mean[count];
		Ay_variance = &stats_variance[count];
		count++;
		// 3
		Az = &data[count][0];
		Az2 = &squares[count];
		Az_offset = &calibration_offsets[count];
		Az_gain =&calibration_gains[count];
		Az_driftrate = &calibration_driftrate[count];
		Az_mean = &stats_mean[count];
		Az_variance = &stats_variance[count];
		count++;
	}
	if ( t_en == 1 )	{
		Thermo = &data[count][0];
		Thermo_offset = &calibration_offsets[count];
		Thermo_gain =&calibration_gains[count];
		Thermo_driftrate = &calibration_driftrate[count];
		Thermo_mean = &stats_mean[count];
		Thermo_variance = &stats_variance[count];
		count++;
	}
	if ( g_en == 1 )	{
		Gx = &data[count][0];
		Gx2 = &squares[count];
		Gx_offset = &calibration_offsets[count];
		Gx_gain =&calibration_gains[count];
		Gx_driftrate = &calibration_driftrate[count];
		Gx_mean = &stats_mean[count];
		Gx_variance = &stats_variance[count];
		count++;
		Gy = &data[count][0];
		Gy2 = &squares[count];
		Gy_offset = &calibration_offsets[count];
		Gy_gain =&calibration_gains[count];
		Gy_driftrate = &calibration_driftrate[count];
		Gy_mean = &stats_mean[count];
		Gy_variance = &stats_variance[count];
		count++;
		Gz = &data[count][0];
		Gz2 = &squares[count];
		Gz_offset = &calibration_offsets[count];
		Gz_gain =&calibration_gains[count];
		Gz_driftrate = &calibration_driftrate[count];
		Gz_mean = &stats_mean[count];
		Gz_variance = &stats_variance[count];
	}

	/* TODO Why????? */
	for (int i=1; i < raw[0]; i++){
		data[i][1] = data[i][0];
		data[i][0] = (raw[i]*scale[i]);
	}
}


/*
 * TODO 
 * Implement these functions
 * mpu_dumpreg - dumps configuration resgiters to a file
 * mpu_log_create - log events to file
 */
void mpu_dumpreg(int *fd, char *filename)
{
	FILE *fp = fopen( filename, "w+");
	printf("Generating config dump array\n");
	fprintf(fp, "Config dump for MPU-6050\n");
	fprintf(fp,"%8s %8s %20% %4s", "reg(hex)", "reg(dec)", "name", "val");
	uint8_t mpureg[128][3];
	for (int i=0; i <128; i++) {
		mpureg[i][0] = i;
		mpureg[i][1] = mpu_comm_read_byte(fd, mpureg[i][0]);
		fprintf(fp, " %2x     %3d    %-20s %2x\n",mpureg[i][0], mpureg[i][0], mpu_regnames[mpureg[i][0]], mpureg[i][1]);
		printf( " %2x     %3d    %-20s %2x\n",mpureg[i][0], mpureg[i][0], mpu_regnames[mpureg[i][0]], mpureg[i][1]);
	}
	fflush(fp);
	fclose(fp);
	printf("--DONE--\n\n");
	getchar();
}

/* Creat a binary logfile */
void mpu_log_create(void)
{
	/*
	struct mpu_archive_header {
		char message[MPU_LOGFILE_MSG_SIZE];
		time_t time_start;
		time_t time_stop;
	} logfile_header;

	struct mpu_archive_data {
		long long sample_counter;
		int16_t raw[16];
		double data[16][2];

	} logfile_data;

	char mpu_ctl_log_filename[L_tmpnam] = "";
	FILE * mpu_ctl_log_fp;
	*/
	mpu_ctl_log_fp = fopen( "mpu6050.log", "w+");
	snprintf(logfile_header.message,MPU_LOGFILE_MSG_SIZE, "%s", "Testing" );
	time(&logfile_header.time_start);
	fwrite(&logfile_header, sizeof(logfile_header), 1, mpu_ctl_log_fp);
	fflush(mpu_ctl_log_fp);
	fclose(mpu_ctl_log_fp);
}
