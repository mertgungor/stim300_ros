#include <stdio.h>
#include <math.h>
#define N_PI (3.141592653589793)
#define M_A (9.80665)

class LinAccInt{
	public:
		LinAccInt(double val,double freq){
			printf("Constructor created \n");
			beta = val;
			sampleFreq = freq;
			printf("beta: %lf, sampleFreq: %lf \n",beta,sampleFreq);
		}
		LinAccInt(){

		}
		
		void MadgwickIMU(double ax, double ay, double az, double gx, double gy, double gz){
			double qDot1 = 0.5 * (-q[1] * gx - q[2] * gy - q[3] * gz);
			double qDot2 = 0.5 * (q[0] * gx + q[2] * gz - q[3] * gy);
			double qDot3 = 0.5 * (q[0] * gy - q[1] * gz + q[3] * gx);
			double qDot4 = 0.5 * (q[0] * gz + q[1] * gy - q[2] * gx);
			
			double nrm = sqrt(ax * ax + ay * ay + az * az);
			ax /= nrm; ay /= nrm; az /= nrm;
			
			double k2q0 = 2.0 * q[0];
			double k2q1 = 2.0 * q[1];
			double k2q2 = 2.0 * q[2];
			double k2q3 = 2.0 * q[3];
			
			double k4q0 = 4.0 * q[0];
			double k4q1 = 4.0 * q[1];
			double k4q2 = 4.0 * q[2];
			
			double k8q1 = 8.0 * q[1];
			double k8q2 = 8.0 * q[2];
			
			double q0q0 = q[0] * q[0];
			double q1q1 = q[1] * q[1];
			double q2q2 = q[2] * q[2];
			double q3q3 = q[3] * q[3];
			
			double s0 = k4q0 * q2q2 + k2q2 * ax + k4q0 * q1q1 - k2q1 *ay;
			double s1 = k4q1 * q3q3 - k2q3 * ax + 4.0 * q0q0 * q[1] - k2q0 * ay - k4q1 + k8q1 * q1q1 + k8q1 * q2q2 + k4q1 *az;
			double s2 = 4.0 * q0q0 * q[2] + k2q0 * ax + k4q2 * q3q3 - k2q3 * ay - k4q2 + k8q2 * q1q1 + k8q2 * q2q2 + k4q2 *az;
			double s3 = 4.0 * q1q1 * q[3] - k2q1 * ax + 4.0 * q2q2 * q[3] - k2q2 * ay;
			
			double nrms = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
			s0 /= nrms; s1 /= nrms; s2 /= nrms; s3 /= nrms;
			
			qDot1 = qDot1 - beta*s0; qDot2 = qDot2 - beta*s1; qDot3 = qDot3 - beta*s2; qDot4 = qDot4 - beta*s3;
			
			q[0] = q[0] + qDot1 * (1.0 / sampleFreq);
			q[1] = q[1] + qDot2 * (1.0 / sampleFreq);
			q[2] = q[2] + qDot3 * (1.0 / sampleFreq);
			q[3] = q[3] + qDot4 * (1.0 / sampleFreq);
			
			nrm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
			q[0] /= nrm; q[1] /= nrm; q[2] /= nrm; q[3] /= nrm;		
			
		}
		
		void printQuat(){
			printf("q0: %.15f, q1: %.15f, q2: %.15f, q3: %.15f\n",q[0],q[1],q[2],q[3]);
		}

		double* get_quat(){
			return q;
		}
		
		void calc_and_set_Quat(double ax, double ay, double az, double heading){
			double r = atan2(ay,az);
			double p = atan((-ax) / (sqrt(ay * ay + az * az)));
			double y = heading;
			
			q[0] = cos(r/2.0) * cos(p/2.0) * cos(y/2.0) + sin(r/2.0) * sin(p/2.0) * sin(y/2.0);
			q[1] = sin(r/2.0) * cos(p/2.0) * cos(y/2.0) - cos(r/2.0) * sin(p/2.0) * sin(y/2.0);
			q[2] = cos(r/2.0) * sin(p/2.0) * cos(y/2.0) + sin(r/2.0) * cos(p/2.0) * sin(y/2.0);
			q[3] = cos(r/2.0) * cos(p/2.0) * sin(y/2.0) - sin(r/2.0) * sin(p/2.0) * cos(y/2.0);			
		}
		
		void go_back_eul(){
			double q1 = q[0]; double q2 = -q[1]; double q3 = -q[2]; double q4 = -q[3];
			double R11 = 2.0 * q1* q1 - 1.0 + 2.0 * q2 * q2;
			double R21 = 2.0 * (q2 * q3 - q1 * q4);
			double R31 = 2.0 * (q2 * q4 + q1 * q3);
			double R32 = 2.0 * (q3 * q4 - q1 * q2);
			double R33 = 2.0 * q1 * q1 - 1.0 + 2.0 * q4 * q4;
			
			eul[0] = atan2(R32,R33); //roll
			eul[1] = -atan((R31) / (sqrt(1.0 - R31 * R31))); //pitch
			eul[2] = atan2(R21,R11); //yaw
		}
		
	private:
		double q[4] = {1,0,0,0};
		double beta = 0.5;
		double sampleFreq = 250.0;
		//double v[3] = {0,0,0};
		double eul[3] = {0,0,0};
		double s = -1.0;	
		
};
