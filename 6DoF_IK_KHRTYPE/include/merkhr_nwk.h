#include "params.h"
#include <math.h>

void IK_Rleg_Rotation(float,float,float,float);
void IK_Lleg_Rotation(float,float,float,float);
float Trj_Complement(float,float,int,float*,int);
float Maxmin(float uin,float umin,float umax);
float CalcDir(float uin);

float AnsQuadEqBig(float A, float B, float C/*Ax^2+Bx+C=0*/);
float AnsQuadEqSmall(float A, float B, float C/*Ax^2+Bx+C=0*/);

float* Rot(float x[3], float y[3], short axis/*1:x,2:y,3:z*/, float deg);
float* Rot2(float x[3], short axis/*1:x,2:y,3:z*/, float deg);
float* CrossProduct(float x1[3], float x2[3], float y[3]);
float InnerProduct(float x1[3], float x2[3]);
float* Normalize(float x[3], float y[3]);
float* Normalize2(float x[3]);
float AngleOfVecs(float x1[3], float x2[3]);
float* SumVect(float y[3], float dy[3],short bairitsu);
float* SumVect2(float x[3], float dx[3], float y[3],short bairitsu);
float CalcNorm(float x[3]);
float* DiscompVect(float v[3], float p[3], float q[3], float s_t_[2]);
float* Euler_PitchRollYawRotation(float x[3]/*mm*/, float y[3], float Pitch/*deg*/, float Roll, float Yaw, float Roll2, float Pitch2 );
void LegIK2test(/*IcsHardSerialClass krs_R,*/ float xd_x/*mm*/,float xd_y, float xd_z, float Rolld/*deg*/,float Pitchd, float Yawd, int i);