
#include "../include/merkhr_nwk.h"


//ダンピング制御用変数：気にしないでOK
float delta_x;
float delta_y;

float delta_xL;
float delta_yL;

float normalizeAngle(float angle) {
    // 角度を [0, 360) の範囲にラップ
    angle = fmod(angle + 360.0f, 360.0f);
    
    // [0, 360) を [-180, 180) に変換
    if (angle > 180.0f)
        angle -= 360.0f;
    
    return angle;
}


float Maxmin(float uin,float umin,float umax){
	// float uout=uin;
	if(uin>umax){
        uin=umax;
    }else if(uin<umin){
        uin=umin;
    }
	return uin;
}

float CalcDir(float uin){
	// float uout=uin;
	float u2=fabsf(uin);
	float y=0;
	if(u2!=0)y=uin/u2;
	return y;
}

float* Rot(float x[3], float y[3], short axis/*1:x,2:y,3:z*/, float deg){
	/*
	 *ベクトルxをaxis回りにdegだけ回転して返す
	 * */

	float Rot[3][3] = {};
	float cosdeg=cos(deg/180*PI);
	float sindeg=sin(deg/180*PI);

	////回転行列準備
	if(axis==1){//x軸回りの回転

		Rot[0][0]=1;  Rot[0][1]=0;      Rot[0][2]=0;
		Rot[1][0]=0;  Rot[1][1]=cosdeg; Rot[1][2]=-sindeg;
		Rot[2][0]=0;  Rot[2][1]=sindeg; Rot[2][2]=cosdeg;

	}else if(axis==2){//y軸回りの回転

		Rot[0][0]=cosdeg;  Rot[0][1]=0; Rot[0][2]=sindeg;
		Rot[1][0]=0;  	   Rot[1][1]=1; Rot[1][2]=0;
		Rot[2][0]=-sindeg; Rot[2][1]=0; Rot[2][2]=cosdeg;

	}else if(axis==3){//z軸周りの回転

		Rot[0][0]=cosdeg;  Rot[0][1]=-sindeg; Rot[0][2]=0;
		Rot[1][0]=sindeg;  Rot[1][1]=cosdeg;  Rot[1][2]=0;
		Rot[2][0]=0; 	   Rot[2][1]=0;       Rot[2][2]=1;

	}

	short i;

	for(i=0;i<3;i++){
		//回転後のベクトルを返す
		y[i] = Rot[i][0]*x[0]+Rot[i][1]*x[1]+Rot[i][2]*x[2];
	}

	return y;
}

float* Rot2(float x[3], short axis/*1:x,2:y,3:z*/, float deg){
	/*
	 *ベクトルxをaxis回りにdegだけ回転して返す
	 *引数をそのまま変換するver
	 * */
	float y[3]={};

	float Rot[3][3] = {};
	float cosdeg=cos(deg/180*PI);
	float sindeg=sin(deg/180*PI);

	//回転行列準備
	if(axis==1){//x軸回りの回転

		Rot[0][0]=1;  Rot[0][1]=0;      Rot[0][2]=0;
		Rot[1][0]=0;  Rot[1][1]=cosdeg; Rot[1][2]=-sindeg;
		Rot[2][0]=0;  Rot[2][1]=sindeg; Rot[2][2]=cosdeg;

	}else if(axis==2){//y軸回りの回転

		Rot[0][0]=cosdeg;  Rot[0][1]=0; Rot[0][2]=sindeg;
		Rot[1][0]=0;  	   Rot[1][1]=1; Rot[1][2]=0;
		Rot[2][0]=-sindeg; Rot[2][1]=0; Rot[2][2]=cosdeg;

	}else if(axis==3){//z軸周りの回転

		Rot[0][0]=cosdeg;  Rot[0][1]=-sindeg; Rot[0][2]=0;
		Rot[1][0]=sindeg;  Rot[1][1]=cosdeg;  Rot[1][2]=0;
		Rot[2][0]=0; 	   Rot[2][1]=0;       Rot[2][2]=1;

	}

	short i;

	for(i=0;i<3;i++){
		//回転後のベクトルを返す
		y[i] = Rot[i][0]*x[0]+Rot[i][1]*x[1]+Rot[i][2]*x[2];
	}

	for(i=0;i<3;i++){
		//回転後のベクトルを返す
		x[i] = y[i];
	}

	return x;
}

float* CrossProduct(float x1[3], float x2[3], float y[3]/*出力*/){
	/*
	 * x1,x2の外積yを返す関数
	 * */
	y[0]=x1[1]*x2[2]-x1[2]*x2[1];
	y[1]=x1[2]*x2[0]-x1[0]*x2[2];
	y[2]=x1[0]*x2[1]-x1[1]*x2[0];

	return y;
}
float InnerProduct(float x1[3], float x2[3]){
	/*
	 * x1,x2の内積を返す関数
	 */
	return x1[0]*x2[0]+x1[1]*x2[1]+x1[2]*x2[2];
}

float* Normalize(float x[3], float y[3]/*?o??*/){
	/*
	 * x方向の大きさ1のベクトルyを返す関数
	 * */

	float norm = sqrtf(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
	if(norm>0){
		y[0]=x[0]/norm; y[1]=x[1]/norm; y[2]=x[2]/norm;
	}else{
		y[0]=0; y[1]=0; y[2]=0;
	}

	return y;
}
float* Normalize2(float x[3]/*入力兼出力*/){
	/*
	 * ベクトルxの大きさを正規化して返す
	 * */

	float norm = sqrtf(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
	if(norm>0){
		x[0]=x[0]/norm; x[1]=x[1]/norm; x[2]=x[2]/norm;
	}else{
		x[0]=0; x[1]=0; x[2]=0;
	}

	return x;
}
float* ScalarTimes(float x[3]/*入力兼出力*/, float bairitsu){
	/*
	 * ベクトルxをbairitsu倍して返す
	 * */

	x[0]=x[0]*bairitsu; x[1]=x[1]*bairitsu; x[2]=x[2]*bairitsu;

	return x;
}
float AngleOfVecs(float a[3], float b[3]){
	/*
	 * 二つのベクトルx1, x2のなす角を求める関数
	 * 戻り値はrad
	 * ！！！負の値は出せない！！！
	 */
	float x1[3]={a[0],a[1],a[2]};
	float x2[3]={b[0],b[1],b[2]};

	float rad;
	Normalize2(x1);
	Normalize2(x2);

	rad=acos((x1[0]*x2[0]+x1[1]*x2[1]+x1[2]*x2[2])
			/(sqrtf(x1[0]*x1[0]+x1[1]*x1[1]+x1[2]*x1[2])
			*sqrtf(x2[0]*x2[0]+x2[1]*x2[1]+x2[2]*x2[2])));

	return rad;

}

float* SumVect(float y[3], float dy[3],short bairitsu){
	/*
	 * ベクトルyにdyのbairitsu倍を足して返す関数
	 */
		y[0]=y[0]+bairitsu * dy[0];
		y[1]=y[1]+bairitsu * dy[1];
		y[2]=y[2]+bairitsu * dy[2];

	return y;
}

float* SumVect2(float x[3], float dx[3], float y[3],short bairitsu){
	/*
	 * ベクトルxにdxを足したyを返す関数
	 */
	y[0]=x[0]+bairitsu * dx[0];
	y[1]=x[1]+bairitsu * dx[1];
	y[2]=x[2]+bairitsu * dx[2];

	return y;
}

float CalcNorm(float x[3]){

	float norm=x[0]*x[0]+x[1]*x[1]+x[2]*x[2];

	return norm;

}

float* DiscompVect(float v[3], float p[3], float q[3], float s_t_[2]){
	/*
	 * v = s・p + t・qに対し, v, p, qが既知の時にs, tを求める
	 */

	s_t_[0]=InnerProduct(p, v);
	s_t_[1]=InnerProduct(q, v);

	return s_t_;
}

float SelectMin3(float u1,float u2,float u3){
	/*
	入力の最小値を求めて返す（3入力）
	*/
	float min=u1;
	if(u2<min)min=u2;
	if(u3<min)min=u3;
	return min;
}

float AnsQuadEqBig(float A, float B, float C/*Ax^2+Bx+C=0*/){
	/*
	Ax^2+Bx+C=0の解の大きいほうを返す
	*/	
	return (-B+sqrtf(B*B-4*A*C))/(2*A);
}

float AnsQuadEqSmall(float A, float B, float C/*Ax^2+Bx+C=0*/){
	/*
	Ax^2+Bx+C=0の解の大きいほうを返す
	*/	
	return (-B-sqrtf(B*B-4*A*C))/(2*A);
}

float CalcNearestAnglePos(float deg_old, float deg_new){
	/*
		等価な+-2PIor+-0の角度に対し、前回値に最も近い値を返す。
	*/

	float deg1=deg_new-2*180;
	float deg2=deg_new;
	float deg3=deg_new+2*180;
	float return_deg=deg_old;

	float size1=fabsf(deg1-deg_old);
	float size2=fabsf(deg2-deg_old);
	float size3=fabsf(deg3-deg_old);

	float min=SelectMin3(size1,size2,size3);

	if(min==size1){
		return_deg=deg1;
	}else if(min==size2){
		return_deg=deg2;
	}else{
		return_deg=deg3;
	}
	return_deg=Maxmin(return_deg, -180, 180);
	return return_deg;
}

float* Euler_PitchRollYawRotation(float x[3]/*mm*/, float y[3], float Pitch/*deg*/, float Roll, float Yaw, float Pitch2, float Roll2){
	/*
	 * xのPitch Roll Yawの順番のオイラー角回転yを求める
	 * ここでの順序変えることで、オイラー角の回転順序を変えて指定できる
	 * 座標変換行列は回転行列とはかける順序逆なこと注意
	 * (y=RotY*RotR*RotY*x)
	 */
	y[0]=x[0];
	y[1]=x[1];
	y[2]=x[2];

	Rot2(y,1,Roll2);
	Rot2(y,2,-Pitch2);

	Rot2(y,3,Yaw);
	Rot2(y,1,Roll);
	Rot2(y,2,Pitch);

	return y;
}


void LegIK2test(/*IcsHardSerialClass krs_R, */float xd_x/*mm*/,float xd_y, float xd_z, float Rolld/*deg*/,float Pitchd, float Yawd, int i/*R:0, L:1*/){

	/*
	 *KHRタイプ軸構成6DoFIK関数
	 *x,y,z,roll,pitch,yawを指定可能
	 *しかし、気合で解いたためどうやって解いたかのドキュメントが残っていない
	 *標準姿勢付近ではだいたい正しいはずだが、pitch or roll姿勢がPI/2くらい大きいとバグる
	 *とりあえずそれらしく動くが、おそらく探せばたくさんのバグがあるので注意
	*/

	//単位ベクトル定義
	float normal_z[3]={0,0,1};
	float normal_x[3]={1,0,0};
	float normal_y[3]={0,1,0};

	// float lyofs=5.0;//足の付け根yaw軸のオフセット

	//xd求める
	float xd[3]={};
	if(i==0){
		xd_y=-xd_y;
	}else{}
	xd[0]=xd_x;
	xd[1]=xd_y;
	xd[2]=xd_z;

	//xp求める
	float xp[3]={};
	float lsoleofs_z[3]={0,0,lsoleoffset};
	//くるぶし球面座標から見たxpを回転
	if(i==0){
		Euler_PitchRollYawRotation(lsoleofs_z, xp, Pitchd, Rolld, Yawd, delta_x, -delta_y);
	}else{
		Euler_PitchRollYawRotation(lsoleofs_z, xp, Pitchd, Rolld, Yawd, delta_xL, delta_yL);
	}
	SumVect(xp, xd, 1);//xpを原点から見た値に変換

	//v求める
	float v_NotRotated[3]={};
	float v_Rotated[3]={};
	float v[3]={};

	SumVect2(normal_x, lsoleofs_z, v_NotRotated, 1);//くるぶし球面座標から見たv導出
	//くるぶし球面座標から見たvを回転
	if(i==0){
		Euler_PitchRollYawRotation(v_NotRotated, v_Rotated, Pitchd, Rolld, Yawd, delta_x, -delta_y);
	}else{
		Euler_PitchRollYawRotation(v_NotRotated, v_Rotated, Pitchd, Rolld, Yawd, delta_xL, delta_yL);
	}
	SumVect2(v_Rotated, xd, v, 1);//vを原点から見た値に変換

	//v_dash求める
	float v_dash[3]={};
	SumVect2(v,xp ,v_dash, -1);

	//theta0求める
	float A_dash=v_dash[1]-v_dash[2]/xp[2]*xp[1];
	float B_dash=v_dash[0]-v_dash[2]/xp[2]*xp[0];
	float theta0=0;

	if(fabsf(v_dash[0])>th_value&&fabsf(v_dash[1])<th_value&&fabsf(v_dash[2])<th_value){
		theta0=0;
	}else if(fabsf(v_dash[0])<th_value&&fabsf(v_dash[1])>th_value&&fabsf(v_dash[2])<th_value){
		theta0=-PI/2;
	}else{
		theta0=asin(-v_dash[2] / xp[2] * lyofs / sqrtf(A_dash*A_dash + B_dash*B_dash)) - atan2(A_dash, B_dash);
	}
	//xo求める
	float xo[3]={};
	xo[0]=lyofs*sin(theta0);
	xo[1]=lyofs*cos(theta0);
	xo[2]=0;

	//xp_dash求める(p_plane上でのxp)
	float xp_dash[3]={};
	SumVect2(xp, xo ,xp_dash, -1);

	//p(p-planeの水平方向基底ベクトル)求める
	float p[3]={};
	p[0]=cos(theta0);
	p[1]=-sin(theta0);
	p[2]=0;

	//n_pr求める(p, qの法線ベクトル, p_planeの法線ベクトル)
	float n_pq[3]={};
	CrossProduct(p, xp_dash, n_pq);//n_pqが0になったらv_dashで再計算するのも良いかも
	if(sqrtf(n_pq[0]*n_pq[0] + n_pq[1]*n_pq[1]+n_pq[2]*n_pq[2])<th_value){
		CrossProduct(p, v_dash, n_pq);
	}
	Normalize2(n_pq);

	//q(p-planeのもう一つの基底ベクトル)求める
	float q[3]={};
	CrossProduct(n_pq, p, q);
	Normalize2(q);

	//theta1求める
	float theta1=AngleOfVecs(xo, q)-PI/2;//0<theta1<PIならこれで対応可能、可動範囲的には十分

	//thetap求める
	float v_s_t[2]={};//s,t座標から見たv
	DiscompVect(v_dash, p, q, v_s_t);//p-plane上での表現求めてatanで求めるver, 計算量一定で自然
	float thetap=atan2(-v_s_t[1],v_s_t[0]);

	//pのp-plane座標(s, t)を求める
	float p_s_t[2]={};
	DiscompVect(xp_dash, p, q, p_s_t);
	float s=p_s_t[0];
	float t=p_s_t[1];

	//theta2,3,4求める
	float t_dash=t-loffset*(1+cos(thetap));
	float s_dash=s-loffset*sin(thetap);//	float s_dash=s+loffset*sin(thetap);
	float theta2=acos((s_dash*s_dash+t_dash*t_dash+l1*l1-l2*l2)/(2*l1*sqrtf(s_dash*s_dash+t_dash*t_dash)))+atan2(s_dash, t_dash);//acos??-?????H
	float theta3=atan2(s_dash-l1*sin(theta2),t_dash-l1*cos(theta2))-theta2;//膝
	if(theta3>0){theta3=theta3-2*PI;}//バグへの応急処置
	float theta4=thetap-theta2-theta3;
	
	//xpd求める
	float xpd[3]={};
	SumVect2(xd, xp ,xpd, -1);

	//theta5求める
	float theta5=AngleOfVecs(n_pq,xpd)-PI/2;//-PI/2<theta5<PI/2????????OK

	//足サーボ書き込み関数に渡す(ここでradからdegに変換)
	if(i==0){
		// Write_angle_R_Leg(/*-*/theta0/PI*180,-theta1/PI*180,-theta2/PI*180,theta3/PI*180,theta4/PI*180,theta5/PI*180);
	}else{
		// Write_angle_L_Leg(theta0/PI*180,theta1/PI*180,-theta2/PI*180,theta3/PI*180,theta4/PI*180,-theta5/PI*180);
	}
	printf("theta0~5 : %f, %f, %f. %f. %f, %f\n", theta0, theta1, theta2, theta3, theta4, theta5);
}