#include "MotionCal.h"

MotionCal::MotionCal() {
	UpPlatform = new Platform_InitParameter;
	DownPlatform = new Platform_InitParameter;
	InitParam();
}

MotionCal::~MotionCal() {
	
	delete UpPlatform;
	delete DownPlatform;

}
void MotionCal::InitParam() {
	
	/*中位时平台动坐标系相对静坐标系的坐标*/
	vector<double> Upplatform{ 0, 0, 815.95 };
	//vector<double> Upplatform{ 0, 0, -815.95 };
	vector<double> Downplatform{ 0, 0, 999.7 };
	
	/*动平台铰点布置角度*/
	vector<double> Upplatform1Angle{ -9, -111, -129, 129, 111, 9 };
	//vector<double> Upplatform1Angle{ -51, -69, 189, 171, 69, 51 };
	vector<double> Downplatform1Angle{ -7.5, -112.5, -127.5, 127.5, 112.5, 7.5 };
	
	//静平台铰点布置角度
	vector<double> Upplatform2Angle{ -51, -69, 189, 171, 69, 51 };
	//vector<double> Upplatform2Angle{ -9, -111, -129, 129, 111, 9 };
	vector<double> Downplatform2Angle{ -50, -70, 190, 170, 70, 50 };

	//上平台参数初始化
	//静平台r，动平台R
	UpPlatform->r = 650;
	//UpPlatform->r = 600;
	UpPlatform->R = 600;
	//UpPlatform->R = 650;
	UpPlatform->l_mid = 130;
	UpPlatform->L_mid = 932.0;//疑点
	UpPlatform->mMotionParam.X = 0;
	UpPlatform->mMotionParam.Y = 0;
	UpPlatform->mMotionParam.Z = 0;
	UpPlatform->mMotionParam.ROLL = 0;
	UpPlatform->mMotionParam.PITCH = 0;
	UpPlatform->mMotionParam.YAW = 0;

	UpPlatform->Platform1 = Upplatform;
	UpPlatform->Platform1Angle = Upplatform1Angle;
	UpPlatform->Platform2Angle = Upplatform2Angle;

	//下平台参数初始化
	DownPlatform->r = 1200;
	DownPlatform->R = 1100;
	DownPlatform->l_mid = 230;
	//DownPlatform->L_mid = 1304.977;
	DownPlatform->L_mid = 1304.99;
	DownPlatform->mMotionParam.X = 0;
	DownPlatform->mMotionParam.Y = 0;
	DownPlatform->mMotionParam.Z = 0;
	DownPlatform->mMotionParam.ROLL = 0;
	DownPlatform->mMotionParam.PITCH = 0;
	DownPlatform->mMotionParam.YAW = 0;

	DownPlatform->Platform1 = Downplatform;
	DownPlatform->Platform1Angle = Downplatform1Angle;
	DownPlatform->Platform2Angle = Downplatform2Angle;

}

void MotionCal::InputParam(Platform_InitParameter *Plat,float x, float y, float z, float roll, float pitch, float yaw) {
	
	Plat->mMotionParam.X = x;
	Plat->mMotionParam.Y = y;
	Plat->mMotionParam.Z = z;
	Plat->mMotionParam.ROLL = roll;
	Plat->mMotionParam.PITCH = pitch;
	Plat->mMotionParam.YAW = yaw;

}
/*求杆长Lenth*/
void MotionCal::Calculating(Platform_InitParameter *Plat) {
	
	//动平台圆心点
	P = { { Plat->mMotionParam.X + Plat->Platform1[0] }, { Plat->mMotionParam.Y + Plat->Platform1[1] }, { Plat->mMotionParam.Z + Plat->Platform1[2] } };
	
	//动平台的6个铰点，在动平台坐标系中的位置矢量
	bR1 = { { Plat->R * cos(Plat->Platform1Angle[0] * PI / 180) }, { Plat->R * sin(Plat->Platform1Angle[0] * PI / 180) }, { 0 } };
	bR2 = { { Plat->R * cos(Plat->Platform1Angle[1] * PI / 180) }, { Plat->R * sin(Plat->Platform1Angle[1] * PI / 180) }, { 0 } };
	bR3 = { { Plat->R * cos(Plat->Platform1Angle[2] * PI / 180) }, { Plat->R * sin(Plat->Platform1Angle[2] * PI / 180) }, { 0 } };
	bR4 = { { Plat->R * cos(Plat->Platform1Angle[3] * PI / 180) }, { Plat->R * sin(Plat->Platform1Angle[3] * PI / 180) }, { 0 } };
	bR5 = { { Plat->R * cos(Plat->Platform1Angle[4] * PI / 180) }, { Plat->R * sin(Plat->Platform1Angle[4] * PI / 180) }, { 0 } };
	bR6 = { { Plat->R * cos(Plat->Platform1Angle[5] * PI / 180) }, { Plat->R * sin(Plat->Platform1Angle[5] * PI / 180) }, { 0 } };

	//静平台的6个铰点，在静平台坐标系中的位置矢量
	Br1 = { { Plat->r * cos(Plat->Platform2Angle[0] * PI / 180) }, { Plat->r * sin(Plat->Platform2Angle[0] * PI / 180) }, { 0 } };
	Br2 = { { Plat->r * cos(Plat->Platform2Angle[1] * PI / 180) }, { Plat->r * sin(Plat->Platform2Angle[1] * PI / 180) }, { 0 } };
	Br3 = { { Plat->r * cos(Plat->Platform2Angle[2] * PI / 180) }, { Plat->r * sin(Plat->Platform2Angle[2] * PI / 180) }, { 0 } };
	Br4 = { { Plat->r * cos(Plat->Platform2Angle[3] * PI / 180) }, { Plat->r * sin(Plat->Platform2Angle[3] * PI / 180) }, { 0 } };
	Br5 = { { Plat->r * cos(Plat->Platform2Angle[4] * PI / 180) }, { Plat->r * sin(Plat->Platform2Angle[4] * PI / 180) }, { 0 } };
	Br6 = { { Plat->r * cos(Plat->Platform2Angle[5] * PI / 180) }, { Plat->r * sin(Plat->Platform2Angle[5] * PI / 180) }, { 0 } };
	
	//旋转矩阵，XYZ型
	rotz = { { cos(Plat->mMotionParam.YAW * PI / 180), -sin(Plat->mMotionParam.YAW * PI / 180), 0 },
			 { sin(Plat->mMotionParam.YAW * PI / 180), cos(Plat->mMotionParam.YAW * PI / 180), 0 },
			 { 0, 0, 1 } };
	roty = { { cos(Plat->mMotionParam.PITCH * PI / 180), 0, sin(Plat->mMotionParam.PITCH * PI / 180) },
			 { 0, 1, 0 },
			 { -sin(Plat->mMotionParam.PITCH * PI / 180), 0, cos(Plat->mMotionParam.PITCH * PI / 180) } };
	rotx = { { 1, 0, 0 },
			 { 0, cos(Plat->mMotionParam.ROLL * PI / 180), -sin(Plat->mMotionParam.ROLL * PI / 180) },
			 { 0, sin(Plat->mMotionParam.ROLL * PI / 180), cos(Plat->mMotionParam.ROLL * PI / 180) } };
	TransM = multiply(rotz, multiply(roty, rotx));
	
	//动平台的6个铰点，在静平台坐标系中的位置矢量
	br1 = plus_mat(multiply(TransM, bR1), P);
	br2 = plus_mat(multiply(TransM, bR2), P);
	br3 = plus_mat(multiply(TransM, bR3), P);
	br4 = plus_mat(multiply(TransM, bR4), P);
	br5 = plus_mat(multiply(TransM, bR5), P);
	br6 = plus_mat(multiply(TransM, bR6), P);
	
	//动平台的6个铰点位置矢量，减去，静平台的6个铰点位置矢量，得到每个杆长矢量
	L1 = minus_mat(br1, Br1);
	L2 = minus_mat(br2, Br2);
	L3 = minus_mat(br3, Br3);
	L4 = minus_mat(br4, Br4);
	L5 = minus_mat(br5, Br5);
	L6 = minus_mat(br6, Br6);

	//求模，得到每个杆的杆长
	Plat->Lenth[0] = sqrt(L1[0][0] * L1[0][0] + L1[1][0] * L1[1][0] + L1[2][0] * L1[2][0]) - Plat->L_mid + Plat->l_mid;
	Plat->Lenth[1] = sqrt(L2[0][0] * L2[0][0] + L2[1][0] * L2[1][0] + L2[2][0] * L2[2][0]) - Plat->L_mid + Plat->l_mid;
	Plat->Lenth[2] = sqrt(L3[0][0] * L3[0][0] + L3[1][0] * L3[1][0] + L3[2][0] * L3[2][0]) - Plat->L_mid + Plat->l_mid;
	Plat->Lenth[3] = sqrt(L4[0][0] * L4[0][0] + L4[1][0] * L4[1][0] + L4[2][0] * L4[2][0]) - Plat->L_mid + Plat->l_mid;
	Plat->Lenth[4] = sqrt(L5[0][0] * L5[0][0] + L5[1][0] * L5[1][0] + L5[2][0] * L5[2][0]) - Plat->L_mid + Plat->l_mid;
	Plat->Lenth[5] = sqrt(L6[0][0] * L6[0][0] + L6[1][0] * L6[1][0] + L6[2][0] * L6[2][0]) - Plat->L_mid + Plat->l_mid;

}
/*存疑*/
void MotionCal::NowMotion(Platform_InitParameter *Plat) {


	//旋转矩阵，XYZ型
	rotz = { { cos(Plat->mMotionParam.YAW * PI / 180), -sin(Plat->mMotionParam.YAW * PI / 180), 0 },
	{ sin(Plat->mMotionParam.YAW * PI / 180), cos(Plat->mMotionParam.YAW * PI / 180), 0 },
	{ 0, 0, 1 } };
	roty = { { cos(Plat->mMotionParam.PITCH * PI / 180), 0, sin(Plat->mMotionParam.PITCH * PI / 180) },
	{ 0, 1, 0 },
	{ -sin(Plat->mMotionParam.PITCH * PI / 180), 0, cos(Plat->mMotionParam.PITCH * PI / 180) } };
	rotx = { { 1, 0, 0 },
	{ 0, cos(Plat->mMotionParam.ROLL * PI / 180), -sin(Plat->mMotionParam.ROLL * PI / 180) },
	{ 0, sin(Plat->mMotionParam.ROLL * PI / 180), cos(Plat->mMotionParam.ROLL * PI / 180) } };
	TransM = multiply(rotz, multiply(roty, rotx));
	//P_s_t, P_t_n, n_t, P_n_g
	P_n_g = { { 0 },
	{ 0 },
	{ 999.7 },
	};
	P_s_t = { { 0 },
	{ 0 },
	{ 1000 },
	};
	P_t_n = { { Plat->mMotionParam.X },
	{ Plat->mMotionParam.Y },
	{ Plat->mMotionParam.Z },
	};
	//cout << TransM[0][0] << endl;
	O_g = (multiply(TransM, P_s_t));
	O_g = plus_mat(multiply(TransM, P_s_t), P_t_n);
	O_g = plus_mat(O_g, P_n_g);

	Plat->Position[0] = O_g[0][0];
	Plat->Position[1] = O_g[1][0];
	Plat->Position[2] = O_g[2][0] - 1999.7;
	cout << "X1:	" << Plat->Position[0] << endl;
	cout << "Y1:	" << Plat->Position[1] << endl;
	cout << "Z1:	" << Plat->Position[2] << endl;
}