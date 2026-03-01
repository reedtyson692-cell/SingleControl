#ifndef MOTIONCAL_H
#define MOTIONCAL_H
//六自由度解算（黄东龙著）
#include <math.h>
#include <vector>
#include "matrix.h"

#define r_  1200
#define R_  1100
#define PI 3.14159265
using namespace std;
typedef struct{
	double  X;
	double  Y;
	double  Z;
	double ROLL;
	double PITCH;
	double YAW;
}MotionParam;/*动平台位移量*/

typedef struct {
	/*静平台半径*/
	double r;
	/*动平台半径*/
	double R;
	/*平台中位时电缸的伸长量*/
	double l_mid;
	/*平台中位时电缸缸体的总长*/
	double L_mid;
	vector<double> Platform1;
	vector<double> Platform1Angle;
	vector<double> Platform2Angle;
	MotionParam mMotionParam;
	/*电杆伸长量*/
	double Lenth[6];
	double nowpos[6];
	/*记录偏移量*/
	double Position[6];
}Platform_InitParameter;

class MotionCal {
public:
	MotionCal();
	~MotionCal();
	void InitParam();
	void InputParam(Platform_InitParameter *Plat,float x, float y, float z, float roll, float pitch, float yaw);
	/*求杆长Lenth*/
	void Calculating(Platform_InitParameter *Plat);

	void NowMotion(Platform_InitParameter *Plat);

public:
	//Length length;
	Platform_InitParameter *UpPlatform;
	Platform_InitParameter *DownPlatform;

private:
	
	/*动平台圆心点*/
	vector<vector<double>>P;
	/*动平台的6个铰点，在动平台坐标系中的位置矢量*/
	vector<vector<double>>bR1, bR2, bR3, bR4, bR5, bR6;
	/*静平台的6个铰点，在静平台坐标系中的位置矢量*/
	vector<vector<double>>Br1, Br2, Br3, Br4, Br5, Br6;
	/*旋转矩阵，XYZ型	TransM = rotz * roty * rotx*/
	vector<vector<double>>rotz, roty, rotx;//TransM;
	/*动平台的6个铰点，在静平台坐标系中的位置矢量*/
	vector<vector<double>>br1, br2, br3, br4, br5, br6;
	/*动平台的6个铰点位置矢量，减去，静平台的6个铰点位置矢量，得到每个杆长矢量*/
	vector<vector<double>>L1, L2, L3, L4, L5, L6;
	/*存疑*/
	vector<vector<double>>TransM, P_n_g, P_s_t, P_t_n, O_g;
	
};
#endif // !MOTIONCAL_H
