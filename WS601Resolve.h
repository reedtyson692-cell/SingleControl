#include <iostream>
using namespace std;
//中间那个IMU+力传感器
struct WS601Struct{
	double angle[3];
	double g[3];
	double a[3];
	double pos;
	double vel;
	double temp;
	double ts;
};

void WS601Resolve(struct WS601Struct *pws, unsigned char *data);