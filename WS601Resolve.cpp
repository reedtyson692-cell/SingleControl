#include "WS601Resolve.h"

using namespace std;
void WS601Resolve(struct WS601Struct *pws, unsigned char *data)
{
	short tempInt16;
	int index, tempInt32;
	index = 3;

	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->angle[0] = tempInt16*360.0 / 32768;
	index = index + 2;
	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->angle[1] = tempInt16*360.0 / 32768;
	index = index + 2;
	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->angle[2] = tempInt16*360.0 / 32768;
	index = index + 2;

	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->vel = tempInt16*10.0 / 32768;
	index = index + 2;
	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->pos = tempInt16*20.0 / 32768;
	index = index + 2;

	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->g[0] = tempInt16*150.0 / 32768;
	index = index + 2;
	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->g[1] = tempInt16*150.0 / 32768;
	index = index + 2;
	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->g[2] = tempInt16*150.0 / 32768;
	index = index + 2;

	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->a[0] = tempInt16*2.0 / 32768;
	index = index + 2;
	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->a[1] = tempInt16*2.0 / 32768;
	index = index + 2;
	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->a[2] = tempInt16*2.0 / 32768;
	index = index + 2;

	tempInt16 = data[index] + (((int)data[index + 1]) << 8);
	pws->temp = tempInt16*200.0 / 32768;
	index = index + 2;

	tempInt32 = data[index] + (((unsigned int)data[index + 1]) << 8) + (((unsigned int)data[index + 2]) << 16) + (((unsigned int)data[index + 3]) << 24);
	pws->ts = tempInt32*0.01;
	index = index + 4;
}