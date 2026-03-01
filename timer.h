#include <stdlib.h>
#include <stdio.h>
#include "compatibility.h"
#include "bdaqctrl.h"
using namespace Automation::BDaq;



class timer{
public:
	timer();;
	~timer();

public:
	void initialize();
	void startcount();
	void stopcount();
	static void BDAQCALL OnCounterEvent(void *sender, CntrEventArgs *args, void * userParam);


private:
	TimerPulseCtrl* timerPulseCtrl;
	double targetFrequence;
	double currentFrequence;
	int eventCount;
	ErrorCode ret = Success;
};