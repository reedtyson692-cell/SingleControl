#include "timer.h"
//#include "QString"
#include <iostream>
#define deviceDescription L"PCI-1751,BID#0"
using namespace std;
int32 channelStart = 0;
int32 channelCount = 1;

timer::timer() //:eventCount(0),targetFrequence(1000),currentFrequence(0)
{
	timerPulseCtrl = TimerPulseCtrl::Create();
	timerPulseCtrl->addTimerTickHandler(OnCounterEvent, this);
}

timer::~timer(){

}

void timer::initialize(){
	DeviceInformation devInfo(deviceDescription);
	const wchar_t* profilePath = L"PCI-1751.xml";
	ret = timerPulseCtrl->setSelectedDevice(devInfo);
	ret = timerPulseCtrl->LoadProfile(profilePath);
	ret = timerPulseCtrl->setChannelCount(1);
	ret = timerPulseCtrl->setChannelStart(1);

}

void timer::startcount(){
	ErrorCode ret = Success;
	for (int i = timerPulseCtrl->getChannelStart(); i < timerPulseCtrl->getChannelStart() + timerPulseCtrl->getChannelCount(); i++){
		ret = timerPulseCtrl->getChannels()->getItem(i).setFrequency(1000);
	}
	//CheckError(errorCode);
	ret = timerPulseCtrl->setEnabled(true);
	if (ret == Success)
	{
		printf("\n¶¨Ê±Æ÷Æô¶¯");
	}
	if (ret != Success)
	{
		printf("\nSorry4, There are some errors occurred, Error Code: 0x");
	}
	for (int i = timerPulseCtrl->getChannelStart(); i < timerPulseCtrl->getChannelStart() + timerPulseCtrl->getChannelCount(); i++){
		currentFrequence = timerPulseCtrl->getChannels()->getItem(i).getFrequency();
	}
	//printf("\nStart Timer  Frequency is : %d", currentFrequence);
}

void timer::stopcount(){
	ErrorCode ret = Success;
	//QLOG_INFO() << "Stop Timer Counter" << currentFrequence;
	ret = timerPulseCtrl->setEnabled(false);
	//CheckError(errorCode);
}

void timer::OnCounterEvent(void *sender, CntrEventArgs *args, void * userParam){
	static int i = 0;
	static int j = 0;
	i++;
	if (i % 1000 == 0 && i != 0){
		j++;
		cout << endl << i;
	}
	//j++;
	//cout << endl << i;
}