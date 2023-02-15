#include "rpmctrl.h"

void RPMCTRL_Init(int steps, float corrfacShort, float corrfacLong, RPMCTRL_t* rpmCtrl)
{
	rpmCtrl->dutyCycle = 0;
	rpmCtrl->eternalTicks = 0;
	rpmCtrl->corrfacShort = corrfacShort;
	rpmCtrl->corrfacLong = corrfacLong;
	rpmCtrl->sum = 0;
	rpmCtrl->timestamp_us = 0;
	rpmCtrl->tmpTicks = 0;
	rpmCtrl->rpm = 0;
	rpmCtrl->rpmDest = 0;
	rpmCtrl->lastRpmDest = 0;
	rpmCtrl->restart = false;
	rpmCtrl->steps = steps;
	rpmCtrl->ignore = false;
	rpmCtrl->recordSamples = 0;
	rpmCtrl->isRecord = false;
}

void RPMCTRL_Tick(RPMCTRL_t* ctrl, volatile uint32_t timestamp_us, bool isLong)
{

	// Update eternal odometry tick counter
	if(ctrl->rpmDest > 0)
	{
		ctrl->eternalTicks++;
	}
	else if(ctrl->dutyCycle < 0)
	{
		ctrl->rpmDest--;
	}

	// Handle timestamp overflows and calculate time offset
	uint32_t offset = 0;

	if(timestamp_us < ctrl->timestamp_us)
	{
		offset = (uint32_t)(0xffff - ctrl->timestamp_us / 100) * 100 + timestamp_us;
	}
	else
	{
		offset = timestamp_us - ctrl->timestamp_us;
	}
	ctrl->timestamp_us = timestamp_us;
	if(isLong)
	{
		offset = (uint32_t)(offset * ctrl->corrfacLong);
	}
	else
	{
		offset = (uint32_t)(offset * ctrl->corrfacShort);
	}

	// Calculate average RPM since last control calculation
	int rpm = 0;
	if(5000 < offset && 0 != ctrl->rpmDest)
	{
	    float denum = (offset * ctrl->steps) / 1000.0 / 1000.0;
		if(0.0 != denum)
		{
			rpm = (int)(60.0 / denum);
		}
	}
	else
	{
		rpm = 0;
	}

	if(ctrl->rpmDest < 0)
	{
	  rpm = -rpm;
	}
	ctrl->rpm = rpm;
	// Handle recording
	if(ctrl->isRecord && !ctrl->recordTorque && QUEUE_Size((QUEUE_t*)ctrl->recordQueue) < ctrl->recordSamples * 2)
	{
		if(!QUEUE_IsFull16((QUEUE_t*)ctrl->recordQueue) && offset > 5000)
		{
			QUEUE_Push16((uint16_t)(offset / 100), (QUEUE_t*)ctrl->recordQueue);
		}
	}
}

void RPMCTRL_SetAvgTorque(RPMCTRL_t* ctrl, uint16_t torque)
{

	ctrl->avgTorque = torque;
	// Handle recording
	if(ctrl->isRecord && ctrl->recordTorque && QUEUE_Size((QUEUE_t*)ctrl->recordQueue) < ctrl->recordSamples * 2)
	{
		if(!QUEUE_IsFull16((QUEUE_t*)ctrl->recordQueue))
		{
			QUEUE_Push16(torque, (QUEUE_t*)ctrl->recordQueue);
		}
	}

}

void RPMCTRL_Update(RPMCTRL_t* ctrl, CTRL_t* ctrl_base)
{

	if(!ctrl->ignore)
	{

		// Actual control loop execution

			ctrl->dutyCycle = CTRL_Calculate((float)ctrl->rpm, (float)ctrl->rpmDest, ctrl_base, ctrl->restart);
		ctrl->rpmDest = ctrl->lastRpmDest;
	}
}

void RPMCTRL_SetDest(RPMCTRL_t* ctrl, int dest, CTRL_t* ctrl_base)
{

	if(ctrl->ignore)
	{
		ctrl->dutyCycle = dest;
	}
	else
	{
		// Restart if necessary to prevent runaway
		if(0 == dest || (dest * ctrl->lastRpmDest) < 0)
		{
			ctrl->dutyCycle = 0;
			ctrl->rpmDest = 0;
			ctrl->rpm = 0;
			ctrl->sum = 0;
			CTRL_Calculate(0.0, 0.0, ctrl_base, true);
		}
		ctrl->lastRpmDest = dest;
	}
}

void RPMCTRL_Ignore(RPMCTRL_t* ctrl, bool ignore, CTRL_t* ctrl_base)
{

	ctrl->ignore = ignore;
	if(ctrl->ignore)
	{
		CTRL_Calculate(0.0, 0.0, ctrl_base, true);
	}
}

void RPMCTRL_StartRecord(RPMCTRL_t* ctrl, uint32_t samples, QUEUE_t* sampleQueue, bool torque)
{

	if(samples > MEMDEPTH_RECORD)
	{
		samples = MEMDEPTH_RECORD;
	}
	ctrl->recordQueue = sampleQueue;
	QUEUE_Reset((QUEUE_t*)ctrl->recordQueue);
	ctrl->recordSamples = samples;
	ctrl->isRecord = true;
	ctrl->recordTorque = torque;
}

bool RPMCTRL_IsRecordDone(RPMCTRL_t* ctrl)
{
	return (QUEUE_Size((QUEUE_t*)ctrl->recordQueue) >= 2 * ctrl->recordSamples && ctrl->isRecord);
}

void RPMCTRL_StopRecord(RPMCTRL_t* ctrl)
{

	ctrl->recordSamples = 0;
	ctrl->isRecord = false;
}

