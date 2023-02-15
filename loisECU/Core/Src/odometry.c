/*
 * odometry.c
 *
 *  Created on: Nov 19, 2022
 *      Author: jonas
 */
#include "odometry.h"

void ODOMETRY_Init(int steps, float wheelradius_m, float wheelwidth_m, ODOMETRY_t* odom)
{
	odom->x = 0.0;
	odom->y = 0.0;
	odom->o = 0.0;
	odom->dx = 0.0;
	odom->dy = 0.0;
	odom->w = 0.0;
	odom->steps = steps;
	odom->wheelscope_m = 2.0 * M_PI * wheelradius_m;
	odom->wheelwidth_m = wheelwidth_m;
	odom->timestamp_us = 0;
	odom->first = true;
}

void ODOMETRY_Update(ODOMETRY_t* odom, int rpmLeft, int rpmRight, uint32_t timestamp_us)
{
	if(odom->first)
	{
		odom->timestamp_us = timestamp_us;
		odom->first = false;
	}
	else
	{
		uint32_t offset_us = 0;

		if(timestamp_us < odom->timestamp_us)
		{
			offset_us = (uint32_t)(0xffff - odom->timestamp_us / 100) * 100 + timestamp_us;
		}
		else
		{
			offset_us = timestamp_us - odom->timestamp_us;
		}
		odom->timestamp_us = timestamp_us;
		float dt_sec = (float)offset_us / 1000.0 / 1000.0;
	    float vLeft = odom->wheelscope_m * (float)rpmLeft / 60.0;
	    float vRight = odom->wheelscope_m * (float)rpmRight / 60.0;

	    float R = 0;
	    if(vLeft == vRight) // axial rotation
	    {
	    	R = 0.5 * odom->wheelwidth_m;
	    }
	    else
	    {
	    	R = 0.5 * odom->wheelwidth_m * (vLeft + vRight) / (vRight - vLeft);
	    }
	    float w = (vRight - vLeft) / odom->wheelwidth_m;
	    float ICCx = odom->x - R * sin(odom->o);
	    float ICCy = odom->y + R * cos(odom->o);
        float x = odom->x;
        float y = odom->y;
        float o = odom->o;
        odom->x = cos(w * dt_sec) * (x - ICCx) - sin(w * dt_sec) * (y - ICCy) + ICCx;
        odom->y = sin(w * dt_sec) * (x - ICCx) + cos(w * dt_sec) * (y - ICCy) + ICCy;
        odom->o = odom->o + w * dt_sec;
        odom->dx = (odom->x - x) / dt_sec;
        odom->dy = (odom->y - y) / dt_sec;
        odom->w = (odom->o - o) / dt_sec;
	}
}
