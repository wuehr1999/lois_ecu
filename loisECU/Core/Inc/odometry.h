/*
 * odometry.h
 *
 *  Created on: Nov 19, 2022
 *      Author: jonas
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

typedef struct ODOMETRY_t
{
	float x, y, o;
	float dx, dy, w;
	uint32_t timestamp_us;
	int steps;
	float wheelscope_m;
	float wheelwidth_m;
	volatile bool first;
}ODOMETRY_t;

void ODOMETRY_Init(int steps, float wheelradius_m, float wheelwidth_m, ODOMETRY_t* odom);

void ODOMETRY_Update(ODOMETRY_t* odom, int rpmLeft, int rpmRight, uint32_t timestamp_us);

#endif /* INC_ODOMETRY_H_ */
