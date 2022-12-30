#include "ctrl.h"

void CTRL_Init(float Kp, float Tn, float Td, float T, float maxIn, float maxOut, CTRL_t* ctrl)
{
  ctrl->Kp = Kp;
  ctrl->Tn = Tn;
  ctrl->Td = Td;
  ctrl->T = T;
  ctrl->maxIn = maxIn;
  ctrl->maxOut = maxOut;

  ctrl->d0 = Kp * ((Tn + T/2.0) * (0 + T/2.0)) / (Tn * (Td + T/2.0));
  ctrl->d1 = -2 * Kp * (Tn * 0 - (T/2.0) * (T/2.0)) / (Tn * (Td + T/2.0));
  ctrl->d2 = Kp * ((Tn - T/2.0) * (0 - T/2.0)) / (Tn * (Td + T/2.0));
  ctrl->c1 = (2.0 * Td) / (Td + T/2.0);
  ctrl->c2 = -(Td - T/2.0) / (Td + T/2.0);

  CTRL_Calculate(0.0, 0.0, ctrl, true);
}

float CTRL_Calculate(float curr, float dest, CTRL_t* ctrl, bool restart)
{
  if(restart)
  {
    ctrl->ek_1 = 0.0;
    ctrl->ek_2 = 0.0;
    ctrl->uk_1 = 0.0;
    ctrl->uk_2 = 0.0;
    ctrl->e = 0.0;
    ctrl->e_sum = 0.0;
  }

  // Limit input and output
  if(dest > ctrl->maxIn)
  {
    dest = ctrl->maxIn;
  }
  else if(dest < -ctrl->maxIn)
  {
    dest = -ctrl->maxIn;
  }

  // Do controller calculation
  ctrl->ek = dest - curr;
  ctrl->uk = ctrl->d0 * ctrl->ek + ctrl->d1 * ctrl->ek_1 + ctrl->d2 * ctrl->ek_2
            + ctrl->c1 * ctrl->uk_1 + ctrl->c2 * ctrl->uk_2;

  // Update registers with anti windup check
  if(ctrl->uk < ctrl->maxOut && ctrl->uk > -ctrl->maxOut)
  {
	  ctrl->uk_2 = ctrl->uk_1;
	  ctrl->uk_1 = ctrl->uk;
  }
  ctrl->ek_2 = ctrl->ek_1;
  ctrl->ek_1 = ctrl->ek;

  if((dest * ctrl->uk) < 0)
  {
	  ctrl->uk = 0;
  }
  return ctrl->uk;
}
