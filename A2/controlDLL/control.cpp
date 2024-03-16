// controlDLL.cpp : Defines the entry point for the DLL application.

#include "servo.h"
#include "param.h"
#include "control.h"
// #include "UiAgent.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
#include <iostream>

using std::min;
using std::max;

void PrintDebug(GlobalVariables& gv);

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
  // This code runs before the first servo loop 
  
}

void PreprocessControl(GlobalVariables& gv)
{
  // This code runs on every servo loop, just before the control law
   
  if ((gv.dof == 3) || (gv.dof == 6)) {
    // get the correct joint angles depending on the current mode:
    double q1,q2,q3;
    if (gv.dof == 3) {
      q1 = gv.q[0];
      q2 = gv.q[1];
      q3 = gv.q[2];
    } else if (gv.dof == 6) {
      q1 = gv.q[1];
      q2 = gv.q[2];
      q3 = gv.q[4];
    }

    // Variable that holds the torque exerted by gravity for each joint
    PrVector3 g123 = PrVector3(0,0,0);

    // Compute g123 here!
    double r1 = R2;
    double r2 = 0.189738;
    double r3 = R6;
    double l1 = L2;
    double l2 = L3;
    double l3 = L6;
    double m1 = M2;
    double m2 = M3 + M4 + M5;
    double m3 = M6;    
    double g = -9.81; 
    double pi = 3.14;
    
    double c1 = cos(q1);  
    double c12 = cos(-pi/2 + q1 + q2);  
    double c123 = cos(-pi/2 + q1 + q2 + q3);   
    
    
    
    g123[0] = -r1 * m1 * g * c1 - (l1 * c1 + r2 * c12) * m2 * g - (l1 * c1 + l2 * c12 + r3 * c123) * m3 * g;  
    g123[1] = -r2 * m2 * g * c12 - (l2 * c12 + r3 * c123) * m3 * g;
    g123[2] = -r3 * m3 * g * c123;


    // maps the torques to the right joint indices depending on the current mode:
  //   if (gv.dof == 3) {
  //     gv.G[0] = g123[0];
  //     gv.G[1] = g123[1];
  //     gv.G[2] = g123[2];
  //   } else if (gv.dof == 6) {
  //     gv.G[1] = g123[0];
  //     gv.G[2] = g123[1];
  //     gv.G[4] = g123[2];
  //   }
  //   // printing example, do not leave print inthe handed in solution 
  //   // printVariable(g123, "g123");
  } else {
     gv.G = PrVector(gv.G.size());
   }   
}

void PostprocessControl(GlobalVariables& gv) 
{
  // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initOpenControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initNjholdControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initJholdControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initNjmoveControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initJmoveControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initNjgotoControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
} 

void initJgotoControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

double time_init_2;
double time_init_3;  

struct CubicSpline {
  	double t0, tf;
  	PrVector a0, a1, a2, a3;
};
CubicSpline spline;
  
  

double computeTf(GlobalVariables& gv)
{
  float tfmax = 0.0;
  
  for (int i=0; i<3; i++) {
    tfmax = max( abs( 3/(2 * gv.dqmax[i]) * (gv.qd[i] - gv.q[i]) ), tfmax);
    tfmax = max( sqrt( abs( 6 / gv.ddqmax[i] * (gv.qd[i] - gv.q[i]) ) ), tfmax);
  };
  
  return tfmax;
}

void initNjtrackControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
  
  double tf = computeTf(gv);
  
  spline.a0 = gv.q;
  if (gv.dof==6){
  spline.a1 = PrVector(gv.G.size());
  }else{
  spline.a1 = PrVector(3);
  }
  // spline.a1 = PrVector(0.0);  how to create yero vector?
  //spline.a1[0] = 0.0;
  //spline.a1[1] = 0.0;
  //spline.a1[3] = 0.0;
  spline.a2 = 3/pow(tf,2) * (gv.qd - gv.q);
  spline.a3 = -2/pow(tf,3) * (gv.qd - gv.q);
  
  spline.t0 = gv.curTime;
  spline.tf = gv.curTime + tf;
  
}

void initJtrackControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initXtrackControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
} 

void initNholdControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initHoldControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initNgotoControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
} 

void initGotoControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
} 

void initNtrackControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initTrackControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
} 

void initPfmoveControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
} 

void initLineControl(GlobalVariables& gv) 
{
  // Control Initialization Code Here
}

void initProj1Control(GlobalVariables& gv) 
{

  gv.qd[0] = 0.096;
  gv.qd[1] = 0.967;
  gv.qd[2] = -1.061;

  initNjtrackControl(gv);
}

void initProj2Control(GlobalVariables& gv) 
{
  // Control Initialization Code Here

  time_init_2 = gv.curTime; 

}

void initProj3Control(GlobalVariables& gv) 
{

  time_init_3 = gv.curTime; 
  // Control Initialization Code Here
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{
  gv.tau = gv.G;
  
  // this only works on the real robot unless the function is changed to use cout
  // the handed in solution must not contain any printouts
  // PrintDebug(gv);
}

void openControl(GlobalVariables& gv)
{
  // floatControl(gv);  // Remove this line when you implement this controller
}

void njholdControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void jholdControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void njmoveControl(GlobalVariables& gv)
{
  gv.tau = - gv.kp * (gv.q - gv.qd);
}

void jmoveControl(GlobalVariables& gv)
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void njgotoControl(GlobalVariables& gv) 
{
  gv.tau = - gv.kp * (gv.q - gv.qd) + gv.G;
}

void jgotoControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
  
}



void njtrackControl(GlobalVariables& gv) 
{
  PrVector qd, dqd;
  
  if (gv.curTime < spline.tf) {
    qd = spline.a0 + spline.a1 * (gv.curTime-spline.t0) + spline.a2 * pow(gv.curTime-spline.t0, 2) + spline.a3 * pow(gv.curTime-spline.t0, 3); 
    dqd = spline.a1 + 2 * spline.a2 * (gv.curTime-spline.t0) + 3* spline.a3 * pow(gv.curTime-spline.t0, 2);
    
    gv.tau = - gv.kp * (gv.q - qd) - gv.kv * (gv.qd - dqd) + gv.G;
  }
  else {
    floatControl(gv); 
  }
}

void jtrackControl(GlobalVariables& gv)
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void nxtrackControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void xtrackControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void nholdControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void holdControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void ngotoControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void gotoControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void ntrackControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void trackControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void pfmoveControl(GlobalVariables& gv) 
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void lineControl(GlobalVariables& gv)
{
  floatControl(gv);  // Remove this line when you implement this controller
}

void proj1Control(GlobalVariables& gv) 
{
  njtrackControl(gv);
}

void proj2Control(GlobalVariables& gv) 
{

  PrVector3 F = PrVector3(0,0,0);
  double r = 0.2;
  double offset_x = 0.6;
  double offset_y = 0.35;
  double pi = 3.14;
  double time = gv.curTime - time_init_2;
  double beta_dist = (2* pi / 5)*time;
  double beta_vel = 2* pi / 5;

  gv.xd[0] = cos(beta_dist) * r + offset_x; 
  gv.xd[1] = sin(beta_dist) * r + offset_y;
  gv.xd[2] = 0;

  gv.dxd[0] = -sin(beta_vel)* r;
  gv.dxd[1] = cos(beta_vel)* r;
  gv.dxd[2] = 0;

  F = -gv.kp*(gv.x - gv.xd) - gv.kv*(gv.dx - gv.dxd);

  gv.tau = gv.Jtranspose * F + gv.G;

}

void proj3Control(GlobalVariables& gv) 
{
  PrVector3 F = PrVector3(0,0,0);
  double beta_dist;
  double beta_vel;
  double r = 0.2;
  double offset_x = 0.6;
  double offset_y = 0.35;
  double pi = 3.14;
  double time = gv.curTime - time_init_3;
  double beta_acc = 2 * (pi / 25);
  double beta_vel_max = 2 * (pi /5);
  double t_b = beta_vel_max / beta_acc; //5s
  double u_f = 6 * pi;
  double u_t_b = 0.5 * beta_acc * pow(t_b,2); // pi
  double u_lin = u_f - 2 * u_t_b; //4pi
  double t_lin = u_lin / beta_vel_max; //10s
  double t_f = 2*t_b + t_lin; //20s

  
  
  if(time < t_b){ 
    beta_vel = (beta_acc)*time; 
    beta_dist = 0.5*(beta_acc)*pow(time,2);

  } 
  else if(time >= t_b && time < t_b + t_lin ){
    beta_dist = u_t_b + (beta_vel_max)*(time-t_b);
    beta_vel = beta_vel_max;

  } 
  else if(time >= t_b + t_lin && time <= t_f){
    beta_vel = beta_vel_max -(beta_acc)*(time-(t_b+t_lin)); 
    //beta_dist = u_f - 0.5*(beta_acc)*pow(t_f-time,2);
    beta_dist = beta_vel_max*(time-(t_b+t_lin))-0.5*(beta_acc)*pow(time-(t_b+t_lin),2) +5 * pi;

  }
  else{
    beta_vel = 2 * (pi /5);
    beta_dist = 0;
  }

  gv.xd[0] = cos(beta_dist) * r + offset_x; 
  gv.xd[1] = sin(beta_dist) * r + offset_y;
  gv.xd[2] = 0;

  gv.dxd[0] = -sin(beta_vel)* r;
  gv.dxd[1] = cos(beta_vel)* r;
  gv.dxd[2] = 0;
  

  F = -gv.kp*(gv.x - gv.xd) - gv.kv*(gv.dx - gv.dxd);

  gv.tau = gv.Jtranspose * F + gv.G;





}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
  // Replace this code with any debug information you'd like to get
  // when you type "pdebug" at the prompt.
  printf( "This sample code prints the torque and mass\n" );
  gv.tau.display( "tau" );
  gv.A.display( "A" );
}

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/
