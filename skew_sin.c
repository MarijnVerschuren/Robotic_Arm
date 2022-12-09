/*
 * sfuntmpl.c: C template for a level 2 S-function.
 *
 *  -------------------------------------------------------------------------
 *  | See matlabroot/simulink/src/sfuntmpl.doc for a more detailed template |
 *  -------------------------------------------------------------------------
 *
 * Copyright (c) 1990-1998 by The MathWorks, Inc. All Rights Reserved.
 * $Revision: 1.1 $
 */

/* Reference Generator
 * Erwin Schrijver, March 2001 / Hans-Martin Duringhof, December 2003
 */

/*
Hiddo Super, 20 March 2003
Modified: Incorrect initialization of inputport Q1, directfeedtrough flag was not set.
From Matlab R12.1 and upwards, this generates a segmentation fault.
*/


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl with the name of your S-function).
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  refgennew

#define PI 3.141592653589793

/* Discrete states */
#define STARTTIME 0
#define STARTPOS1 1
#define ENDPOS1   2
#define STATE     3
#define              STOPPED 0
#define              RUNNING 1

/* Inputs */
#define Q1      0
#define QD1     1
#define START   2

/* Outputs */
#define QR1      0
#define QDR1     1
#define QDDR1    2
#define READY    3

#define SETUP_TIME_PARAM            (ssGetSFcnParam(S,0))
#define SETUP_TIME                  ((real_T) mxGetPr(SETUP_TIME_PARAM)[0])
#define TYPE_PARAM                  (ssGetSFcnParam(S,1))
#define FUNCTION_TYPE               ((real_T) mxGetPr(TYPE_PARAM)[0])
#define ACC_PARAM                   (ssGetSFcnParam(S,2))
#define MAX_ACCELERATION            ((real_T) mxGetPr(ACC_PARAM)[0])
#define VEL_PARAM                   (ssGetSFcnParam(S,3))
#define MAX_VELOCITY                ((real_T) mxGetPr(VEL_PARAM)[0])

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include <math.h>

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* See sfuntmpl.doc for more details on the macros below */

    ssSetNumSFcnParams(S, 4);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 4);

    if (!ssSetNumInputPorts(S, 3)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortWidth(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 0);
    ssSetInputPortDirectFeedThrough(S, 2, 0);

    if (!ssSetNumOutputPorts(S, 4)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortWidth(S, 2, 1);
    ssSetOutputPortWidth(S, 3, 1);

    ssSetNumSampleTimes(S, 0);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  { real_T *x=ssGetRealDiscStates(S);

    x[STATE]     = STOPPED;
    x[STARTPOS1] = 0.0;
    x[ENDPOS1]   = 0.0;
    x[STARTTIME] = 0.0;
  }
#endif /* MDL_INITIALIZE_CONDITIONS */

#define ZERO_ORDER   0
#define FIRST_ORDER  1
#define SECOND_ORDER 2
#define THIRD_ORDER  3
#define SKEWSINE     4

#define ABS(x)  (((x)>0)?(x):(-(x)))

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *x           = ssGetRealDiscStates(S);
    real_T *qr          = ssGetOutputPortRealSignal(S,QR1);	
    real_T *qdr         = ssGetOutputPortRealSignal(S,QDR1);	
    real_T *qddr        = ssGetOutputPortRealSignal(S,QDDR1);	
    real_T *ready       = ssGetOutputPortRealSignal(S,READY);	
    InputRealPtrsType u = ssGetInputPortRealSignalPtrs(S,Q1);

    real_T hm1,rtime,tm,acc,t1,h1,vmax;
    int type=(int)FUNCTION_TYPE;

    hm1   = x[ENDPOS1] - x[STARTPOS1];
    rtime = ssGetT(S)  - x[STARTTIME];
    tm    = SETUP_TIME;
    acc   = ABS(MAX_ACCELERATION);
    vmax  = ABS(MAX_VELOCITY);

    if (x[STATE]==STOPPED) {
	qr[0]    = *u[0];
	qdr[0]   = 0.0;
	qddr[0]  = 0.0;	
	ready[0] = 1.0;
    } else {
	if (rtime<=tm || tm==0.0) {
	  switch(type) {
	    case SKEWSINE:
              if (tm==0.0 && acc!=0.0 && vmax!=0.0) {
                t1=PI*vmax/acc;
                h1=vmax*t1/2.0;
                tm=t1+(ABS(hm1)-h1)/vmax;
				if ((ABS(hm1)-h1) <= 0.0) {
					tm=sqrt(2*PI*ABS(hm1)/acc);
					t1=tm;
					h1=ABS(hm1);
				}
              } else if (acc == 0.0 || acc<(2*PI*ABS(hm1)/(tm*tm))) {
                t1=tm;
                h1=hm1;
                acc=2*PI*hm1/(tm*tm);
              } else {
                t1=tm-sqrt(tm*tm-2*PI*ABS(hm1)/acc);
                h1=t1*t1*acc/(2*PI);
              }
              if (hm1<0.0) {
                acc=-ABS(acc);
                h1=-ABS(h1);
              }

              ready[0]= 0.0;

              if (rtime<(t1/2.0)) {
                qr[0]   = ((h1/t1)*rtime - (1/(2*PI))*h1*sin((2*PI*(rtime))/t1))+x[STARTPOS1];
                qdr[0]  = (h1/t1 - (h1/t1)*cos((2*PI*(rtime))/t1));
                qddr[0] = 2*h1*sin((2*PI*(rtime))/t1)*PI/(t1*t1);
              } else if (rtime < (tm-(t1/2.0))) {
				qr[0]   = (2*h1/t1)*(rtime-(t1/2.0)) + h1/2.0 + x[STARTPOS1];
                qdr[0]  = 2*h1/t1;
                qddr[0] = 0.0;
              } else if (rtime < tm) {
                rtime = t1-tm+rtime;
                qr[0]   = ((h1/t1)*rtime - (1/(2*PI))*h1*sin((2*PI*(rtime))/t1)) + x[STARTPOS1] + hm1 - h1;
                qdr[0]  = (h1/t1 - (h1/t1)*cos((2*PI*(rtime))/t1));
                qddr[0] = 2*h1*sin((2*PI*(rtime))/t1)*PI/(t1*t1);               
              } else {
                qr[0]   = x[ENDPOS1];
                qdr[0]  = 0.0;
                qddr[0] = 0.0;	
                ready[0]= 1.0;
              }
              break;
            case ZERO_ORDER:
              qr[0]   = x[ENDPOS1];
              qdr[0]  = 0.0;
              qddr[0] = 0.0;	
              ready[0]= 1.0;
              break;
            case FIRST_ORDER:
              qr[0]   = (hm1/tm)*rtime + x[STARTPOS1];
              qdr[0]  = hm1/tm;
              qddr[0] = 0.0;
              ready[0]= 0.0;
              break;
            case SECOND_ORDER: 
              if (tm==0.0 && acc!=0.0 && vmax!=0.0) {
                t1=vmax/acc;
                tm=2*t1+(ABS(hm1)-acc*t1*t1)/vmax;
              } else if (acc==0.0 || (acc*acc*tm*tm - 4*acc*ABS(hm1)) < 0) {
                t1=tm/2.0;
                acc=4*hm1/(tm*tm);
              } else t1 = tm/2.0 - sqrt(acc*acc*tm*tm - 4*acc*ABS(hm1))/(2*acc);
	      
			  if (hm1>0.0) acc=ABS(acc);
              else         acc=-ABS(acc);
              
              ready[0]= 0.0;

			  if (rtime<t1) {
                qr[0]   = acc*rtime*rtime/2.0 + x[STARTPOS1];
                qdr[0]  = acc*rtime;
                qddr[0] = acc;
              } else if (rtime < (tm-t1)) {
                qr[0]   = acc*t1*(rtime-t1) + acc*t1*t1/2.0 + x[STARTPOS1];
                qdr[0]  = acc*t1;
                qddr[0] = 0.0;
              } else if (rtime <tm) {
                qr[0]   = -acc*(rtime-tm)*(rtime-tm)/2.0 + hm1 + x[STARTPOS1];
                qdr[0]  = acc*(tm-rtime);
                qddr[0] = -acc;
              } else {
                qr[0]   = x[ENDPOS1];
                qdr[0]  = 0.0;
                qddr[0] = 0.0;	
                ready[0]= 1.0;
			  }
              break;
            case THIRD_ORDER:
              if (tm==0.0 && acc!=0.0 && vmax!=0.0) {
                t1=vmax/acc;
                acc=acc/t1;
                tm=4*t1+(ABS(hm1)-2*acc*t1*t1*t1)/vmax;
                if (hm1<0.0) acc=-ABS(acc);
              } else if (acc==0.0 || (tm*tm*acc*acc-8.0*acc*ABS(hm1))<0.0) {
                t1=tm/4.0;
                acc=32.0*hm1/(tm*tm*tm);
              } else {
		t1=tm/4.0 - sqrt(tm*tm*acc*acc-8.0*acc*ABS(hm1))/(4.0*acc);
		acc=acc/t1;
                if (hm1<0.0) acc=-ABS(acc);
              }

              ready[0]= 0.0;

              if (rtime<t1) {
                qr[0]   = acc*rtime*rtime*rtime/6.0 + x[STARTPOS1];
                qdr[0]  = acc*rtime*rtime/2.0;
                qddr[0] = acc*rtime;
              } else if (rtime < (t1*2.0)) {
                qr[0]   = -acc*rtime*rtime*rtime/6.0 + acc*rtime*rtime*t1 - acc*t1*t1*rtime + acc*t1*t1*t1/3.0 + x[STARTPOS1];
                qdr[0]  = -acc*rtime*rtime/2.0 + 2*acc*rtime*t1 - acc*t1*t1;
                qddr[0] = -acc*rtime + 2*acc*t1;
              } else if (rtime < (tm-2.0*t1)) {
  				qr[0]   = acc*t1*t1*(rtime-2.0*t1) + acc*t1*t1*t1 + x[STARTPOS1];
                qdr[0]  = acc*t1*t1;
                qddr[0] = 0.0;
              } else if (rtime < (tm - t1)) {
                rtime  -= tm - 4.0*t1;
                qr[0]   = -acc*rtime*rtime*rtime/6.0 + acc*rtime*rtime*t1 - acc*t1*t1*rtime + acc*t1*t1*t1/3.0 + acc*t1*t1*(tm-4.0*t1) + x[STARTPOS1];
                qdr[0]  = -acc*rtime*rtime/2.0 + 2*acc*rtime*t1 - acc*t1*t1;
                qddr[0] = -acc*rtime + 2*acc*t1;
			  } else if (rtime < tm) {
                rtime  -= tm - 4.0*t1;
                qr[0]   = acc*rtime*rtime*rtime/6.0 - 2*acc*t1*rtime*rtime + 8*acc*t1*t1*rtime - 26.0*acc*t1*t1*t1/3.0 + acc*t1*t1*(tm-4.0*t1) + x[STARTPOS1];
                qdr[0]  = acc*rtime*rtime/2.0 - acc*rtime*4*t1 + 8*acc*t1*t1;
                qddr[0] = acc*rtime - acc*4*t1;
              } else {
                qr[0]   = x[ENDPOS1];
                qdr[0]  = 0.0;
                qddr[0] = 0.0;	
                ready[0]= 1.0;
              }
              break;
          }
	} else {
          qr[0]   = x[ENDPOS1];
          qdr[0]  = 0.0;
          qddr[0] = 0.0;	
          ready[0]= 1.0;
	}
   }
}








// https://nl.mathworks.com/help/simulink/sfg/example-of-a-basic-c-mex-s-function.html