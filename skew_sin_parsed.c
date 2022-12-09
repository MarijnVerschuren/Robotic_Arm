double max_vel;						// vmax
double max_acc;						// acc

double target_x;
double start_x;
uint32_t start_t;					// in ticks

double dx = target_x - start_x;		// hm1

bool skew_sin(double pos, double* pos_out, double* vel_out, double* acc_out) {
	bool ready;
	// qr -> pos_out
	// qdr -> vel_out
	// qddr -> acc_out


	double tm;		// ? ( = SETUP_TIME ??) (DT of sym)
	double t1;		// ?
	double h1;		// ?

	double dt = HAL_GetTick() - start_t;  // rtime


	// TODO:
	if (rtime<=tm || tm==0.0) {  // ?????
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
	}
	else {
		qr[0]   = x[ENDPOS1];
		qdr[0]  = 0.0;
		qddr[0] = 0.0;	
		ready[0]= 1.0;
	}
}