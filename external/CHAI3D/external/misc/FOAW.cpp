#include <cmath>
#include <stdio.h>
#include "FOAW.h"


double FOAW::vel(double currPosition) {
	//printf("sdfasdf\n");
	int n = 1;
	yHistory[historyIndex] = currPosition;
	double yk = currPosition;
	int do_more = 1;
	while ((do_more == 1) && (n<=MAX_WINDOW)) {
        double yk_n;
		if((historyIndex-n) < 0) {
			yk_n = yHistory[MAX_WINDOW+(historyIndex-n)];
		} else {
			yk_n = yHistory[historyIndex-n];
		}
		double a = ((MAX_WINDOW*yk_n) + ((double)(n-MAX_WINDOW)*yk))/(double)n;
		double b = (yk-yk_n)/((double)n*SAM_PER);
		for(int i = 1;i <= n;i++) {
			double determ;
			if((historyIndex-i) < 0) {
				determ = abs(yHistory[MAX_WINDOW+(historyIndex-i)] - (a+(b*(MAX_WINDOW-i)*SAM_PER)));
			} else {
				determ = abs(yHistory[historyIndex-i] - (a+(b*(MAX_WINDOW-i)*SAM_PER)));
			}
			if(determ > POS_RES) {
				do_more = 0;
			}
		}
		if(do_more == 1) {
			n++;
		}
	}
	n = n-1;
	//printf("%d\n",n);
	double rtn;
	if((historyIndex-n) < 0) {
		rtn = ((yk - yHistory[MAX_WINDOW+(historyIndex-n)])/(n*SAM_PER));
	} else {
		rtn = ((yk - yHistory[historyIndex-n])/(n*SAM_PER));
	}
	historyIndex = (historyIndex + 1)%MAX_WINDOW;
	return rtn;
}