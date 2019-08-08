//First-Order Adaptive Windowing (FOAW) filter
//Reference: ???
//Author: Seokhee Jeon
//Last Modified: 2010-08-05

#pragma once
#ifndef FOAW_H
#define FOAW_H

#define MAX_WINDOW	100			//maximum average window
#define SAM_PER	  0.0005				//sampling period
#define POS_RES	0.00001					//position resolution

class FOAW
{
public:
	FOAW() {
		historyIndex = 0;
	};
	//current velocity in, filtered velocity out
	double vel(double currPosition);
	double yHistory[MAX_WINDOW];
	int historyIndex;
};

#endif