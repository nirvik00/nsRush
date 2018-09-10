#pragma once

#include <cmath>

class Pt
{
public:
	float x, y;
	int locked = 0;
	Pt();
	Pt(float, float);
	float di(Pt);
	void setup(float, float);

	Pt nor(Pt, Pt);
	Pt sc(Pt, float);

	//Pt intx(Pt, Pt, Pt, Pt);
};

