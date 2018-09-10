#include "Pt.h"



Pt::Pt()
{
}

Pt::Pt(float x_, float y_) { x = x_; y = y_; }

float Pt::di(Pt p) { return sqrt((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y)); }

void Pt::setup(float x_, float y_) { x = x_; y = y_; }

Pt Pt::nor(Pt a, Pt b) {
	Pt u((b.x - a.x) / a.di(b), (b.y - a.y) / a.di(b));
	return Pt(x-u.y,y+u.x);
}

Pt Pt::sc(Pt a, float g) {
	Pt u((a.x - x)*g / di(a), (a.y - y)*g / di(a));
	return Pt(x+u.x, y+u.y);
}
/*
Pt Pt::intx(Pt p, Pt q, Pt r, Pt s) {
	float a1 = q.y - p.y; float b1 = p.x - q.x; float c1 = a1*q.x + b1*q.y;
	float a2 = s.y - r.y; float b2 = r.x - s.x; float c2 = a2*s.x + b2*s.y;
	float det = a1*b2 - b2*a1; if (det == 0) { return Pt(-1, -1); }
	float m = c1*b2 - c2*b1; float n = c2*a1 - c1*a2; Pt I(m, n);
	float Ip = I.di(p); float Iq = I.di(q); float pq = p.di(q);
	float Ir = I.di(r); float Is = I.di(s); float rs = r.di(s);
	if ((Ip + Iq - pq < 5) && (Is + Ir - rs < 1)) { return Pt(m, n); }
	else { return Pt(-1, -1); }
}
*/