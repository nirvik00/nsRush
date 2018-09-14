#include "ofApp.h"

vector<Seg> ofApp::getsegvec(vector<Pt>ptvec) {
	vector<Seg> newptvec; newptvec.clear();
	for (int i = 0; i < ptvec.size() - 1; i++) {
		Pt A, B, C; Pt a, b, c;
		if (i == 0) {
			a = ptvec[7]; b = ptvec[0]; c = ptvec[1];
			A.setup(a.x + (b.x - a.x)*Curvature, a.y + (b.y - a.y)*Curvature);
			C.setup(c.x + (b.x - c.x)*Curvature, c.y + (b.y - c.y)*Curvature);
			B = b;
		}
		else {
			a = ptvec[i - 1]; b = ptvec[i]; c = ptvec[i + 1];
			A.setup(a.x + (b.x - a.x)*Curvature, a.y + (b.y - a.y)*Curvature);
			C.setup(c.x + (b.x - c.x)*Curvature, c.y + (b.y - c.y)*Curvature);
			B = b;
		}
		newptvec.push_back(Seg(A, C));
	}	
	return newptvec;
}

vector<vector<Pt>> ofApp::getcrvpts(vector<Pt>ptvec) {
	vector<vector<Pt>>nestptvec;
	for (int i = 0; i < ptvec.size() - 1; i++) {
		vector<Pt> newptvec; newptvec.clear();
		Pt A, B, C; Pt a, b, c;
		if (i == 0) {
			a = ptvec[7]; b = ptvec[0]; c = ptvec[1];
			A.setup(a.x + (b.x - a.x)*Curvature, a.y + (b.y - a.y)*Curvature);
			C.setup(c.x + (b.x - c.x)*Curvature, c.y + (b.y - c.y)*Curvature);
			B = b;
		}
		else {
			a = ptvec[i - 1]; b = ptvec[i]; c = ptvec[i + 1];
			A.setup(a.x + (b.x - a.x)*Curvature, a.y + (b.y - a.y)*Curvature);
			C.setup(c.x + (b.x - c.x)*Curvature, c.y + (b.y - c.y)*Curvature);
			B = b;
		}
		//bezier
		newptvec.push_back(A); vector<Pt> pts = Lerp(A, B, C);
		for (int j = 1; j < pts.size(); j++) {
			newptvec.push_back(pts[j]);
		}
		newptvec.push_back(C);
		nestptvec.push_back(newptvec);
	}
	return nestptvec;
}

vector<Pt>ofApp::Lerp(Pt a, Pt b, Pt c) {
	Pt ab[20]; Pt bc[20]; vector<Pt> pts;
	int k = 0;
	float n = 1.f / 20.f;
	for (float i = 0.f; i < 1.f; i += n) {
		ab[k].setup(a.x + (b.x - a.x)*i, a.y + (b.y - a.y)*i);
		bc[k].setup(b.x + (c.x - b.x)*i, b.y + (c.y - b.y)*i);
		k++;
	}
	k = 0;
	for (float i = 0.f; i < 1.f; i += n) {
		float x = ab[k].x + (bc[k].x - ab[k].x)*i;
		float y = ab[k].y + (bc[k].y - ab[k].y)*i;
		Pt e(x, y);
		pts.push_back(e);
		k++;
	}
	return pts;
}

vector<Pt> ofApp::gensmoothboundary(vector<Pt>ptvec) {
	vector<Pt> newptvec; newptvec.clear();
	for (int i = 0; i < ptvec.size() - 1; i++) {
		Pt A, B, C; Pt a, b, c;
		if (i == 0) {
			a = ptvec[7]; b = ptvec[0]; c = ptvec[1];
			A.setup(a.x + (b.x - a.x)*Curvature, a.y + (b.y - a.y)*Curvature);
			C.setup(c.x + (b.x - c.x)*Curvature, c.y + (b.y - c.y)*Curvature);
			B = b;
		}
		else {
			a = ptvec[i - 1]; b = ptvec[i]; c = ptvec[i + 1];
			A.setup(a.x + (b.x - a.x)*Curvature, a.y + (b.y - a.y)*Curvature);
			C.setup(c.x + (b.x - c.x)*Curvature, c.y + (b.y - c.y)*Curvature);
			B = b;
		}
		//bezier
		newptvec.push_back(A); vector<Pt> pts = Lerp(A, B, C);
		for (int j = 1; j < pts.size(); j++) {
			newptvec.push_back(pts[j]);
		}
		newptvec.push_back(C);
	}
	newptvec.push_back(newptvec[0]);
	return newptvec;
}

vector<Pt> ofApp::gensmoothboundary(Pt a, Pt b, Pt c, Pt d) {
	vector<Pt> abc = Lerp(a, b, c);
	return abc;
}

vector<Pt> ofApp::gensmoothspinecurve() {
	vector<Pt> newptvec; newptvec.clear();
	A = globaldiaA; F = globaldiaB; Pt U(F.x - A.x, F.y - A.y); Pt V(-U.y / A.di(F), U.x / A.di(F));
	B.setup(A.x + U.x*0.25, A.y + U.y*0.20); C.setup(A.x + U.x*0.40, A.y + U.y*0.40);
	D.setup(A.x + U.x*0.60, A.y + U.y*0.60); E.setup(A.x + U.x*0.80, A.y + U.y*0.80);

	if (controlspine == 0) {
		pB.setup(B.x + V.x * SpineDisplacement, B.y + V.y * SpineDisplacement); pB.locked = 0;
		pC.setup(C.x + V.x * SpineDisplacement * 2, C.y + V.y * SpineDisplacement * 2); pC.locked = 0;
		pD.setup(D.x + V.x * -SpineDisplacement * 2, D.y + V.y * -SpineDisplacement * 2); pD.locked = 0;
		pE.setup(E.x + V.x * -SpineDisplacement, E.y + V.y * -SpineDisplacement); pE.locked = 0;
	}
	spineptvec.clear();
	for (float t = 0.f; t < 1.f; t += 0.01) {
		if (t > 0.f) {
			ofSetColor(0, 0, 255); ofFill();
			float x = (A.x*pow((1 - t), 5)) + (pB.x * 5 * pow((1 - t), 4)*t) + (pC.x * 10 * pow((1 - t), 3)*pow(t, 2)) + (pD.x * 10 * pow(1 - t, 2)*pow(t, 3)) + pE.x * 5 * (1 - t)*pow(t, 4) + F.x*pow(t, 5);
			float y = (A.y*pow((1 - t), 5)) + (pB.y * 5 * pow((1 - t), 4)*t) + (pC.y * 10 * pow((1 - t), 3)*pow(t, 2)) + (pD.y * 10 * pow(1 - t, 2)*pow(t, 3)) + pE.y * 5 * (1 - t)*pow(t, 4) + F.y*pow(t, 5);
			spineptvec.push_back(Pt(x, y));
		}		
	}
	return spineptvec;

}

vector<Pt> ofApp::geninternalboundary(vector<Pt> ptvec, float g) {
	vector<Pt>newptvec; newptvec.clear();
	for (int i = 0; i < ptvec.size() - 1; i++) {
		Pt A, B, C;
		if (i == 0) {
			A = ptvec[7];
			B = ptvec[0];
			C = ptvec[1];
		}
		else if (i < ptvec.size() - 2) {
			A = ptvec[i - 1];
			B = ptvec[i];
			C = ptvec[i + 1];
		}
		else {
			A = ptvec[i - 1];
			B = ptvec[i];
			C = ptvec[0];
		}
		Pt q = initPeripheralSys(A, B, C, g);
		newptvec.push_back(q);
	}
	newptvec.push_back(newptvec[0]);
	return newptvec;
}


void ofApp::setup(){
	parameters.setName("Controls");
	parameters.add(boundaryconfigblank.set("Boundary config"));
	parameters.add(CurvaTure.set("Curvature", 0.75, 0.51, 0.99f));
	parameters.add(showCurveSeg.set("Place Curve Seg", true));
	parameters.add(CurveSegP0.set("Crv Seg P-0", 4, 1, 10));
	parameters.add(PeripheralCellDepth.set("P. Depth", 75, 5, 125));
	parameters.add(PeripheralCellLength.set("P. Length", 25, 5, 75));
	
	parameters.add(Corridor0.set("P. Cor Depth ", 25, 1, 75));

	parameters.add(DoorDepth.set("Door Depth ", 15, 5, 25));
	//parameters.add(color0.set("Color P.region", 225, ofColor(0, 0), 255));
	//parameters.add(color1.set("Color I.region", 225, ofColor(0, 0), 255));
	parameters.add(controlpts.set("Control Pts", false));
	parameters.add(rushconfigblank.set("Rush config"));
	parameters.add(rush.set("Rush Config", false));
	parameters.add(intgrid0.set("I. Grid-1", 1, 0, 10));
	parameters.add(intgrid1.set("I. Grid-2 ", 2, 0, 10));
	parameters.add(intgrid2.set("I. Grid-3 ", 1, 0, 10));
	parameters.add(fixint0.set("Fix-0 ILU. Config", false));
	parameters.add(fixint1.set("Fix-1 IRU. Config", false));
	parameters.add(fixint2.set("Fix-2 IRD. Config", false));
	parameters.add(fixint3.set("Fix-3 ILD. Config", false));
	parameters.add(fixint4.set("Fix-4 IC. Config", false));
	parameters.add(fixint5.set("Fix-5 ICR. Config", false));
	parameters.add(fixint6.set("Fix-6 ICL. Config", false));

	//parameters.add(intSpineCtrl.set("I. Spine Ctrl", 0.50, -0.95, 0.95));
	parameters.add(generalconfigblank.set("General Config"));
	parameters.add(spinecontrolpts.set("Spine Control Pts", false));
	parameters.add(spinecurvature.set("Spine Curvature", 0.01, 0.005, 0.99f));
	
	parameters.add(Corridor1.set("I. Cor Depth", 10, 1, 50));
	parameters.add(Corridor2.set("I. Cor2 Depth", 10, 1, 25));
	parameters.add(SpineDisplacement.set("I. Spine Ctrl", 0, -200, 200));
	parameters.add(spinedivpts.set("I. Grid-1", 15, 1, 50));
	parameters.add(showintspinequads.set("Show I Grids", true));

	parameters.add(showintregion.set("Show I region", false));
	parameters.add(subdivsystem.set("Subdiv I Grids", true));
	parameters.add(intsubdiv.set("Number of subdiv", 1,0,3));
	parameters.add(perisystem.set("Peripheral I Grids", false));

	gui.setup(parameters);
	gui.setBackgroundColor(ofColor(255,150,255));
	L = PeripheralCellLength; W = PeripheralCellLength; Corridor = Corridor0;
	ofSetBackgroundColor(255); ofSetColor(0); ofFill();
	iniptvec.push_back(Pt(550, 100)); iniptvec.push_back(Pt(950, 250)); iniptvec.push_back(Pt(1350, 100));
	iniptvec.push_back(Pt(1200, 450)); iniptvec.push_back(Pt(1350, 850)); iniptvec.push_back(Pt(950, 700));
	iniptvec.push_back(Pt(550, 850)); iniptvec.push_back(Pt(700, 450)); iniptvec.push_back(Pt(550, 100));
	oriptvec = iniptvec;

}

void ofApp::update(){
	L = PeripheralCellLength; W = PeripheralCellDepth; Corridor = Corridor0; Curvature = CurvaTure;

	revptvec.clear(); intptvec.clear(); revintptvec.clear(); intgridptvec.clear(); revintgridptvec.clear();
	revptvec = gensmoothboundary(iniptvec);

	straightSeg.clear(); crvpts.clear();
	intptvec = geninternalboundary(iniptvec, W);
	revintptvec= gensmoothboundary(intptvec);
	straightSeg = getsegvec(intptvec); straightSeg.push_back(straightSeg[0]);
	crvpts=getcrvpts(intptvec);
	
	intgridptvec= geninternalboundary(iniptvec, W+Corridor);
	revintgridptvec = gensmoothboundary(intgridptvec);
	intStraightSeg= getsegvec(intgridptvec); intStraightSeg.push_back(intStraightSeg[0]);

	A0 = intStraightSeg[0].A; B0 = intStraightSeg[0].B;
	A1 = intStraightSeg[1].A; B1 = intStraightSeg[1].B;
	A2 = intStraightSeg[2].A; B2 = intStraightSeg[2].B;
	A3 = intStraightSeg[3].A; B3 = intStraightSeg[3].B;
	A4 = intStraightSeg[4].A; B4 = intStraightSeg[4].B;
	A5 = intStraightSeg[5].A; B5 = intStraightSeg[5].B;
	A6 = intStraightSeg[6].A; B6 = intStraightSeg[6].B;
	A7 = intStraightSeg[7].A; B7 = intStraightSeg[7].B;
	A8 = intStraightSeg[8].A; B8 = intStraightSeg[8].B;

	diagpts.clear(); 
	diagpts.push_back(A0); diagpts.push_back(A1); diagpts.push_back(A2); diagpts.push_back(A3); diagpts.push_back(A4); diagpts.push_back(A5);
	diagpts.push_back(B0); diagpts.push_back(B1); diagpts.push_back(B2); diagpts.push_back(B3); diagpts.push_back(B4); diagpts.push_back(B5);
	diagpts.push_back(A6); diagpts.push_back(A7); diagpts.push_back(A8); diagpts.push_back(B6); diagpts.push_back(B7); diagpts.push_back(B8);
	
	float maxD = -1000;
	for (int i = 0; i < diagpts.size(); i++) {
		Pt a = diagpts[i];
		for (int j = 0; j < diagpts.size(); j++) {
			Pt b = diagpts[j];
			if (a.di(b) > maxD) {
				maxD = a.di(b); globaldiaA = a; globaldiaB = b;
			}			
		}
	}
	
	//function<bool(Seg, Seg)>sorter = sortSegDesc();
	//sort(diagseg.begin(), diagseg.end(), sorter);

	//spine 
	A = globaldiaA; F = globaldiaB; Pt U(F.x - A.x, F.y - A.y); Pt V(-U.y / A.di(F), U.x / A.di(F));
	B.setup(A.x + U.x*0.25, A.y + U.y*0.20); C.setup(A.x + U.x*0.40, A.y + U.y*0.40);
	D.setup(A.x + U.x*0.60, A.y + U.y*0.60); E.setup(A.x + U.x*0.80, A.y + U.y*0.80);

	if (controlspine == 0) {
		pB.setup(B.x + V.x * SpineDisplacement, B.y + V.y * SpineDisplacement); pB.locked = 0;
		pC.setup(C.x + V.x * SpineDisplacement * 2, C.y + V.y * SpineDisplacement * 2); pC.locked = 0;
		pD.setup(D.x + V.x * -SpineDisplacement * 2, D.y + V.y * -SpineDisplacement * 2); pD.locked = 0;
		pE.setup(E.x + V.x * -SpineDisplacement, E.y + V.y * -SpineDisplacement); pE.locked = 0;
	}
	gensmoothspinecurve();
}

void ofApp::draw() {
	trivec.clear();
	//plot smooth boundaries	
	float sW = 2; ofColor colr = (ofColor(255, 0, 0, 50)); 
	ofColor fill2(colr);
	path2.clear();
	path2.setStrokeColor(ofColor(0,0,0,255));
	path2.setStrokeWidth(sW); path2.setFillColor(fill2);
	path2.moveTo(revptvec[0].x, revptvec[0].y);
	for (int i = 1; i < revptvec.size(); i++) {
		path2.lineTo(revptvec[i].x, revptvec[i].y);
		//ofEllipse(revptvec[i].x,revptvec[i].y,10,10);
	}
	path2.draw();

	path3.clear();
	path3.setStrokeColor(ofColor(150, 150, 0, 155));
	path3.setStrokeWidth(sW); path3.setFillColor(ofColor(250, 250, 250));
	path3.moveTo(revintptvec[0].x, revintptvec[0].y);
	for (int j = 1; j < revintptvec.size(); j++) { path3.lineTo(revintptvec[j].x, revintptvec[j].y); }
	path3.draw();

	//plot normal segments at points on straight segments
	ofSetColor(ofColor(0,0,0,255)); ofSetLineWidth(sW);
	for (int i = 1; i < straightSeg.size(); i++) {
		Pt a = straightSeg[i - 1].B; Pt b = straightSeg[i].A;
		int n = a.di(b) / L;
		Pt u((b.x - a.x) / a.di(b), (b.y - a.y) / a.di(b));
		vector<Pt> pts;
		for (int j = 1; j < n + 1; j++) {
			Pt e(a.x + (u.x*j*L), a.y + (u.y*j*L));
			pts.push_back(e);
		}
		for (int j = 0; j < pts.size(); j++) {
			Pt e = pts[j]; Pt f(e.x - (u.y), e.y + (u.x));
			for (int k = 1; k < revptvec.size(); k++) {
				Pt g(revptvec[k - 1].x, revptvec[k - 1].y); Pt h(revptvec[k].x, revptvec[k].y);
				Pt I = intxPt2(e, f, g, h);
				if (I.di(e) < W*1.5 && I.di(e) > 0.75*W && I.x > 0) {
					ofLine(e.x, e.y, I.x, I.y);
					trivec.push_back(Tri(I, e, b));
					break;
				}
			}
		}
	}

	if (showCurveSeg == 1) {
		//plot curve points normal segment
		vector<Seg>prevseg; ofSetColor(ofColor(0,0,0,255)); ofSetLineWidth(sW);
		int mod = CurveSegP0;
		//plot point segments of curve & absorb in vector
		for (int i = 0; i < crvpts.size(); i++) {
			for (int j = 1; j < crvpts[i].size() - 1; j++) {
				if (j % mod == 0) {
					Pt p = crvpts[i][j];	Pt MP = crvpts[i][j]; int mpIn = j;
					Pt P = crvpts[i][mpIn - 1]; Pt E = MP; Pt R = crvpts[i][mpIn + 1];
					Pt U((R.x - P.x) / P.di(R), (R.y - P.y) / P.di(R));
					Pt F(E.x - U.y, E.y + U.x);
					for (int k = 1; k < revptvec.size(); k++) {
						Pt g(revptvec[k - 1].x, revptvec[k - 1].y); Pt h(revptvec[k].x, revptvec[k].y);
						Pt I = intxPt2(E, F, g, h);
						if (I.di(E) < W*2.f && I.di(E) > 0.75*W && I.x > 0) {
							ofLine(E.x, E.y, I.x, I.y);
							prevseg.push_back(Seg(E, I));
							trivec.push_back(Tri(I, E, R));
							break;
						}
					}
				}
			}
		}
		//plot mid point of crv 
		if (prevseg.size() == 0) {
			for (int i = 0; i < crvpts.size(); i++) {
				Pt p = crvpts[i][10];	Pt MP = crvpts[i][10]; int mpIn = 10;
				Pt P = crvpts[i][mpIn - 1]; Pt E = MP; Pt R = crvpts[i][mpIn + 1];
				Pt U((R.x - P.x) / P.di(R), (R.y - P.y) / P.di(R));
				Pt F(E.x - U.y, E.y + U.x);
				for (int k = 1; k < revptvec.size(); k++) {
					Pt g(revptvec[k - 1].x, revptvec[k - 1].y); Pt h(revptvec[k].x, revptvec[k].y);
					Pt I = intxPt2(E, F, g, h);
					if (I.di(E) < W*2.f && I.di(E) > 0.75*W && I.x > 0) {
						ofLine(E.x, E.y, I.x, I.y);
						prevseg.push_back(Seg(E, I));
						trivec.push_back(Tri(I, E, R));
						break;
					}
				}
			}
		}

	}
	
	/*
	//plot normal segments at curve segments
	for (int i = 1; i < crvpts.size()-1; i++) {
		if (i % 4 != 0) { continue;	}
		Pt p = crvpts[i - 1]; Pt e = crvpts[i]; Pt r = crvpts[i + 1];
		Pt u((r.x - p.x) / p.di(r), (r.y - p.y) / p.di(r));
		Pt f(e.x-u.y, e.y+u.x);
		for (int j = 1; j < revptvec.size(); j++) {
			Pt g(revptvec[j - 1].x, revptvec[j - 1].y); Pt h(revptvec[j].x, revptvec[j].y);
			Pt I = intxPt2(e, f, g, h);
			if (I.di(e) < W*2.f && I.di(e) > 0.75*W && I.x > 0) {
				int sum = 0;
				for (int k = 0; k < prevseg.size(); k++) {
					Pt m = prevseg[k].A; Pt n = prevseg[k].A;
					int J = doesintx(I, e, m, n);
					if (J == 1) { sum++; }
				}
				if (sum == 0) {
					ofLine(e.x, e.y, I.x, I.y);
					prevseg.push_back(Seg(e, I));
					trivec.push_back(Tri(I, e, r));
					break;
				}
			}
		}
	}
	*/

	//plot door swings
	float dw = DoorDepth; ofSetColor(ofColor(0, 0, 0, 255));	
	for (int i = 0; i < trivec.size(); i++) {
		Pt n = trivec[i].A; Pt e = trivec[i].B; Pt f = trivec[i].C;
		Pt u((f.x - e.x) / e.di(f), (f.y - e.y) / e.di(f));
		Pt v((n.x - e.x) / e.di(n), (n.y - e.y) / e.di(n));
		vector<Pt> doorpts;
		for (float j = 0.f; j < (PI / 2); j += PI / 20) {
			float A = 2 * PI - j;
			float x = u.x*cos(A)*dw - u.y*sin(A)*dw;
			float y = u.x*sin(A)*dw + u.y*cos(A)*dw;
			Pt r(e.x + x, e.y + y);
			doorpts.push_back(r);
		}
		Pt p0 = doorpts[0];
		doorpts.push_back(e);

		ofPath door; ofColor dcolr(255, 255, 255, 255); door.setStrokeWidth(1); door.setFillColor(dcolr); door.setStrokeColor(dcolr);
		door.moveTo(doorpts[0].x, doorpts[0].y);
		for (int j = 0; j < doorpts.size(); j++) {
			door.lineTo(doorpts[j].x, doorpts[j].y);
		}
		door.close();
		door.draw();

		ofPath doorswing; ; doorswing.setStrokeWidth(2); doorswing.setStrokeColor(ofColor(0, 0, 0));
		door.moveTo(doorpts[0].x, doorpts[0].y);
		for (int j = 0; j < doorpts.size(); j++) {
			doorswing.lineTo(doorpts[j].x, doorpts[j].y);
		}
		doorswing.draw();
	}

	if (controlpts == true) {
		//control lines - initial polygon
		for (int i = 0; i < iniptvec.size(); i++) {
			if (iniptvec[i].locked == 1) {
				ofSetColor(255, 0, 0, 100); ofFill();
				ofEllipse(iniptvec[i].x, iniptvec[i].y, 25, 25);
				ofNoFill(); ofSetColor(255, 0, 0, 100);
				ofEllipse(iniptvec[i].x, iniptvec[i].y, 35, 35);
			}
			ofSetColor(100, 100, 100); ofEllipse(iniptvec[i].x, iniptvec[i].y, 10, 10);
			if (i > 0) {
				ofSetColor(225, 225, 225); ofSetLineWidth(1); ofLine(iniptvec[i - 1].x, iniptvec[i - 1].y, iniptvec[i].x, iniptvec[i].y);
				ofNoFill(); ofSetColor(50, 50, 50);	ofEllipse(iniptvec[i].x, iniptvec[i].y, 35, 35);
			}
		}
	}

	ofFill(); ofSetColor(ofColor(0, 0, 0, 50)); ofDrawRectangle(15, 675, 350, 150);
	ofSetColor(0, 0, 0);
	string MSG = "Keyboard controls:";
	MSG += "\n------------------";
	MSG += "\nPress 'r' 'R' to reset to original drawing";
	MSG += "\nPress 'w' 'W' to regen int config";
	MSG += "\nPress 's' 'S' to save image";	
	MSG += "\nPress 'e' 'E' to control spine";
	MSG += "\nPress 'f' 'F' to stop control spine";
	MSG += "\nPress 'g' 'G' to subdivide I quads";
	MSG += "\nPress 'h' 'H' to gen peripheral I quads";
	ofDrawBitmapString(MSG, 20, 700);
	string title = "PhD student: Nirvik Saha (G.I.T.) \t\tadvisor: Dennis R Shelden (G.I.T.) \t\tadvisor: John R Haymaker (P + W)";
	ofSetColor(0, 0, 0, 255); ofDrawBitmapString(title, 100, ofGetHeight() - 30);
	gui.draw();


	if (rush == 1) {
		ofSetColor(0); ofSetLineWidth(1);
		for (int i = 0; i < quads0.size(); i++) { quads0[i].display(); }
		for (int i = 0; i < quads1.size(); i++) { quads1[i].display(); }
		for (int i = 0; i < quads2.size(); i++) { quads2[i].display(); }
		for (int i = 0; i < quads3.size(); i++) { quads3[i].display(); }
		for (int i = 0; i < quads4.size(); i++) { quads4[i].display(); }
		for (int i = 0; i < quads5.size(); i++) { quads5[i].display(); }
		for (int i = 0; i < quads6.size(); i++) { quads6[i].display(); }
	}	


	//show internal regions
	if (showintregion == true) {
		path4.clear();
		path4.setStrokeColor(ofColor(0, 0, 0, 155)); path4.setFillColor(ofColor(255, 255, 255));
		path4.setStrokeWidth(sW);
		path4.moveTo(revintgridptvec[0].x, revintgridptvec[0].y);
		for (int j = 1; j < revintgridptvec.size(); j++) {
			path4.lineTo(revintgridptvec[j].x, revintgridptvec[j].y);
		}
		path4.draw();

		//show the spine line
		for (int i = 1; i < spineptvec.size(); i++) {
			Pt a = spineptvec[i - 1]; Pt b = spineptvec[i];
			ofSetColor(0, 0, 0, 50); ofSetLineWidth(1); ofDrawLine(a.x, a.y, b.x, b.y);
			ofEllipse(a.x, a.y, 3, 3);
		}
	}

	//only plotting of spine vector
	if (spinecontrolpts == true) {
		if (pB.locked == 1) {
			ofFill(); ofSetColor(255, 0, 0, 50);
			ofEllipse(pB.x, pB.y, 25, 25);
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150); ofDrawLine(pB.x, pB.y, B.x, B.y);
		}
		else {
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150);
			ofEllipse(pB.x, pB.y, 25, 25);
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150); ofDrawLine(pB.x, pB.y, B.x, B.y);
		}

		if (pC.locked == 1) {
			ofFill(); ofSetColor(255, 0, 0, 50);
			ofEllipse(pC.x, pC.y, 25, 25);
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150); ofDrawLine(pC.x, pC.y, C.x, C.y);
		}
		else {
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150);
			ofEllipse(pC.x, pC.y, 25, 25);
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150); ofDrawLine(pC.x, pC.y, C.x, C.y);
		}

		if (pD.locked == 1) {
			ofFill(); ofSetColor(255, 0, 0, 50);
			ofEllipse(pD.x, pD.y, 25, 25);
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150); ofDrawLine(pD.x, pD.y, D.x, D.y);
		}
		else {
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150);
			ofEllipse(pD.x, pD.y, 25, 25);
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150); ofDrawLine(pD.x, pD.y, D.x, D.y);
		}

		if (pE.locked == 1) {
			ofFill(); ofSetColor(255, 0, 0, 50);
			ofEllipse(pE.x, pE.y, 25, 25);
			ofSetLineWidth(1); ofSetColor(255, 0, 0, 150); ofDrawLine(pE.x, pE.y, E.x, E.y);
		}
		else {
			ofSetLineWidth(1); ofSetColor(0, 0, 0, 150);
			ofEllipse(pE.x, pE.y, 25, 25);
			ofSetLineWidth(1); ofSetColor(255, 0, 0, 150); ofDrawLine(pE.x, pE.y, E.x, E.y);
		}
		//plot the spine
		for (int i = 1; i < spineptvec.size(); i++) {
			Pt a = spineptvec[i - 1]; Pt b = spineptvec[i];
			ofDrawLine(a.x, a.y, b.x, b.y);
		}
	}

	vector<Pt> spinept;
	for (int i = 1; i < spineptvec.size(); i+=spinedivpts) {
		spinept.push_back(spineptvec[i]); 
	} 
	spinept.push_back(spineptvec[spineptvec.size()-1]);

	//spine top 
	vector<Seg> spineupvec; vector<Seg> spinednvec;
	for (int i = 1; i < spinept.size(); i++) {
		Pt a = spinept[i - 1]; Pt b = spinept[i]; 
		Pt u((b.x - a.x)*Corridor1/a.di(b), (b.y - a.y)*Corridor1 / a.di(b));	
		Pt aup(a.x + u.y, a.y - u.x); Pt adn(a.x - u.y, a.y + u.x); 
		if (showintregion == true) {
			ofEllipse(aup.x, aup.y, 15, 15); ofEllipse(adn.x, adn.y, 15, 15);
		}
		
		Pt u_((b.x - a.x) * 200 / a.di(b), (b.y - a.y) * 200 / a.di(b));
		Pt a_(aup.x + u_.y*100, aup.y - u_.x*100); 
		for (int j = 1; j < revintgridptvec.size(); j++) {
			Pt r = revintgridptvec[j - 1]; Pt s = revintgridptvec[j];
			Pt I = intxPt4(a, a_, r, s);
			if (I.x > 0 && I.y > 0) {
				spineupvec.push_back(Seg(aup, I));
				break;
			}
		}
		Pt b_(adn.x - u_.y * 100, adn.y + u_.x * 100);
		for (int j = 1; j < revintgridptvec.size(); j++) {
			Pt r = revintgridptvec[j - 1]; Pt s = revintgridptvec[j];
			Pt I = intxPt4(a, b_, r, s);
			if (I.x > 0 && I.y > 0) {
				spinednvec.push_back(Seg(adn, I));
				break;
			}
		}
	}

	int SUBDIV = intsubdiv;
	vector<Quad> intsubdivquadvec;

	//setup ortho adjusted quads inside & across spine - above corridor
	for (int i = 1; i < spineupvec.size(); i++) {
		Pt A = spineupvec[i - 1].A; Pt B = spineupvec[i - 1].B;
		Pt C = spineupvec[i].A; Pt D = spineupvec[i].B;
		if (A.di(B) > C.di(D)) {
			Pt u(A.x - C.x, A.y - C.y);
			Pt E(D.x + u.x, D.y + u.y);
			Pt B_ = intxPt2(D, E, A, B);
			if (B_.x > 0 && C.di(D)>25) {
				Quad q(A, B_, D, C);
				intsubdivquadvec.push_back(q);
			}				
		}
		else {
			Pt u(C.x - A.x, C.y - A.y);
			Pt E(B.x + u.x, B.y + u.y);
			Pt D_ = intxPt2(B, E, C, D);
			if (D_.x > 0 && A.di(B)>25) {
				Quad q(A, B, D_, C);
				intsubdivquadvec.push_back(q);
			}				
		}
	}

	//setup ortho adjusted quads inside & across spine - below corridor
	for (int i = 1; i < spinednvec.size(); i++) {
		Pt A = spinednvec[i - 1].A; Pt B = spinednvec[i - 1].B;
		Pt C = spinednvec[i].A; Pt D = spinednvec[i].B;
		Quad q;
		if (A.di(B) > C.di(D)) {
			Pt u(A.x - C.x, A.y - C.y);
			Pt E(D.x + u.x, D.y + u.y);
			Pt B_ = intxPt2(D, E, A, B);
			if (B_.x > 0 && C.di(D)>25) {
				q.setupPreservePts(A, B_, D, C);
				intsubdivquadvec.push_back(q);
			}
		}
		else {
			Pt u(C.x - A.x, C.y - A.y);
			Pt E(B.x + u.x, B.y + u.y);
			Pt D_ = intxPt2(B, E, C, D);
			if (D_.x > 0 && A.di(B)>25) {
				q.setupPreservePts(A, B, D_, C);
				intsubdivquadvec.push_back(q);
			}				
		}
	}

	if (showintspinequads == true && perisystem==false) {
		for (int i = 0; i < intsubdivquadvec.size(); i++) {
			intsubdivquadvec[i].display2();
		}
	}

	//peripheral system reg the collected quads
	vector<Tri>trivec2;
	if (perisystem == true) {
		intperipgeneratequads.clear();
		for (int i = 0; i < intsubdivquadvec.size(); i++) {
			Quad q = intsubdivquadvec[i];
			Pt a = q.pts[0]; 
			Pt b = q.pts[1]; 
			Pt c = q.pts[2]; 
			Pt d = q.pts[3];
			Pt e((a.x + d.x) / 2, (a.y + d.y) / 2);
			Pt f((b.x + c.x) / 2, (b.y + c.y) / 2);
			float w = Corridor2;
			Pt a_(a.x + (e.x - a.x)*w*0.5 / e.di(a), a.y + (e.y - a.y)*w*0.5 / e.di(a));
			Pt b_(b.x + (f.x - b.x)*w*0.5 / f.di(b), b.y + (f.y - b.y)*w*0.5 / f.di(b));
			Pt c_(c.x + (f.x - c.x)*w*0.5 / f.di(c), c.y + (f.y - c.y)*w*0.5 / f.di(c));
			Pt d_(d.x + (e.x - d.x)*w*0.5 / e.di(d), d.y + (e.y - d.y)*w*0.5 / e.di(d));

			Pt u((f.x - e.x)/e.di(f), (f.y - e.y) / e.di(f));
			float j = 0.f; float L = 50.f;
			while (j < a.di(b)) {
				Pt p(a_.x + u.x*j, a_.y + u.y*j);
				Pt q(e.x + u.x*j, e.y + u.y*j);
				Pt r(a_.x + u.x*(j + L), a_.y + u.y*(j + L));
				Pt s(e.x + u.x*(j + L), e.y + u.y*(j + L));

				Pt p_(d_.x + u.x*j, d_.y + u.y*j);
				Pt q_(e.x + u.x*j, e.y + u.y*j);
				Pt r_(d_.x + u.x*(j + L), d_.y + u.y*(j + L));
				Pt s_(e.x + u.x*(j + L), e.y + u.y*(j + L));

				if (a_.di(q) < e.di(f)  && d_.di(r_) < e.di(f)) {
					trivec2.push_back(Tri(p, r, s));
					trivec2.push_back(Tri(s_,r_,p_));
					Quad q2(p, r, s, q); intperipgeneratequads.push_back(q2);
					Quad q3(p_, r_, s_, q_); intperipgeneratequads.push_back(q3);
				}
				j += L;
			}
		}
		for (int i = 0; i < intperipgeneratequads.size(); i++) {
			intperipgeneratequads[i].display2();
		}

		//door swing
		float dw = DoorDepth; ofSetColor(ofColor(0, 0, 0, 255));
		for (int i = 0; i < trivec2.size(); i++) {
			Pt n = trivec2[i].A; Pt e = trivec2[i].B; Pt f = trivec2[i].C;
			Pt u((f.x - e.x) / e.di(f), (f.y - e.y) / e.di(f));
			Pt v((n.x - e.x) / e.di(n), (n.y - e.y) / e.di(n));
			Pt n_(e.x + (n.x - e.x)*dw / e.di(n), e.y + (n.y - e.y)*dw / e.di(n));
			Pt f_(e.x + (f.x - e.x)*dw / e.di(f), e.y + (f.y - e.y)*dw / e.di(f));
			ofSetLineWidth(1); ofSetColor(255,0,0); 
			ofLine(n_.x, n_.y, f_.x, f_.y);
			ofEllipse(e.x, e.y, 5, 5);
		}
	
	}//end of peripheral systems

	//subdivide the collected quads - interior subdiv across spine
	if (generatesubdivsystem == 1 && subdivsystem == true) {
		for (int i = 0; i < intsubdivquadvec.size(); i++) {
			subdivQuadVec.clear(); subdiv(intsubdivquadvec[i], 0, SUBDIV);
			vector<Quad> quadvec = subdivQuadVec;
			subdiv(intsubdivquadvec[i], 0, SUBDIV);
			for (int j = 0; j < quadvec.size(); j++) {
				intsubdivgeneratequads.push_back(quadvec[j]);
			}				
		}
	}

	//display the subdivided quads across spine
	if (subdivsystem == true) {
		for (int i = 0; i < intsubdivgeneratequads.size(); i++) {
			intsubdivgeneratequads[i].display();
		}
	}else{ 
		intsubdivgeneratequads.clear(); intsubdivquadvec.clear(); 
	}

	generatesubdivsystem = 0;	

	if (showintspinequads == false) {
		spineupvec.clear(); spinednvec.clear(); intsubdivgeneratequads.clear();
	}

}

void ofApp::keyPressed(int key){
	if (key == 'r' || key == 'R') {
		iniptvec = oriptvec;

	}
	if (key == 's' || key == 'S') {
		global_image_counter++;
		ofImage screenshot;
		screenshot.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
		screenshot.saveImage("screenshot" + to_string(global_image_counter) + ".png");
	}
	if (key == 'w' || key == 'W') {
		intRushConfig();
	}
	if (key == 'e' || key == 'E') {
		controlspine = 1;
	}
	if (key == 'f' || key == 'F') {
		controlspine = 0;
		SpineDisplacement = 0; SpineDisplacement.set(0);
	}
	if (key == 'g' || key == 'G') { 
		intsubdivgeneratequads.clear();
		generatesubdivsystem = 1; 		
	}
}

void ofApp::keyReleased(int key){

}

void ofApp::mouseMoved(int x, int y ){
	float S = 35;
	for (int i = 0; i < iniptvec.size(); i++) {
		Pt m(x, y);	if (m.di(iniptvec[i]) < S) { iniptvec[i].locked = 1; }
	}
	for (int i = 0; i < iniptvec.size(); i++) {
		if (iniptvec[i].di(Pt(x, y)) > S) {
			iniptvec[i].locked = 0;
		}
	}

	//lock control spine points
	if (Pt(x, y).di(pB) < 25) {	pB.locked = 1; }
	else if (Pt(x, y).di(pB) > 25) { pB.locked = 0; }

	if (Pt(x, y).di(pC) < 25) {	pC.locked = 1; }
	else if (Pt(x, y).di(pC) > 25) { pC.locked = 0; }

	if (Pt(x, y).di(pD) < 25) {	pD.locked = 1; }
	else if (Pt(x, y).di(pD) > 25) { pD.locked = 0; }

	if (Pt(x, y).di(pE) < 25) { pE.locked = 1; }
	else if (Pt(x, y).di(pE) > 25) { pE.locked = 0; }
}

void ofApp::mouseDragged(int x, int y, int button){
	for (int i = 0; i < iniptvec.size(); i++) { 
		if (iniptvec[i].locked==1) { iniptvec[i].setup(x, y); }
	}
	intRushConfig();

	//lock control spine points
	if (pB.locked == 1) { pB.setup(x, y); }
	if (pC.locked == 1) { pC.setup(x, y); }
	if (pD.locked == 1) { pD.setup(x, y); }
	if (pE.locked == 1) { pE.setup(x, y); }
}

void ofApp::mousePressed(int x, int y, int button){
	for (int i = 0; i < iniptvec.size(); i++) {
		if (iniptvec[i].locked == 1) { iniptvec[i].setup(x, y); }
	}
}

void ofApp::mouseReleased(int x, int y, int button){
	for (int i = 0; i < iniptvec.size(); i++) {
		if (iniptvec[i].locked == 1) { 
			iniptvec[i].setup(x, y); 
			iniptvec[i].locked = 0; 
		}
	}
	for (int i = 0; i < iniptvec.size(); i++) {
		iniptvec[i].locked = 0;
	}
}

void ofApp::mouseEntered(int x, int y){

}

void ofApp::mouseExited(int x, int y){

}

void ofApp::windowResized(int w, int h){

}

void ofApp::gotMessage(ofMessage msg){

}

void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

Pt ofApp::intxPt(Pt p, Pt q, Pt r, Pt s) {
	float a1 = q.y - p.y; float b1 = p.x - q.x;	float c1 = a1*q.x + b1*q.y;
	float a2 = s.y - r.y; float b2 = r.x - s.x; float c2 = a2*s.x + b2*s.y;
	float det = a1*b2 - a2*b1; if (det == 0) { return Pt(-100, -100); }
	float mx = (c1*b2 - c2*b1) / det; float my = (c2*a1 - c1*a2) / det;
	return Pt(mx, my);
}

Pt ofApp::intxPt2(Pt p, Pt q, Pt r, Pt s) {
	float a1 = q.y - p.y; float b1 = p.x - q.x;	float c1 = a1*q.x + b1*q.y;
	float a2 = s.y - r.y; float b2 = r.x - s.x; float c2 = a2*s.x + b2*s.y;
	float det = a1*b2 - a2*b1; if (det == 0) { return Pt(-100, -100); }
	float mx = (c1*b2 - c2*b1) / det; float my = (c2*a1 - c1*a2) / det;
	Pt I(mx, my);
	if (abs(I.di(r) + I.di(s) - r.di(s)) < 1) { return Pt(mx, my); }
	else { return Pt(-100, 100); }
}

Pt ofApp::intxPt4(Pt p, Pt q, Pt r, Pt s) {
	float a1 = q.y - p.y; float b1 = p.x - q.x;	float c1 = a1*q.x + b1*q.y;
	float a2 = s.y - r.y; float b2 = r.x - s.x; float c2 = a2*s.x + b2*s.y;
	float det = a1*b2 - a2*b1; if (det == 0) { return Pt(-100, -100); }
	float mx = (c1*b2 - c2*b1) / det; float my = (c2*a1 - c1*a2) / det;
	Pt I(mx, my);
	if ((abs(I.di(r) + I.di(s) - r.di(s)) < 1) && (abs(I.di(p) + I.di(q) - p.di(q)) < 1)) {
		return Pt(mx, my); 
	}
	else { return Pt(-100, 100); }
}

int ofApp::doesintx(Pt p, Pt q, Pt r, Pt s) {
	float a1 = q.y - p.y; float b1 = p.x - q.x;	float c1 = a1*q.x + b1*q.y;
	float a2 = s.y - r.y; float b2 = r.x - s.x; float c2 = a2*s.x + b2*s.y;
	float det = a1*b2 - a2*b1; if (det == 0) { return 0; }
	float mx = (c1*b2 - c2*b1) / det; float my = (c2*a1 - c1*a2) / det;
	Pt I(mx, my);
	if (abs(I.di(r) + I.di(s) - r.di(s)) < 1) { return 1; }
	else { return 0; }
}

/*	angular bisector at B for triangle ABC	*/
Pt ofApp::initPeripheralSys(Pt p, Pt q, Pt r, float g) {
	Pt u_((q.x - p.x) / p.di(q), (q.y - p.y) / p.di(q)); Pt u(-u_.y, u_.x);
	Pt v_((r.x - q.x) / r.di(q), (r.y - q.y) / r.di(q)); Pt v(-v_.y, v_.x);
	Pt pq((p.x + q.x) / 2, (p.y + q.y) / 2);
	Pt qr((q.x + r.x) / 2, (q.y + r.y) / 2);
	Pt P1(p.x + u.x*g, p.y + u.y*g); Pt Q(q.x + u.x*g, q.y + u.y*g);
	Pt P2(q.x + v.x*g, q.y + v.y*g); Pt R(r.x + v.x*g, r.y + v.y*g);
	Pt I=intxPt(P1, Q, P2, R);
	return I;
}

float ofApp::heron(Pt a, Pt b, Pt c) {
	float l = a.di(b);
	float m = a.di(c);
	float n = c.di(b);
	float s = (l + m + n) / 2;
	float ar = sqrt(s*(s - l)*(s - m)*(s - n));
	return ar;
}

int ofApp::heronContains(Pt p, Pt a, Pt b, Pt c) {
	float ar = heron(a, b, c);
	float ar0 = heron(a, p, b);
	float ar1 = heron(b, p, c);
	float ar2 = heron(c, p, a);
	float diff = abs(ar - (ar0 + ar1 + ar2));
	if (diff < 10) {
		return 1;
	}
	else {
		return 0;
	}
}

Pt ofApp::proj(Pt a, Pt b, Pt c) {
	//proj b on ac
	Pt u(a.x - c.x, a.y - c.y); Pt v(b.x - c.x, b.y - c.y);
	float e = (u.x*v.x + u.y*v.y) / (c.di(a)*c.di(a));	
	Pt r(c.x + (e*u.x), c.y + (e*u.y));
	return r;
}

/* init subdiv process  */
vector<Quad> ofApp::initSubdiv(Pt a, Pt b, Pt c, Pt d, int t, vector<Quad> quad) {
	subdivQuadVec.clear(); 
	Quad q(a,b,c,d);
	subdiv(q, 0, t);
	//subdivVec.push_back(subdivQuadVec);
	//quad.push_back(subdivQuadVec);
	for (int i = 0; i < subdivQuadVec.size(); i++) {
		quad.push_back(subdivQuadVec[i]);
	}
	subdivQuadVec.clear();
	return quad;
}

void ofApp::intRushConfig() {
	vector<Pt>spineptvec;
	spineptvec.push_back(A0); spineptvec.push_back(A1);
	spineptvec.push_back(A2); spineptvec.push_back(A3);
	spineptvec.push_back(A4); spineptvec.push_back(A5);
	spineptvec.push_back(A6); spineptvec.push_back(A7);
	spineptvec.push_back(A8);

	vector<Seg>spinesegvec;
	for (int i = 0; i < spineptvec.size(); i++) {
		for (int j = 0; j < spineptvec.size(); j++) {
			Pt a = spineptvec[i]; Pt b = spineptvec[j];
			if (a.di(b) > 1) {
				spinesegvec.push_back(Seg(a, b));
			}
		}
	}
	function<bool(Seg, Seg)>sorter = sortSegDesc();
	sort(spinesegvec.begin(), spinesegvec.end(), sorter);

	ofNoFill();
	Pt a = spinesegvec[0].A; Pt c = spinesegvec[0].B;
	Pt U((c.x - a.x) / a.di(c), (c.y - a.y) / a.di(c)); Pt r((a.x + c.x) / 2, (a.y + c.y) / 2);
	float minD = 1000000;
	for (int i = 0; i < spineptvec.size(); i++) {
		Pt p = spineptvec[i];
		Pt q = proj(a, p, c);
		float D = p.di(q);
		if (D < minD && D > 1.f) { minD = D; }
	}

	Pt b(r.x - U.y*minD, r.y + U.x*minD); Pt w(r.x + (b.x - r.x)*intSpineCtrl, r.y + (b.y - r.y)*intSpineCtrl);

	vector<Pt>midptvec;
	for (float t = 0; t < 1; t += 0.01) {
		float x = (a.x*pow((1 - t), 2)) + (w.x * 2 * (1 - t)*t) + c.x*pow(t, 2);
		float y = (a.y*pow((1 - t), 2)) + (w.y * 2 * (1 - t)*t) + c.y*pow(t, 2);
		midptvec.push_back(Pt(x, y));
	}

	for (int j = 0; j < midptvec.size() - 1; j++) {	Pt e = midptvec[j]; Pt f = midptvec[j + 1]; }

	Pt b7(B7.x + (A0.x - B7.x)*Corridor1 / (B7.di(A0)), B7.y + (A0.y - B7.y)*Corridor1 / (B7.di(A0)));
	Pt a1(A1.x + (B0.x - A1.x)*Corridor1 / (B0.di(A1)), A1.y + (B0.y - A1.y)*Corridor1 / (B0.di(A1)));
	Pt b1(B1.x + (A2.x - B1.x)*Corridor1 / (A2.di(B1)), B1.y + (A2.y - B1.y)*Corridor1 / (A2.di(B1)));
	Pt a3(A3.x + (B2.x - A3.x)*Corridor1 / (B2.di(A3)), A3.y + (B2.y - A3.y)*Corridor1 / (B2.di(A3)));
	Pt a5(A5.x + (B4.x - A5.x)*Corridor1 / (B4.di(A5)), A5.y + (B4.y - A5.y)*Corridor1 / (B4.di(A5)));
	Pt b3(B3.x + (A4.x - B3.x)*Corridor1 / (A4.di(B3)), B3.y + (A4.y - B3.y)*Corridor1 / (A4.di(B3)));
	Pt a7(A7.x + (B6.x - A7.x)*Corridor1 / (B6.di(A7)), A7.y + (B6.y - A7.y)*Corridor1 / (B6.di(A7)));
	Pt b5(B5.x + (A6.x - B5.x)*Corridor1 / (A6.di(B5)), B5.y + (A6.y - B5.y)*Corridor1 / (A6.di(B5)));

	vector<Quad> quad;
	if (fixint0 == 0 && (B7.di(A0))>50) {	quads0.clear();	quads0 = initSubdiv(A0, B0, a1, b7, intgrid0, quads0); } // inter
	if (fixint1 == 0 && (B0.di(A1))>50) {	quads1.clear();	quads1 = initSubdiv(A2, B2, a3, b1, intgrid0, quads1); } // inter
	if (fixint2 == 0 && (A2.di(B1))>50) {	quads2.clear();	quads2 = initSubdiv(b3, A4, B4, a5, intgrid0, quads2); } // inter
	if (fixint3 == 0 && (B2.di(A3))>50) {	quads3.clear();	quads3 = initSubdiv(b5, A6, B6, a7, intgrid0, quads3); } // inter
	
	if (fixint4 == 0) { quads4.clear();	quads4 = initSubdiv(A1, B1, A5, B5, intgrid1, quads4); } // center part

	/*
	Pt a1_ = proj(B7, A1, A3); Pt b5_ = proj(A7, B5, B3); // adjust for left -center
	Pt b1_ = proj(B7, B1, A3); Pt a5_ = proj(A7, A5, B3); // right for right -center
	if (fixint6 == 0) { quads6.clear();	quads6 = initSubdiv(B7, a1_, b5_, A7, intgrid2, quads6); } // left center
	if (fixint5 == 0) {	quads5.clear();	quads5 = initSubdiv(b1_, A3, B3, a5_, intgrid2, quads5); } // right center
	*/

	if (fixint6 == 0) { quads6.clear();	quads6 = initSubdiv(B7, A1, B5, A7, intgrid2, quads6); } // left center
	if (fixint5 == 0) { quads5.clear();	quads5 = initSubdiv(B1, A3, B3, A5, intgrid2, quads5); } // right center

	if (B7.di(A0) < 50) { quads0.clear(); }
	if (B0.di(A1) < 50) { quads1.clear(); }
	if (A2.di(B1) < 50) { quads2.clear(); }
	if (B2.di(A3) < 50) { quads3.clear(); }

	//rush = 0;
	//rush.set(false);
}

/*	recursive function for subdividion process	*/
void ofApp::subdiv(Quad Q, int t, int w) {
	Pt a, b, c, d;
	a.setup(Q.pts[0].x, Q.pts[0].y);
	b.setup(Q.pts[1].x, Q.pts[1].y);
	c.setup(Q.pts[2].x, Q.pts[2].y);
	d.setup(Q.pts[3].x, Q.pts[3].y);
	int g = (int)ofRandom(0, 2);
	Quad R, S; Pt m, n;
	if (g == 0) {
		m.setup((a.x + b.x) / 2, (a.y + b.y) / 2);
		n.setup((c.x + d.x) / 2, (c.y + d.y) / 2);
		R.setupPreservePts(a, m, n, d);
		S.setupPreservePts(m, b, c, n);
	}
	else {
		m.setup((b.x + c.x) / 2, (b.y + c.y) / 2);
		n.setup((a.x + d.x) / 2, (a.y + d.y) / 2);
		R.setupPreservePts(a, b, m, n);
		S.setupPreservePts(n, m, c, d);
	}
	if (t < w) {
		t++;
		subdiv(R, t, w);
		subdiv(S, t, w);
	}
	else {
		subdivQuadVec.push_back(R);
		subdivQuadVec.push_back(S);
	}
}



