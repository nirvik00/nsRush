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

vector<Pt>ofApp::Lerp(Pt a, Pt b, Pt c) {
	Pt ab[20]; Pt bc[20]; vector<Pt> pts;
	int k = 0;
	float n = 1.f / 20.f;
	for (float i = 0.f; i < 1.f; i+=n) {
		ab[k].setup(a.x + (b.x - a.x)*i, a.y + (b.y - a.y)*i);
		bc[k].setup(b.x + (c.x - b.x)*i, b.y + (c.y - b.y)*i);
		k++;
	}
	k = 0;
	for (float i = 0.f; i < 1.f; i+=n) {
		float x = ab[k].x + (bc[k].x - ab[k].x)*i;
		float y = ab[k].y + (bc[k].y - ab[k].y)*i;
		Pt e(x, y);
		pts.push_back(e);
		k++;
	}
	return pts;
}

void ofApp::setup(){
	parameters.setName("Controls");
	parameters.add(CurvaTure.set("Curvature", 0.51, 0.51, 0.99f));
	parameters.add(showCurveSeg.set("Place Curve Seg", true));
	parameters.add(CurveSegP0.set("Crv Seg P-0", 4, 1, 10));
	parameters.add(PeripheralCellDepth.set("P. Depth", 75, 5, 125));
	parameters.add(PeripheralCellLength.set("P. Length", 25, 5, 75));
	parameters.add(intSpineCtrl.set("I. Spine Ctrl", 0.75, 0.5, 0.95));
	parameters.add(Corridor0.set("P. Cor Depth ", 5, 1, 75));
	parameters.add(Corridor1.set("I. Cor Depth", 5, 1, 15));
	parameters.add(DoorDepth.set("Door Depth ", 15, 5, 25));
	parameters.add(color0.set("Color P.region", 225, ofColor(0, 0), 255));
	parameters.add(color1.set("Color I.region", 225, ofColor(0, 0), 255));
	parameters.add(controlpts.set("Control Pts", false));
	gui.setup(parameters);
	gui.setBackgroundColor(ofColor(255,255,255));
	L = PeripheralCellLength; W = PeripheralCellLength; Corridor = Corridor0;
	ofSetBackgroundColor(255); ofSetColor(0); ofFill();
	iniptvec.push_back(Pt(350, 100)); iniptvec.push_back(Pt(750, 250)); iniptvec.push_back(Pt(1150, 100));
	iniptvec.push_back(Pt(1000, 450)); iniptvec.push_back(Pt(1150, 850)); iniptvec.push_back(Pt(750, 700));
	iniptvec.push_back(Pt(350, 850)); iniptvec.push_back(Pt(500, 450)); iniptvec.push_back(Pt(350, 100));
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
}

void ofApp::draw() {
	trivec.clear();
	//plot smooth boundaries	
	float sW = 2; ofColor colr=(ofColor(0,0,0,255)); ofColor fill2(color0);//fill color from interface
	path2.clear();
	path2.setStrokeColor(colr);
	path2.setStrokeWidth(sW); path2.setFillColor(fill2);
	path2.moveTo(revptvec[0].x, revptvec[0].y);
	for (int i = 1; i < revptvec.size(); i++) {
		path2.lineTo(revptvec[i].x, revptvec[i].y);
		//ofEllipse(revptvec[i].x,revptvec[i].y,10,10);
	}
	path2.draw();

	path3.clear();
	path3.setStrokeColor(colr);
	path3.setStrokeWidth(sW); path3.setFillColor(ofColor(255, 255, 255));
	path3.moveTo(revintptvec[0].x, revintptvec[0].y);
	for (int j = 1; j < revintptvec.size(); j++) { path3.lineTo(revintptvec[j].x, revintptvec[j].y); }
	path3.draw();

	path4.clear();
	path4.setStrokeColor(colr); ofColor fill4(color1);//fill color from interface
	path4.setStrokeWidth(sW); path4.setFillColor(ofColor(fill4));
	path4.moveTo(revintgridptvec[0].x, revintgridptvec[0].y);
	for (int j = 1; j < revintgridptvec.size(); j++) { 
		path4.lineTo(revintgridptvec[j].x, revintgridptvec[j].y); 
		//ofEllipse(revintgridptvec[j].x, revintgridptvec[j].y, 10, 10);
	}
	path4.draw();

	
	//plot normal segments at points on straight segments
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
					ofSetColor(colr); ofSetLineWidth(sW); ofLine(e.x, e.y, I.x, I.y);
					trivec.push_back(Tri(I, e, b));
					break;
				}
			}
		}
	}

	if (showCurveSeg == 1) {
		//plot curve points normal segment
		vector<Seg>prevseg; ofSetColor(0); ofSetLineWidth(sW);
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
	float dw = DoorDepth;
	for (int i = 0; i < trivec.size(); i++) {
		Pt n = trivec[i].A; Pt e = trivec[i].B; Pt f = trivec[i].C;
		Pt u((f.x - e.x) / e.di(f), (f.y - e.y) / e.di(f));
		Pt v((n.x - e.x) / e.di(n), (n.y - e.y) / e.di(n));
		vector<Pt> doorpts;
		for (float j = 0.f; j < (PI/2); j+=PI/20) {
			float A = 2*PI-j;
			float x = u.x*cos(A)*dw - u.y*sin(A)*dw;
			float y = u.x*sin(A)*dw + u.y*cos(A)*dw;
			Pt r(e.x + x, e.y + y);
			doorpts.push_back(r);
		}
		Pt p0 = doorpts[0];
		doorpts.push_back(e);

		ofPath door; ofColor dcolr(255, 255, 255, 255); door.setStrokeWidth(1); door.setFillColor(dcolr); door.setStrokeColor(dcolr); 
		door.moveTo(doorpts[0].x,doorpts[0].y);
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
			if (iniptvec[i].locked == 1) { ofSetColor(255, 0, 0, 100); ofFill(); 
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
	
	ofFill();ofSetColor(ofColor(0, 0, 0, 50)); ofDrawRectangle(15, 400, 350, 100);
	ofSetColor(0, 0, 0);
	string MSG = "Keyboard controls:";
	MSG += "\n------------------";
	MSG += "\nPress 'r' 'R' to reset to original drawing";
	MSG += "\nPress 's' 'S' to save image";
	ofDrawBitmapString(MSG, 20, 425);
	string title = "PhD student: Nirvik Saha (G.I.T.) \t\tadvisor: Dennis R Shelden (G.I.T.) \t\tadvisor: John R Haymaker (P + W)";
	ofSetColor(0, 0, 0, 255); ofDrawBitmapString(title, 100, ofGetHeight()-30);
	gui.draw();







	Pt A = intStraightSeg[0].A; Pt B = intStraightSeg[1].A; Pt C = intStraightSeg[2].A; 
	Pt D = intStraightSeg[3].A; Pt E = intStraightSeg[4].A; Pt F = intStraightSeg[5].A;
	Pt a = Pt((A.x + B.x) / 2, (A.y + B.y) / 2); 
	Pt b = Pt((C.x + D.x) / 2, (C.y + D.y) / 2);
	Pt c = Pt((D.x + E.x) / 2, (D.y + E.y) / 2);
	//proj b on ac
	Pt u(a.x - c.x, a.y - c.y); Pt v(b.x - c.x, b.y - c.y);
	float e = (u.x*v.x + u.y*v.y) / a.di(c)*a.di(b);
	Pt r(a.x + e*u.x, a.y + e*u.y);
	Pt w((b.x - r.x)*intSpineCtrl / r.di(b), (b.x - r.x)*intSpineCtrl / r.di(b));

	ofNoFill();  ofSetColor(0, 0, 0, 255); ofSetLineWidth(3); 
	ofDrawBitmapStringHighlight("A", A.x, A.y);
	ofDrawBitmapStringHighlight("B", B.x, B.y);
	ofDrawBitmapStringHighlight("C", C.x, C.y);
	ofDrawBitmapStringHighlight("D", D.x, D.y);
	ofDrawBitmapStringHighlight("E", E.x, E.y);
	ofDrawBitmapStringHighlight("F", F.x, F.y);

	for (float t = 0; t<1; t += 0.01) {
		float x = (a.x*pow((1 - t), 2)) + (w.x * 2 * (1 - t)*t) + c.x*pow(t, 2);
		float y = (a.y*pow((1 - t), 2)) + (w.y * 2 * (1 - t)*t) + c.y*pow(t, 2); 
		ofFill();  ofSetColor(0, 0, 0, 255); ofSetLineWidth(1); ofEllipse(x, y, 10, 10);
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
}

void ofApp::mouseDragged(int x, int y, int button){
	for (int i = 0; i < iniptvec.size(); i++) {
		if (iniptvec[i].locked==1) { iniptvec[i].setup(x, y); }
	}
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