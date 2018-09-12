#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "Pt.h"
#include <vector>


struct Quad {
public:
	Pt A, B, C, D;
	Pt pts[4];
	Quad (){}
	Quad(Pt a, Pt b, Pt c, Pt d) {
		A = a; B = b; C = c; D = d;
		pts[0] = A; pts[1] = B;
		pts[2] = C; pts[3] = D;
	}
	void setupPreservePts(Pt a, Pt b, Pt c, Pt d) {
		A = a; B = b; C = c; D = d;
		pts[0] = A; pts[1] = B;
		pts[2] = C; pts[3] = D;
	}
	void display() {
		ofPath path;
		path.setFillColor(ofColor(0, 150, 155, 50));
		path.setStrokeWidth(2);
		path.setStrokeColor(ofColor(0, 0, 0, 255));
		path.moveTo(A.x, A.y);
		path.lineTo(B.x, B.y);
		path.lineTo(C.x, C.y);
		path.lineTo(D.x, D.y);
		path.close();
		path.draw();
	}
};

struct Seg{
public:
	Pt A,B;
	Seg();
	Seg(Pt a, Pt b) { A = a; B = b; }
	void setup(Pt a, Pt b) { A = a; B = b; }
};

struct sortSegDesc {
	bool operator() (Seg a, Seg b) {
		return a.A.di(a.B) > b.A.di(b.B);
	}
};

struct Tri {
	Pt A, B, C;
	Tri(Pt a, Pt b, Pt c) { A = a; B = b; C = c; }
};

struct sortTriDesc {
	bool operator() (Tri a, Tri b) {
		float u = a.A.di(a.B) + a.A.di(a.C) + a.B.di(a.C);
		float v = b.A.di(b.B) + b.A.di(b.C) + b.B.di(b.C);
		return  u > v;
	}
};

class ofApp : public ofBaseApp{

	public:
		//ofApp() {}
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
	
		/* NS METHODS */
		vector<Seg> getsegvec(vector<Pt>);
		vector<vector<Pt>> getcrvpts(vector<Pt>);
		vector<Pt> gensmoothboundary(vector<Pt>);
		vector<Pt> geninternalboundary(vector<Pt>, float);
		vector<Pt> Lerp(Pt, Pt, Pt);
		Pt intxPt(Pt, Pt, Pt, Pt);
		Pt intxPt2(Pt, Pt, Pt, Pt);
		Pt intxPt4(Pt, Pt, Pt, Pt);
		int doesintx(Pt, Pt, Pt, Pt);
		Pt initPeripheralSys(Pt, Pt, Pt, float);
		float heron(Pt, Pt, Pt);
		int heronContains(Pt, Pt, Pt, Pt);
		Pt proj(Pt, Pt, Pt);
		vector<Quad> initSubdiv(Pt, Pt, Pt, Pt, int, vector<Quad>);
		void subdiv(Quad, int, int);
		void intRushConfig();
		vector<Pt> gensmoothspinecurve();

		/* NS VARIABLES */
		vector<Pt> oriptvec;
		vector<Pt> iniptvec;
		vector<Pt> revptvec;
		vector<Pt> intptvec;
		vector<Pt> revintptvec;
		vector<Pt> intgridptvec;
		vector<Pt> revintgridptvec;
		ofPath path2, path3, path4;

		//global variables
		int global_image_counter = 0;
		vector<Tri> trivec;
		vector<Seg>straightSeg;
		vector<vector<Pt>>crvpts; //used
		vector<Seg>intStraightSeg; //used
		vector<Pt>diagpts; //used

		int controlspine = 0;
		vector<Pt> spineptvec;
		

		float L, W, Corridor, Curvature;
		//vector<vector<Quad>>subdivVec; 
		vector<Quad>subdivQuadVec;
		Pt A0, A1, A2, A3, A4, A5, A6, A7, A8;
		Pt B0, B1, B2, B3, B4, B5, B6, B7, B8;
		Pt A, B, pB, C, pC, D, pD, E, pE, F, pF;
		vector<Quad> quads0;
		vector<Quad> quads1;
		vector<Quad> quads2;
		vector<Quad> quads3;
		vector<Quad> quads4;
		vector<Quad> quads5;
		vector<Quad> quads6;

		/*	gui parameters and objects	*/
		//int rush = 0; 
		Pt globaldiaA; Pt globaldiaB;

		ofParameterGroup parameters;
		ofParameter<float>CurvaTure;
		ofParameter<int>CurveSegP0;
		ofParameter<bool>showCurveSeg;
		ofParameter<float>PeripheralCellDepth;
		ofParameter<int>PeripheralCellLength;
		ofParameter<int>Corridor0;

		
		
		ofParameter<float>DoorDepth;
		ofParameter<ofColor> color0;
		ofParameter<ofColor> color1;
		
		ofParameter<bool>controlpts;
		ofParameter<bool>rush;
		ofParameter<int>intgrid0;
		ofParameter<int>intgrid1;
		ofParameter<int>intgrid2;
		
		ofParameter<bool>fixint0;
		ofParameter<bool>fixint1;
		ofParameter<bool>fixint2;
		ofParameter<bool>fixint3;
		ofParameter<bool>fixint4;
		ofParameter<bool>fixint5;
		ofParameter<bool>fixint6;

		ofParameter<bool>spinecontrolpts;
		ofParameter<int>Corridor1;
		ofParameter<float>intSpineCtrl;
		ofParameter<float>SpineDisplacement;
		ofParameter<float>spinecurvature;		
		ofParameter<int>spinedivpts;
		ofParameter<bool>showintspinequads;
		ofParameter<bool>showintregion;
		ofxPanel gui;

		// camera
		ofEasyCam cam;
};
