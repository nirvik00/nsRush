#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "Pt.h"
#include <vector>


struct Seg{
public:
	Pt A,B;
	Seg(Pt a, Pt b) { A = a; B = b; }
};

struct Tri {
	Pt A, B, C;
	Tri(Pt a, Pt b, Pt c) { A = a; B = b; C = c; }
};

class ofApp : public ofBaseApp{

	public:
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
		int doesintx(Pt, Pt, Pt, Pt);
		Pt initPeripheralSys(Pt, Pt, Pt, float);
		float heron(Pt, Pt, Pt);
		int heronContains(Pt, Pt, Pt, Pt);

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
		vector<vector<Pt>>crvpts;
		vector<Seg>intStraightSeg;
		float L, W, Corridor, Curvature;


		/*	gui parameters and objects	*/
		ofParameterGroup parameters;
		ofParameter<float>CurvaTure;
		ofParameter<int>CurveSegP0;
		ofParameter<bool>showCurveSeg;
		ofParameter<float>PeripheralCellDepth;
		ofParameter<int>PeripheralCellLength;
		ofParameter<int>Corridor0;

		ofParameter<float>intSpineCtrl;
		ofParameter<int>Corridor1;
		ofParameter<float>DoorDepth;
		ofParameter<ofColor> color0;
		ofParameter<ofColor> color1;
		ofParameter<bool>controlpts;
		ofxPanel gui;

		// camera
		ofEasyCam cam;
};
