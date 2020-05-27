#pragma once

#include "ofMain.h"
#include  "ofxAssimpModelLoader.h"
#include "box.h"
#include "ray.h"
#include "Octree.h"
#include "Sprite.h"
#include "ofxGui.h"
#include "Particle.h"
#include "ParticleEmitter.h"

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
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
		void toggleSelectTerrain();
		void setCameraTarget();
		void loadVbo();

		bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);

		ofLight keyLight, fillLight, rimLight;

		ofEasyCam cam;
		ofCamera cam1, cam2;
		ofCamera *theCam;

		ofxAssimpModelLoader mars, lander;
		ofLight light;
		Box boundingBox;
		Box landerBoundingBox;
		Box landingZone;
		Box meshBounds(const ofMesh &);
		vector<Box> level1, level2, level3;
		
		ofTime timer;

		ofMesh terrain;
		
		Octree octree;

		ParticleEmitter landerEmitter;
		ParticleEmitter exhaustEmitter;
		GravityForce grav;
		ImpulseForce impulse;
		ThrusterForce thruster;
		TurbulenceForce turbF;
		ImpulseRadialForce radialF;
		GravityForce particleGrav;

		ofSoundPlayer thrusterSound;

		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
		bool bPointSelected;
		bool bLanded;
		bool bLanderLoaded;
		bool bTerrainSelected;
	
		ofVec3f selectedPoint;
		ofVec3f intersectPoint;
		ofVec3f landerRayPoint;
		ofVec3f rayLanderDir;

		ofVec3f minLander;
		ofVec3f maxLander;

		int altitude;

		const float selectionRange = 4.0;

		void checkCollisions();
		void getAltitude(ofVec3f rayPoint);

		

		// textures
		//
		ofTexture  particleTex;

		// shaders
		//
		ofVbo vbo;
		ofShader shader;
};
