#pragma once
//----------------------------------------------------------------------------------
//
// Basic Sprite Class
//
//
//  Kevin M. Smith - CS 134 SJSU

#include "ofMain.h"
#include  "ofxAssimpModelLoader.h"

//  General Sprite class  (similar to a Particle)
//

class Sprite {
public:
	Sprite();
	void draw();
	void update();
	void setModel(ofxAssimpModelLoader);
	ofVec3f trans, scale;
	float	rot;
	bool	bSelected;
	void setPosition(ofVec3f);
	float speed;    //   in pixels/sec
	ofVec3f velocity; // in pixels/sec
	ofVec3f acceleration;
	ofVec3f forces;
	float   mass;
	ofxAssimpModelLoader model;
	string name;
	bool haveImage;
	void startAnim();
	void stopAnim();
	void advanceFrame();
	void    integrate();
	int ntiles_x, ntiles_y;
	int nframes;
	float damping;
	int frame = 0;
	int row = 0;
	int col = 0;
	glm::vec3 pos;
	float width, height, voff;
	bool bAnimRunning = false;
	float lastTimeRec;
};