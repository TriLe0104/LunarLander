#include "Sprite.h"

//
// Basic Sprite Object
//
Sprite::Sprite() {
	speed = 0;
	velocity = ofVec3f(0, 0, 0);
	acceleration = ofVec3f(0, 0, 0);
	bSelected = false;
	haveImage = false;
	name = "UnamedSprite";
	width = 20;
	height = 20;
	trans = ofVec3f(0, 0, 0);
	scale = ofVec3f(1, 1, 1);
	rot = 0;
	damping = .99;
}

void Sprite::setPosition(ofVec3f pos) {
	trans = pos;
}

void Sprite::setModel(ofxAssimpModelLoader img) {
	model = img;
	haveImage = true;
}


//  Render the sprite
//
void Sprite::draw() {
	if (haveImage) {
		model.drawFaces();
	}
	else {
		model.drawWireframe();
	}
}

// Remove a sprite from the sprite system. Note that this function is not currently
// used. The typical case is that sprites automatically get removed when the reach
// their lifespan.
//

void Sprite::startAnim() {

	frame = 0;
	bAnimRunning = true;
	lastTimeRec = ofGetSystemTimeMillis();
}

void Sprite::stopAnim() {
	bAnimRunning = false;
}

void Sprite::update() {
	if (!bAnimRunning) return;

	float curTime = ofGetSystemTimeMillis();
	if ((curTime - lastTimeRec) > 50) {
		advanceFrame();
		lastTimeRec = curTime;
	}
}

void Sprite::advanceFrame() {
	if (frame == (nframes - 1)) {
		col = 0;
		row = 0;
		frame = 0;  // cycle back to first frame
	}
	else {
		frame++;
		if (col == ntiles_x - 1) col = 0; else col++;
		row = frame / ntiles_x;
	}
}

void Sprite::integrate() {

	// check for 0 framerate to avoid divide errors
	//
	float framerate = ofGetFrameRate();
	if (framerate < 1.0) return;

	// interval for this step
	//
	float dt = 1.0 / framerate;

	// update position based on velocity
	//
	pos += (velocity * dt);

	// update acceleration with accumulated paritcles forces
	// remember :  (f = ma) OR (a = 1/m * f)
	//
	ofVec3f accel = acceleration;    // start with any acceleration already on the particle
	accel += (forces * (1.0 / mass));
	velocity += accel * dt;

	// add a little damping for good measure
	//
	velocity *= damping;

	// clear forces on particle (they get re-added each step)
	//
	forces.set(0, 0, 0);
}