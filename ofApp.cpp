
//--------------------------------------------------------------
//
//  Kevin M. Smith 
//
//  Mars HiRise Project - startup scene
// 
//  This is an openFrameworks 3D scene that includes an EasyCam
//  and example 3D geometry which I have reconstructed from Mars
//  HiRis photographs taken the Mars Reconnaisance Orbiter
//
//  You will use this source file (and include file) as a starting point
//  to implement assignment 5  (Parts I and II)
//
//  Please do not modify any of the keymappings.  I would like 
//  the input interface to be the same for each student's 
//  work.  Please also add your name/date below.

//  Please document/comment all of your work !
//  Have Fun !!
//
//  Student Name:   < Tri Le >
//  Date: <04/21/19>


#include "ofApp.h"
#include "Util.h"



//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup(){

	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bLanderLoaded = false;
	bTerrainSelected = true;
//	ofSetWindowShape(1024, 768);
	cam.setDistance(25);
	cam.setNearClip(.1);
	cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
	cam1.setGlobalPosition(glm::vec3(200, 100, 0));
	cam1.lookAt(glm::vec3(0, 1, 0));
	cam2.setGlobalPosition(glm::vec3(0, 150, 0));
	cam2.lookAt(glm::vec3(0, 0, 0));
	theCam = &cam2;
	ofSetVerticalSync(true);
	cam.disableMouseInput();
	ofEnableSmoothing();
	ofEnableDepthTest();
	ofEnableLighting();

	// Setup 3 - Light System
// 
	keyLight.setup();
	keyLight.enable();
	keyLight.setAreaLight(1, 1);
	keyLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	keyLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	keyLight.setSpecularColor(ofFloatColor(1, 1, 1));

	keyLight.rotate(45, ofVec3f(0, 1, 0));
	keyLight.rotate(-45, ofVec3f(1, 0, 0));
	keyLight.setPosition(5, 5, 5);

	fillLight.setup();
	fillLight.enable();
	fillLight.setSpotlight();
	fillLight.setScale(.05);
	fillLight.setSpotlightCutOff(15);
	fillLight.setAttenuation(2, .001, .001);
	fillLight.setAmbientColor(ofFloatColor(1, 0.1, 0.1));
	fillLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	fillLight.setSpecularColor(ofFloatColor(1, 1, 1));
	fillLight.rotate(180, ofVec3f(0, 1, 0));

	rimLight.setup();
	rimLight.enable();
	rimLight.setSpotlight();
	rimLight.setScale(.05);
	rimLight.setSpotlightCutOff(30);
	rimLight.setAttenuation(.2, .001, .001);
	rimLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	rimLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	rimLight.setSpecularColor(ofFloatColor(1, 1, 1));
	rimLight.rotate(-90, ofVec3f(0, 1, 0));
	rimLight.setPosition(0, 50, 0);
	initLightingAndMaterials();

	// texture loading
//
	ofDisableArbTex();     // disable rectangular textures

	// load textures
	//
	if (!ofLoadImage(particleTex, "images/HappyFace.png")) {
		cout << "Particle Texture File: images/dot.png not found" << endl;
		ofExit();
	}

	// load the shader
	//
#ifdef TARGET_OPENGLES
	shader.load("shaders_gles/shader");
#else
	shader.load("shaders/shader");
#endif

	mars.loadModel("geo/moon-houdini.obj");
	mars.setScaleNormalization(false);

	if (lander.loadModel("geo/lander.obj")) {
		lander.setScaleNormalization(false);
//		lander.setPosition(60, -180, 0);
		bLanderLoaded = true;
	}

	landerBoundingBox = Box(Vector3(minLander.x, minLander.y, minLander.z), Vector3(maxLander.x, maxLander.y, maxLander.z));
	landingZone = Box(Vector3(0, 0, 0), Vector3(50, 20, 50));
	boundingBox = meshBounds(lander.getMesh(0));
	terrain = mars.getMesh(0);

	octree.create(terrain, 5);
	cout << "Octree created in: " << ofGetElapsedTimef() << endl;

	//Lander particle
	landerEmitter.setRate(1);
	landerEmitter.setOneShot(true);
	landerEmitter.setLifespan(-1);
	landerEmitter.setPosition(ofVec3f(60, -180, 0));
	landerEmitter.start();

	ParticleSystem *sys = landerEmitter.sys;
	grav.set(ofVec3f(0, -.3, 0));
	sys->addForce(&grav);
	sys->addForce(new TurbulenceForce(ofVec3f(-.1, -.1, -.1), ofVec3f(.1, .1, .1)));
	sys->addForce(&impulse);
	sys->addForce(&thruster);
	//ray cast from lander
	rayLanderDir = ofVec3f(0, -1, 0);
	rayLanderDir.normalize();
	bPointSelected = true;

	
	//Lander's exhaust particle
	turbF.set(ofVec3f(0, -25, 0), ofVec3f(0, -50, 0));
	radialF.setHeight(5);
	radialF.set(5);
	particleGrav.set(ofVec3f(0, -20, 0));
	exhaustEmitter.setOneShot(true);
	exhaustEmitter.setParticleRadius(5);
	exhaustEmitter.setEmitterType(RadialEmitter);
	exhaustEmitter.setLifespanRange(ofVec2f(.1, .2));
	exhaustEmitter.setMass(.1);
	exhaustEmitter.setDamping(0);
	exhaustEmitter.setGroupSize(10);
	exhaustEmitter.sys->addForce(&turbF);
	exhaustEmitter.sys->addForce(&particleGrav);
	exhaustEmitter.sys->addForce(&impulse);
	exhaustEmitter.sys->addForce(&radialF);

	thrusterSound.load("sounds/thrusters.wav");
}
void ofApp::checkCollisions() {

	// for each particle, determine if we hit the groud plane.
	//
	for (int i = 0; i < landerEmitter.sys->particles.size(); i++) {
		
		ofVec3f vel = landerEmitter.sys->particles[i].velocity; // velocity of particle
		ofVec3f pos = landerEmitter.sys->particles[i].position;
		Vector3 point = Vector3(pos.x, pos.y, pos.z);
		if (!landingZone.inside(point))
		{
			bLanded = false;
			//bounce the lander upward when it hits the ground and when not within the landing zone area
			if (altitude <= 4)
			{
				impulse.apply(ofVec3f(0, 4, 0));
				impulse.updateForce(&landerEmitter.sys->particles[0]);
			}
		}
		else
		{
			if (altitude <= 4)
			{
				cout << "Landed" << endl;
				ofVec3f xnorm = ofVec3f(1, 0, 0);
				ofVec3f ynorm = ofVec3f(0, 1, 0);
				ofVec3f znorm = ofVec3f(0, 0, 1);
				ofVec3f f1 = ((-vel.dot(xnorm))*xnorm);
				ofVec3f f2 = ((-vel.dot(ynorm))*ynorm);
				ofVec3f f3 = ((-vel.dot(znorm))*znorm);
				ofVec3f ftotal = f1 + f2 + f3;
				landerEmitter.sys->particles[i].forces += (ofGetFrameRate() * ftotal);
				bLanded = true;
			}
			
		}
	}
}
// load vertex buffer in preparation for rendering
//
void ofApp::loadVbo() {
	if (exhaustEmitter.sys->particles.size() < 1) return;

	vector<ofVec3f> sizes;
	vector<ofVec3f> points;
	for (int i = 0; i < exhaustEmitter.sys->particles.size(); i++) {
		points.push_back(exhaustEmitter.sys->particles[i].position);
		sizes.push_back(ofVec3f(10));
	}
	// upload the data to the vbo
	//
	int total = (int)points.size();
	vbo.clear();
	vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
	vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
}
//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {
	checkCollisions();

	landerEmitter.update();
	ofVec3f v = landerEmitter.sys->particles[0].position;
	lander.setPosition(v.x, v.y, v.z);
	
	exhaustEmitter.setPosition(v);
	exhaustEmitter.update();

	fillLight.setPosition(v);

	minLander = lander.getSceneMin() + lander.getPosition();
	maxLander = lander.getSceneMax() + lander.getPosition();
	cam1.setPosition(v.x+50, v.y + 20, v.z);
	cam2.setPosition(v.x, v.y + 50, v.z);
	cam.setPosition(50, v.y, 0);
	//ray cast from lander
	landerRayPoint = v;
	getAltitude(v);
}
//--------------------------------------------------------------
void ofApp::draw(){
	loadVbo();
//	ofBackgroundGradient(ofColor(20), ofColor(0));   // pick your own background
	ofBackground(ofColor::black);
//	cout << ofGetFrameRate() << endl;
	theCam->begin();
	ofPushMatrix();
	if (bWireframe) {                    // wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
		mars.drawWireframe();
		if (bLanderLoaded) {
			lander.drawWireframe();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}
	else {
		ofEnableLighting();              // shaded mode
		mars.drawFaces();

		if (bLanderLoaded) {
			lander.drawFaces();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}


	if (bDisplayPoints) {                // display points as an option    
		glPointSize(3);
		ofSetColor(ofColor::green);
		mars.drawVertices();
	}

	// highlight selected point (draw sphere around selected point)
	//
	
	if (bPointSelected) {
		ofSetColor(ofColor::blue);
		ofFill();
		ofDrawSphere(selectedPoint, 1);
	}

	ofNoFill();
	ofSetColor(ofColor::white);

//	octree.draw(5, 0);
//	octree.drawLeafNodes(octree.root);

//	landerEmitter.draw();

	//bounding box
//	ofDrawBox(lander.getPosition(), maxLander.x - minLander.x, maxLander.y - minLander.y, maxLander.z - minLander.z);

	//landing zone
	if(bLanded)
	{
		ofSetColor(ofColor::green);
		string landed = "Landing Successful!";
		ofDrawBitmapString(landed, ofVec3f(0, 0, 0));
	}
	else
	{
		string altitudeDisplay;
		altitudeDisplay += "Altitude: " + std::to_string(altitude);
		ofDrawBitmapString(altitudeDisplay, landerEmitter.sys->particles[0].position + ofVec3f(10,0,0));
		ofSetColor(ofColor::red);
	}
	ofDrawBox(
		landingZone.max().x() - landingZone.min().x(),
		landingZone.max().y() - landingZone.min().y(),
		landingZone.max().z() - landingZone.min().z());
	//ofDrawBox(50, 10, 50);


	ofPopMatrix();

	glDepthMask(GL_FALSE);
	ofSetColor(255, 100, 90);
	// this makes everything look glowy :)
	//
	ofEnableBlendMode(OF_BLENDMODE_ADD);
	ofEnablePointSprites();

	shader.begin();

	// draw particle emitter here..
	particleTex.bind();
	vbo.draw(GL_POINTS, 0, (int)exhaustEmitter.sys->particles.size());
	particleTex.unbind();

	theCam->end();
	shader.end();

	ofDisablePointSprites();
	ofDisableBlendMode();
	ofEnableAlphaBlending();

	// set back the depth mask
	//
	glDepthMask(GL_TRUE);
}

// 

// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {

	ofPushMatrix();
	ofTranslate(location);

	ofSetLineWidth(1.0);

	// X Axis
	ofSetColor(ofColor(255, 0, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));
	

	// Y Axis
	ofSetColor(ofColor(0, 255, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

	// Z Axis
	ofSetColor(ofColor(0, 0, 255));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

	ofPopMatrix();
}


void ofApp::keyPressed(int key) {

	switch (key) {
	case 'C':
	case 'c':
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else cam.enableMouseInput();
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'H':
	case 'h':
		break;
		break;
	case 'r':
		cam.reset();
		break;
	case 's':
		savePicture();
		break;
	case 't':
		setCameraTarget();
		break;
	case 'u':
		break;
	case 'v':
		togglePointsDisplay();
		break;
	case 'V':
		break;
	case 'w':
		toggleWireframeMode();
		break;
	case OF_KEY_RIGHT:
		if (thruster.getThrust().x < 1.5)
		{
			thruster.add(ofVec3f(.2, 0, 0));
			exhaustEmitter.sys->reset();
			exhaustEmitter.start();
			thrusterSound.play();
		}
		break;
	case OF_KEY_LEFT:
		if (-1.5 < thruster.getThrust().x)
		{
			thruster.add(ofVec3f(-.2, 0, 0));
			exhaustEmitter.sys->reset();
			exhaustEmitter.start();
			thrusterSound.play();
		}
		break;
	case OF_KEY_UP:
		if (-1.5 < thruster.getThrust().z)
		{
			thruster.add(ofVec3f(0, 0, -.2));
			exhaustEmitter.sys->reset();
			exhaustEmitter.start();
			thrusterSound.play();
		}
		break;
	case OF_KEY_DOWN:
		if (thruster.getThrust().z < 1.5)
		{
			thruster.add(ofVec3f(0, 0, .2));
			exhaustEmitter.sys->reset();
			exhaustEmitter.start();
			thrusterSound.play();
		}
		break;
	case ' ':
		if (thruster.getThrust().y < 1.5)
		{
			thruster.add(ofVec3f(0, .2, 0));
			exhaustEmitter.sys->reset();
			exhaustEmitter.start();
			thrusterSound.play();
		}
		break;
	case OF_KEY_ALT:
		cam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = true;
		break;
	case OF_KEY_SHIFT:
		if (-1.5 < thruster.getThrust().y)
		{
			thruster.add(ofVec3f(0, -.2, 0));
			exhaustEmitter.sys->reset();
			exhaustEmitter.start();
			thrusterSound.play();
		}
		break;
	case OF_KEY_DEL:
		break;
	case OF_KEY_F1:
		theCam = &cam2;
		break;
	case OF_KEY_F2:
		theCam = &cam1;
		break;
	case OF_KEY_F3:
		theCam = &cam;
		break;
	default:
		break;
	}
}

void ofApp::toggleWireframeMode() {
	bWireframe = !bWireframe;
}

void ofApp::toggleSelectTerrain() {
	bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
	bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {

	switch (key) {
	
	case OF_KEY_ALT:
		cam.disableMouseInput();
		bAltKeyDown = false;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = false;
		break;
	case OF_KEY_SHIFT:
		break;
	default:
		break;

	}
}

Box ofApp::meshBounds(const ofMesh & mesh) {
	int n = mesh.getNumVertices();
	ofVec3f v = mesh.getVertex(0);
	ofVec3f max = v;
	ofVec3f min = v;
	for (int i = 1; i < n; i++) {
		ofVec3f v = mesh.getVertex(i);

		if (v.x > max.x) max.x = v.x;
		else if (v.x < min.x) min.x = v.x;

		if (v.y > max.y) max.y = v.y;
		else if (v.y < min.y) min.y = v.y;

		if (v.z > max.z) max.z = v.z;
		else if (v.z < min.z) min.z = v.z;
	}
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}


//--------------------------------------------------------------

void ofApp::mousePressed(int x, int y, int button) {
	/*
	ofTime timer = ofGetCurrentTime();
	ofVec3f mouse(mouseX, mouseY);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z), Vector3(rayDir.x, rayDir.y, rayDir.z));
	TreeNode nodeIntersect = octree.root;

		if (octree.intersect(ray, octree.root, nodeIntersect,0))
		{
			selectedPoint = terrain.getVertex(nodeIntersect.points[0]);
			altitude = lander.getPosition().y - selectedPoint.y;
			cout << "Intersect at x:" << selectedPoint.x << " y:"<< selectedPoint.y << " z:"<< selectedPoint.z << " node point: " << nodeIntersect.points[0] << endl;
			cout << "Time in ms: " << ofGetCurrentTime().getAsMilliseconds() - timer.getAsMilliseconds() << endl;
			bPointSelected = true;
		}
		*/
}

void ofApp::getAltitude(ofVec3f rayPoint)
{
	Ray rayLander = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z), Vector3(rayLanderDir.x, rayLanderDir.y, rayLanderDir.z));
	TreeNode nodeIntersect = octree.root;

	if (octree.intersect(rayLander, octree.root, nodeIntersect, 0))
	{
		selectedPoint = terrain.getVertex(nodeIntersect.points[0]);
		altitude = lander.getPosition().y - selectedPoint.y;
		bPointSelected = true;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {


}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}


// Set the camera to use the selected point as it's new target
//  
void ofApp::setCameraTarget() {

}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}



//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {

	static float ambient[] =
	{ .5f, .5f, .5, 1.0f };
	static float diffuse[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float position[] =
	{5.0, 5.0, 5.0, 0.0 };

	static float lmodel_ambient[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float lmodel_twoside[] =
	{ GL_TRUE };


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
//	glEnable(GL_LIGHT1);
	glShadeModel(GL_SMOOTH);
} 

void ofApp::savePicture() {
	ofImage picture;
	picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	picture.save("screenshot.png");
	cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//

void ofApp::dragEvent(ofDragInfo dragInfo) {

	ofVec3f point;
	mouseIntersectPlane(ofVec3f(0, 0, 0), cam.getZAxis(), point);

	if (lander.loadModel(dragInfo.files[0])) {
		lander.setScaleNormalization(false);
		lander.setScale(.005, .005, .005);
		lander.setPosition(point.x, point.y, point.z);
		bLanderLoaded = true;
	}
	else cout << "Error: Can't load model" << dragInfo.files[0] << endl;
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
	glm::vec3 mouse(mouseX, mouseY, 0);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}
