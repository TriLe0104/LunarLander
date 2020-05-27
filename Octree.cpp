//  Kevin M. Smith - Basic Octree Class - CS134/235 4/18/18
//


#include "Octree.h"
 

// draw Octree (recursively)
//
void Octree::draw(TreeNode & node, int numLevels, int level) {
/*
	if (level >= numLevels) return;

	ofSetColor(ofColor::yellow);
	if (level == 0)
		drawBox(node.box);

	ofSetColor(ofColor::red);
	if (level == 1)
		drawBox(node.box);

	ofSetColor(ofColor::green);
	if(level == 2)
		drawBox(node.box);

	ofSetColor(ofColor::blue);
	if (level == 3)
		drawBox(node.box);
*/
	ofSetColor(ofColor::yellow);
	if (level == 4)
	{
		drawBox(node.box);
	}

	level++;
	for (int i = 0; i < node.children.size(); i++) {
		draw(node.children[i], numLevels, level);
	}
}

// draw only leaf Nodes
//
void Octree::drawLeafNodes(TreeNode & node) {
	//if it doesn't have any children, it's a leaf
	if (node.children.empty())
	{
			ofSetColor(ofColor::yellow);
			drawBox(node.box);
	}
	else
	{
		for (int i = 0; i < node.children.size(); i++)
		{
			drawLeafNodes(node.children[i]);
		}
	}
}


//draw a box from a "Box" class  
//
void Octree::drawBox(const Box &box) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
	float w = size.x();
	float h = size.y();
	float d = size.z();
	ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box Octree::meshBounds(const ofMesh & mesh) {
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
	cout << "vertices: " << n << endl;
//	cout << "min: " << min << "max: " << max << endl;
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

// getMeshPointsInBox:  return an array of indices to points in mesh that are contained 
//                      inside the Box.  Return count of points found;
//
int Octree::getMeshPointsInBox(const ofMesh & mesh, const vector<int>& points, 
	Box & box, vector<int> & pointsRtn)
{
	int count = 0;
	for (int i = 0; i < points.size(); i++) {
		ofVec3f v = mesh.getVertex(points[i]);
		if (box.inside(Vector3(v.x, v.y, v.z))) {
			count++;
			pointsRtn.push_back(points[i]);
		}
	}
	return count;
}



//  Subdivide a Box into eight(8) equal size boxes, return them in boxList;
//
void Octree::subDivideBox8(const Box &box, vector<Box> & boxList) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	float xdist = (max.x() - min.x()) / 2;
	float ydist = (max.y() - min.y()) / 2;
	float zdist = (max.z() - min.z()) / 2;
	Vector3 h = Vector3(0, ydist, 0);

	//  generate ground floor
	//
	Box b[8];
	b[0] = Box(min, center);
	b[1] = Box(b[0].min() + Vector3(xdist, 0, 0), b[0].max() + Vector3(xdist, 0, 0));
	b[2] = Box(b[1].min() + Vector3(0, 0, zdist), b[1].max() + Vector3(0, 0, zdist));
	b[3] = Box(b[2].min() + Vector3(-xdist, 0, 0), b[2].max() + Vector3(-xdist, 0, 0));

	boxList.clear();
	for (int i = 0; i < 4; i++)
		boxList.push_back(b[i]);

	// generate second story
	//
	for (int i = 4; i < 8; i++) {
		b[i] = Box(b[i - 4].min() + h, b[i - 4].max() + h);
		boxList.push_back(b[i]);
	}
}

void Octree::create(const ofMesh & geo, int numLevels) {
	// initialize octree structure
	//
	root.box = meshBounds(geo);
	for (int i = 0; i < geo.getNumVertices(); i++)
	{
		root.points.push_back(i);
	}
	int level = 0;
	subdivide(geo, root, numLevels, level);
}

void Octree::subdivide(const ofMesh & mesh, TreeNode & node, int numLevels, int level) {
	if (level >= numLevels) return;
	//list to store 8 divided boxes
	vector<Box> list;
	vector<int> point;
	TreeNode childNode;
	//dividing the parent's box into 8 smaller boxes
	subDivideBox8(node.box, list);
	for (int i = 0; i < list.size(); i++)
	{
		childNode.box = list[i];
		//Checking if points in the parent's box are in the child's box, add to child's point list if it is.
		getMeshPointsInBox(mesh, node.points, childNode.box, childNode.points);
		//If child box is not empty, add to original parent node list of child boxes
		if (!childNode.points.empty())
		{
			node.children.push_back(childNode);
		}
	}
	//If a child has more than one vertex, call subdivide recursively
	level++;
	for(int i = 0; i < node.children.size(); i++)
	{
		if(node.children[i].points.size() > 1)
		subdivide(mesh, node.children[i], numLevels, level);
	}
}

//Starting with the root node, if the ray intersects the box call the function recursively to find the intersecting leaf node
bool Octree::intersect(const Ray &ray, const TreeNode & node, TreeNode & nodeRtn, int level) {
	if (level == 5)
	{
		nodeRtn = node;
		return true;
	}
	
	if (node.box.intersect(ray, -1000, 1000))
	{
		level++;
		for (int i = 0; i < node.children.size(); i++) {
			intersect(ray, node.children[i], nodeRtn, level);
		}
	}
	else
	{
		return false;
	}
}


/*
	if (node.box.intersect(ray, -1000, 1000))
	{
		nodeRtn = node;
		for (int i = 0; i < node.children.size(); i++)
		{
			intersect(ray, node.children[i], nodeRtn);
		}

		return true;
	}
	return false;
*/

/*
if (node.box.intersect(ray, -1000, 1000))
	{
		if (node.children.empty())
		{
			nodeRtn = node;
			return true;
		}
		else
		{
			for (int i = 0; i < node.children.size(); i++)
			{
				intersect(ray, node.children[i], nodeRtn);
			}
		}
	}
	return false;
*/