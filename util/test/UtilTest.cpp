/*
 * Test cases for utility classes.
 */

#include <iostream>
using std::cout;
using std::endl;

#include <util/interface/Util.hpp>
using util::Indents;

#include <util/interface/BoundingBox.hpp>
#include <util/interface/PointCloud.hpp>

int main(int argc, char** argv) {
	cout << "Running utility unit tests" << endl;
	
	/********************** BOUNDINGBOX TESTS *************************/
	cout << "Running BoundingBox class unit tests" << endl;
	util::BoundingBox box_1(glm::vec3(-1.0f), glm::vec3(1.0f));	
	cout << Indents(2) << box_1 << endl;
	
	cout << Indents(2) << "extents: " << box_1.extents() << ", expected (2, 2, 2)" << endl;
	cout << Indents(2) << "center: " << box_1.center() << ", expected (0, 0, 0)" << endl;
	
	box_1.extend(glm::vec3(2, 0, 0));
	cout << Indents(2) << "extending by point (2, 0, 0), now: " << box_1 << ", expected (-1,-1,-1)->(2,1,1)" << endl;
	
	box_1.extend(util::BoundingBox(glm::vec3(2.0f, -1.0f, -1.0f), glm::vec3(4.0f, 1.0f, 1.0f)));
	cout << Indents(2) << "extending by box (2,-1,-1)->(4,1,1), now: " << box_1 << ", expected ???" << endl;
	
	util::BoundingBox left, right;
	box_1.min() = glm::vec3(-1.0f);
	box_1.max() = glm::vec3(1.0f);
	box_1.split(X, 0.5, left, right);
	cout << Indents(2) << "split box at x=0.5, got left=" << left << ", right=" << right << endl;
	
	/********************** POINTCLOUD TESTS *************************/
	cout << "Running PointCloud class unit tests" << endl;
	util::PointCloud cloud;
	if (!cloud.readOFF("cow.off")) { 
		cout << " unable to read PointCloud from file" << endl;
	}
	cout << " read PointCloud: " << cloud << endl;
	cout << " writing PointCloud to file UtilityTest_PointCloud.off" << endl;
	if (!cloud.writeOFF("UtilityTest_PointCloud.off")) { 
		cout << " unable to write PointCloud to file" << endl;
	}
	
	glm::vec3 p(0.0f);
	const unsigned k = 16;
	cout << " running knn query with k=" << k << endl;
	util::PointCloud::IndexVector nearest = cloud.collectKNearest(p, k);
	cout << " got " << nearest.size() << " points" << endl;
	
	const float radius = 2.0f;
	cout << " running in-radius query with r=" << radius << endl;
	util::PointCloud::IndexVector inRadius = cloud.collectInRadius(p, radius);
	cout << " got " << inRadius.size() << " points" << endl;
	
	cout << " kd-tree holding the point cloud" << endl;
	cloud.print(std::cout);
	
	return 0;
}
