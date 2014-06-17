/*
 * Test cases for kdtree construction and collect query.
 */

#include <iostream>
using std::cout;
using std::endl;

#include <util/interface/Util.hpp>
using util::Indents;

#include <util/interface/BoundingBox.hpp>
#include <util/interface/PointCloud.hpp>

int main(int argc, char** argv) {
	cout << "Running kdtree unit tests" << endl;

	util::PointCloud cloud;
	cloud.readOFF("cow.off");

	const unsigned iterations = 10000;

	unsigned points = 0;

	for (unsigned i = 0; i < iterations; i++) {
		points += cloud.collectInRadius(Point3f(0.0f), 2.0f, util::PointCloud::Mode_kdtree).size();
	}

	cout << points << endl;

	return 0;
}
