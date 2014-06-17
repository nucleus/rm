/*
 * Implementation file for utility classes and functions.
 */

#include <util/interface/Util.hpp>

Point3f util::median(PointPtrVector& _data, Axis axis) {
	typedef PointPtrVector::iterator Iterator;
	Iterator first = _data.begin();
	Iterator last = _data.end();
	Iterator middle = first + (last - first) / 2;
	std::nth_element(first, middle, last, PointDimComparator(axis));
	return *(*middle);
}

std::string util::toString(Axis axis) {
	switch (axis) {
		case X: return "X";
		case Y: return "Y";
		case Z: return "Z";
		case W: return "W";
		default: break;
	}
	return "";
}

double util::elapsedTime(timeval& start, timeval& end) {
	return ((end.tv_sec - start.tv_sec) + ((end.tv_usec - start.tv_usec) / 1000000.0f));
}

float util::wendland(float d, float h) {
    if (d < 0 || d > h)
        return 0.0f;
    return glm::pow((1 - d/h), 4.0f)*(4*d/h + 1);
}

std::ostream& operator<<(std::ostream& out, const glm::vec3& vec) {
	out << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
	return out;
}

std::ostream& operator<<(std::ostream& out, const util::Indents& indents) {
	for (unsigned i = 0; i < indents.indents; i++) { out << " "; }
	return out;
}
