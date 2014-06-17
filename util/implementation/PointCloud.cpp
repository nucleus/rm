/*
 * Source file for PointCloud class.
 */

#include <fstream>
#include <algorithm>
#include <GL/glew.h>

#include <util/interface/PointCloud.hpp>
#include <glm/interface/gtx/norm.hpp>

#ifdef REPORT_LEVEL
#undef REPORT_LEVEL
#endif

#define REPORT_LEVEL 1
#include <util/interface/Debug.hpp>

/////////////////////////////////////////////////////////////////////////////////////

// taken from http://notmagi.me/closest-point-on-line-aabb-and-obb-to-point/
static float pointAABBDistance(const Point3f& point, const util::BoundingBox& box) {
	if (box.contains(point)) {
		return 0.0f;
	}

	glm::vec3 q(0.0f), v(0.0f);

	v.x = point.x;
	if (v.x < box.min().x) { v.x = box.min().x; }
	if (v.x > box.max().x) { v.x = box.max().x; }
	q.x = v.x;

	v.y = point.y;
	if (v.y < box.min().y) { v.y = box.min().y; }
	if (v.y > box.max().y) { v.y = box.max().y; }
	q.y = v.y;

	v.z = point.z;
	if (v.z < box.min().z) { v.z = box.min().z; }
	if (v.z > box.max().z) { v.z = box.max().z; }
	q.z = v.z;

	return glm::distance2(point, q);
}

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// PointResultSet class ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////


util::PointCloud::PointResultSet::PointResultSet(unsigned k, float radius): m_k(k), m_radius(radius) {

}

util::PointCloud::IndexVector util::PointCloud::PointResultSet::getIndices() const {
	IndexVector result;
	result.reserve(m_records.size());
	for (auto it = m_records.begin(); it != m_records.end(); ++it) {
		result.push_back(it->second);
	}
	return result;
}

void util::PointCloud::PointResultSet::insert(unsigned index, float distance) {
	m_records.insert(std::make_pair(distance, index));

	if (m_k) {
		if (m_records.size() > m_k) {
			auto end = m_records.end();
			m_records.erase(--end);
		}
	}
}

float util::PointCloud::PointResultSet::maximumDistance() const {
	if (m_k) {
		if (m_records.empty()) {
			return std::numeric_limits<float>::max();
		}
		auto last = m_records.rbegin();
		return last->first;
	} else {
		return m_radius;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// PointCloud class ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

struct PointDistanceComparator {
	bool operator()(const util::PointCloud::IndexDistPair& lhs, const util::PointCloud::IndexDistPair& rhs) {
		return lhs.second < rhs.second;
	}
};

void util::PointCloud::computeAllSquaredDistances(const Point3f& point, PointDistances& distances) const {
	distances.resize(m_points.size());
	for (unsigned i = 0; i < distances.size(); i++) {
		distances[i] = std::make_pair(i, glm::distance2(m_points[i], point));
	}
}

util::PointCloud::IndexVector util::PointCloud::collectKNearest(const Point3f& point, unsigned k, PointGatherMode mode) {
	switch (mode) {
	case Mode_naive: return _collectKNearestNaive(point, k);
	case Mode_kdtree: return _collectKNearestKDTree(point, k);
	default: break;
	}
	return IndexVector();
}

util::PointCloud::IndexVector util::PointCloud::collectInRadius(const Point3f& point, float radius, PointGatherMode mode) {
	switch (mode) {
	case Mode_naive: return _collectInRadiusNaive(point, radius);
	case Mode_kdtree: return _collectInRadiusKDTree(point, radius);
	case Mode_naive_2d: return _collectInRadiusNaive2D(point, radius);
	default: break;
	}
	return IndexVector();
}

util::PointCloud::IndexVector util::PointCloud::_collectKNearestNaive(const Point3f& point, unsigned k) {
	PointDistances distances;
	computeAllSquaredDistances(point, distances);
	std::sort(distances.begin(), distances.end(), PointDistanceComparator());

	IndexVector result;
	for (unsigned i = 0; i < distances.size() && i < k; i++) {
		result.push_back( distances[i].first );
	}

	return result;
}

util::PointCloud::IndexVector util::PointCloud::_collectInRadiusNaive(const Point3f& point, float radius) {
	PointDistances distances;
	computeAllSquaredDistances(point, distances);

	float squaredRadius = radius * radius;

	IndexVector result;
	for (unsigned i = 0; i < distances.size(); i++) {
		if (distances[i].second < squaredRadius) {
			result.push_back( distances[i].first );
		}
	}

	return result;
}

util::PointCloud::IndexVector util::PointCloud::_collectKNearestKDTree(const Point3f& point, unsigned k) {
	PointResultSet results(k, 0.0f);
	_gatherPoints(m_root, m_bounds, point, results);
	return results.getIndices();
}

util::PointCloud::IndexVector util::PointCloud::_collectInRadiusKDTree(const Point3f& point, float radius) {
	PointResultSet results(0, radius * radius);
	_gatherPoints(m_root, m_bounds, point, results);
	return results.getIndices();
}

util::PointCloud::IndexVector util::PointCloud::_collectInRadiusNaive2D(const Point3f& point, float radius) {
	PointDistances distances;
	distances.resize(m_points.size());
	for (unsigned i = 0; i < distances.size(); i++) {
		distances[i] = std::make_pair(i, glm::distance2(glm::vec2(m_points[i].x, m_points[i].y), glm::vec2(point.x, point.y)));
	}

	float squaredRadius = radius * radius;

	IndexVector result;
	for (unsigned i = 0; i < distances.size(); i++) {
		if (distances[i].second < squaredRadius) {
			result.push_back( distances[i].first );
		}
	}

	return result;
}

void util::PointCloud::_gatherPoints(Node* node, const util::BoundingBox& box, const Point3f& point, PointResultSet& results) {
	if (node->leaf()) {
		Node::ObjectVector& objs = node->objects();
		for (unsigned i = 0; i < objs.size(); i++) {
			float distance = glm::distance2(point, *(objs[i]));
			if (distance < results.maximumDistance()) {
				results.insert(objs[i] - &m_points[0], distance);
			}
		}
		return;
	}

	util::BoundingBox left, right;
	box.split(node->m_axis, node->m_split, left, right);

	if (pointAABBDistance(point, left) < results.maximumDistance()) {
		_gatherPoints(node->leftChild(), left, point, results);
	}
	if (pointAABBDistance(point, right) < results.maximumDistance()) {
		_gatherPoints(node->rightChild(), right, point, results);
	}
}

/////////////////////////////////////////////////////////////////////////////////////

PointVector util::PointCloud::getPoints(const IndexVector& indices) const {
	PointVector result;
	result.resize(indices.size());
	for (unsigned i = 0; i < indices.size(); i++) {
		result[i] = m_points[indices[i]];
	}
	return result;
}

/////////////////////////////////////////////////////////////////////////////////////

bool util::PointCloud::readOFF(const std::string& filename, bool normalizeModel) {
	std::ifstream in(filename.c_str());
	std::string off("OFF");
	
	char buf[OFF_PARSER_BUF_LENGTH];
	unsigned numVertices = 0, numTriangles = 0, numEdges = 0;
	
	bool isOFF = false;
	
	if (in.good()) {
		
		report("PointCloud::readOFF: Loading file " << filename);
		
		if (!empty()) {
			reset();
		}
		
		in.getline(buf, OFF_PARSER_BUF_LENGTH);
		isOFF = !off.compare(0, std::string::npos, buf, off.length());
		if (!isOFF) {
			reportE("Input file not in OFF format");
			return false;
		}

		in >> numVertices >> numTriangles >> numEdges;
		m_points.resize(numVertices);
		m_colors.resize(numVertices);
		
		PointPtrVector ptrs;
		ptrs.resize(numVertices);
		
		float x, y, z, nx, ny, nz;
		Point3f minPoint(std::numeric_limits<float>::max());
		Point3f maxPoint(std::numeric_limits<float>::min());
		Point3f average(0.0f);

		// sweep to get all vertex coords
		for(int i = 0; i < numVertices; i++) {
			in >> x >> y >> z;
			Point3f point(x,y,z);
			minPoint = glm::min(minPoint, point);
			maxPoint = glm::max(maxPoint, point);
			average += point;
			m_points[i] = point;
			m_colors[i] = glm::vec3(1.0f, 1.0f, 1.0f);
			ptrs[i] = &m_points[i];
		}
		
		// if model is displaced from origin, move to origin if desired
		if (normalizeModel) {
			average /= numVertices;
			for (unsigned i = 0; i < numVertices; i++) {
				m_points[i] -= average;
			}
			minPoint -= average;
			maxPoint -= average;
		}
		
		// construct bounding box
		m_bounds = util::BoundingBox(minPoint, maxPoint);

		report("PointCloud::readOFF: Loading completed (" << numVertices << " points, " << m_bounds << ")"
				<< ", constructing kd-tree (max depth = " << m_maxDepth << ", min points = " << m_minPoints << ")");
		
		// construct kd-tree
		float time = 0.0f;
		GET_TIME( build(ptrs, m_bounds) )

		report("PointCloud::readOFF: Construction completed, took " << time / 1000.0f << " ms");
	} else {
		reportE("Could not open input file");
		return false;
	}

	in.close();
	
	return true;
}
		
bool util::PointCloud::writeOFF(const std::string& filename) const {
	std::ofstream out(filename.c_str());
	if (out.good()) {
		out << "OFF" << std::endl;
		out << m_points.size() << " 0 0" << std::endl;
		for (auto it = m_points.begin(); it != m_points.end(); ++it) {
			const Point3f& p = *it;
			out << " " << p.x << " " << p.y << " " << p.z << std::endl;
		}
	} else {
		reportE("Could not open output file");
		return false;
	}
	
	out.close();
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////

void util::PointCloud::draw() const {
	glBegin(GL_POINTS);
	for (unsigned i = 0; i < m_points.size(); i++) {
		glColor3f(m_colors[i].r, m_colors[i].g, m_colors[i].b);
		glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
	}
	glEnd();
	
	if (m_drawBBox) {
		m_bounds.draw();
	}

	if (m_drawKDTree) {
		drawTree(m_root, m_bounds);
	}
}

void util::PointCloud::drawTree(Node* node, const BoundingBox& box) const {
	box.draw();
	if (node->leaf()) {
		return;
	}
	BoundingBox left, right;
	box.split(node->m_axis, node->m_split, left, right);
	drawTree(node->leftChild(), left);
	drawTree(node->rightChild(), right);
}

void util::PointCloud::setDrawColor(const IndexVector& indices, const Color3f& color) {
	for (unsigned i = 0; i < indices.size(); i++) {
		m_colors[indices[i]] = color;
	}
}

void util::PointCloud::resetColor() {
	std::fill(m_colors.begin(), m_colors.end(), Color3f(1.0f, 1.0f, 1.0f));
}

/////////////////////////////////////////////////////////////////////////////////////

void util::PointCloud::subdivide(Node* node, const util::BoundingBox& box, unsigned depth) {
	
	if (depth > m_maxDepth || node->objects().size() < m_minPoints) {
		node->setLeaf();
		return;
	}
	
	Axis axis = box.dominantAxis();
	float split = 0.0f;
	switch (m_splitMode) {
		case Mode_median:
			split = median(node->objects(), axis)[axis];
			break;
		case Mode_mean:
			for (unsigned i = 0; i < node->objects().size(); i++) {
				split += (*(node->objects()[i]))[axis];
			}
			split /= node->objects().size();
			break;
		default: break;
	}
	
	node->m_axis = axis;
	node->m_split = split;
	
	BoundingBox left, right;
	box.split(axis, split, left, right);
	
	node->allocateChildren();
	
	for (unsigned i = 0; i < node->objects().size(); i++) {
		Point3f* p = node->objects()[i];
		if ((*p)[axis] < split) {
			node->leftChild()->objects().push_back(p);
		} else {
			node->rightChild()->objects().push_back(p);
		}
	}
	
	node->objects().clear();
	
	subdivide(node->leftChild(), left, depth+1);
	subdivide(node->rightChild(), right, depth+1);
}

void util::PointCloud::reset() {
	m_points.clear();
	m_colors.clear();
	m_bounds = util::BoundingBox(Point3f(0.0f), Point3f(0.0f));
	clear();
}

void util::PointCloud::printNode(std::ostream& out, Node* node, unsigned indents) const {
	std::string axis = util::toString(node->m_axis);
	const bool leaf = node->leaf();
	
	out << util::Indents(indents) << "KDTreeNode: " << axis << "-split @ " << node->m_split;
	out << ", leaf = " << (leaf ? "yes" : "no") << ", objects = " << node->objects().size() << std::endl;

	if (node->leaf()) { return; }
	
	printNode(out, node->leftChild(), indents + 1);
	printNode(out, node->rightChild(), indents + 1);
}

/////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& out, const util::PointCloud& cloud) {
	out << "PointCloud(" << cloud.size() << " points, " << cloud.boundingBox() << ")";
	return out;
}
