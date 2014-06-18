/*
 * Source file for ImplicitSurface class.
 */

#include <fstream>
#include <algorithm>
#include <GL/glew.h>

#include <util/interface/ImplicitSurface.hpp>
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


util::ImplicitSurface::PointResultSet::PointResultSet(unsigned k, float radius): m_k(k), m_radius(radius) {

}

util::ImplicitSurface::IndexVector util::ImplicitSurface::PointResultSet::getIndices() const {
	IndexVector result;
	result.reserve(m_records.size());
	for (auto it = m_records.begin(); it != m_records.end(); ++it) {
		result.push_back(it->second);
	}
	return result;
}

void util::ImplicitSurface::PointResultSet::insert(unsigned index, float distance) {
	m_records.insert(std::make_pair(distance, index));

	if (m_k) {
		if (m_records.size() > m_k) {
			auto end = m_records.end();
			m_records.erase(--end);
		}
	}
}

float util::ImplicitSurface::PointResultSet::maximumDistance() const {
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
/////////////////////////// ImplicitSurface class ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

Point3f util::ImplicitSurface::getNearest(const Point3f& point) {
	IndexVector resultIdx = _collectKNearestKDTree(point, 1);
	PointVector result;
	getPoints(resultIdx, result);
	return result[0];
}

util::ImplicitSurface::IndexVector util::ImplicitSurface::collectInRadius(const Point3f& point, float radius) {
	return _collectInRadiusKDTree(point, radius);
}

util::ImplicitSurface::IndexVector util::ImplicitSurface::_collectKNearestKDTree(const Point3f& point, unsigned k) {
	PointResultSet results(k, 0.0f);
	_gatherPoints(m_root, m_bounds, point, results);
	return results.getIndices();
}

util::ImplicitSurface::IndexVector util::ImplicitSurface::_collectInRadiusKDTree(const Point3f& point, float radius) {
	PointResultSet results(0, radius * radius);
	_gatherPoints(m_root, m_bounds, point, results);
	return results.getIndices();
}

void util::ImplicitSurface::_gatherPoints(Node* node, const util::BoundingBox& box, const Point3f& point, PointResultSet& results) {
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

void util::ImplicitSurface::intersect(const util::RayVector& rays, PointNormalData& intersections, bool enableGPU) const {
	if (enableGPU) {
		_intersectGPU(rays, intersections);
	} else {
		_intersectCPU(rays, intersections);
	}
}

void util::ImplicitSurface::_intersectGPU(const util::RayVector& rays, PointNormalData& intersections) const {
	intersections.clear();
}

void util::ImplicitSurface::_intersectCPU(const util::RayVector& rays, PointNormalData& intersections) const {
	intersections.clear();
	for (unsigned i = 0; i < rays.size(); i++) {
		Point3f hit, normal;
		if (_intersect(rays[i], hit, normal)) {
			intersections.push_back(std::make_pair(hit, normal));
		}
	}
}

bool util::ImplicitSurface::_intersect(const util::Ray& ray, Point3f& hit, Point3f& normal) const {
	const unsigned steps = m_rmSteps;
	float tnear = ray.tNear, tfar = ray.tFar;
	
	// clip ray against bounding box
	bool intersects = _clipRayAgainstBounds(ray, tnear, tfar);
	if (!intersects) {
		return false;
	}
	
	tnear += 0.00001f;
	
	// march ray through surface
	float step = (tfar - tnear) / steps;
	
	// set sign to unknown at the beginning
	bool sign, signWasSet = false;
	
	// trace forward through the bounding box
	for (unsigned i = 1; i < steps-1; i++) { // -1 to not get too close to tfar
		Point3f target = ray(tnear + i * step);
		float implicitValue; Point3f implicitNormal;
		_evaluate(target, &implicitValue, &implicitNormal);
		
		if (!signWasSet) {
			if (implicitValue < 0.0f) {
				sign = false;
				signWasSet = true;
			} else if (implicitValue > 0.0f) {
				sign = true;
				signWasSet = true;
			}
		} else {
			bool enteredSurface = (sign == true && implicitValue < 0.0f) || (sign == false && implicitValue > 0.0f);
			if (enteredSurface) { // ray entered surface
			
				// compute bounds between this and the last step for backward trace
				float tStart = tnear + (i-1) * step;
				float tEnd = tnear + i * step;
				float smallStep = (tEnd - tStart) / steps;
				
				// trace backwards from step that changed the sign
				for (int j = steps-1; j >= 0; j--) {
					target = ray(tStart + j * smallStep);
					_evaluate(target, &implicitValue, &normal);
					
					bool exitedSurface = (sign == true && implicitValue > 0.0f) || (sign == false && implicitValue < 0.0f);
					if (exitedSurface) {
						hit = target;
						normal = implicitNormal;
						return true;
					}
				}
			}
		}
	}
	
	return false;
}


// courtesy of Jeroen Baert (http://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms)
bool util::ImplicitSurface::_clipRayAgainstBounds(const util::Ray& ray, float& tnear, float& tfar) const {
    glm::vec3 T_1, T_2;
    double t_near = std::numeric_limits<double>::min();
    double t_far = std::numeric_limits<double>::max();

	Point3f min = m_bounds.min(), max = m_bounds.max();
	
    for (int i = 0; i < 3; i++){
        if (ray.d[i] == 0){
            if ((ray.o[i] < min[i]) || (ray.o[i] > max[i])) {
                return false;
            }
        } else {
            T_1[i] = (min[i] - ray.o[i]) / ray.d[i];
            T_2[i] = (max[i] - ray.o[i]) / ray.d[i];

            if(T_1[i] > T_2[i]){
				glm::vec3 tmp = T_1;
				T_1 = T_2;
				T_2 = tmp;
            }
            if (T_1[i] > t_near){
                t_near = T_1[i];
            }
            if (T_2[i] < t_far){
                t_far = T_2[i];
            }
            if( (t_near > t_far) || (t_far < 0) || (std::abs(t_far - t_near) < 0.0001f) ){
                return false;
            }
        }
    }
    tnear = t_near; tfar = t_far;
    return true;
}

void util::ImplicitSurface::_voxel(const Point3f& p, Point3i& base, Point3f& interp) const {
	glm::vec3 extents = m_bounds.extents();
	Point3i dims = m_grid.dimensions();
	
	Point3f coords;
	FOR_EACH_AXIS {
		coords[axis] = (p[axis] - m_bounds.min()[axis]) / extents[axis] * (dims[axis] - 1);
	}
	
	base = Point3i(floor(coords.x), floor(coords.y), floor(coords.z));
	interp = Point3f(coords.x - base.x, coords.y - base.y, coords.z - base.z);
}

// trilinearly interpolates the value of the implicit function f from WLS values at grid locations
void util::ImplicitSurface::_evaluate(const Point3f& p, float* value, Point3f* normal) const {
	
	// determine target voxel
	Point3i base;
	Point3f interp;
	_voxel(p, base, interp);
	
	if (value) {
		float f000 = m_grid.value(base.x, base.y, base.z);
		float f100 = m_grid.value(base.x + 1, base.y, base.z);
		float f010 = m_grid.value(base.x, base.y + 1, base.z);
		float f110 = m_grid.value(base.x + 1, base.y + 1, base.z);
		float f001 = m_grid.value(base.x, base.y, base.z + 1);
		float f101 = m_grid.value(base.x + 1, base.y, base.z + 1);
		float f011 = m_grid.value(base.x, base.y + 1, base.z + 1);
		float f111 = m_grid.value(base.x + 1, base.y + 1, base.z + 1);
		
		float x00 = util::lerp<float>(f000, f100, interp.x);
		float x10 = util::lerp<float>(f010, f110, interp.x);
		float x01 = util::lerp<float>(f001, f101, interp.x);
		float x11 = util::lerp<float>(f011, f111, interp.x);
		
		float y0 = util::lerp<float>(x00, x10, interp.y);
		float y1 = util::lerp<float>(x01, x11, interp.y);
		
		*value = util::lerp<float>(y0, y1, interp.z);	
	}
	
	if (normal) {
		Point3f f000 = m_grid.normal(base.x, base.y, base.z);
		Point3f f100 = m_grid.normal(base.x + 1, base.y, base.z);
		Point3f f010 = m_grid.normal(base.x, base.y + 1, base.z);
		Point3f f110 = m_grid.normal(base.x + 1, base.y + 1, base.z);
		Point3f f001 = m_grid.normal(base.x, base.y, base.z + 1);
		Point3f f101 = m_grid.normal(base.x + 1, base.y, base.z + 1);
		Point3f f011 = m_grid.normal(base.x, base.y + 1, base.z + 1);
		Point3f f111 = m_grid.normal(base.x + 1, base.y + 1, base.z + 1);
		
		Point3f x00 = util::lerp<Point3f>(f000, f100, interp.x);
		Point3f x10 = util::lerp<Point3f>(f010, f110, interp.x);
		Point3f x01 = util::lerp<Point3f>(f001, f101, interp.x);
		Point3f x11 = util::lerp<Point3f>(f011, f111, interp.x);
		
		Point3f y0 = util::lerp<Point3f>(x00, x10, interp.y);
		Point3f y1 = util::lerp<Point3f>(x01, x11, interp.y);
		
		*normal = util::lerp<Point3f>(y0, y1, interp.z);	
	}
}

/////////////////////////////////////////////////////////////////////////////////////

void util::ImplicitSurface::getPoints(const IndexVector& indices, PointVector& points) const {	
	points.resize(indices.size());
	for (unsigned i = 0; i < indices.size(); i++) {
		points[i] = m_points[indices[i]];
	}
}

void util::ImplicitSurface::getValues(const IndexVector& indices, std::vector<float>& values) const {
	values.resize(indices.size());
	for (unsigned i = 0; i < indices.size(); i++) {
		values[i] = m_values[indices[i]];
	}
}

void util::ImplicitSurface::getNormals(const IndexVector& indices, PointVector& normals) const {
	normals.resize(indices.size());
	for (unsigned i = 0; i < indices.size(); i++) {
		normals[i] = m_normals[indices[i]];
	}
}

/////////////////////////////////////////////////////////////////////////////////////

bool util::ImplicitSurface::readOFF(const std::string& filename, bool normalizeModel) {
	std::ifstream in(filename.c_str());
	std::string noff("NOFF");
	
	char buf[OFF_PARSER_BUF_LENGTH];
	unsigned numVertices = 0, numTriangles = 0, numEdges = 0;
	
	bool isNOFF = false;
	
	if (in.good()) {
		
		report("ImplicitSurface::readOFF: Loading file " << filename);
		
		if (!empty()) {
			_reset();
		}
		
		in.getline(buf, OFF_PARSER_BUF_LENGTH);
		isNOFF = !noff.compare(0, std::string::npos, buf, noff.length());
		if (!isNOFF) {
			reportE("Input file not in NOFF format");
			return false;
		}

		in >> numVertices >> numTriangles >> numEdges;
		m_numPoints = numVertices;
		m_points.resize(numVertices * 3);
		m_values.resize(numVertices * 3);
		m_colors.resize(numVertices);
		m_normals.resize(numVertices * 3);
		
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
			m_values[i] = 0.0f;
			ptrs[i] = &m_points[i];
			
			in >> nx >> ny >> nz;
			m_normals[i] = glm::normalize(Point3f(nx, ny, nz));
		}
		
		resetColor();
		
		m_center = average / (float)numVertices;
		
		// if model is displaced from origin, move to origin if desired
		if (normalizeModel) {
			for (unsigned i = 0; i < numVertices; i++) {
				m_points[i] -= m_center;
			}
			minPoint -= m_center;
			maxPoint -= m_center;
			m_center = Point3f(0.0f);
		}
		
		// normalize model size
		float diag = glm::length(util::BoundingBox(minPoint, maxPoint).extents()) / 4.0f;
		minPoint = Point3f(std::numeric_limits<float>::max());
		maxPoint = Point3f(std::numeric_limits<float>::min());
		for(int i = 0; i < numVertices; i++) {
			m_points[i] = m_points[i] / diag;
			minPoint = glm::min(minPoint, m_points[i]);
			maxPoint = glm::max(maxPoint, m_points[i]);
		}
		
		// construct slightly enlarged bounding box
		float diagonalLength = glm::length(util::BoundingBox(minPoint, maxPoint).extents());
		glm::vec3 diagonal = glm::normalize(util::BoundingBox(minPoint, maxPoint).extents());
		minPoint = minPoint - 0.01f * diagonalLength * diagonal;
		maxPoint = maxPoint + 0.01f * diagonalLength * diagonal;
		m_bounds = util::BoundingBox(minPoint, maxPoint);
		
		// construct grid
		m_grid = util::Grid3D(minPoint, maxPoint, m_grid.dimensions());

		report("ImplicitSurface::readOFF: Loading completed (" << numVertices << " points, " << m_bounds << ")"
				<< ", constructing kd-tree (max depth = " << m_maxDepth << ", min points = " << m_minPoints << ")");
		
		// construct kd-tree
		float time = 0.0f;
		GET_TIME( build(ptrs, m_bounds) )
		report("ImplicitSurface::readOFF: Initial tree construction completed, took " << time / 1000.0f << " ms");
		
		// for each point in the original dataset, construct boundary conditions
		float eps = 0.001f * diagonalLength;
		
		report("ImplicitSurface::readOFF: Computing boundary conditions");
		GET_TIME( _generateBoundaryConditions() );
		report("ImplicitSurface::readOFF: Computed boundary conditions, took " << time / 1000.0f << " ms");

		// construct full kd-tree
		ptrs.resize(m_points.size());
		for (int i = 0; i < m_points.size(); i++) {
			ptrs[i] = &m_points[i];
		}
		clear();
		GET_TIME( build(ptrs, m_bounds) )
		report("ImplicitSurface::readOFF: Final tree construction completed, took " << time / 1000.0f << " ms");
	} else {
		reportE("Could not open input file");
		return false;
	}

	in.close();
	
	return true;
}

void util::ImplicitSurface::_generateBoundaryConditions() {
	float eps = glm::length(m_bounds.extents()) * 0.001f;
	
	#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < m_numPoints; i++) {
		const Point3f& p = m_points[i];
		const Point3f& norm = m_normals[i];
		
		Point3f closest(0.0f);
		float localEps = eps;
		
		// positive direction
		while (true) {
			Point3f positiveOffset = p + norm * localEps;
			closest = getNearest(positiveOffset);
			if (closest == p) {
				m_points[i + m_numPoints] = positiveOffset;
				m_values[i + m_numPoints] = localEps;
				m_normals[i + m_numPoints] = Point3f(0.0f);
				break;
			} else {
				localEps /= 2.0f;
			}
		}
		
		localEps = -eps;
		
		// negative direction
		while (true) {
			Point3f negativeOffset = p + norm * localEps;
			closest = getNearest(negativeOffset);
			if (closest == p) {
				m_points[i + 2 * m_numPoints] = negativeOffset;
				m_values[i + 2 * m_numPoints] = localEps;
				m_normals[i + 2 * m_numPoints] = Point3f(0.0f);
				break;
			} else {
				localEps /= 2.0f;
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////

void util::ImplicitSurface::draw() const {
	glBegin(GL_POINTS);
	for (unsigned i = 0; i < m_numPoints; i++) {
		glColor3f(m_colors[i].r, m_colors[i].g, m_colors[i].b);
		glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
	}
	glEnd();

	float diag = glm::length(m_bounds.extents());
	glBegin(GL_LINES);
	for (unsigned i = 0; i < m_numPoints; i++) {
		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
		glVertex3f(m_points[i].x + m_normals[i].x / diag / 20.0f,
					m_points[i].y + m_normals[i].y / diag / 20.0f,
					m_points[i].z + m_normals[i].z / diag / 20.0f);
	}
	glEnd();
}

void util::ImplicitSurface::setDrawColor(const IndexVector& indices, const Color3f& color) {
	for (unsigned i = 0; i < indices.size(); i++) {
		m_colors[indices[i]] = color;
	}
}

void util::ImplicitSurface::resetColor() {
	std::fill(m_colors.begin(), m_colors.end(), Color3f(1.0f, 1.0f, 1.0f));
}

/////////////////////////////////////////////////////////////////////////////////////

void util::ImplicitSurface::subdivide(Node* node, const util::BoundingBox& box, unsigned depth) {
	
	if (depth > m_maxDepth || node->objects().size() < m_minPoints) {
		node->setLeaf();
		return;
	}
	
	Axis axis = box.dominantAxis();
	float split = median(node->objects(), axis)[axis];
	
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

void util::ImplicitSurface::printNode(std::ostream& out, Node* node, unsigned indents) const {
	std::string axis = util::toString(node->m_axis);
	const bool leaf = node->leaf();
	
	out << util::Indents(indents) << "KDTreeNode: " << axis << "-split @ " << node->m_split;
	out << ", leaf = " << (leaf ? "yes" : "no") << ", objects = " << node->objects().size() << std::endl;

	if (node->leaf()) { return; }
	
	printNode(out, node->leftChild(), indents + 1);
	printNode(out, node->rightChild(), indents + 1);
}

///////////////////////////////////////////////////////////////////////////////////////////

void util::ImplicitSurface::_reset() {
	m_points.clear();
	m_normals.clear();
	m_values.clear();
	m_colors.clear();
	m_grid = Grid3D(Point3f(0.0f), Point3f(0.0f), Point3i(10));
	m_bounds = util::BoundingBox(Point3f(0.0f), Point3f(0.0f));
	clear();
}

void util::ImplicitSurface::_updateGridDim(Axis axis, unsigned dim) {
	Point3i dims = m_grid.dimensions();
	dims[axis] = dim;
	// report("ImplicitSurface: Creating grid (dims = (" << dims.x << "," << dims.y << "," << dims.z << "), bounds = " << m_bounds << ")");
	m_grid = util::Grid3D(m_bounds.min(), m_bounds.max(), dims);
}


