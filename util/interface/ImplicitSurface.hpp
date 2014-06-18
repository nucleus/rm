/*
 * Header file for ImplicitSurface class.
 */

#ifndef IMPLICIT_SURFACE_HPP
#define IMPLICIT_SURFACE_HPP

#include <vector>
#include <map>

#include <util/interface/Util.hpp>
#include <util/interface/BoundingBox.hpp>
#include <util/interface/KDTree.hpp>
#include <util/interface/Grid3D.hpp>
#include <util/interface/Ray.hpp>

#define OFF_PARSER_BUF_LENGTH 1024

// CUDA interface
void launchRaymarchKernel();

namespace util {
	class ImplicitSurface : public KDTree<Point3f> {
	public:
		typedef std::pair< unsigned, Point3f > IndexPointPair;
		typedef std::vector< IndexPointPair > PointIndices;
		typedef std::vector< unsigned > IndexVector;

		typedef KDTreeNode< Point3f > Node;

	public:
		class PointResultSet {
		public:
			typedef std::pair< float, unsigned > PointRecord;
			typedef std::multimap< float, unsigned > PointRecords;

		public:
			PointResultSet(unsigned k, float radius);

		public:
			size_t size() const { return m_records.size(); }

		public:
			IndexVector getIndices() const;

			void insert(unsigned index, float distance);

			float maximumDistance() const;

		public:
			unsigned m_k;
			float m_radius;

		protected:
			PointRecords m_records;
		};

	public:
		ImplicitSurface(): 
			m_grid(Point3f(0.0f), Point3f(0.0f), Point3i(10)),
			m_maxDepth(24),
			m_minPoints(8),
			m_numPoints(0),
			m_rmSteps(100) { }
			
		virtual ~ImplicitSurface() {
			clear();
		}
		
	public:
		size_t size() const {
            return m_points.size();
		}
		
		Point3f center() const {
			return m_center;
		}
		
		BoundingBox boundingBox() const {
			return m_bounds;
		}
		
		bool empty() const {
			return (m_points.size() == 0);
		}
		
		const PointVector& points() const {
			return m_points;
		}
		
		Grid3D& grid() {
			return m_grid;
		}
		
		const Grid3D& grid() const {
			return m_grid;
		}
		
	public:
		Point3f getNearest(const Point3f& point);
		IndexVector collectInRadius(const Point3f& point, float radius);
		
		void getPoints(const IndexVector& indices, PointVector& points) const;
		void getValues(const IndexVector& indices, std::vector<float>& values) const;
		void getNormals(const IndexVector& indices, PointVector& normals) const;
		
	public:
		void intersect(const util::RayVector& rays, PointNormalData& intersections, bool enableGPU) const;
		
	protected:
		void _intersectGPU(const util::RayVector& rays, PointNormalData& intersections) const;
		void _intersectCPU(const util::RayVector& rays, PointNormalData& intersections) const;
		
		bool _intersect(const util::Ray& ray, Point3f& hit, Point3f& normal) const;
	public:
		bool readOFF(const std::string& filename, bool normalizeModel = false);
			
	public:
		void setTreeMaxDepth(unsigned depth) { m_maxDepth = depth; }
		void setTreeLeafSize(unsigned size) { m_minPoints = size; }
		
		void setGridDimX(unsigned dim) { _updateGridDim(X, dim); }
		void setGridDimY(unsigned dim) { _updateGridDim(Y, dim); }
		void setGridDimZ(unsigned dim) { _updateGridDim(Z, dim); }
		
		void setRayMarchingSteps(unsigned steps) { m_rmSteps = steps; }
			
	public:
		void draw() const;
		
		void resetColor();
		
		void setDrawColor(const IndexVector& indices, const Color3f& color);
		
	protected:
		IndexVector _collectInRadiusKDTree(const Point3f& point, float radius);
		IndexVector _collectKNearestKDTree(const Point3f& point, unsigned k);

		void _gatherPoints(Node* node, const util::BoundingBox& box, const Point3f& point, PointResultSet& results);

	protected:
		void subdivide(Node* node, const util::BoundingBox& box, unsigned depth = 0);
		
		void printNode(std::ostream& out, Node* node, unsigned indents = 0) const;
		
	protected:
		void _generateBoundaryConditions();
		
		bool _clipRayAgainstBounds(const util::Ray& ray, float& tnear, float& tfar) const;
		
		void _evaluate(const Point3f& p, float* value = NULL, Point3f* normal = NULL) const;
		void _voxel(const Point3f& p, Point3i& base, Point3f& interp) const;
		
		void _updateGridDim(Axis axis, unsigned dim);
		
		void _reset();
		
	protected:
		PointVector m_points;
		PointVector m_normals;
		std::vector<float> m_values;
		
		Point3f m_center;
		BoundingBox m_bounds;
		
		Grid3D m_grid;
		
		ColorVector m_colors;
		
		unsigned m_maxDepth, m_minPoints;
		unsigned m_numPoints;
		
		unsigned m_rmSteps;
	};
}

#endif
