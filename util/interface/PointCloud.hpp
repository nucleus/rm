/*
 * Header file for PointCloud class.
 */

#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include <iostream>
#include <vector>
#include <map>
#include <cassert>

#include <util/interface/Util.hpp>
#include <util/interface/BoundingBox.hpp>
#include <util/interface/KDTree.hpp>

#define OFF_PARSER_BUF_LENGTH 1024

namespace util {
	class PointCloud : public KDTree<Point3f> {
	public:
		typedef std::pair< unsigned, float > IndexDistPair;
		typedef std::pair< unsigned, Point3f > IndexPointPair;
		typedef std::vector< IndexDistPair > PointDistances;
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

		enum SplitPositionMode {
			Mode_mean = 0,
			Mode_median,
			NUM_SPLIT_MODES
		};
		
		enum PointGatherMode {
			Mode_naive = 0,
			Mode_kdtree,
			Mode_naive_2d,
			NUM_GATHER_MODES
		};

	public:
		PointCloud():
			m_splitMode(Mode_median),
			m_drawBBox(false),
			m_drawKDTree(false),
			m_maxDepth(24),
			m_minPoints(8) {}
			
		virtual ~PointCloud() {
			clear();
		}
		
	public:
		size_t size() const {
            return m_points.size();
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
		
	public:
		IndexVector collectKNearest(const Point3f& point, unsigned k, PointGatherMode mode = Mode_naive);
		IndexVector collectInRadius(const Point3f& point, float radius, PointGatherMode mode = Mode_naive);
		PointVector getPoints(const IndexVector& indices) const;
		
	public:
		bool readOFF(const std::string& filename, bool normalizeModel = false);
		bool writeOFF(const std::string& filename) const;
		
	public:
		void setTreeMaxDepth(unsigned depth) { m_maxDepth = depth; }
		void setTreeLeafSize(unsigned size) { m_minPoints = size; }
		
		void setBBoxDraw(bool enable = true) { m_drawBBox = enable; }
		void setKDTreeDraw(bool enable = false) { m_drawKDTree = enable; }
			
	public:
		void draw() const;
		
		void drawTree(Node* node, const BoundingBox& box) const;
		
		void resetColor();
		void setDrawColor(const IndexVector& indices, const Color3f& color);
		
	protected:
		void computeAllSquaredDistances(const Point3f& point, PointDistances& distances) const;
		
		IndexVector _collectKNearestNaive(const Point3f& point, unsigned k);
		IndexVector _collectInRadiusNaive(const Point3f& point, float radius);
		IndexVector _collectKNearestKDTree(const Point3f& point, unsigned k);
		IndexVector _collectInRadiusKDTree(const Point3f& point, float radius);
		IndexVector _collectInRadiusNaive2D(const Point3f& point, float radius);

		void _gatherPoints(Node* node, const util::BoundingBox& box, const Point3f& point, PointResultSet& results);

	protected:
		void subdivide(Node* node, const util::BoundingBox& box, unsigned depth = 0);
		
		void printNode(std::ostream& out, Node* node, unsigned indents = 0) const;

	protected:
		void reset();
		
	protected:
		PointVector m_points;
		
		BoundingBox m_bounds;
	
		SplitPositionMode m_splitMode;
		
		ColorVector m_colors;
		bool m_drawBBox, m_drawKDTree;
		
		unsigned m_maxDepth, m_minPoints;
	};
}

std::ostream& operator<<(std::ostream& out, const util::PointCloud& cloud);

#endif
