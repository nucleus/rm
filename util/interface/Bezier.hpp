 /*
 * Header file for BezierCurve class.
 */

#ifndef BEZIER_HPP
#define BEZIER_HPP

#include <util/interface/Util.hpp>
#include <glm/interface/gtx/norm.hpp>

namespace util {

	class CubicBezierCurve {
	public:
		enum RasterizationMode {
			Mode_direct = 0,
			Mode_casteljau,
			NUM_MODES
		};

		typedef std::vector< Point2f > PointVector2D;

	public:
		CubicBezierCurve();
		virtual ~CubicBezierCurve();

	public:
		void rasterize(unsigned k = 100, RasterizationMode mode = Mode_direct);

	public:
		void setControlPoint(unsigned i, const Point2f& p);
		Point2f getControlPoint(unsigned i) const;
		Point2f& operator[](unsigned i);
		const Point2f& operator[](unsigned i) const;

		unsigned getClosestControlPointIdx(const Point2f& point) const;

	protected:
		void _rasterizeDirect(unsigned k);
		void _rasterizeCasteljau(unsigned k);

	public:
		virtual void draw();

	protected:

		const unsigned m_degree;

		bool m_rasterized;
		
		PointVector2D m_controlPoints;
		PointVector2D m_rasterizedPoints;
	};

}
#endif
