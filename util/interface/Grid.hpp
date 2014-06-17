#ifndef GRID_H
#define GRID_H

#include <util/interface/Util.hpp>
#include <util/interface/PointCloud.hpp>
#include <util/interface/BoundingBox.hpp>

namespace util {
    class Grid {

    public:
        // Constructor
        Grid();

        // Functions
        bool create(Point3f min, Point3f max, int m, int n);
        void setRenderGrid(bool enable);
        PointVector* getPoints();
        Point3f at(int i, int j);
        const Point3f at(int i, int j) const;
        void setZat(int i, int j, float z);
        void setAt(int i, int j, Point3f point);

        // OpenGL
        void draw() const;
		
		void clear() {
			m_points.clear();
			m_colors.clear();
			m_min = Point3f(std::numeric_limits<float>::max());
			m_max = Point3f(std::numeric_limits<float>::min());
		}

    public:
        size_t size() const { return m_points.size(); }
        unsigned rows() const { return m_m; }
        unsigned cols() const { return m_n; }

    protected:
        PointVector m_points;
        ColorVector m_colors;
        Point3f m_min;
        Point3f m_max;
        unsigned m_m;
        unsigned m_n;
        float m_deltaX;
        float m_deltaY;
        bool m_render;

    };
}

#endif // GRID_H
