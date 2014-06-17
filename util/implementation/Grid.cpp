#include <util/interface/Grid.hpp>
#include <GL/glew.h>
#include <glm/interface/gtx/norm.hpp>

util::Grid::Grid() : m_min(Point3f(0, 0, 0)),
m_max(Point3f(0, 0, 0)),
m_m(1),
m_n(1)
{ }

bool util::Grid::create(Point3f min, Point3f max, int m, int n)
{
    m_min = min;
    m_max = max;
    m_m = m;
    m_n = n;
    m_deltaX = (max.x - min.x)/(m-1);
    m_deltaY = (max.y - min.y)/(n-1);

    float deltaX = (max.x - min.x)/(m-1);
    float deltaY = (max.y - min.y)/(n-1);
    m_points.resize(m*n);
    m_colors.resize(m*n);

    for (unsigned i=0; i<m*n; i++)
    {
        Point3f newPoint;
        newPoint.x = min.x + (i%m) * deltaX;
        newPoint.y = min.y + (i/m) * deltaY;
        newPoint.z = 0;
        m_points[i] = newPoint;
        m_colors[i] = Color3f(1.0f, 1.0f, 1.0f);
    }

    return true;
}

void util::Grid::setRenderGrid(bool enable)
{
    m_render = enable;
}

PointVector* util::Grid::getPoints()
{
    return &m_points;
}

Point3f util::Grid::at(int i, int j)
{
    return m_points[i*m_m + j];
}

const Point3f util::Grid::at(int i, int j) const
{
    return m_points[i*m_m + j];
}

void util::Grid::setZat(int i, int j, float z)
{
    m_points[i*m_m + j].z = z;
}

void util::Grid::setAt(int i, int j, Point3f point)
{
    //std::cout << point << std::endl;
    m_points[i*m_m + j] = point;
}

void util::Grid::draw() const {
    if (m_render)
    {
        // Draw points
        GLfloat oldPointSize;
        glGetFloatv(GL_POINT_SIZE, &oldPointSize);
        glPointSize(3);

        glBegin(GL_POINTS);
        for (unsigned i= 0; i < m_points.size(); i++) {
            //glColor3f(m_colors[i].r, m_colors[i].g, m_colors[i].b);
            glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
        }
        glEnd();

        glPointSize(oldPointSize);

		if(m_points.size()>0){
        // Draw grid
        glBegin(GL_LINES);
        for(unsigned i=0; i<m_n; i++)
        {
			glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(m_points[i*m_m].x, m_points[i*m_m].y, m_points[i*m_m].z);
			glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(m_points[(i+1)*m_m-1].x, m_points[(i+1)*m_m-1].y, m_points[(i+1)*m_m-1].z);
        }

        for(unsigned i=0; i<m_m; i++)
        {
			glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
			glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(m_points[i + (m_n-1)*m_m].x, m_points[i + (m_n-1)*m_m].y, m_points[i + (m_n-1)*m_m].z);
        }
        glEnd();
		}
    }
}
