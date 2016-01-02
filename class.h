#include "bmp-manager.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstring>
#include <cmath>
#include <vector>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <cstdlib>

using namespace std;

class Point {
	public:
		double x,y,z;
		Point() {}
		Point(double ix, double iy, double iz): x(ix), y(iy), z(iz)  {}

		Point operator+(const Point& p) const {
			return Point(this->x + p.x, this->y + p.y, this->z + p.z);
		}
		Point operator-(const Point& p) const {
			return Point(this->x - p.x, this->y - p.y, this->z - p.z);
		}
		Point operator*(const double c) const {
			return Point(this->x * c, this->y * c, this->z * c);
		}
		Point operator/(const double c) const {
			return Point(this->x / c, this->y / c, this->z / c);
		}
		double length() {
			return sqrt(x * x + y * y + z * z);
		}
		void normalize() {
			double len = length();
			x = x / len;
			y = y / len;
			z = z / len;
		}
		double distance(Point p) {
			return sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) + (z - p.z) * (z - p.z));
		}
		double getDim(int dim) {
			if(dim == 0)
				return y;
			if(dim == 1)
				return x;
			if(dim == 2)
				return z;
		}
};

class Color {
	public:
		double r,g,b;		// real number from 0.0 to 1.0
		Color() {}
		Color(double ir, double ig, double ib): r(ir), g(ig), b(ib)  {}
		Color operator*(const double l) const {
			return Color(r * l, g * l, b * l);
		}
		Color operator+(Color c) const {
			Color ret = Color(r + c.r, g + c.g, b + c.b);
			if(ret.r > 1) {
				ret.r = 1;
			}
			if(ret.g > 1) {
				ret.g = 1;
			}
			if(ret.b > 1) {
				ret.b = 1;
			}
			return ret;
		}
		Pixel toPixel() const {
			return Pixel((int)(r*255.), (int)(g*255.), (int)(b*255.));
		}
};

class GeometricalObject {
	public:
		double trans; 		//对应透明程度	
		double reflect;		//对应光滑程度 trans和reflect加起来应该小于1，剩下的就是本来的颜色
		GeometricalObject() {}
		virtual bool isTri()=0;
		virtual bool isSphere()=0;
		virtual pair<double, double> getDimRange(int dim)=0;
};

class Sphere : public GeometricalObject {
	public:
		Point c;
		double r;
		Color color;
		Sphere() {}
		Sphere(Point c, double r, Color color, double trans, double reflect) {
			this->c = c;
			this->r = r;
			this->color = color;
			this->trans = trans;
			this->reflect = reflect;
		}
		Sphere(Point c, double r, Color color) {
			this->c = c;
			this->r = r;
			this->color = color;
			this->trans = 0;
			this->reflect = 0;
		}
		pair<double, double> getDimRange(int dim) {
			double first, second;
			switch (dim) { 
				case 0:
					first = c.y - r;
					second = c.y + r;
					break;
				case 1:
					first = c.z - r;
					second = c.z + r;
					break;
				case 2:
					first = c.x - r;
					second = c.x + r;
					break;
			}
			return make_pair(first, second);
		}
		bool isTri() {
			return false;
		}
		bool isSphere() {
			return true;
		}
};

struct Triple {
	int p[3];
	Triple(int p0, int p1, int p2) {
		p[0] = p0;
		p[1] = p1;
		p[2] = p2;
	}
};

class Tri : public GeometricalObject {
	public:
		Point p[3];
		Point norm[3];
		Color color;
		Tri() {}
		Tri(Point ip0, Point ip1, Point ip2, Color ic, double trans, double reflect) {
			p[0] = ip0;
			p[1] = ip1;
			p[2] = ip2;
			this->color = ic;
			this->trans = trans;
			this->reflect = reflect;
		}

		Tri(Point ip0, Point ip1, Point ip2, Color ic) {
			p[0] = ip0;
			p[1] = ip1;
			p[2] = ip2;
			this->color = ic;
			this->trans = 0;
			this->reflect = 0;
		}

		Tri(Point ip0, Point ip1, Point ip2, Point norm0, Point norm1, Point norm2, Color ic, double trans, double reflect) {
			p[0] = ip0;
			p[1] = ip1;
			p[2] = ip2;
			norm[0] = norm0;
			norm[1] = norm1;
			norm[2] = norm2;
			this->color = ic;
			this->trans = trans;
			this->reflect = reflect;
		}

		Tri(Point ip0, Point ip1, Point ip2, Point norm0, Point norm1, Point norm2, Color ic) {
			p[0] = ip0;
			p[1] = ip1;
			p[2] = ip2;
			norm[0] = norm0;
			norm[1] = norm1;
			norm[2] = norm2;
			this->color = ic;
			this->trans = 0;
			this->reflect = 0;
		}

		pair<double, double> getDimRange(int dim) {
			double first, second;
			switch (dim) { 
				case 0:
					first = min(p[2].y, min(p[0].y, p[1].y));
					second = max(p[2].y, max(p[0].y, p[1].y));
					break;
				case 1:
					first = min(p[2].z, min(p[0].z, p[1].z));
					second = max(p[2].z, max(p[0].z, p[1].z));
					break;
				case 2:
					first = min(p[2].x, min(p[0].x, p[1].x));
					second = max(p[2].x, max(p[0].x, p[1].x));
					break;
			}
			return make_pair(first, second);
		}

		bool isSphere() {
			return false;
		}
		bool isTri() {
			return true;
		}
};

class ViewRay {
	public:
		Point p,dir;
		ViewRay() {}
		ViewRay(Point ip, Point idir) {
			p.x = ip.x;
			p.y = ip.y;
			p.z = ip.z;
			dir.x = idir.x;
			dir.y = idir.y;
			dir.z = idir.z;
			dir.normalize();
		}
};

