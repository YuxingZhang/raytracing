#include "class.h"
#include <algorithm>
using namespace std;

Point cross(Point p1, Point p2);
double dot(Point p1, Point p2);
int sign(double x);
const int LEAFSIZE = 50;

class Intersection {
	public:
		Point p;
		double t; // p = ray.p + t * ray.dir
		GeometricalObject *m;
		double alpha, beta, gamma; // p = alpha*m->p[0] + beta*m->p[1] + (...)

		Intersection() {}

		void init() { 
			t = 1E32;
			return;
		}
		
		void updateBy(Intersection &inter) {
			if (inter.t < this->t) {
				this->t = inter.t;
				this->p = inter.p;
				this->m = inter.m;
				this->alpha = inter.alpha;
				this->beta = inter.beta;
				this->gamma = inter.gamma;
			}
		}
};

class BBox {
	public:
		Point Max, Min;

		BBox() {
			Max = Point(0,0,0);
			Min = Point(0,0,0);
		}

		BBox(Point Max, Point Min) {
			this->Max = Max;
			this->Min = Min;
		}

		BBox(const vector<GeometricalObject*> &m) {
			Max = Point(0,0,0);
			Min = Point(0,0,0);
			build(m);
		}

		bool checkRay(ViewRay r) {
			//slab algo
			double INFINTY = 1e32;
			if (checkInside(r.p))
				return true;
			double t00,t01,t10,t11,t20,t21,t0min,t0max,t1min,t1max,t2min,t2max,tmin,tmax;
			tmin = -INFINTY;
			tmax = INFINTY;
			if (sign(r.dir.y) == 0) {
				if (sign(r.p.y - Max.y) > 0 || sign(r.p.y - Min.y) < 0)
					return false;
			} else {
				t00 = (Min.y - r.p.y) / r.dir.y;
				t01 = (Max.y - r.p.y) / r.dir.y;
				t0min = min(t00, t01);
				t0max = max(t00, t01);
				tmin = t0min;
				tmax = t0max;
			}
			if (sign(r.dir.x) == 0) {
				if (sign(r.p.x - Max.x) > 0 || sign(r.p.x - Min.x) < 0)
					return false;
			} else {
				t10 = (Min.x - r.p.x) / r.dir.x;
				t11 = (Max.x - r.p.x) / r.dir.x;
				t1min = min(t10, t11);
				t1max = max(t10, t11);
				tmin = max(tmin, t1min);
				tmax = min(tmax, t1max);
			}
			if (sign(r.dir.z) == 0) {
				if (sign(r.p.z - Max.z) > 0 || sign(r.p.z - Min.z) < 0)
					return false;
			} else {
				t20 = (Min.z - r.p.z) / r.dir.z;
				t21 = (Max.z - r.p.z) / r.dir.z;
				t2min = min(t20, t21);
				t2max = max(t20, t21);
				tmin = max(tmin, t2min);
				tmax = min(tmax, t2max);
			}
			if (sign(tmin - tmax) < 0 && sign(tmax - 0) > 0)
				return true;
			return false;
		}

		bool checkInside(Point p) {
			if (sign(p.x - Min.x) < 0 || sign(p.x - Max.x) > 0)
				return false;
			if (sign(p.y - Min.y) < 0 || sign(p.y - Max.y) > 0)
				return false;
			if (sign(p.z - Min.z) < 0 || sign(p.z - Max.z) > 0)
				return false;
			return true;
		}

		void build(const vector<GeometricalObject*> &m) {
			for(size_t i = 0; i < m.size(); i++) {
				if (m[i]->isTri()) {
					Tri *t = (Tri*) m[i];
					for(int j = 0; j < 3; j++) {
						build(t->p[j]);
					}
				} else {
					Sphere *s = (Sphere*) m[i];
					build(s->c + Point(s->r,s->r,s->r));
					build(s->c - Point(s->r,s->r,s->r));
				}
			}
		}

		void build(Point p) {
			if (p.x < Min.x)
				Min.x = p.x;
			if (p.y < Min.y)
				Min.y = p.y;
			if (p.z < Min.z)
				Min.z = p.z;
			if (p.x > Max.x)
				Max.x = p.x;
			if (p.y > Max.y)
				Max.y = p.y;
			if (p.z > Max.z)
				Max.z = p.z;
		}

		void split(int dim, double cut, BBox &lbox, BBox &rbox) {
			Point lMax, rMin;
			switch(dim) {
				case 0:						//cut the y axis
					lMax = Point(Max.x, cut, Max.z);
					rMin = Point(Min.x, cut, Min.z);
					lbox = BBox(lMax, Min);
					rbox = BBox(Max, rMin);
					break;
				case 1:						//cut the z axis
					lMax = Point(Max.x, Max.y, cut);
					rMin = Point(Min.x, Min.y, cut);
					lbox = BBox(lMax, Min);
					rbox = BBox(Max, rMin);
					break;
				case 2:						//cut the x axis
					lMax = Point(cut, Max.y, Max.z);
					rMin = Point(cut, Min.y, Min.z);
					lbox = BBox(lMax, Min);
					rbox = BBox(Max, rMin);
					break;
			}
//			cout << lbox.Min.x << ' ' << lbox.Min.y << ' ' << lbox.Min.z << ' ' << lbox.Max.x << ' ' << lbox.Max.y << ' ' << lbox.Max.z << endl;
//			cout << rbox.Min.x << ' ' << rbox.Min.y << ' ' << rbox.Min.z << ' ' << rbox.Max.x << ' ' << rbox.Max.y << ' ' << rbox.Max.z << endl;
		}

		/*
		double getSurfaceArea(void) {
			double length = fabs(max[0] - min[0]);
			double width = fabs(max[1] - min[1]);
			double height = fabs(max[2] - min[2]);
			return 2.0 * (length * width + length * height + width * height);
		}

		double getAvgArea(void) {
			double length = fabs(max[0] - min[0]);
			double width = fabs(max[1] - min[1]);
			double height = fabs(max[2] - min[2]);
			return (length * width + length * height + width * height) / 3.;
		}
		*/

};

//int __ = 0;

class KdTreeNode {
	public:
		bool isLeaf;
		vector<GeometricalObject*> m;
		KdTreeNode *lc, *rc;
		BBox box;

		KdTreeNode(vector<GeometricalObject*> &v, int dim, BBox box, int &count) {
			this->box = box;
			count++;
			if ((int)v.size() < LEAFSIZE) {
				isLeaf = true;
				this->m = v;
				return;
			}
			isLeaf = false;
			// TODO: can be optimized
			vector<double> list;
			for(size_t i = 0; i < v.size(); ++i) {
				pair<double, double> p = v[i]->getDimRange(dim); // dim in [0, 3)
				list.push_back((p.first + p.second) / 2);
			}
			sort(list.begin(), list.end());
			double cut = (list[(int)list.size() / 2] + list[((int)list.size() / 2) - 1]) / 2;
			// END
			vector<GeometricalObject*> lm, rm;
			for(size_t i = 0; i < v.size(); ++i) {
				pair<double, double> p = v[i]->getDimRange(dim); // dim in [0, 3)
				if (p.first <= cut) lm.push_back(v[i]);
				if (p.second > cut) rm.push_back(v[i]);
			}
			BBox lbox, rbox;
			box.split(dim, cut, lbox, rbox);
			lc = new KdTreeNode(lm, (dim + 1) % 3, lbox, count);
			rc = new KdTreeNode(rm, (dim + 1) % 3, rbox, count);
		}

		bool find_first_intersect(ViewRay ray, Intersection &intersection) {
			if (!box.checkRay(ray)){
				return false;
			}
			if (isLeaf) {
				bool res = false;
				for(size_t i = 0; i < m.size(); ++i) {
					Intersection tmp;
					if (getIntersect(m[i], ray, tmp) && box.checkInside(tmp.p)) {
						res = true;
						intersection.updateBy(tmp);
					}
				}
				return res;
			}
			if (lc->box.checkInside(ray.p)) {
				if (lc->find_first_intersect(ray, intersection))
					return true;
				return rc->find_first_intersect(ray, intersection);
			} else if (rc->box.checkInside(ray.p)) {
				if (rc->find_first_intersect(ray, intersection))
					return true;
				return lc->find_first_intersect(ray, intersection);
			} else {
				bool find = false;
				if (rc->find_first_intersect(ray, intersection))
					find = true;
				if (lc->find_first_intersect(ray, intersection))
					find = true;
				return find;
			}
		}

		double det(	double a11, double a12, double a13,
				double a21, double a22, double a23,
				double a31, double a32, double a33) {
			return a11 * a22 * a33 + a12 * a23 * a31 + a21 * a32 * a13
				- a31 * a22 * a13 - a32 * a23 * a11 - a21 * a12 * a33;
		}

		bool getIntersect(GeometricalObject* m, ViewRay r, Intersection &intersection) {
			if(m->isSphere()){
				Sphere *s = (Sphere*) m;
				double t1 = 0, t2 = 0;
				double b = dot(r.dir, r.p - s->c);
				double c = dot(r.p - s->c, r.p - s->c) - s->r * s->r;
				double delta = b * b - c;
				if(sign(delta) < 0){
					return false;
				} else if(sign(delta) > 0) {
					t1 = - b - sqrt(delta);
					t2 = - b + sqrt(delta);
				} else {
					t1 = t2 = - b;
				}
				if (sign(t1) > 0) {
					intersection.p = r.p + r.dir * t1;
					intersection.t = t1;
					intersection.m = m;
					return true;
				} else if (sign(t2)  > 0) {
					intersection.p = r.p + r.dir * t2;
					intersection.t = t2;
					intersection.m = m;
					return true;
				}
				return false;
			} else {
				Tri *tri = (Tri*) m;
				Point e1 = tri->p[0] - tri->p[1];
				Point e2 = tri->p[0] - tri->p[2];
				Point s = tri->p[0] - r.p;
				Point rd = r.dir;
				double t,beta,gamma;
				double denominator = det(rd.x, e1.x, e2.x, rd.y, e1.y, e2.y, rd.z, e1.z, e2.z);
				if(sign(denominator) != 0) {
					t = det(s.x, e1.x, e2.x, s.y, e1.y, e2.y, s.z, e1.z, e2.z) / denominator;
					beta = det(rd.x, rd.y, rd.z, s.x, s.y, s.z, e2.x, e2.y, e2.z) / denominator;
					gamma = det(rd.x, rd.y, rd.z, e1.x, e1.y, e1.z, s.x, s.y, s.z) / denominator;
					if(sign(t) <= 0)
						return false;
					if(sign(beta) < 0 || sign(gamma) < 0 || sign(1. - beta - gamma) < 0)
						return false;
					intersection.p = r.p + rd * t;
					intersection.t = t;
					intersection.m = m;
					intersection.beta = beta;
					intersection.gamma = gamma;
					intersection.alpha = (1. - beta - gamma);
					return true;
				}
				return false;
			}
		}
};

