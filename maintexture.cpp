#include "tree.h"

using namespace std;

//BmpManager img(1080,1600);
//BmpManager img(960,1440);
BmpManager img(540,800);
vector<Point> lightSource;
cv::Mat3b texture;

bool antiAlias = false;
int antiAliasMode = 1; //1 is 3 * 3, 2 is 5 * 5
bool softShadow = false;
int softShadowNum = 2;
bool smooth = true;
const double PI = 3.141592653589793;
const double EPS = 1e-10;
const int TRACINGDEPTH = 1;
const double REFRACTINDEX = 2.2;
const int WIDTH = 3000;
const int AMPLIFICATION = 24;
Color WHITE = Color(1,1,1);
Color BLACK = Color(0,0,0);
Color GRAY = Color(0.2,0.2,0.2);
Color BACKGROUD = Color(0.2,0.2,0.2);
KdTreeNode *root;


Point cross(Point p1, Point p2) {
	return Point(p1.y * p2.z - p2.y * p1.z, p1.z * p2.x - p2.z * p1.x, p1.x * p2.y - p2.x * p1.y);
}

double dot(Point p1, Point p2) {
	return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

int sign(double x) {
	if(x < EPS && x > -EPS)
		return 0;
	if(x >= EPS)
		return 1;
	return -1;
}

pair<double, double> phong(Point n, Point light, Point p, Point vp){
	Point L = light - p;
	L.normalize();
	Point N = n;
	N.normalize();
	Point V = vp - p;
	V.normalize();
	Point R = N * dot(L, N) * 2 - L;
	R.normalize();
	double kd = 0.7;
	double ks = 0.3;
	double imd = 0.8;
	double ims = 0.6;
	int alpha = 47;
	double dif = max(0., kd * dot(L, N) * imd);
	double high = max(0., ks * pow(dot(R, V), alpha) * ims);
	return make_pair(dif, high);
}

bool inShadow(Point &light, Intersection &intersection) {
	bool res = false;
	Intersection inP;
	inP.init();
	ViewRay sd = ViewRay(intersection.p, light - intersection.p);
	if(root->find_first_intersect(sd, inP) && inP.p.distance(intersection.p) < light.distance(intersection.p)) {
		res = true;
	}
	return res;
}


Color tracing(vector<GeometricalObject*>& m, const ViewRay &r, int depth) {
	if(depth <= 0) return Color(0,0,0); Color ans = BACKGROUD; Point norm;					//store the norm vector of the closest mesh from the view point.
	double high;
	Intersection intersection;
	intersection.init();

	if (!root->find_first_intersect(r, intersection)) {
		return BLACK;
	}
	if(intersection.m->isTri()) {
		if (smooth && sign(((Tri*)(intersection.m))->norm[0].length()) > 0
				 && sign(((Tri*)(intersection.m))->norm[1].length()) > 0
				 && sign(((Tri*)(intersection.m))->norm[2].length()) > 0) {
			norm = ((Tri*)(intersection.m))->norm[0] * intersection.alpha
				+ ((Tri*)(intersection.m))->norm[1] * intersection.beta
				+ ((Tri*)(intersection.m))->norm[2] * intersection.gamma;
		} else {
			norm = cross(((Tri*)intersection.m)->p[1] - ((Tri*)intersection.m)->p[0], ((Tri*)intersection.m)->p[2] - ((Tri*)intersection.m)->p[0]);
		}
	} else {
		norm = intersection.p - ((Sphere*)intersection.m)->c;
	}
	norm.normalize();

	if(sign(dot(norm, r.p - intersection.p)) >= 0) {
		double l = 0.1;
		high = 0;
		if(softShadow) for(size_t j = 0; j < lightSource.size(); j++) {
			int shadowNum = 0;
			double shadowRatio;
			for(int dx = -softShadowNum; dx <= softShadowNum; dx++) for(int dy = -softShadowNum; dy <= softShadowNum; dy++) {
				Point lightSample = Point(lightSource[j].x + dx * 20, lightSource[j].y + dy * 20, lightSource[j].z);
				if(!inShadow(lightSample, intersection))
					shadowNum++;
			}
			shadowRatio = (shadowNum / 1.) / (double)((2 * softShadowNum + 1) * (2 * softShadowNum + 1));
			pair<double, double> pho = phong(norm, lightSource[j], intersection.p, r.p);
			l += pho.first * shadowRatio;
			high += pho.second * shadowRatio;
		} else for(size_t j = 0; j < lightSource.size(); j++) {
			if(!inShadow(lightSource[j], intersection)) {
				pair<double, double> pho = phong(norm, lightSource[j], intersection.p, r.p);
				l += pho.first;
				high += pho.second;
			}
		}
		if(l > 1)
			l = 1;
		if(high > 1)
			high = 1;
		if(intersection.m == m[0] || intersection.m == m[1])
			high = 0;
		if(intersection.m != m[0] && intersection.m != m[1]) {
			if(intersection.m->isTri())
				ans = ((Tri*)intersection.m)->color * l;
			else
				ans = ((Sphere*)intersection.m)->color * l;
		} else {
			double x, y;
			if(intersection.m == m[0]) {
				x = intersection.alpha * 2000. + intersection.beta * 0. + intersection.gamma * 2000.;
				y = intersection.alpha * 0. + intersection.beta * 0. + intersection.gamma * 2000.;
			} else {
				x = intersection.alpha * 2000. + intersection.beta * 0. + intersection.gamma * 0.;
				y = intersection.alpha * 2000. + intersection.beta * 0. + intersection.gamma * 2000.;
			}
			cv::Point point((int)x % 400,(int)y % 400);
			const cv::Vec3b& bgr = texture(point);
			ans = Color(((double)bgr[2]) / 255., ((double)bgr[1]) / 255., ((double)bgr[0]) / 255.) * l;
			/*
			int iy = (sign(intersection.p.y) >= 0) ? (int)(intersection.p.y / 60.) : (int)(intersection.p.y / 60.) - 1;
			int ix = (sign(intersection.p.x) >= 0) ? (int)(intersection.p.x / 60.) : (int)(intersection.p.x / 60.) - 1;
			if((iy + ix) % 2 == 0) {
				ans = WHITE * l;
			} else {
				ans = GRAY * l;
			}
			*/
		}
	} else {
		ans = Color(0,0,0);
		high = 0;
	}

	Point I = r.dir;
	I.normalize();
	ans = ans * (1 - intersection.m->trans - intersection.m->reflect); 
	ans = ans + Color(1,1,1) * high;

	bool inside = false;
	bool totalRef = false;
	if(sign(dot(norm, I)) > 0)
		inside = true;
	if(sign(dot(norm, r.dir)) != 0 && sign(intersection.m->trans) > 0) {
		double rIndex = REFRACTINDEX;
		if(sign(dot(norm, I)) > 0) {
			norm = norm * (-1);
			rIndex = (1. / rIndex);
		}
		norm.normalize();
		double cosi = - dot(I, norm);
		double sini2 = 1 - cosi * cosi;
		double sint2 = sini2 / (rIndex * rIndex);
		if(sign(sint2 - 1) <= 0) {
			double cost = sqrt(1 - sint2);
			Point refractDir = I * (1. / rIndex) + norm * ((1. / rIndex) * cosi - cost);
			refractDir.normalize();
			ans = ans + tracing(m, ViewRay(intersection.p, refractDir), depth - 1) * intersection.m->trans;
		} else {
			totalRef = true;
		}
	}
	if((sign(intersection.m->reflect) > 0 && !inside) || totalRef) {
		Point reflectDir = norm * dot(r.dir * (-1), norm) * 2 - r.dir * (-1);
		reflectDir.normalize();
		Color temp = tracing(m, ViewRay(intersection.p, reflectDir), depth - 1);
		if(totalRef)
			ans = ans + temp * (intersection.m->reflect + intersection.m->trans);
		else 
			ans = ans + temp * intersection.m->reflect;
	}


	return ans;
}

void render(vector<GeometricalObject*>& m, Point vp) {
	for(int x = 0; x < img.w; x++) for(int y = 0; y < img.h; y++) {
		if(y % 100 == 0)
			cout << 100 * (double)(x * img.h + y) / (double)(img.h * img.w) << "%" << endl;
		Color ans(0, 0, 0);
		if(antiAlias && antiAliasMode == 1) {
			for(int ix = -1; ix <= 1; ix++) for(int iy = -1; iy <= 1; iy++) {
				double dx = ix * 0.3, dy = iy * 0.3;
				ViewRay ray = ViewRay(vp, Point(-500., x + dx - img.w / 2., y + dy - img.h / 2.));
				ray.dir.normalize();
				ans = ans + tracing(m, ray, TRACINGDEPTH) * (1 / 9.);
			}
		}
		if(antiAlias && antiAliasMode == 2) {
			for(int ix = -2; ix <= 2; ix++) for(int iy = -2; iy <= 2; iy++) {
				double dx = ix * 0.2, dy = iy * 0.2;
				ViewRay ray = ViewRay(vp, Point(-500., x + dx - img.w / 2., y + dy - img.h / 2.));
				ray.dir.normalize();
				ans = ans + tracing(m, ray, TRACINGDEPTH) * (1 / 25.);
			}
		}
		if(!antiAlias) {
			ViewRay ray = ViewRay(vp, Point(-500., x - img.w / 2., y - img.h / 2.));
			ray.dir.normalize();
			ans = tracing(m, ray, TRACINGDEPTH);
		}
		img.pixels[img.h - 1 - y][x] = ans.toPixel();
		/*
		count++;
		if (count >= 1000) {
			count = 0;
			img.print_to_file("Dragon.bmp");
		}
		*/
	}
}

void rotateAxis(Point *p, int dim, double angle) {
	double x = p->x;
	double y = p->y;
	double z = p->z;
	switch (dim) {
		case 0:					//clockwise about the x axis;
			p->x = x * 1. 		+ y * 0. 		+ z * 0.;
			p->y = x * 0. 		+ y * cos(angle) 	+ z * sin(angle);
			p->z = x * 0. 		+ y * (-sin(angle))	+ z * cos(angle);
			break;
		case 1:					//clockwise about the y axis;
			p->x = x * cos(angle) 	+ y * 0. 		+ z * (-sin(angle));
			p->y = x * 0. 		+ y * 1.	 	+ z * 0.;
			p->z = x * sin(angle)	+ y * 0.		+ z * cos(angle);
			break;
		case 2:					//clockwise about the z axis;
			p->x = x * cos(angle)	+ y * sin(angle)	+ z * 0.;
			p->y = x * (-sin(angle))+ y * cos(angle) 	+ z * 0.;
			p->z = x * 0. 		+ y * 0.		+ z * 1.;
			break;
	}
	return;
}

void rotate(Point* point) {
	double angle = PI / 2;
	rotateAxis(point, 0, angle * (-1));
	rotateAxis(point, 2, angle * (-1.1));
	rotateAxis(point, 1, PI * (-0.125));
	return;
}

void readScene(vector<GeometricalObject*> &m) {
	ifstream fin("dragon.obj");
	char c;
	double x, y, z;
	long int p[3];
	Color color = Color(200. / 255.,200. / 255.,200. / 255.);
	vector<Point> v;
	vector<Point> norm;
	vector<Triple> inputTri;
	string temp;
	double minz = 1000;
	while (fin >> c) {
		if (c == 'v') {
			fin >> x >> y >> z;
			x = AMPLIFICATION * x;
			y = AMPLIFICATION * y;
			z = AMPLIFICATION * z;
			Point point = Point(x,y,z);
			rotate(&point);
			if(point.z < minz)
				minz = point.z;
			v.push_back(point);
			norm.push_back(Point(0,0,0));
		} else if(c == 'f') {
			fin >> p[0] >> p[1] >> p[2];

			for(int i = 0; i < 3; i++)
				p[i]--;

			Point normal = cross(v[p[1]] - v[p[0]], v[p[2]] - v[p[0]]);
			normal.normalize();

			for(int i = 0; i < 3; i++) {
				norm[p[i]] = norm[p[i]] + normal;
			}

			Triple tri = Triple(p[0], p[1], p[2]);

			inputTri.push_back(tri);
		} else if(c == '#') {
			getline(fin, temp);
		}
	}
	for (size_t i = 0; i < norm.size(); i++) {
		norm[i].normalize();
	}
	for (size_t i = 0; i < inputTri.size(); i++) {
		m.push_back(new Tri(v[inputTri[i].p[0]], v[inputTri[i].p[1]], v[inputTri[i].p[2]], norm[inputTri[i].p[0]], norm[inputTri[i].p[1]], norm[inputTri[i].p[2]], color));
	}
	cout << "minz = " << minz << endl;
	return;
}

void createScene(vector<GeometricalObject*> &m) { 		// why &Tri() is local variable and new Tri() is not? TODO
	m.push_back(new Tri(Point(-500,600,-1), Point(-500,-600,-1), Point(500,600,-1), WHITE, 0, 0.));
	m.push_back(new Tri(Point(500,600,-1), Point(-500,-600,-1), Point(500,-600,-1), WHITE, 0, 0.));
	for(int i = 0; i < 3; i++) {
		rotateAxis(&((Tri*)m[0])->p[i], 1, PI * (-0.125));
		rotateAxis(&((Tri*)m[1])->p[i], 1, PI * (-0.125));
	}
}

int main() {
	texture = cv::imread("texture.bmp", CV_LOAD_IMAGE_COLOR);
	vector<GeometricalObject*> m;
	lightSource.push_back(Point(320,180,560));
	lightSource.push_back(Point(440,-180,320));
	createScene(m);
	readScene(m);
	BBox box(m);
	for(size_t i = 0; i < lightSource.size(); i++) {
		box.build(lightSource[i]);
	}
	int count = 0;
	root = new KdTreeNode(m, 0, box, count);
	Point viewPoint = Point(360, 0, 60);
	cout << m.size() << endl;
	render(m, viewPoint);
	img.print_to_file("DragonTexture.bmp");
}
