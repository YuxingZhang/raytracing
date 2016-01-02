#include <cstring>
#include <cstdio>
#include <vector>
#include <string>
#include <cstdlib>
using namespace std;

struct Pixel {
	int r, g, b;
	Pixel() { r=g=b=0; }
	Pixel(int r, int g, int b)
		: r(r), g(g), b(b) {}
};

struct BmpManager {
	vector<vector<Pixel> > pixels;
	int h, w;
	BmpManager() { h=w=0; }
	BmpManager(int h, int w) {
		this->h=h;
		this->w=w;
		pixels.resize(h);
		for(int i=0; i<h; ++i)
			pixels[i].resize(w);
	}
	void print_to_file(string file_name) {
	//printf("x\n"); fflush(stdout);
		FILE *f;
		int filesize=54+3*w*h;
	//printf("a\n"); fflush(stdout);
		unsigned char *img=(unsigned char*)malloc(3*w*h);
		for(int i=0; i<h; i++) {
			for(int j=0; j<w; j++) {
				int x=j, y=i;
				int r=pixels[i][j].r;
				int g=pixels[i][j].g;
				int b=pixels[i][j].b;
				img[(x+y*w)*3+2]=(unsigned char)(r);
				img[(x+y*w)*3+1]=(unsigned char)(g);
				img[(x+y*w)*3+0]=(unsigned char)(b);
			}
		}

		unsigned char bmpfileheader[14]={'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
		unsigned char bmpinfoheader[40]={40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
		unsigned char bmppad[3] = {0,0,0};
	//printf("b\n");

		bmpfileheader[ 2] = (unsigned char)(filesize    );
		bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
		bmpfileheader[ 4] = (unsigned char)(filesize>>16);
		bmpfileheader[ 5] = (unsigned char)(filesize>>24);

		bmpinfoheader[ 4] = (unsigned char)(       w    );
		bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
		bmpinfoheader[ 6] = (unsigned char)(       w>>16);
		bmpinfoheader[ 7] = (unsigned char)(       w>>24);
		bmpinfoheader[ 8] = (unsigned char)(       h    );
		bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
		bmpinfoheader[10] = (unsigned char)(       h>>16);
		bmpinfoheader[11] = (unsigned char)(       h>>24);
	//printf("c\n");

		f = fopen(file_name.c_str(),"wb");
		fwrite(bmpfileheader,1,14,f);
		fwrite(bmpinfoheader,1,40,f);
		for(int i=0; i<h; i++) {
			fwrite(img+(w*(h-i-1)*3),3,w,f);
			fwrite(bmppad,1,(4-(w*3)%4)%4,f);
		}
		fclose(f);
	}
};
