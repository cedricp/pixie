#ifndef DEEPIMAGE_H
#define DEEPIMAGE_H

#include "common/algebra.h"
#include <string.h>
#include <string>
#include <vector>
#include <iostream>

#define RED_IDX    0
#define GREEN_IDX  1
#define BLUE_IDX   2
#define ALPHA_IDX  3

class deepPixel{
public:
	deepPixel(){
		depth = 0.;
		rgba[0] = 0.;
		rgba[1] = 0.;
		rgba[2] = 0.;
		rgba[3] = 0.;
	}
	deepPixel(const deepPixel& pix){
		memcpy(this, &pix, sizeof(float)*5);
	}

	deepPixel& operator = (const deepPixel& pix){
		rgba[0] = pix.rgba[0];
		rgba[1] = pix.rgba[1];
		rgba[2] = pix.rgba[2];
		rgba[3] = pix.rgba[3];
		depth = pix.depth;
		return *this;
	};

	float depth;
	float rgba[4];
};

struct impl;

class deepImage
{
public:
	deepImage();
	~deepImage();

	void setFilename(std::string filename);
	void setCompression(float compression);
	void setSize(int w, int h);
	void getSize(int &w, int &h);
	void setCameraMatrix(matrix m);
	void setPixelDepth(const int &x, const int &y, std::vector<deepPixel>& pixel, bool merge = false);
	bool write();
	bool read(const std::string &filename);
	bool read_append(const std::string &filename);
	void set_pixel_color_list(std::vector<deepPixel>& pixel, const int &x, const int &y);
	bool get_pixel_color_list(std::vector<deepPixel> &list, int x, int y);
	void set_size(int w, int h);
	void doPixelIntegration(std::vector<deepPixel> &depthPixels, std::vector<deepPixel> *depthPixels_append = NULL, bool reorderFragments=false);
	float* to2DBufferFloat();
	char* to2DBufferChar();

	static bool compose_depth_values(colorrgba color, std::vector<deepPixel> &list, bool reorder_list);
	static bool sort_pixel_depth( const deepPixel& val1 , const deepPixel& val2 )
	{
		if( val1.depth != val2.depth )
			return ( val1.depth < val2.depth );
		else
			return val1.rgba[3] > val2.rgba[3];
	}

private:
	impl* internalData;
};




#endif

