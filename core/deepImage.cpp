#include "deepImage.h"

#include "error.h"

#include <OpenEXR/half.h>
#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImathBox.h>


#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfPartType.h>
#include <OpenEXR/ImfDeepScanLineOutputFile.h>
#include <OpenEXR/ImfDeepScanLineInputFile.h>
#include <OpenEXR/ImfDeepFrameBuffer.h>
#include <OpenEXR/ImfDeepTiledOutputFile.h>
#include <OpenEXR/ImfStandardAttributes.h>

#include <algorithm>

#define coord2index(x,y) ( y*internalData->width ) + x

bool sortPixelByDepth( const deepPixel& d1 , const deepPixel& d2 )
{
	if( d1.depth != d2.depth )
	{
		return ( d1.depth < d2.depth );
	}
	else
	{
		return d1.rgba[3] > d2.rgba[3];
	}
}

struct impl{
	std::vector< std::vector<deepPixel> > pixelList;
	int width,height;
	std::string fileName;
	float compression;
	matrix cameraMatrix;
	bool valid;
};

deepImage::deepImage()
{
	internalData = new impl;
	internalData->fileName = "";
	internalData->width  = -1;
	internalData->height = -1;
	internalData->pixelList.clear();
	internalData->compression = 0.8;
	internalData->valid  = false;
}

deepImage::~deepImage()
{
	delete internalData;
}

void deepImage::setFilename(std::string filename)
{
	internalData->fileName = filename;
}

void
deepImage::setCompression(float compression)
{
	internalData->compression = compression;
}

void
deepImage::setSize(int w, int h)
{
	internalData->width  = w;
	internalData->height = h;
	internalData->pixelList.resize(w*h);
}

void
deepImage::getSize(int &w, int &h)
{
	w = internalData->width;
	h = internalData->height;
}

void
deepImage::setPixelDepth(const int &x, const int &y, std::vector<deepPixel>& pixel, bool merge)
{
	int index = coord2index(x,y);
	if (index >= internalData->pixelList.size() || internalData->width < 0 || internalData->height < 0){
		error(CODE_RANGE,"deepImage::setPixelDepth : You are about to write outside image boundaries: %i %i", x, y);
		return;
	}
	if (merge)
		doPixelIntegration(internalData->pixelList[index], &pixel, true);
	else
		internalData->pixelList[index] = pixel;
}

void
deepImage::setCameraMatrix(matrix m)
{
	movmm(internalData->cameraMatrix, m);
}

static bool sortPixels( const deepPixel& val1 , const deepPixel& val2 )
{
	if( val1.depth != val2.depth )
	{
		return ( val1.depth < val2.depth );
	}
	else
	{
		return val1.rgba[3] > val2.rgba[3];
	}
}

void
deepImage::doPixelIntegration(std::vector<deepPixel> &depthPixels, std::vector<deepPixel> *depthPixels_append, bool reorderFragements)
{
    bool can_skip = true;
    if (depthPixels_append)
    	can_skip = depthPixels_append->empty();

    if( depthPixels.empty() && can_skip)
    {
        return;
    }

    if (reorderFragements)
    	std::sort(depthPixels.begin(), depthPixels.end(), sortPixels);

    deepPixel tempColor;
	std::vector<deepPixel> colorDepth;

	// Do the integration of depth values
	if (!depthPixels.empty()){
		std::vector<deepPixel>::iterator begin 	= depthPixels.begin();
		std::vector<deepPixel>::iterator end 	= depthPixels.end();
		std::vector<deepPixel>::iterator it 	= begin;

		colorrgba colorOffset;
		initv4(colorOffset, 0.);
		std::vector<deepPixel>::iterator it2;

		float d, new_depth, current_depth;
		colorrgba slopeMin, slopeMax;
		colorrgba currentColor;
		initv4(currentColor, 0.);

		tempColor.depth = begin->depth;
		initv4(tempColor.rgba, 0.);

		while(true)
		{
			if (tempColor.rgba[ALPHA_IDX] >= 1.) {
				break;
			}

			initv4(currentColor, tempColor.rgba);

			current_depth = it->depth;

			it2 = it + 1;
			initv4(slopeMax, FLT_MAX);
			initv4(slopeMin, -FLT_MAX);
			initv4(colorOffset, 0.);

			while( it2 != end )
			{
				addvv4(colorOffset, it2->rgba);
				d = it2->depth - current_depth;

				if( d > FLT_EPSILON )
				{
					d = 1. / d;
					slopeMin[0] = maxx( slopeMin[0], ( colorOffset[0] - internalData->compression ) * d );
					slopeMax[0] = minn( slopeMax[0], ( colorOffset[0] + internalData->compression ) * d );
					slopeMin[1] = maxx( slopeMin[1], ( colorOffset[1] - internalData->compression ) * d );
					slopeMax[1] = minn( slopeMax[1], ( colorOffset[1] + internalData->compression ) * d );
					slopeMin[2] = maxx( slopeMin[2], ( colorOffset[2] - internalData->compression ) * d );
					slopeMax[2] = minn( slopeMax[2], ( colorOffset[2] + internalData->compression ) * d );
					slopeMin[3] = maxx( slopeMin[3], ( colorOffset[3] - internalData->compression ) * d );
					slopeMax[3] = minn( slopeMax[3], ( colorOffset[3] + internalData->compression ) * d );

					if( slopeMax[0] <= slopeMin[0] ||
						slopeMax[1] <= slopeMin[1] ||
						slopeMax[2] <= slopeMin[2] ||
						slopeMax[3] <= slopeMin[3] )
					{
						subvv4(colorOffset, it2->rgba);
						break;
					}
				}
				it2++;
			}

			it = it2 - 1;
			new_depth = it->depth;
			tempColor.depth = new_depth;
			addvv4(tempColor.rgba, currentColor, colorOffset);

			if(tempColor.rgba[ALPHA_IDX] >= 1.f) {
				tempColor.rgba[ALPHA_IDX] = 1.f;

				deepPixel pixel;
				pixel.depth = tempColor.depth;
				tempColor.rgba[RED_IDX] 	= maxx(tempColor.rgba[RED_IDX], 0.f);
				tempColor.rgba[GREEN_IDX] 	= maxx(tempColor.rgba[GREEN_IDX], 0.f);
				tempColor.rgba[BLUE_IDX] 	= maxx(tempColor.rgba[BLUE_IDX], 0.f);
				tempColor.rgba[ALPHA_IDX] 	= clampf( 0.f, tempColor.rgba[ALPHA_IDX], 1.f);
				subvv4(pixel.rgba, tempColor.rgba, currentColor);
				divvv4(pixel.rgba, (1.f - currentColor[ALPHA_IDX]));
				colorDepth.push_back(pixel);
				break;
			}

			if (it == end - 1) {
				deepPixel pixel;

				colorrgba tmp;
				tmp[RED_IDX] 	= maxx(tempColor.rgba[RED_IDX], 0.f);
				tmp[GREEN_IDX] 	= maxx(tempColor.rgba[GREEN_IDX], 0.f);
				tmp[BLUE_IDX] 	= maxx(tempColor.rgba[BLUE_IDX], 0.f);
				tmp[ALPHA_IDX] 	= clampf( 0.f, tempColor.rgba[ALPHA_IDX], 1.f);
				pixel.depth 	= tempColor.depth;

				subvv4( pixel.rgba, tmp, currentColor );
				divvv4( pixel.rgba, (1.f - currentColor[ALPHA_IDX]) );
				colorDepth.push_back(pixel);
				break;
			}
			deepPixel pixel;
			tempColor.rgba[RED_IDX] 	= maxx(tempColor.rgba[RED_IDX], 0.f);
			tempColor.rgba[GREEN_IDX] 	= maxx(tempColor.rgba[GREEN_IDX], 0.f);
			tempColor.rgba[BLUE_IDX] 	= maxx(tempColor.rgba[BLUE_IDX], 0.f);
			tempColor.rgba[ALPHA_IDX] 	= clampf( 0.f, tempColor.rgba[ALPHA_IDX], 1.f);
			pixel.depth 				= tempColor.depth;

			subvv4( pixel.rgba, tempColor.rgba, currentColor );
			divvv4( pixel.rgba, (1.f - currentColor[ALPHA_IDX]) );
			colorDepth.push_back(pixel);
		}
    }

    if (depthPixels_append && !depthPixels_append->empty()){
    	colorDepth.insert(colorDepth.end(), depthPixels_append->begin(), depthPixels_append->end());
    }

    depthPixels.clear();
    depthPixels = colorDepth;

    return;
}

using namespace std;
using namespace Imf;
using namespace Imath;
using namespace Iex;

bool
deepImage::write()
{

    Box2i dataWindow;
    Box2i displayWindow;
    Array2D< float* > dataZ;
    Array2D< half* > dataR;
    Array2D< half* > dataG;
    Array2D< half* > dataB;
    Array2D< half* > dataA;

    Array2D< unsigned int > sampleCount;

    sampleCount.resizeErase( internalData->height, internalData->width);
    dataZ.resizeErase( internalData->height, internalData->width);
    dataR.resizeErase( internalData->height, internalData->width);
    dataG.resizeErase( internalData->height, internalData->width);
    dataB.resizeErase( internalData->height, internalData->width);
    dataA.resizeErase( internalData->height, internalData->width);

    Imf_2_1::Header header(internalData->width, internalData->height, 1, V2f (0, 0), 1, INCREASING_Y, RLE_COMPRESSION);

    M44f proj_matrix;
    int count = 0;
    for (int i = 0; i < 4;++i)
    	for (int j = 0; j < 4;++j)
    		proj_matrix[i][j] = internalData->cameraMatrix[count++];

    // FIXME: add support for orthos cameras
    //header.insert ("ortho", FloatAttribute ((float)m_ortho));
    //if (m_ortho)
    	//header.insert ("projection_factor", FloatAttribute ((float)m_ortho_proj));
    //else
    	//header.insert ("projection_factor", FloatAttribute ((float)m_fov));
    header.insert ("camera_mx", M44fAttribute (proj_matrix));
    header.insert ("deep_compression", FloatAttribute (internalData->compression));


    header.channels().insert("R", Channel(HALF));
    header.channels().insert("G", Channel(HALF));
    header.channels().insert("B", Channel(HALF));
    header.channels().insert("A", Channel(HALF));

    header.channels().insert("Z", Channel(FLOAT));


    header.setType(DEEPSCANLINE);


    DeepFrameBuffer frameBuffer;


    frameBuffer.insertSampleCountSlice (Slice (UINT,
                                       (char *) (&sampleCount[0][0]),
                                       sizeof (unsigned int) * 1,           // xStride
                                       sizeof (unsigned int) * internalData->width));     // yStride
    frameBuffer.insert ("Z",
                      DeepSlice (FLOAT,
                      (char *) (&dataZ[0][0]),
                      sizeof (float *) * 1,                 // xStride for pointer array
                      sizeof (float *) * internalData->width,             // yStride for pointer array
                      sizeof (float) * 1));                 // stride for Z data sample
  	frameBuffer.insert ("R",
                      DeepSlice (HALF,
                      (char *) (&dataR[0][0]),
                      sizeof (half *) * 1,                 // xStride for pointer array
                      sizeof (half *) * internalData->width,             // yStride for pointer array
                      sizeof (half) * 1));                 // stride for O data sample
    frameBuffer.insert ("G",
                      DeepSlice (HALF,
                      (char *) (&dataG[0][0]),
                      sizeof (half *) * 1,                 // xStride for pointer array
                      sizeof (half *) * internalData->width,             // yStride for pointer array
                      sizeof (half) * 1));                 // stride for O data sample
  	frameBuffer.insert ("B",
                      DeepSlice (HALF,
                      (char *) (&dataB[0][0]),
                      sizeof (half *) * 1,                 // xStride for pointer array
                      sizeof (half *) * internalData->width,             // yStride for pointer array
                      sizeof (half) * 1));                 // stride for O data sample

  	frameBuffer.insert ("A",
                      DeepSlice (HALF,
                      (char *) (&dataA[0][0]),
                      sizeof (half *) * 1,                 // xStride for pointer array
                      sizeof (half *) * internalData->width,             // yStride for pointer array
                      sizeof (half) * 1));                 // stride for O data sample

    DeepScanLineOutputFile file(internalData->fileName.c_str(), header);

  	file.setFrameBuffer(frameBuffer);

	std::vector<deepPixel> list;
	for ( int i = 0; i < internalData->height;++i){
    	for (int j = 0; j < internalData->width;++j){
    		int imgIdx = coord2index(j, i);
    		std::vector<deepPixel>& list = internalData->pixelList[imgIdx];
            if (list.empty()) {
                sampleCount[i][j]= 0;
                continue;
            }

            float *pixel_dataZ = new float[list.size()];
            half *pixel_dataR = new half[list.size()];
            half *pixel_dataG = new half[list.size()];
            half *pixel_dataB = new half[list.size()];
            half *pixel_dataA = new half[list.size()];

            dataZ[i][j] = pixel_dataZ;
            dataR[i][j] = pixel_dataR;
            dataG[i][j] = pixel_dataG;
            dataB[i][j] = pixel_dataB;
            dataA[i][j] = pixel_dataA;

            size_t list_count = 0;
            for (unsigned int s = 0; s  < list.size(); ++s) {
                pixel_dataZ[s] = (float)list[s].depth;
                pixel_dataR[s] = (half)list[s].rgba[RED_IDX] ;
                pixel_dataG[s] = (half)list[s].rgba[GREEN_IDX];
                pixel_dataB[s] = (half)list[s].rgba[BLUE_IDX];
                pixel_dataA[s] = (half)list[s].rgba[ALPHA_IDX];
                list_count++;
            }

            sampleCount[i][j]= list_count;
    	}
    	file.writePixels(1);
    }

	for ( int i = 0; i < internalData->height;++i){
		for (int j = 0; j < internalData->width;++j){
			if (sampleCount[i][j] == 0)
				continue;
			delete[] dataZ[i][j];
			delete[] dataR[i][j];
			delete[] dataG[i][j];
			delete[] dataB[i][j];
			delete[] dataA[i][j];
		}
	}
	return true;
}

bool
deepImage::read_append(const std::string &filename)
{
	deepImage img;
	img.read(filename);
	int imgW, imgH;
	img.getSize(imgW, imgH);

	if (internalData->width != imgW && internalData->height != imgH){
		return false;
	}

	for(int y = 0; y < imgH; ++y){
		for(int x = 0; x < imgW; ++x){
    		int imgIdx = coord2index(x, y);
    		std::vector<deepPixel>& list1 = internalData->pixelList[imgIdx];
    		std::vector<deepPixel>& list2 = img.internalData->pixelList[imgIdx];
    		doPixelIntegration(list1, &list2, true);
		}
	}

	return true;
}

bool
deepImage::read(const std::string &filename)
{
    Box2i displayWindow;
    Box2i dataWindow;
    Array2D< float* > dataZ;
    Array2D< half* > dataR;
    Array2D< half* > dataG;
    Array2D< half* > dataB;
    Array2D< half* > dataA;
    Array2D< unsigned int > sampleCount;

    try{
		DeepScanLineInputFile file(filename.c_str());
		const Header& header = file.header();

	    int bucket_size = 32;
	    bool ortho = false;
	    double projection_factor = 1.0;


	    const FloatAttribute* bucket_size_attr = header.findTypedAttribute<FloatAttribute>("bucket_size");
	    if (bucket_size_attr)
	    	bucket_size = (int)bucket_size_attr->value();

	    const FloatAttribute* ortho_attr = header.findTypedAttribute<FloatAttribute>("otho");
	    if (ortho_attr)
	    	ortho = (bool)ortho_attr->value();

	    const FloatAttribute* projection_factor_attr = header.findTypedAttribute<FloatAttribute>("projection_factor");
	    if (projection_factor_attr)
	    	projection_factor = (bool)projection_factor_attr->value();


	    const FloatAttribute* deep_comp_attr = header.findTypedAttribute<FloatAttribute>("deep_compression");
	    if (deep_comp_attr)
	    	internalData->compression = deep_comp_attr->value();

	    dataWindow 		= header.dataWindow();
	    displayWindow 	= header.displayWindow();
	    int width 		= dataWindow.max.x - dataWindow.min.x + 1;
	    int height 		= dataWindow.max.y - dataWindow.min.y + 1;

	    sampleCount.resizeErase(height, width);
	    dataZ.resizeErase(height, width);
	    dataR.resizeErase(height, width);
	    dataG.resizeErase(height, width);
	    dataB.resizeErase(height, width);
	    dataA.resizeErase(height, width);

	    DeepFrameBuffer frameBuffer;

	    frameBuffer.insertSampleCountSlice (Slice (UINT,
											(char *) (&sampleCount[0][0]
											- dataWindow.min.x
											- dataWindow.min.y * width),
											sizeof (unsigned int) * 1,
											// xStride
											sizeof (unsigned int) * width));
											// yStride

	    frameBuffer.insert ("Z",
							DeepSlice (FLOAT,
							(char *) (&dataZ[0][0]
							- dataWindow.min.x
							- dataWindow.min.y * width),
							sizeof (float *) * 1,
							// xStride for pointer array
							sizeof (float *) * width,
							// yStride for poi
							sizeof (float) * 1)); // stride for Z data sample

	    frameBuffer.insert ("R",
							DeepSlice (HALF,
							(char *) (&dataR[0][0]
							- dataWindow.min.x
							- dataWindow.min.y * width),
							sizeof (half *) * 1,
							// xStride for pointer array
							sizeof (half *) * width,
							// yStride for pointer array
							sizeof (half) * 1));

	    frameBuffer.insert ("G",
							DeepSlice (HALF,
							(char *) (&dataG[0][0]
							- dataWindow.min.x
							- dataWindow.min.y * width),
							sizeof (half *) * 1,
							// xStride for pointer array
							sizeof (half *) * width,
							// yStride for pointer array
							sizeof (half) * 1));

	    frameBuffer.insert ("B",
							DeepSlice (HALF,
							(char *) (&dataB[0][0]
							- dataWindow.min.x
							- dataWindow.min.y * width),
							sizeof (half *) * 1,
							// xStride for pointer array
							sizeof (half *) * width,
							// yStride for pointer array
							sizeof (half) * 1));

	    frameBuffer.insert ("A",
							DeepSlice (HALF,
							(char *) (&dataA[0][0]
							- dataWindow.min.x
							- dataWindow.min.y * width),
							sizeof (half *) * 1,
							// xStride for pointer array
							sizeof (half *) * width,
							// yStride for pointer array
							sizeof (half) * 1));

	    file.setFrameBuffer(frameBuffer);

	    file.readPixelSampleCounts(dataWindow.min.y, dataWindow.max.y);

	    // Init the sucker
	    set_size(width, height);
	    //set_camera_matrix(mx);
	    //set_params(width, height, ortho, projection_factor, mx);

		for ( int i = 0; i < internalData->height;++i){
	    	for (int j = 0; j < internalData->width;++j){
	    		register int sc = sampleCount[i][j];
	    		dataZ[i][j] = new float[sc];
	    		dataR[i][j] = new half[sc];
	    		dataG[i][j] = new half[sc];
	    		dataB[i][j] = new half[sc];
	    		dataA[i][j] = new half[sc];
	    	}
		}

		file.readPixels(dataWindow.min.y, dataWindow.max.y);

		for ( int i = 0; i < internalData->height;++i){
	    	for (int j = 0; j < internalData->width;++j){
	    		int sc = sampleCount[i][j];
	    		std::vector<deepPixel> vals;
	    		for (int k = 0; k < sc; ++k){
	    			deepPixel deep_val;
					deep_val.depth = dataZ[i][j][k];
					deep_val.rgba[0] = dataR[i][j][k];
					deep_val.rgba[1] = dataG[i][j][k];
					deep_val.rgba[2] = dataB[i][j][k];
					deep_val.rgba[3] = dataA[i][j][k];
					vals.push_back(deep_val);
	    		}

	    		delete [] dataZ[i][j];
	    		delete [] dataR[i][j];
	    		delete [] dataG[i][j];
	    		delete [] dataB[i][j];
	    		delete [] dataA[i][j];

	    		set_pixel_color_list(vals, j, i);
	    	}
		}
    } catch( ... ) {
    	std::cerr << "Deep_image : Cannot open image file " << filename << std::endl;
    	return false;
    }
    internalData->valid = true;
    return true;
}

void
deepImage::set_pixel_color_list(std::vector<deepPixel>& pixel, const int &x, const int &y)
{
	if (!internalData->valid)
		return;
	int index = coord2index(x, y);

	if (index >= internalData->pixelList.size() || internalData->width < 0 || internalData->height < 0){
		std::cout << "Waring: deepImage::setPixelDepth : You are about to write outside image boundaries : " << x << "," << y <<  std::endl;
		return;
	}
	internalData->pixelList[index] = pixel;
}

bool
deepImage::get_pixel_color_list(std::vector<deepPixel> &list, int x, int y)
{
	list.clear();
	if (!internalData->valid) return false;

	size_t idx = coord2index(x, y);
	list.insert(list.begin(), internalData->pixelList[idx].begin(), internalData->pixelList[idx].end());
	return !list.empty();
}

void
deepImage::set_size(int w, int h)
{
	internalData->width  = w;
	internalData->height = h;
	internalData->valid  = true;
	internalData->pixelList.resize(w*h);
}

bool
deepImage::compose_depth_values(colorrgba color, std::vector<deepPixel> &list, bool reorder_list)
{
    initv4(color, 0.f);
    if (list.empty()) {
        return false;
    }
    if (reorder_list) {
        std::sort(list.begin(), list.end(), sort_pixel_depth);
    }

    colorrgba tmp_color;

    for (unsigned int l = 0; l < list.size(); ++l) {
    	initv4(tmp_color, list[l].rgba);
    	mulv4f(tmp_color, (1. - color[3]));
        addvv4(color, tmp_color);
        if (color[3] >= (1.0f - C_EPSILON)) {
            color[3] = 1.;
            break;
        }
    }

    return true;
}

float*
deepImage::to2DBufferFloat()
{
	if (!internalData->valid)
		return NULL;

	const int w = internalData->width;
	const int h = internalData->height;

	float* buffer = (float*)malloc(sizeof(float) * w * h * 4);
	float* tmpbuffer = buffer;
	colorrgba color;

	for ( int i = 0; i < h;++i){
    	for (int j = 0; j < w;++j){
    		int imgIdx = coord2index(j, i);
    		std::vector<deepPixel>& deepList = internalData->pixelList[imgIdx];
    		compose_depth_values(color, deepList, true);
    		initv4(tmpbuffer, color);
    		tmpbuffer += 4;
    	}
	}

	return buffer;
}

char*
deepImage::to2DBufferChar()
{
	if (!internalData->valid)
		return NULL;

	const int w = internalData->width;
	const int h = internalData->height;

	char* buffer = (char*)malloc(sizeof(char) * w * h * 4);
	char* tmpbuffer = buffer;
	colorrgba color;

	for ( int i = 0; i < h;++i){
    	for (int j = 0; j < w;++j){
    		int imgIdx = coord2index(j, i);
    		std::vector<deepPixel>& deepList = internalData->pixelList[imgIdx];
    		compose_depth_values(color, deepList, true);
    		tmpbuffer[0] = char(color[0] * 255.0);
    		tmpbuffer[1] = char(color[1] * 255.0);
    		tmpbuffer[2] = char(color[2] * 255.0);
    		tmpbuffer[3] = char(color[3] * 255.0);
    		tmpbuffer += 4;
    	}
	}

	return buffer;
}
