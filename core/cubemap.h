#ifndef CUBEMAP_H
#define CUBEMAP_H

#include <common/containers.h>

#include <OpenEXR/ImathVec.h>
#include <OpenEXR/ImathMath.h>
#include <OpenEXR/ImathBox.h>
#include <OpenEXR/ImathColor.h>
#include <OpenEXR/ImathFun.h>

using namespace Imath;

class cubemap
{
    public:
        /// Identifiers for each cube face direction
        enum Face
        {
            Face_xp = 0, ///< x+
            Face_yp, ///< y+
            Face_zp, ///< z+
            Face_xn, ///< x-
            Face_yn, ///< y-
            Face_zn, ///< z-
            Face_end,
            Face_begin = Face_xp
        };

        cubemap(const int& faceRes, const int& nchans, const float* defaultPix);

        void reset();

        float* face(const int& which);
        const float* face(const int& which) const;


        static Face faceIndex(const V3f& p);

        static Face neighbourU(const int& faceIdx, const int& side);

        static Face neighbourV(const int& faceIdx, const int& side);

        static void faceCoords(const int& faceIdx, const V3f& p, float& u, float& v);

        static float dotFaceNormal(const int& faceIdx, const V3f& vec);

        static V3f faceNormal(const int& faceIdx);

        V3f rayDirection(const int& faceIdx, const int& u, const int& v) const;

        float pixelSize(const int& u, const int& v) const;

        static V3f canonicalFaceCoords(const int& faceIdx, const V3f& p);

        /// Face side resolution
        int res() const { return m_res; }
        /// Number of channels per pixel
        int nchans() const { return m_nchans; }
        /// Total size of all faces in number of texels
        int size() const { return Face_end*m_res*m_res; }

    private:

        static V3f direction(const int& faceIdx, const float& u, const float& v);

        /// Square face resolution
        int m_res;
        /// Number of channels per pixel
        int m_nchans;
        /// Number of floats needed to store a face
        int m_faceSize;
        /// Pixel face storage
        CArray<float> m_pixels;
        CArray<float> m_defaultPixels;
        /// Storage for pixel ray directions
        CArray<V3f> m_directions;
        /// Pixels on a unit cube are not all equal in angular size
        CArray<float> m_pixelSizes;
};



#endif
