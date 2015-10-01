#include "cubemap.h"
#include <cstring>

/// faceRes gives face resolution.  Faces are square.  nchans gives
/// the number of channels in each pixel, and defaultPix gives the
/// values of the channels which will be used when reset() is called.
cubemap::cubemap(const int& faceRes, const int& nchans, const float* defaultPix)
                    : m_res(faceRes),
                    m_nchans(nchans),
                    m_faceSize(nchans*faceRes*faceRes)
                    {

    m_pixels.reserve(m_faceSize*Face_end);
    m_defaultPixels.reserve(m_faceSize*Face_end);
    m_directions.reserve(Face_end*faceRes*faceRes);
    m_pixelSizes.reserve(m_faceSize);
    // Cache direction vectors
    for(int face = 0; face < Face_end; ++face)
    {
        for(int iv = 0; iv < m_res; ++iv)
        for(int iu = 0; iu < m_res; ++iu)
        {
            // directions of pixels go through pixel centers
            float u = (0.5f + iu)/faceRes*2.0f - 1.0f;
            float v = (0.5f + iv)/faceRes*2.0f - 1.0f;
            m_directions[(face*m_res + iv)*m_res + iu] =
                direction(face, u, v);
        }
    }
    for(int iv = 0; iv < m_res; ++iv)
    for(int iu = 0; iu < m_res; ++iu)
    {
        float u = (0.5f + iu)/faceRes*2.0f - 1.0f;
        float v = (0.5f + iv)/faceRes*2.0f - 1.0f;
        m_pixelSizes[iv*m_res + iu] = 1.0f/V3f(u,v,1).length2();
    }
    float* pix = m_defaultPixels.array;
    for(int i = 0, iend = size(); i < iend; ++i, pix += m_nchans)
        for(int c = 0; c < m_nchans; ++c)
            pix[c] = defaultPix[c];

}

/// Reset buffer to default (non-rendered) state.
void cubemap::reset()
{
    memcpy(m_pixels.array, m_defaultPixels.array,
           sizeof(float)*size()*m_nchans);
}

/// Get raw data store for face
float* cubemap::face(const int& which)
{
    assert(which >= Face_begin && which < Face_end);
    return &m_pixels[0] + which*m_faceSize;
}

/// Get raw data store for face
const float* cubemap::face(const int& which) const
{
    assert(which >= Face_begin && which < Face_end);
    return &m_pixels[0] + which*m_faceSize;
}



/// Get a neighbouring face in u direction
///
/// \param faceIdx - current face index
/// \param side - which side to look (0 == left, 1 == right)
///
// +---+---+---+  +---+---+---+  +---+---+---+
// |+z |+x |-z |  |-x |+y |+x |  |-x |+z |+x |
// +---+---+---+  +---+---+---+  +---+---+---+
//
// +---+---+---+  +---+---+---+  +---+---+---+
// |-z |-x |+z |  |-x |-y |+x |  |+x |-z |-x |
// +---+---+---+  +---+---+---+  +---+---+---+
//
cubemap::Face cubemap::neighbourU(const int& faceIdx, const int& side)
{
    static Face neighbourArray[6][2] = {
        {Face_zp, Face_zn}, {Face_xn, Face_xp}, {Face_xn, Face_xp},
        {Face_zn, Face_zp}, {Face_xn, Face_xp}, {Face_xp, Face_xn}
    };
    return neighbourArray[faceIdx][side];
}

/// Get a neighbouring face in v direction
///
/// \param faceIdx - current face index
/// \param side - which side to look (0 == bottom, 1 == top)
///
// +---+   +---+   +---+   +---+   +---+   +---+
// |+y |   |-z |   |+y |   |+y |   |+z |   |+y |
// +---+   +---+   +---+   +---+   +---+   +---+
// |+x |   |+y |   |+z |   |-x |   |-y |   |-z |
// +---+   +---+   +---+   +---+   +---+   +---+
// |-y |   |+z |   |-y |   |-y |   |-z |   |-y |
// +---+   +---+   +---+   +---+   +---+   +---+
cubemap::Face cubemap::neighbourV(const int& faceIdx, const int& side)
{
    static Face neighbourArray[6][2] = {
        {Face_yn, Face_yp}, {Face_zp, Face_zn}, {Face_yn, Face_yp},
        {Face_yn, Face_yp}, {Face_zn, Face_zp}, {Face_yn, Face_yp}
    };
    return neighbourArray[faceIdx][side];
}

/// Get index of face which direction p sits inside.
cubemap::Face cubemap::faceIndex(const V3f& p)
{
    V3f absp = V3f(fabs(p.x), fabs(p.y), fabs(p.z));
    if(absp.x >= absp.y && absp.x >= absp.z)
        return (p.x > 0) ? cubemap::Face_xp : cubemap::Face_xn;
    else if(absp.y >= absp.x && absp.y >= absp.z)
        return (p.y > 0) ? cubemap::Face_yp : cubemap::Face_yn;
    else
    {
        assert(absp.z >= absp.x && absp.z >= absp.y);
        return (p.z > 0) ? cubemap::Face_zp : cubemap::Face_zn;
    }
}

/// Get coordinates on face
///
/// The coordinates are in the range -1 <= u,v <= 1, if faceIdx is
/// obtained using the faceIndex function.  Coordinates outside this
/// range are legal, as long as p has nonzero component in the
/// direction of the normal of the face.
///
/// \param faceIdx - index of current face
/// \param p - position (may lie outside cone of current face)
void cubemap::faceCoords(const int& faceIdx, const V3f& p, float& u, float& v)
{
    V3f P = canonicalFaceCoords(faceIdx, p);
    assert(P.z != 0);
    float zinv = 1.0/P.z;
    u = P.x*zinv;
    v = P.y*zinv;
}

/// Compute dot product of vec with face normal on given face
float cubemap::dotFaceNormal(const int &faceIdx, const V3f& vec)
{
    assert(faceIdx < Face_end && faceIdx >= Face_begin);
    return (faceIdx < 3) ? vec[faceIdx] : -vec[faceIdx-3];
}

/// Compute face normal
V3f cubemap::faceNormal(const int& faceIdx)
{
    static V3f normals[6] = {
        V3f(1,0,0), V3f(0,1,0), V3f(0,0,1),
        V3f(-1,0,0), V3f(0,-1,0), V3f(0,0,-1)
    };
    return normals[faceIdx];
}

/// Get direction vector for pixel on given face.
V3f cubemap::rayDirection(const int &faceIdx, const int& u, const int& v) const
{
    return m_directions[(faceIdx*m_res + v)*m_res + u];
}

/// Return relative size of pixel.
///
/// Compared to a pixel in the middle of the cube face, pixels in the
/// corners of the cube have a smaller angular size.  We must take
/// this into account when integrating the radiosity.
///
/// \param u,v - face coordinates
float cubemap::pixelSize(const int& u, const int& v) const
{
    return m_pixelSizes[m_res*v + u];
}

/// Reorder vector components into "canonical face coordinates".
///
/// The canonical coordinates correspond to the coordinates on the +z
/// face.  If we let the returned vector be q then (q.x, q.y)
/// correspond to the face (u, v) coordinates, and q.z corresponds to
/// the signed depth out from the face.
V3f cubemap::canonicalFaceCoords(const int &faceIdx, const V3f& p)
{
    switch(faceIdx)
    {
        case Face_xp: return V3f(-p.z,  p.y, p.x);
        case Face_xn: return V3f(-p.z, -p.y, p.x);
        case Face_yp: return V3f( p.x, -p.z, p.y);
        case Face_yn: return V3f(-p.x, -p.z, p.y);
        case Face_zp: return V3f( p.x,  p.y, p.z);
        case Face_zn: return V3f( p.x, -p.y, p.z);
        default: assert(0 && "invalid face"); return V3f();
    }
}

/// Get direction vector for position on a given face.
///
/// Roughly speaking, this is the opposite of the faceCoords function
V3f cubemap::direction(const int& faceIdx, const float& u, const float& v)
{
    switch(faceIdx)
    {
        case Face_xp: return V3f( 1, v,-u).normalized();
        case Face_yp: return V3f( u, 1,-v).normalized();
        case Face_zp: return V3f( u, v, 1).normalized();
        case Face_xn: return V3f(-1, v, u).normalized();
        case Face_yn: return V3f( u,-1, v).normalized();
        case Face_zn: return V3f(-u, v,-1).normalized();
        default: assert(0 && "unknown face"); return V3f();
    }
}

