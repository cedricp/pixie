#ifndef GLOBAL_ILLUM_H
#define GLOBAL_ILLUM_H

#include "cubemap.h"
#include <algorithm>
#include <vector>

inline float dot(Imath::V3f a, Imath::V3f b)
{
    return a^b;
}

//------------------------------------------------------------------------------
/// Integrator for ambient occlusion.
///
/// The job of an integrator class is to save the microrasterized data
/// somewhere (presumably in a microbuffer) and integrate it at the end of the
/// rasterization to give the final illumination value.
class OcclusionIntegrator
{
    public:
        /// Create integrator with given resolution of the environment map
        /// faces.
        OcclusionIntegrator(int faceRes)
            : m_buf(faceRes, 1, defaultPixel()),
            m_face(0)
        {
            clear();
        }

        /// Get direction of the ray
        Imath::V3f rayDirection(const int& iface, const int& u, const int& v)
        {
            return m_buf.rayDirection(iface, u, v);
        }

        /// Get at the underlying buffer
        const cubemap& microBuf()
        {
            return m_buf;
        }

        /// Get desired resolution of environment map faces
        int res()
        {
            return m_buf.res();
        }

        /// Reset buffer to default state
        void clear()
        {
            m_buf.reset();
        };

        /// Set extra data (eg, radiosity) associated with the current point
        /// being shaded.
        void setPointData(const float*) { }

        /// Set the face to which subsequent calls of addSample will apply
        void setFace(const int& iface)
        {
            m_face = m_buf.face(iface);
        };

        /// Add a rasterized sample to the current face
        ///
        /// \param u,v - coordinates of face
        /// \param distance - distance to sample
        /// \param coverage - estimate of pixel coverage due to this sample
        void addSample(const int& u, const int& v, const float& distance, const float& coverage)
        {
            float* pix = m_face + (v*m_buf.res() + u) * m_buf.nchans();
            // There's more than one way to combine the coverage.
            //
            // 1) The usual method of compositing.  This assumes that
            // successive layers of geometry are uncorrellated so that each
            // attenuates the layer before, but a bunch of semi-covered layers
            // never result in full opacity.
            //
            // 1 - (1 - o1)*(1 - o2);
            //
            // 2) Add the opacities (and clamp to 1 at the end).  This is more
            // appropriate if we assume that we have adjacent non-overlapping
            // surfels.
            pix[0] += coverage;
        }

        /// Compute ambient occlusion based on previously sampled scene.
        ///
        /// This is one minus the zero-bounce light coming from infinity to the
        /// point.
        ///
        /// N is the shading normal; coneAngle is the angle over which to
        /// consider the occlusion in radians.  If coneAngle is not PI/2
        /// radians, the usual cos(theta) weighting is adjusted to
        /// (cos(theta) - cos(coneAngle)) so that it falls continuously to
        /// zero when theta == coneAngle
        float occlusion(const Imath::V3f& N, const float& coneAngle) const
        {
            // Integrate over face to get occlusion.
            float occ = 0;
            float totWeight = 0;
            float cosConeAngle = cos(coneAngle);
            for(int f = cubemap::Face_begin; f < cubemap::Face_end; ++f)
            {
                const float* face = m_buf.face(f);
                for(int iv = 0; iv < m_buf.res(); ++iv)
                for(int iu = 0; iu < m_buf.res(); ++iu, face += m_buf.nchans())
                {
                    float d = dot(m_buf.rayDirection(f, iu, iv), N) -
                              cosConeAngle;
                    if(d > 0)
                    {
                        d *= m_buf.pixelSize(iu, iv);
                        // Accumulate light coming from infinity.
                        occ += d*std::min(1.0f, face[0]);
                        totWeight += d;
                    }
                }
            }
            if(totWeight != 0)
                occ /= totWeight;
            return occ;
        }

    private:
        static float* defaultPixel()
        {
            static float def[1] = {0};
            return def;
        }

        cubemap m_buf;
        float* m_face;
};

class RadiosityIntegrator
{
    public:
        /// Create integrator with given resolution of the environment map
        /// faces.
        RadiosityIntegrator(int faceRes)
            : m_buf(faceRes, 5, defaultPixel()),
            m_face(0),
            m_currRadiosity(0)
        {
            clear();
        }

        /// Get direction of the ray
        Imath::V3f rayDirection(const int &iface, const int& u, const int& v)
        {
            return m_buf.rayDirection(iface, u, v);
        }

        /// Get at the underlying buffer
        const cubemap& getcubemap()
        {
            return m_buf;
        }

        /// Get desired resolution of environment map faces
        int res()
        {
            return m_buf.res();
        }

        /// Reset buffer to default state
        void clear()
        {
            m_buf.reset();
        };

        /// Set extra data (eg, radiosity) associated with the current point
        /// being shaded.
        void setPointData(const float* radiosity)
        {
            m_currRadiosity = Imath::C3f(radiosity[0], radiosity[1], radiosity[2]);
        }

        /// Set the face to which subsequent calls of addSample will apply
        void setFace(const int& iface)
        {
            m_face = m_buf.face(iface);
        };

        /// Add a rasterized sample to the current face
        ///
        /// \param u,v - coordinates of face
        /// \param distance - distance to sample
        /// \param coverage - estimate of pixel coverage due to this sample
        void addSample(const int &u, const int &v, const float &distance, const float& coverage)
        {
            float* pix = m_face + (v*m_buf.res() + u) * m_buf.nchans();
            // TODO: Eventually remove dist if not needed
            float& currDist = pix[0];
            float& currCover = pix[1];
            Imath::C3f& radiosity = *reinterpret_cast<Imath::C3f*>(pix + 2);
            if(distance < currDist)
                currDist = distance;
            if(currCover < 1)
            {
                if(currCover + coverage <= 1)
                {
                    radiosity += coverage*m_currRadiosity;
                    currCover += coverage;
                }
                else
                {
                    radiosity += (1 - currCover)*m_currRadiosity;
                    currCover = 1;
                }
            }
        }

        /// Integrate radiosity based on previously sampled scene.
        ///
        /// N is the shading normal; coneAngle is the angle over which to
        /// consider the occlusion in radians.  If coneAngle is not PI/2
        /// radians, the usual cos(theta) weighting is adjusted to
        /// (cos(theta) - cos(coneAngle)) so that it falls continuously to
        /// zero when theta == coneAngle
        ///
        /// If occlusion is non-null, the amount of occlusion will also be
        /// computed and stored.
        Imath::C3f radiosity(const Imath::V3f& N, const float& coneAngle, float* occlusion = 0) const
        {
            // Integrate incoming light with cosine weighting to get outgoing
            // radiosity
            Imath::C3f rad(0);
            float totWeight = 0;
            float cosConeAngle = cos(coneAngle);
            float occ = 0;
            for(int f = cubemap::Face_begin; f < cubemap::Face_end; ++f)
            {
                float* face = (float*)m_buf.face(f);
                for(int iv = 0; iv < m_buf.res(); ++iv)
                for(int iu = 0; iu < m_buf.res(); ++iu, face += m_buf.nchans())
                {
                    float d = dot(m_buf.rayDirection(f, iu, iv), N) -
                              cosConeAngle;
                    if(d > 0)
                    {
                        d *= m_buf.pixelSize(iu, iv);
                        Imath::C3f& radiosity = *(Imath::C3f*)(face + 2);
                        rad += d*radiosity;
                        occ += d*face[1];
                        totWeight += d;
                    }
                }
            }
            if(totWeight != 0)
            {
                occ /= totWeight;
                rad = (1.0f/totWeight) * rad;
            }
            if(occlusion)
                *occlusion = occ;
            return rad;
        }

    private:
        static float* defaultPixel()
        {
            // depth, foreground_coverage, foreground_rgb, background_rgb
            static float def[] = {FLT_MAX, 0, 0, 0, 0, 0, 0, 0};
            return def;
        }

        cubemap m_buf;
        float* m_face;
        Imath::C3f m_currRadiosity;
};

struct pointOctreeNode
{
    pointOctreeNode()
        : bound(),
        center(0),
        boundRadius(0),
        aggP(0),
        aggN(0),
        aggR(0),
        aggCol(0),
        npoints(0)
    {
        children[0] = children[1] = children[2] = children[3] = 0;
        children[4] = children[5] = children[6] = children[7] = 0;
    }

    /// Data derived from octree bounding box
    Imath::Box3f bound;
    Imath::V3f center;
    float boundRadius;
    // Crude aggregate values for position, normal and radius
    Imath::V3f aggP;
    Imath::V3f aggN;
    float aggR;
    Imath::C3f aggCol;
    // Child nodes, to be indexed as children[z][y][x]
    pointOctreeNode* children[8];
    // bool used;
    /// Number of child points for the leaf node case
    int npoints;
    // Collection of points in leaf.
    CArray<float> data;
};

inline bool sphereOutsideCone(const Imath::V3f& p, const float& plen2, const float& r,
                              const Imath::V3f& n, const float &cosConeAngle, const float& sinConeAngle)
{
    // The actual expressions used here are an optimized version which does
    // the same thing, but avoids calling sqrt().  This makes a difference
    // when this is the primary culling test and you're iterating over lots of
    // points - you need to reject invalid points as fast as possible.
    float x = plen2 - r*r;
    // special case - if the sphere covers the origin, it must intersect the
    // cone.
    if(x < 0)
        return false;
    // Special case - if sphere angle and cone angle add to >= 180 degrees, the
    // sphere and cone must intersect.
    if(cosConeAngle < 0 && x < plen2*cosConeAngle*cosConeAngle)
        return false;
    // General case
    float lhs = x*cosConeAngle * cosConeAngle;
    float rhs = dot(p, n) + r * sinConeAngle;

    return copysign(lhs, cosConeAngle) > copysign(rhs*rhs, rhs);

}


/// Find real solutions of the quadratic equation a*x^2 + b*x + c = 0.
///
/// The equation is assumed to have real-valued solutions; if they are in fact
/// complex only the real parts are returned.
inline void solveQuadratic(const float &a, const float &b, const float& c,
                           float& lowRoot, float& highRoot)
{
    // Avoid NaNs when determinant is negative by clamping to zero.  This is
    // the right thing to do when det would otherwise be zero to within
    // numerical precision.
    float det = std::max(0.0f, b*b - 4*a*c);
    float firstTerm = -b/(2*a);
    float secondTerm = sqrtf(det)/(2*a);
    lowRoot = firstTerm - secondTerm;
    highRoot = firstTerm + secondTerm;
}


/// Render a disk into the cubemapfer using exact visibility testing
///
/// When two surfaces meet at a sharp angle, the visibility of surfels on the
/// adjacent surface needs to be computed more carefully than with the
/// approximate square-based rasterization method.  If not, disturbing
/// artifacts will appear where the surfaces join.
///
/// \param cubemap - buffer to render into.
/// \param p - position of disk center relative to cubemapfer
/// \param n - disk normal
/// \param r - disk radius
template<typename IntegratorT>
static void renderDiskExact(IntegratorT& integrator, const Imath::V3f& p, const Imath::V3f& n, const float& r)
{
    int faceRes = integrator.res();
    float plen2 = p.length2();
    if(plen2 == 0) // Sanity check
        return;
    // Angle from face normal to edge is acos(1/sqrt(3)).
    static float cosFaceAngle = 1.0f/sqrtf(3);
    static float sinFaceAngle = sqrtf(2.0f/3.0f);
    for(int iface = 0; iface < 6; ++iface)
    {
        // Avoid rendering to the current face if the disk definitely doesn't
        // touch it.  First check the cone angle
        if(sphereOutsideCone(p, plen2, r, cubemap::faceNormal(iface),
                             cosFaceAngle, sinFaceAngle))
            continue;
        float dot_pFaceN = cubemap::dotFaceNormal(iface, p);
        float dot_nFaceN = cubemap::dotFaceNormal(iface, n);
        // If the disk is behind the camera and the disk normal is relatively
        // aligned with the face normal (to within the face cone angle), the
        // disk can't contribute to the face and may be culled.
        if(dot_pFaceN < 0 && fabs(dot_nFaceN) > cosFaceAngle)
            continue;
        // Check whether disk spans the perspective divide for the current
        // face.  Note: sin^2(angle(n, faceN)) = (1 - dot_nFaceN*dot_nFaceN)
        if((1 - dot_nFaceN*dot_nFaceN)*r*r >= dot_pFaceN*dot_pFaceN)
        {
            // When the disk spans the perspective divide, the shape of the
            // disk projected onto the face is a hyperbola.  Bounding a
            // hyperbola is a pain, so the easiest thing to do is trace a ray
            // for every pixel on the face and check whether it hits the disk.
            //
            // Note that all of the tricky rasterization rubbish further down
            // could probably be replaced by the following ray tracing code if
            // I knew a way to compute the tight raster bound.
            integrator.setFace(iface);
            for(int iv = 0; iv < faceRes; ++iv)
            for(int iu = 0; iu < faceRes; ++iu)
            {
                // V = ray through the pixel
                Imath::V3f V = integrator.rayDirection(iface, iu, iv);
                // Signed distance to plane containing disk
                float t = dot(p, n)/dot(V, n);
                if(t > 0 && (t*V - p).length2() < r*r)
                {
                    // The ray hit the disk, record the hit
                    integrator.addSample(iu, iv, t, 1.0f);
                }
            }
            continue;
        }
        // If the disk didn't span the perspective divide and is behind the
        // camera, it may be culled.
        if(dot_pFaceN < 0)
            continue;
        // Having gone through all the checks above, we know that the disk
        // doesn't span the perspective divide, and that it is in front of the
        // camera.  Therefore, the disk projected onto the current face is an
        // ellipse, and we may compute a quadratic function
        //
        //   q(u,v) = a0*u*u + b0*u*v + c0*v*v + d0*u + e0*v + f0
        //
        // such that the disk lies in the region satisfying q(u,v) < 0.  To do
        // this, start with the implicit definition of the disk on the plane,
        //
        //   norm(dot(p,n)/dot(V,n) * V - p)^2 - r^2 < 0
        //
        // and compute coefficients A,B,C such that
        //
        //   A*dot(V,V) + B*dot(V,n)*dot(p,V) + C < 0
        float dot_pn = dot(p,n);
        float A = dot_pn*dot_pn;
        float B = -2*dot_pn;
        float C = plen2 - r*r;
        // Project onto the current face to compute the coefficients a0 through
        // to f0 for q(u,v)
        Imath::V3f pp = cubemap::canonicalFaceCoords(iface, p);
        Imath::V3f nn = cubemap::canonicalFaceCoords(iface, n);
        float a0 = A + B*nn.x*pp.x + C*nn.x*nn.x;
        float b0 = B*(nn.x*pp.y + nn.y*pp.x) + 2*C*nn.x*nn.y;
        float c0 = A + B*nn.y*pp.y + C*nn.y*nn.y;
        float d0 = (B*(nn.x*pp.z + nn.z*pp.x) + 2*C*nn.x*nn.z);
        float e0 = (B*(nn.y*pp.z + nn.z*pp.y) + 2*C*nn.y*nn.z);
        float f0 = (A + B*nn.z*pp.z + C*nn.z*nn.z);
        // Finally, transform the coefficients so that they define the
        // quadratic function in *raster* face coordinates, (iu, iv)
        float scale = 2.0f/faceRes;
        float scale2 = scale*scale;
        float off = 0.5f*scale - 1.0f;
        float a = scale2*a0;
        float b = scale2*b0;
        float c = scale2*c0;
        float d = ((2*a0 + b0)*off + d0)*scale;
        float e = ((2*c0 + b0)*off + e0)*scale;
        float f = (a0 + b0 + c0)*off*off + (d0 + e0)*off + f0;
        // Construct a tight bound for the ellipse in raster coordinates.
        int ubegin = 0, uend = faceRes;
        int vbegin = 0, vend = faceRes;
        float det = 4*a*c - b*b;
        // Sanity check; a valid ellipse must have det > 0
        if(det <= 0)
        {
            // If we get here, the disk is probably edge on (det == 0) or we
            // have some hopefully small floating point errors (det < 0: the
            // hyperbolic case we've already ruled out).  Cull in either case.
            continue;
        }
        float ub = 0, ue = 0;
        solveQuadratic(det, 4*d*c - 2*b*e, 4*c*f - e*e, ub, ue);
        ubegin = std::max(0, Imath::ceil(ub));
        uend   = std::min(faceRes, Imath::ceil(ue));
        float vb = 0, ve = 0;
        solveQuadratic(det, 4*a*e - 2*b*d, 4*a*f - d*d, vb, ve);
        vbegin = std::max(0, Imath::ceil(vb));
        vend   = std::min(faceRes, Imath::ceil(ve));
        // By the time we get here, we've expended perhaps 120 FLOPS + 2 sqrts
        // to set up the coefficients of q(iu,iv).  The setup is expensive, but
        // the bound is optimal so it will be worthwhile vs raytracing, unless
        // the raster faces are very small.
        integrator.setFace(iface);
        for(int iv = vbegin; iv < vend; ++iv)
        for(int iu = ubegin; iu < uend; ++iu)
        {
            float q = a*(iu*iu) + b*(iu*iv) + c*(iv*iv) + d*iu + e*iv + f;
            if(q < 0)
            {
                Imath::V3f V = integrator.rayDirection(iface, iu, iv);
                // compute distance to hit point
                float z = dot_pn/dot(V, n);
                integrator.addSample(iu, iv, z, 1.0f);
            }
        }
    }
}


/// Rasterize disk into the given integrator
///
/// N is the normal of the culling cone, with cone angle specified by
/// cosConeAngle and sinConeAngle.  The position of the disk with respect to
/// the centre of the cubemapfer is p, n is the normal of the disk and r is
/// the disk radius.
template<typename IntegratorT>
void renderDisk(IntegratorT& integrator, const Imath::V3f& N, Imath::V3f p, const Imath::V3f &n, const float &r,
                const float& cosConeAngle, const float &sinConeAngle)
{
    float dot_pn = dot(p, n);
    // Cull back-facing points.  In conjunction with the oddball composition
    // rule below, this is very important for smoothness of the result:  If we
    // don't cull the back faces, coverage will be overestimated in every
    // cubemapfer pixel which contains an edge.
    if(dot_pn > 0)
        return;
    float plen2 = p.length2();
    float plen = sqrtf(plen2);
    // If solid angle of bounding sphere is greater than exactRenderAngle,
    // resolve the visibility exactly rather than using a cheap approx.
    //
    // TODO: Adjust exactRenderAngle for best results!
    const float exactRenderAngle = 0.05f;
    float origArea = M_PI*r*r;
    // Solid angle of the bound
    if(exactRenderAngle*plen2 < origArea)
    {
        // Multiplier for radius to make the cracks a bit smaller.  We
        // can't do this too much, or sharp convex edges will be
        // overoccluded (TODO: Adjust for best results!  Maybe the "too
        // large" problem could be worked around using a tracing offset?)
        const float radiusMultiplier = M_SQRT2;
        // Resolve visibility of very close surfels using ray tracing.
        // This is necessary to avoid artifacts where surfaces meet.
        renderDiskExact(integrator, p, n, radiusMultiplier*r);
        return;
    }
    // Figure out which face we're on and get u,v coordinates on that face,
    cubemap::Face faceIndex = cubemap::faceIndex(p);
    int faceRes = integrator.res();
    float u = 0, v = 0;
    cubemap::faceCoords(faceIndex, p, u, v);
    // Compute the area of the surfel when projected onto the env face.
    // This depends on several things:
    // 1) The area of the original disk
    // 2) The angles between the disk normal n, viewing vector p, and face
    // normal.  This is the area projected onto a plane parallel to the env
    // map face, and through the centre of the disk.
    float pDotFaceN = cubemap::dotFaceNormal(faceIndex, p);
    float angleFactor = fabs(dot_pn/pDotFaceN);
    // 3) Ratio of distance to the surfel vs distance to projected point on
    // the face.
    float distFactor = 1.0f/(pDotFaceN*pDotFaceN);
    // Putting these together gives the projected area
    float projArea = origArea * angleFactor * distFactor;
    // Half-width of a square with area projArea
    float wOn2 = sqrtf(projArea)*0.5f;
    // Transform width and position to face raster coords.
    float rasterScale = 0.5f*faceRes;
    u = rasterScale*(u + 1.0f);
    v = rasterScale*(v + 1.0f);
    wOn2 *= rasterScale;
    // Construct square box with the correct area.  This shape isn't
    // anything like the true projection of a disk onto the raster, but
    // it's much cheaper!  Note that points which are proxies for clusters
    // of smaller points aren't going to be accurately resolved no matter
    // what we do.
    struct BoundData
    {
        cubemap::Face faceIndex;
        float ubegin, uend;
        float vbegin, vend;
    };
    // The current surfel can cross up to three faces.
    int nfaces = 1;
    BoundData boundData[3];
    BoundData& bd0 = boundData[0];
    bd0.faceIndex = faceIndex;
    bd0.ubegin = u - wOn2;   bd0.uend = u + wOn2;
    bd0.vbegin = v - wOn2;   bd0.vend = v + wOn2;
    // Detect & handle overlap onto adjacent faces
    //
    // We assume that wOn2 is the same on the adjacent face, an assumption
    // which is true when the surfel is close the the corner of the cube.
    // We also assume that a surfel touches at most three faces.  This is
    // true as long as the surfels don't have a massive solid angle; for
    // such cases the axis-aligned box isn't going to be accurate anyway and
    // the code should have branched into the renderDiskExact function instead.
    if(bd0.ubegin < 0)
    {
        // left neighbour
        BoundData& b = boundData[nfaces++];
        b.faceIndex = cubemap::neighbourU(faceIndex, 0);
        cubemap::faceCoords(b.faceIndex, p, u, v);
        u = rasterScale*(u + 1.0f);
        v = rasterScale*(v + 1.0f);
        b.ubegin = u - wOn2;  b.uend = u + wOn2;
        b.vbegin = v - wOn2;  b.vend = v + wOn2;
    }
    else if(bd0.uend > faceRes)
    {
        // right neighbour
        BoundData& b = boundData[nfaces++];
        b.faceIndex = cubemap::neighbourU(faceIndex, 1);
        cubemap::faceCoords(b.faceIndex, p, u, v);
        u = rasterScale*(u + 1.0f);
        v = rasterScale*(v + 1.0f);
        b.ubegin = u - wOn2;  b.uend = u + wOn2;
        b.vbegin = v - wOn2;  b.vend = v + wOn2;
    }
    if(bd0.vbegin < 0)
    {
        // bottom neighbour
        BoundData& b = boundData[nfaces++];
        b.faceIndex = cubemap::neighbourV(faceIndex, 0);
        cubemap::faceCoords(b.faceIndex, p, u, v);
        u = rasterScale*(u + 1.0f);
        v = rasterScale*(v + 1.0f);
        b.ubegin = u - wOn2;  b.uend = u + wOn2;
        b.vbegin = v - wOn2;  b.vend = v + wOn2;
    }
    else if(bd0.vend > faceRes)
    {
        // top neighbour
        BoundData& b = boundData[nfaces++];
        b.faceIndex = cubemap::neighbourV(faceIndex, 1);
        cubemap::faceCoords(b.faceIndex, p, u, v);
        u = rasterScale*(u + 1.0f);
        v = rasterScale*(v + 1.0f);
        b.ubegin = u - wOn2;  b.uend = u + wOn2;
        b.vbegin = v - wOn2;  b.vend = v + wOn2;
    }
    for(int iface = 0; iface < nfaces; ++iface)
    {
        BoundData& bd = boundData[iface];
        // Range of pixels which the square touches (note, exclusive end)
        int ubeginRas = Imath::clamp(int(bd.ubegin),   0, faceRes);
        int uendRas   = Imath::clamp(int(bd.uend) + 1, 0, faceRes);
        int vbeginRas = Imath::clamp(int(bd.vbegin),   0, faceRes);
        int vendRas   = Imath::clamp(int(bd.vend) + 1, 0, faceRes);
        integrator.setFace(bd.faceIndex);
        for(int iv = vbeginRas; iv < vendRas; ++iv)
        for(int iu = ubeginRas; iu < uendRas; ++iu)
        {
            // Calculate the fraction coverage of the square over the current
            // pixel for antialiasing.  This estimate is what you'd get if you
            // filtered the square representing the surfel with a 1x1 box filter.
            float urange = std::min<float>(iu+1, bd.uend) -
                           std::max<float>(iu,   bd.ubegin);
            float vrange = std::min<float>(iv+1, bd.vend) -
                           std::max<float>(iv,   bd.vbegin);
            float coverage = urange*vrange;
            integrator.addSample(iu, iv, plen, coverage);
        }
    }
}

/// Render point hierarchy into cubemapfer.
template<typename IntegratorT>
static void renderNode(IntegratorT& integrator, Imath::V3f P, Imath::V3f N, float cosConeAngle,
                       float sinConeAngle, float maxSolidAngle, int dataSize,
                       const pointOctreeNode* node, int radiosityIndex, int areaIndex)
{
    // This is an iterative traversal of the point hierarchy, since it's
    // slightly faster than a recursive traversal.
    //
    // The max required size for the explicit stack should be < 200, since
    // tree depth shouldn't be > 24, and we have a max of 8 children per node.
    const pointOctreeNode* nodeStack[400];
    nodeStack[0] = node;
    int stackSize = 1;
    while(stackSize > 0)
    {
        node = nodeStack[--stackSize];
        {
            // Examine node bound and cull if possible
            // TODO: Reinvestigate using (node->aggP - P) with spherical harmonics
            Imath::V3f c = node->center - P;
            if(sphereOutsideCone(c, c.length2(), node->boundRadius, N,
                                cosConeAngle, sinConeAngle))
                continue;
        }
        float r = node->aggR;
        Imath::V3f p = node->aggP - P;
        float plen2 = p.length2();
        // Examine solid angle of interior node bounding sphere to see whether we
        // can render it directly or not.
        //
        // TODO: Would be nice to use dot(node->aggN, p.normalized()) in the solid
        // angle estimation.  However, we get bad artifacts if we do this naively.
        // Perhaps with spherical harmoics it'll be better.
        float solidAngle = M_PI*r*r / plen2;
        if(solidAngle < maxSolidAngle)
        {
            integrator.setPointData(reinterpret_cast<const float*>(&node->aggCol));
            renderDisk(integrator, N, p, node->aggN, r, cosConeAngle, sinConeAngle);
        }
        else
        {
            // If we get here, the solid angle of the current node was too large
            // so we must consider the children of the node.
            //
            // The render order is sorted so that points are rendered front to
            // back.  This greatly improves the correctness of the hider.
            //
            // FIXME: The sorting procedure gets things wrong sometimes!  The
            // problem is that points may stick outside the bounds of their octree
            // nodes.  Probably we need to record all the points, sort, and
            // finally render them to get this right.
            if(node->npoints)
            {
                // Leaf node: simply render each child point.
                //assert(node->npoints <= 8);
                int np =  node->npoints;

                std::pair<float, int> childOrder[8];

                for(int i = 0; i < np; ++i)
                {
                    const float* data = &node->data[i*dataSize];
                    Imath::V3f p = Imath::V3f(data[0], data[1], data[2]) - P;
                    childOrder[i].first = p.length2();
                    childOrder[i].second = i;
                }

                std::sort(childOrder, childOrder + node->npoints);

                for(int i = 0; i < np; ++i)
                {
                    const float* data = &node->data[childOrder[i].second*dataSize];
                    Imath::V3f pp = Imath::V3f(data[0], data[1], data[2]) - P;
                    Imath::V3f nn = Imath::V3f(data[3], data[4], data[5]);
                    float rr = data[6 + areaIndex];
                    integrator.setPointData(data+6+radiosityIndex);
                    renderDisk(integrator, N, pp, nn, rr, cosConeAngle, sinConeAngle);
                }
            }
            else
            {
                // Interior node: render children.
                std::pair<float, const pointOctreeNode*> children[8];
                int nchildren = 0;

                for(int i = 0; i < 8; ++i)
                {
                    pointOctreeNode* child = node->children[i];
                    if(!child)
                        continue;
                    children[nchildren].first = (child->center - P).length2();
                    children[nchildren].second = child;
                    ++nchildren;
                }
                std::sort(children, children + nchildren);

                // Interior node: render each non-null child.  Nodes we want to
                // render first must go onto the stack last.
                for(int i = nchildren-1; i >= 0; --i){
                    nodeStack[stackSize++] = children[i].second;
                }
            }
        }
    }
}



#endif
