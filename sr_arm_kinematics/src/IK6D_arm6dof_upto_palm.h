#ifndef IKFAST6D_HEADER
#define IKFAST6D_HEADER


#define IKFAST_HAS_LIBRARY
#include "ikfast.h" // found inside share/openrave-X.Y/python/ikfast.h
using namespace ikfast;

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>
#include <cstdio>

#ifndef IKFAST_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#endif

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#define IKFAST_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "ikfast exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }

#endif

#if defined(_MSC_VER)
#define IKFAST_ALIGNED16(x) __declspec(align(16)) x
#else
#define IKFAST_ALIGNED16(x) x __attribute((aligned(16)))
#endif

#define IK2PI  ((IkReal)6.28318530717959)
#define IKPI  ((IkReal)3.14159265358979)
#define IKPI_2  ((IkReal)1.57079632679490)

#ifdef _MSC_VER
#ifndef isnan
#define isnan _isnan
#endif
#endif // _MSC_VER

// defined when creating a shared object/dll
#ifdef IKFAST_CLIBRARY
#ifdef _MSC_VER
#define IKFAST_API extern "C" __declspec(dllexport)
#else
#define IKFAST_API extern "C"
#endif
#else
#define IKFAST_API
#endif

// lapack routines
extern "C" {
    void dgetrf_ (const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info);
    void zgetrf_ (const int* m, const int* n, std::complex<double>* a, const int* lda, int* ipiv, int* info);
    void dgetri_(const int* n, const double* a, const int* lda, int* ipiv, double* work, const int* lwork, int* info);
    void dgesv_ (const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info);
    void dgetrs_(const char *trans, const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
    void dgeev_(const char *jobvl, const char *jobvr, const int *n, double *a, const int *lda, double *wr, double *wi,double *vl, const int *ldvl, double *vr, const int *ldvr, double *work, const int *lwork, int *info);
}


using namespace std; // necessary to get std math routines


namespace ikfast
{



inline float IKsqr(float f)
{
    return f*f;
}
inline double IKsqr(double f)
{
    return f*f;
}

inline float IKabs(float f)
{
    return fabsf(f);
}
inline double IKabs(double f)
{
    return fabs(f);
}

inline float IKlog(float f)
{
    return logf(f);
}
inline double IKlog(double f)
{
    return log(f);
}

// allows asin and acos to exceed 1
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)0.000001)
#endif

// used to check input to atan2 for degenerate cases
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)2e-6)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
#endif

inline float IKasin(float f)
{
    IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
    if( f <= -1 ) return -IKPI_2;
    else if( f >= 1 ) return IKPI_2;
    return asinf(f);
}
inline double IKasin(double f)
{
    IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
    if( f <= -1 ) return -IKPI_2;
    else if( f >= 1 ) return IKPI_2;
    return asin(f);
}

// return positive value in [0,y)
inline float IKfmod(float x, float y)
{
    while(x < 0)
    {
        x += y;
    }
    return fmodf(x,y);
}

// return positive value in [0,y)
inline float IKfmod(double x, double y)
{
    while(x < 0)
    {
        x += y;
    }
    return fmod(x,y);
}

inline float IKacos(float f)
{
    IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
    if( f <= -1 ) return IKPI;
    else if( f >= 1 ) return 0;
    return acosf(f);
}
inline double IKacos(double f)
{
    IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
    if( f <= -1 ) return IKPI;
    else if( f >= 1 ) return 0;
    return acos(f);
}
inline float IKsin(float f)
{
    return sinf(f);
}
inline double IKsin(double f)
{
    return sin(f);
}
inline float IKcos(float f)
{
    return cosf(f);
}
inline double IKcos(double f)
{
    return cos(f);
}
inline float IKtan(float f)
{
    return tanf(f);
}
inline double IKtan(double f)
{
    return tan(f);
}
inline float IKsqrt(float f)
{
    if( f <= 0.0f ) return 0.0f;
    return sqrtf(f);
}
inline double IKsqrt(double f)
{
    if( f <= 0.0 ) return 0.0;
    return sqrt(f);
}
inline float IKatan2(float fy, float fx)
{
    if( isnan(fy) )
    {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) )
    {
        return 0;
    }
    return atan2f(fy,fx);
}
inline double IKatan2(double fy, double fx)
{
    if( isnan(fy) )
    {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) )
    {
        return 0;
    }
    return atan2(fy,fx);
}

inline float IKsign(float f)
{
    if( f > 0 )
    {
        return 1.0f;
    }
    else if( f < 0 )
    {
        return -1.0f;
    }
    return 0;
}

inline double IKsign(double f)
{
    if( f > 0 )
    {
        return 1.0;
    }
    else if( f < 0 )
    {
        return -1.0;
    }
    return 0;
}


static inline bool checkconsistency12(const IkReal* Breal)
{
    IkReal norm = 0.1;
    for(int i = 0; i < 11; ++i)
    {
        norm += IKabs(Breal[i]);
    }
    IkReal tol = 1e-6*norm; // have to increase the threshold since many computations are involved
    return IKabs(Breal[0]*Breal[0]-Breal[1]) < tol && IKabs(Breal[0]*Breal[2]-Breal[3]) < tol && IKabs(Breal[1]*Breal[2]-Breal[4]) < tol && IKabs(Breal[2]*Breal[2]-Breal[5]) < tol && IKabs(Breal[0]*Breal[5]-Breal[6]) < tol && IKabs(Breal[1]*Breal[5]-Breal[7]) < tol && IKabs(Breal[2]*Breal[5]-Breal[8]) < tol && IKabs(Breal[0]*Breal[8]-Breal[9]) < tol && IKabs(Breal[1]*Breal[8]-Breal[10]) < tol;
};


/// \brief Solve the det Ax^2+Bx+C = 0 problem using the Manocha and Canny method (1994)
///
/// matcoeffs is of length 54*3, for 3 matrices
static inline void solvedialyticpoly12qep(const IkReal* matcoeffs, IkReal* rawroots, int& numroots)
{
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    IkReal IKFAST_ALIGNED16(M[24*24]) = {0};
    IkReal IKFAST_ALIGNED16(A[12*12]);
    IkReal IKFAST_ALIGNED16(work[24*24*23]);
    int ipiv[12];
    int info, coeffindex;
    const int worksize=24*24*23;
    const int matrixdim = 12;
    const int matrixdim2 = 24;
    numroots = 0;
    // first setup M = [0 I; -C -B] and A
    coeffindex = 0;
    for(int j = 0; j < 6; ++j)
    {
        for(int k = 0; k < 9; ++k)
        {
            M[matrixdim+(j+6)+2*matrixdim*k] = M[matrixdim+j+2*matrixdim*(k+3)] = -matcoeffs[coeffindex++];
        }
    }
    for(int j = 0; j < 6; ++j)
    {
        for(int k = 0; k < 9; ++k)
        {
            M[matrixdim+(j+6)+2*matrixdim*k+matrixdim*2*matrixdim] = M[matrixdim+j+2*matrixdim*(k+3)+matrixdim*2*matrixdim] = -matcoeffs[coeffindex++];
        }
    }
    for(int j = 0; j < 6; ++j)
    {
        for(int k = 0; k < 9; ++k)
        {
            A[(j+6)+matrixdim*k] = A[j+matrixdim*(k+3)] = matcoeffs[coeffindex++];
        }
        for(int k = 0; k < 3; ++k)
        {
            A[j+matrixdim*k] = A[(j+6)+matrixdim*(k+9)] = 0;
        }
    }
    const IkReal lfpossibilities[4][4] = {{1,-1,1,1},{1,0,-2,1},{1,1,2,0},{1,-1,4,1}};
    int lfindex = -1;
    bool bsingular = true;
    do
    {
        dgetrf_(&matrixdim,&matrixdim,A,&matrixdim,&ipiv[0],&info);
        if( info == 0 )
        {
            bsingular = false;
            for(int j = 0; j < matrixdim; ++j)
            {
                if( IKabs(A[j*matrixdim+j]) < 100*tol )
                {
                    bsingular = true;
                    break;
                }
            }
            if( !bsingular )
            {
                break;
            }
        }
        if( lfindex == 3 )
        {
            break;
        }
        // transform by the linear functional
        lfindex++;
        const IkReal* lf = lfpossibilities[lfindex];
        // have to reinitialize A
        coeffindex = 0;
        for(int j = 0; j < 6; ++j)
        {
            for(int k = 0; k < 9; ++k)
            {
                IkReal a = matcoeffs[coeffindex+108], b = matcoeffs[coeffindex+54], c = matcoeffs[coeffindex];
                A[(j+6)+matrixdim*k] = A[j+matrixdim*(k+3)] = lf[0]*lf[0]*a+lf[0]*lf[2]*b+lf[2]*lf[2]*c;
                M[matrixdim+(j+6)+2*matrixdim*k] = M[matrixdim+j+2*matrixdim*(k+3)] = -(lf[1]*lf[1]*a + lf[1]*lf[3]*b + lf[3]*lf[3]*c);
                M[matrixdim+(j+6)+2*matrixdim*k+matrixdim*2*matrixdim] = M[matrixdim+j+2*matrixdim*(k+3)+matrixdim*2*matrixdim] = -(2*lf[0]*lf[1]*a + (lf[0]*lf[3]+lf[1]*lf[2])*b + 2*lf[2]*lf[3]*c);
                coeffindex++;
            }
            for(int k = 0; k < 3; ++k)
            {
                A[j+matrixdim*k] = A[(j+6)+matrixdim*(k+9)] = 0;
            }
        }
    }
    while(lfindex<4);

    if( bsingular )
    {
        return;
    }
    dgetrs_("No transpose", &matrixdim, &matrixdim2, A, &matrixdim, &ipiv[0], &M[matrixdim], &matrixdim2, &info);
    if( info != 0 )
    {
        return;
    }

    // set identity in upper corner
    for(int j = 0; j < matrixdim; ++j)
    {
        M[matrixdim*2*matrixdim+j+matrixdim*2*j] = 1;
    }
    IkReal IKFAST_ALIGNED16(wr[24]);
    IkReal IKFAST_ALIGNED16(wi[24]);
    IkReal IKFAST_ALIGNED16(vr[24*24]);
    int one=1;
    dgeev_("N", "V", &matrixdim2, M, &matrixdim2, wr, wi,NULL, &one, vr, &matrixdim2, work, &worksize, &info);
    if( info != 0 )
    {
        return;
    }
    IkReal Breal[matrixdim-1];
    for(int i = 0; i < matrixdim2; ++i)
    {
        if( IKabs(wi[i]) < tol*100 )
        {
            IkReal* ev = vr+matrixdim2*i;
            if( IKabs(wr[i]) > 1 )
            {
                ev += matrixdim;
            }
            // consistency has to be checked!!
            if( IKabs(ev[0]) < tol )
            {
                continue;
            }
            IkReal iconst = 1/ev[0];
            for(int j = 1; j < matrixdim; ++j)
            {
                Breal[j-1] = ev[j]*iconst;
            }
            if( checkconsistency12(Breal) )
            {
                if( lfindex >= 0 )
                {
                    const IkReal* lf = lfpossibilities[lfindex];
                    rawroots[numroots++] = (wr[i]*lf[0]+lf[1])/(wr[i]*lf[2]+lf[3]);
                }
                else
                {
                    rawroots[numroots++] = wr[i];
                }
                bool bsmall0=IKabs(ev[0]) > IKabs(ev[3]);
                bool bsmall1=IKabs(ev[0]) > IKabs(ev[1]);
                if( bsmall0 && bsmall1 )
                {
                    rawroots[numroots++] = ev[3]/ev[0];
                    rawroots[numroots++] = ev[1]/ev[0];
                }
                else if( bsmall0 && !bsmall1 )
                {
                    rawroots[numroots++] = ev[5]/ev[2];
                    rawroots[numroots++] = ev[2]/ev[1];
                }
                else if( !bsmall0 && bsmall1 )
                {
                    rawroots[numroots++] = ev[9]/ev[6];
                    rawroots[numroots++] = ev[10]/ev[9];
                }
                else if( !bsmall0 && !bsmall1 )
                {
                    rawroots[numroots++] = ev[11]/ev[8];
                    rawroots[numroots++] = ev[11]/ev[10];
                }
            }
        }
    }
};




class IKSolver
{
public:
    IkReal j0,cj0,sj0,htj0,j1,cj1,sj1,htj1,j2,cj2,sj2,htj2,j3,cj3,sj3,htj3,j4,cj4,sj4,htj4,j5,cj5,sj5,htj5,new_r00,r00,rxp0_0,new_r01,r01,rxp0_1,new_r02,r02,rxp0_2,new_r10,r10,rxp1_0,new_r11,r11,rxp1_1,new_r12,r12,rxp1_2,new_r20,r20,rxp2_0,new_r21,r21,rxp2_1,new_r22,r22,rxp2_2,new_px,px,npx,new_py,py,npy,new_pz,pz,npz,pp;
    unsigned char _ij0[2], _nj0,_ij1[2], _nj1,_ij2[2], _nj2,_ij3[2], _nj3,_ij4[2], _nj4,_ij5[2], _nj5;

    bool ik(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions);
    void fk(const IkReal* j, IkReal* eetrans, IkReal* eerot);

};


IKFAST_API const char* getKinematicsHash();

} // end namespace

#endif //IKFAST6D_HEADER




