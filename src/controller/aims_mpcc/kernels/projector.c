/* Exact scalar reference projector. Library prepared offline.
 * Bounded minimization / PPoly evaluation translated from SciPy 1.15.3.
 * Copyright (c) 2001-2002 Enthought, Inc. 2003-2024, SciPy Developers.
 * NumPy-style remainder follows NumPy 1.26.4 (NumPy Developers).
 * Redistribution notices and disclaimers: ../vendor/SCIPY_LICENSE and
 * ../vendor/NUMPY_LICENSE. Both BSD 3-Clause; retain with redistribution.
 * optimize.py module by Travis E. Oliphant: You may copy and use this module
 * as you see fit with no guarantee implied provided you keep this notice
 * in all copies. (Original SciPy module notice.)
 * Compile without floating-point contraction, reassociation, or fast math.
 */
#include <math.h>
#include <stddef.h>

typedef struct {
    const double *grid, *points, *knots, *coeff;
    int grids, intervals;
    double length, step;
} Path;

/* Remainder part of NumPy npy_divmod. Its quotient calculation does not
 * modify the remainder. Unlike fmod alone, preserve positive-divisor zero
 * sign and adjust negative remainders, including rounding to length. */
static double remainder_numpy(double a, double b) {
    double mod = fmod(a, b);
    if (mod != 0.0) {
        if ((b < 0.0) != (mod < 0.0)) mod += b;
    } else {
        mod = copysign(0.0, b);
    }
    return mod;
}

static int interval(const double *knots, int count, double x) {
    int low=0, high=count-2, mid;
    if (!(knots[0] <= x && x <= knots[count-1])) return -1;
    if (x == knots[count-1]) return count-2;
    if (x < knots[1]) high=0;
    while (low < high) {
        mid=low+(high-low)/2;
        if (x < knots[mid]) high=mid;
        else if (x >= knots[mid+1]) low=mid+1;
        else {low=mid; break;}
    }
    return low;
}

static void polynomial(const Path *p, double theta, double *xy) {
    /* curve.numpy wraps once; periodic PPoly.__call__ wraps again. */
    double wrapped=remainder_numpy(theta,p->length);
    double x=p->knots[0]+remainder_numpy(wrapped-p->knots[0],
                                       p->knots[p->intervals]-p->knots[0]);
    int i=interval(p->knots,p->intervals+1,x);
    if (i < 0) {xy[0]=NAN; xy[1]=NAN; return;}
    double s=x-p->knots[i];
    for (int component=0; component<2; ++component) {
        double res=0.0, z=1.0;
        for (int k=0; k<6; ++k) {
            double prefactor=1.0;
            res=res+p->coeff[((size_t)(5-k)*p->intervals+i)*2+component]*z*prefactor;
            if (k < 5) z *= s;
        }
        xy[component]=res;
    }
}

static double objective(const Path *p, double theta, double qx, double qy) {
    double xy[2]; polynomial(p,theta,xy);
    double dx=xy[0]-qx, dy=xy[1]-qy;
    /* np.sum of the two squared components, in that order. */
    return dx*dx+dy*dy;
}

static int coarse(const Path *p, double qx, double qy) {
    int first=0;
    double dx=p->points[0]-qx,dy=p->points[1]-qy;
    double best=dx*dx+dy*dy;
    for (int i=1; i<p->grids; ++i) {
        dx=p->points[2*(size_t)i]-qx;dy=p->points[2*(size_t)i+1]-qy;
        double value=dx*dx+dy*dy;
        if (value < best) {first=i; best=value;}
    }
    return first;
}

static double sign_plus_zero(double x) {
    if (isnan(x)) return x;
    return x > 0.0 ? 1.0 : x < 0.0 ? -1.0 : 1.0;
}

static double maximum_numpy(double a, double b) {
    if (isnan(a)) return a;
    if (isnan(b)) return b;
    return a >= b ? a : b;
}

/* Return wrapped theta plus diagnostics [unwrapped, objective, nfev,
 * status, coarse_index]. Optional trace stores [x,f(x)] for every call. */
static double bounded(const Path *path, double qx, double qy, double *stats,
                      double *trace) {
    const double sqrt_eps=sqrt(2.2e-16);
    const double golden_mean=0.5*(3.0-sqrt(5.0));
    const double xatol=1e-12;
    const int maxfun=500;
    int guess=coarse(path,qx,qy), flag=0;
    double a=path->grid[guess]-path->step, b=path->grid[guess]+path->step;
    double fulc=a+golden_mean*(b-a), nfc=fulc, xf=fulc;
    double rat=0.0, e=0.0, x=xf;
    double fx=objective(path,x,qx,qy), fu=INFINITY;
    int num=1;
    if (trace) {trace[0]=x;trace[1]=fx;}
    double ffulc=fx, fnfc=fx;
    double xm=0.5*(a+b);
    double tol1=sqrt_eps*fabs(xf)+xatol/3.0, tol2=2.0*tol1;
    while (fabs(xf-xm) > (tol2-0.5*(b-a))) {
        int golden=1;
        if (fabs(e) > tol1) {
            golden=0;
            double r=(xf-nfc)*(fx-ffulc);
            double q=(xf-fulc)*(fx-fnfc);
            double p=(xf-fulc)*q-(xf-nfc)*r;
            q=2.0*(q-r);
            if (q > 0.0) p=-p;
            q=fabs(q);
            r=e;e=rat;
            if ((fabs(p) < fabs(0.5*q*r)) && (p > q*(a-xf)) && (p < q*(b-xf))) {
                rat=(p+0.0)/q;
                x=xf+rat;
                if ((x-a) < tol2 || (b-x) < tol2) rat=tol1*sign_plus_zero(xm-xf);
            } else golden=1;
        }
        if (golden) {
            e=xf >= xm ? a-xf : b-xf;
            rat=golden_mean*e;
        }
        double si=sign_plus_zero(rat);
        x=xf+si*maximum_numpy(fabs(rat),tol1);
        fu=objective(path,x,qx,qy);
        if (trace) {trace[2*num]=x;trace[2*num+1]=fu;}
        ++num;
        if (fu <= fx) {
            if (x >= xf) a=xf;else b=xf;
            fulc=nfc;ffulc=fnfc;nfc=xf;fnfc=fx;xf=x;fx=fu;
        } else {
            if (x < xf) a=x;else b=x;
            if (fu <= fnfc || nfc == xf) {
                fulc=nfc;ffulc=fnfc;nfc=x;fnfc=fu;
            } else if (fu <= ffulc || fulc == xf || fulc == nfc) {fulc=x;ffulc=fu;}
        }
        xm=0.5*(a+b);
        tol1=sqrt_eps*fabs(xf)+xatol/3.0;tol2=2.0*tol1;
        if (num >= maxfun) {flag=1;break;}
    }
    if (isnan(xf) || isnan(fx) || isnan(fu)) flag=2;
    if (stats) {stats[0]=xf;stats[1]=fx;stats[2]=num;stats[3]=flag;stats[4]=guess;}
    return remainder_numpy(xf,path->length);
}

/* Live reference tables held by the Python caller for this call. */
double aims_project(const double *grid, const double *points, int grids,
                    const double *knots, const double *coeff, int intervals,
                    double length, double step, double qx, double qy,
                    double *stats, double *trace) {
    Path path={grid,points,knots,coeff,grids,intervals,length,step};
    return bounded(&path,qx,qy,stats,trace);
}

void aims_polynomial(const double *knots,const double *coeff,int intervals,
                     double length,double theta,double *xy) {
    Path path={NULL,NULL,knots,coeff,0,intervals,length,0.};
    polynomial(&path,theta,xy);
}

double aims_remainder(double a,double b) {return remainder_numpy(a,b);}

int aims_projector_abi(void) {return 1;}

/* Validate every live table on every call. No prepared geometry assumptions
 * survive across calls. Python checks dtype/layout/shape/method identities. */
int aims_project_checked(const double *grid,const double *points,int grids,
                         const double *knots,const double *coeff,int intervals,
                         double length,double step,double qx,double qy,
                         double *result) {
    if (grids<1 || intervals<1 || !isfinite(length) || length<=0.0 ||
        !isfinite(step) || step<=0.0 || knots[0]!=0.0 ||
        knots[intervals]!=length) return 0;
    for (int i=0;i<grids;++i) {
        if (!isfinite(grid[i]) || !isfinite(grid[i]-step) ||
            !isfinite(grid[i]+step) || !isfinite(points[2*(size_t)i]) ||
            !isfinite(points[2*(size_t)i+1])) return 0;
    }
    for (int i=0;i<intervals;++i) {
        if (!isfinite(knots[i]) || !isfinite(knots[i+1]) ||
            !(knots[i+1]>knots[i])) return 0;
    }
    for (size_t i=0;i<12*(size_t)intervals;++i) {
        if (!isfinite(coeff[i])) return 0;
    }
    Path path={grid,points,knots,coeff,grids,intervals,length,step};
    *result=bounded(&path,qx,qy,NULL,NULL);
    return 1;
}
