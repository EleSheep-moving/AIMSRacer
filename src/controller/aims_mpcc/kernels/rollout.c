/* Independent 2 ms midpoint integration. No optimizer-generated transitions. */
#include <math.h>
#include <string.h>

static void rhs(const double *x, const double *u, double target,
                double wheelbase, double tau, double understeer, double *r) {
    r[0]=x[3]*cos(x[2]); r[1]=x[3]*sin(x[2]);
    r[2]=x[3]*tan(x[5])/(wheelbase*(1.0+understeer*x[3]*x[3]));
    r[3]=u[0]; r[4]=u[2]; r[5]=(target-x[5])/tau;
}

int aims_rollout(const double *initial, const double *applied, const double *controls,
                 int n, int count, double wheelbase, double tau, double understeer,
                 double bias, double *out) {
    if(n<1 || count<1 || !(wheelbase>0) || !(tau>0)) return -1;
    double x[6], a[6], mid[6], b[6];
    memcpy(x,initial,6*sizeof(double)); memcpy(out,x,6*sizeof(double));
    double previous=applied[1]; int row=1;
    for(int k=0;k<n;k++) {
        const double *u=controls+3*k;
        for(int j=0;j<count;j++) {
            double target=previous+(u[1]-previous)*(j+1)/(double)count+bias;
            for(int step=0;step<10;step++) {
                rhs(x,u,target,wheelbase,tau,understeer,a);
                for(int d=0;d<6;d++) mid[d]=x[d]+.001*a[d];
                rhs(mid,u,target,wheelbase,tau,understeer,b);
                for(int d=0;d<6;d++) {
                    x[d]+=.002*b[d];
                    if(!isfinite(x[d])) return -2;
                }
            }
            memcpy(out+6*row++,x,6*sizeof(double));
        }
        previous=u[1];
    }
    return 0;
}
