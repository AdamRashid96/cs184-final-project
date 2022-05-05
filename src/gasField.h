#ifndef GAS_FIELD_H
#define GAS_FIELD_H

#include <vector>

#include "CGL/CGL.h"
#include "fieldCell.h"
#include "util/random_util.h"

using namespace CGL;
using namespace std;

class GasField {
public:
    GasField(int N, double cell_size, double density,
            double ambient_temperature, double base_pressure, double initial_velocity) {
        
        this->width = N;
        this->height = N;
        this->depth = N;
        this->size = (N + 2) * (N + 2) * (N + 2);
        this->cell_size = 1.0 / N; //Assuming Square

        u = new double[size];
        u_prev = new double[size];
        v = new double[size];
        v_prev = new double[size];
        w = new double[size];
        w_prev = new double[size];
        d = new double[size];
        d0 = new double[size];
        T = new double[size];
        T0 = new double[size];

        clear_data();

        // Initialize cell values
        for (int i = 0; i < width + 2; i++) {
            for (int j = 0; j < height + 2; j++) {
                for (int k = 0; k < depth + 2; k++) {
                    // Vector3D velocity = get_sample() * random_uniform() * initial_velocity;
                    Vector3D velocity = Vector3D(0, 0, 0);
                    u[IDX(i, j, k)] = velocity.x;
                    u_prev[IDX(i, j, k)] = velocity.x;
                    v[IDX(i, j, k)] = velocity.y;
                    v_prev[IDX(i, j, k)] = velocity.y;
                    w[IDX(i, j, k)] = velocity.z;
                    w_prev[IDX(i, j, k)] = velocity.z;
                }
            }
        }
    }

    ~GasField() {
        
    }


    int IDX(int x, int y, int z) {
        return x + y * (width + 2) + z * (height + 2) * (width + 2);
    }

    void SWAP(double *x0 , double *x) {
        double *tmp=x0;
        x0=x;
        x=tmp;
    }

    void clear_data(){
	    for (int i=0 ; i<size ; i++ ) {
		    u[i] = v[i] = w[i] = u_prev[i] = v_prev[i] =w_prev[i] = T[i] = T0[i] = d[i] = d0[i] = 0.0;
	    }
    }
    
    /* Return the world space position of the cell at the given indices. */
    Vector3D CellPos(int x, int y, int z) {
        return origin - cell_size / 2 * Vector3D(width + 2, height + 2, depth + 2) + cell_size * Vector3D(x, y, z);
    }

    /* Return the velocity vector of the cell at the given indices. */
    Vector3D VelAt(int x, int y, int z) {
        return Vector3D(u[IDX(x, y, z)], v[IDX(x, y, z)], w[IDX(x, y, z)]);
    }

    Vector3D VelAt(Vector3D position) {
        int x = (position.x - origin.x) / cell_size + 0.5 * (width + 2);
        int y = (position.y - origin.y) / cell_size + 0.5 * (height + 2);
        int z = (position.z - origin.z) / cell_size + 0.5 * (depth + 2);

        if (x < 0 || x >= width + 2 || y < 0 || y >= height + 2 || z < 0 || z >= depth + 2) {
            return NULL;
        }

        return VelAt(x, y, z);
    }

    void add_source(int width, int height, int depth, double * x, double * s, double dt){
        int size=(width+2)*(height+2)*(width+2);
        for (int i=0 ; i<size ; i++ ) {
            x[i] += dt*s[i];
        }
    }

    void set_bnd (int M, int N, int O, int b, double * x) {
        // bounds are cells at faces of the cube
        int i, j;
        //setting faces
        for (i=1 ; i<=M ; i++ ) {
            for ( j=1 ; j<=N ; j++ ) {
                x[IDX(i,j,0 )] = b==3 ? -x[IDX(i,j,1)] : x[IDX(i,j,1)];
                x[IDX(i,j,O+1)] = b==3 ? -x[IDX(i,j,O)] : x[IDX(i,j,O)];
            }
        }

        for (i=1 ; i<=N ; i++ ) {
            for ( j=1 ; j<=O ; j++ ) {
                x[IDX(0  ,i, j)] = b==1 ? -x[IDX(1,i,j)] : x[IDX(1,i,j)];
                x[IDX(M+1,i, j)] = b==1 ? -x[IDX(M,i,j)] : x[IDX(M,i,j)];
            }
        }

        for (i=1; i<=M ; i++ ) {
            for ( j=1 ; j<=O ; j++ ) {
                x[IDX(i,0,j )] = b==2 ? -x[IDX(i,1,j)] : x[IDX(i,1,j)];
                x[IDX(i,N+1,j)] = b==2 ? -x[IDX(i,N,j)] : x[IDX(i,N,j)];
            }
        }

        //Setting edges
        for ( i=1; i<=M; i++) {
            x[IDX(i,  0,  0)] = 1.0/2.0*(x[IDX(i,1,  0)]+x[IDX(i,  0,  1)]);
            x[IDX(i,N+1,  0)] = 1.0/2.0*(x[IDX(i,N,  0)]+x[IDX(i,N+1,  1)]);
            x[IDX(i,  0,O+1)] = 1.0/2.0*(x[IDX(i,0,  O)]+x[IDX(i,  1,O+1)]);
            x[IDX(i,N+1,O+1)] = 1.0/2.0*(x[IDX(i,N,O+1)]+x[IDX(i,N+1,  O)]);
        }
        
        for ( i=1; i<=N; i++) {
            x[IDX(0,  i,  0)] = 1.0/2.0*(x[IDX(1,i,  0)]+x[IDX(0,  i,  1)]);
            x[IDX(M+1,i,  0)] = 1.0/2.0*(x[IDX(M,i,  0)]+x[IDX(M+1,i,  1)]);
            x[IDX(0,  i,O+1)] = 1.0/2.0*(x[IDX(0,i,  O)]+x[IDX(1,  i,O+1)]);
            x[IDX(M+1,i,O+1)] = 1.0/2.0*(x[IDX(M,i,O+1)]+x[IDX(M+1,i,  O)]);
        }
        
        for (i=1; i<=O; i++) {
            x[IDX(0,  0,  i)] = 1.0/2.0*(x[IDX(0,  1,i)]+x[IDX(1,  0,  i)]);
            x[IDX(0,  N+1,i)] = 1.0/2.0*(x[IDX(0,  N,i)]+x[IDX(1,  N+1,i)]);
            x[IDX(M+1,0,  i)] = 1.0/2.0*(x[IDX(M,  0,i)]+x[IDX(M+1,1,  i)]);
            x[IDX(M+1,N+1,i)] = 1.0/2.0*(x[IDX(M+1,N,i)]+x[IDX(M,  N+1,i)]);
        }

        //setting corners
        x[IDX(0  ,0, 0  )] = 1.0/3.0*(x[IDX(1,0,0  )]+x[IDX(0  ,1,0)]+x[IDX(0 ,0,1)]);
        x[IDX(0  ,N+1, 0)] = 1.0/3.0*(x[IDX(1,N+1, 0)]+x[IDX(0  ,N, 0)] + x[IDX(0  ,N+1, 1)]);

        x[IDX(M+1,0, 0 )] = 1.0/3.0*(x[IDX(M,0,0  )]+x[IDX(M+1,1,0)] + x[IDX(M+1,0,1)]);
        x[IDX(M+1,N+1,0)] = 1.0/3.0*(x[IDX(M,N+1,0)]+x[IDX(M+1,N,0)]+x[IDX(M+1,N+1,1)]);

        x[IDX(0  ,0, O+1 )] = 1.0/3.0*(x[IDX(1,0,O+1  )]+x[IDX(0  ,1,O+1)]+x[IDX(0 ,0,O)]);
        x[IDX(0  ,N+1, O+1)] = 1.0/3.0*(x[IDX(1,N+1, O+1)]+x[IDX(0  ,N, O+1)] + x[IDX(0  ,N+1, O)]);

        x[IDX(M+1,0, O+1 )] = 1.0/3.0*(x[IDX(M,0,O+1  )]+x[IDX(M+1,1,O+1)] + x[IDX(M+1,0,O)]);
        x[IDX(M+1,N+1,O+1)] = 1.0/3.0*(x[IDX(M,N+1,O+1)]+x[IDX(M+1,N,O+1)]+x[IDX(M+1,N+1,O)]);
    }

    void lin_solve ( int M, int N, int O, int b, double * x, double * x0, double a, double c ) {
	    int i, j, k, l;

        // iterate the solver
        for ( l=0 ; l<relax_steps ; l++ ) {
            // update for each cell
            for (i=1 ; i<=M ; i++ ) { 
                for (j=1 ; j<=N ; j++ ) { 
                    for (k=1 ; k<=O ; k++ ) {
                        x[IDX(i,j,k)] = (x0[IDX(i,j,k)] + a*(x[IDX(i-1,j,k)]+x[IDX(i+1,j,k)]+x[IDX(i,j-1,k)]+x[IDX(i,j+1,k)]+x[IDX(i,j,k-1)]+x[IDX(i,j,k+1)]))/c;
                    }
                }
            }
            set_bnd (M, N, O, b, x);
        }
    }

    void diffuse(int M, int N, int O, int b, double * x, double * x0, double diff, double dt ) {
        double a=dt*diff*M*N*O;
        lin_solve ( M, N, O, b, x, x0, a, 1+6*a );
    }

    void advect(int b, double* x0, double* x, double* uu, double* vv, double* ww, double dt) {
        int i, j, k, i0, j0, k0, i1, j1, k1;
        int N = width;
        double sx0, sx1, sy0, sy1, sz0, sz1, v0, v1;
        double xx, yy, zz, dt0;
        dt0 = dt*N;
        for (k=1; k<=N; k++)
        {
            for (j=1; j<=N; j++)
            {
                for (i=1; i<=N; i++)
                {
                    xx = i-dt0*uu[IDX(i,j,k)];
                    yy = j-dt0*vv[IDX(i,j,k)];
                    zz = k-dt0*ww[IDX(i,j,k)];
                    if (xx<0.5) xx=0.5f; if (xx>N+0.5) xx=N+0.5f; i0=(int)xx; i1=i0+1;
                    if (yy<0.5) yy=0.5f; if (yy>N+0.5) yy=N+0.5f; j0=(int)yy; j1=j0+1;
                    if (zz<0.5) zz=0.5f; if (zz>N+0.5) zz=N+0.5f; k0=(int)zz; k1=k0+1;
                    sx1 = xx-i0; sx0 = 1-sx1;
                    sy1 = yy-j0; sy0 = 1-sy1;
                    sz1 = zz-k0; sz0 = 1-sz1;
                    v0 = sx0*(sy0*x0[IDX(i0,j0,k0)]+sy1*x0[IDX(i0,j1,k0)])+sx1*(sy0*x0[IDX(i1,j0,k0)]+sy1*x0[IDX(i1,j1,k0)]);
                    v1 = sx0*(sy0*x0[IDX(i0,j0,k1)]+sy1*x0[IDX(i0,j1,k1)])+sx1*(sy0*x0[IDX(i1,j0,k1)]+sy1*x0[IDX(i1,j1,k1)]);
                    x[IDX(i,j,k)] = sz0*v0 + sz1*v1;
                }
            }
        }
        set_bnd(N, N, N, b, x);
    }

    void advect_cool(int b, double* x0, double* x, double* y0, double* y, double* uu, double* vv, double* ww, double dt) {
        int i, j, k, i0, j0, k0, i1, j1, k1;
        int N = width;
        double sx0, sx1, sy0, sy1, sz0, sz1, v0, v1;
        double xx, yy, zz, dt0, c0;
        dt0 = dt*N;
        c0 = 1.0f - cooling*dt;
        for (k=1; k<=N; k++)
        {
            for (j=1; j<=N; j++)
            {
                for (i=1; i<=N; i++)
                {
                    xx = i-dt0*uu[IDX(i,j,k)];
                    yy = j-dt0*vv[IDX(i,j,k)];
                    zz = k-dt0*ww[IDX(i,j,k)];
                    if (xx<0.5) xx=0.5f; if (xx>N+0.5) xx=N+0.5f; i0=(int)xx; i1=i0+1;
                    if (yy<0.5) yy=0.5f; if (yy>N+0.5) yy=N+0.5f; j0=(int)yy; j1=j0+1;
                    if (zz<0.5) zz=0.5f; if (zz>N+0.5) zz=N+0.5f; k0=(int)zz; k1=k0+1;
                    sx1 = xx-i0; sx0 = 1-sx1;
                    sy1 = yy-j0; sy0 = 1-sy1;
                    sz1 = zz-k0; sz0 = 1-sz1;
                    v0 = sx0*(sy0*x0[IDX(i0,j0,k0)]+sy1*x0[IDX(i0,j1,k0)])+sx1*(sy0*x0[IDX(i1,j0,k0)]+sy1*x0[IDX(i1,j1,k0)]);
                    v1 = sx0*(sy0*x0[IDX(i0,j0,k1)]+sy1*x0[IDX(i0,j1,k1)])+sx1*(sy0*x0[IDX(i1,j0,k1)]+sy1*x0[IDX(i1,j1,k1)]);
                    x[IDX(i,j,k)] = sz0*v0 + sz1*v1;
                    v0 = sx0*(sy0*y0[IDX(i0,j0,k0)]+sy1*y0[IDX(i0,j1,k0)])+sx1*(sy0*y0[IDX(i1,j0,k0)]+sy1*y0[IDX(i1,j1,k0)]);
                    v1 = sx0*(sy0*y0[IDX(i0,j0,k1)]+sy1*y0[IDX(i0,j1,k1)])+sx1*(sy0*y0[IDX(i1,j0,k1)]+sy1*y0[IDX(i1,j1,k1)]);
                    y[IDX(i,j,k)] = (sz0*v0 + sz1*v1)*c0;
                }
            }
        }
        set_bnd(N, N, N, b,d);
    }

    // void advect(int M, int N, int O, int b, double * d, double * d0, double * u, double * v, double * w, double dt) {
    //     int i, j, k, i0, j0, k0, i1, j1, k1;
    //     double x, y, z, s0, t0, s1, t1, u1, u0, dtx,dty,dtz;
        
    //     dtx=dty=dtz=dt*M;

    //     for (i=1 ; i<=M ; i++ ) { 
    //         for ( j=1 ; j<=N ; j++ ) { 
    //             for ( k=1 ; k<=O ; k++ ) {
    //                 x = i-dtx*u[IDX(i,j,k)]; y = j-dty*v[IDX(i,j,k)]; z = k-dtz*w[IDX(i,j,k)];
    //                 if (x<0.5f) x=0.5f; if (x>M+0.5f) x=M+0.5f; i0=(int)x; i1=i0+1;
    //                 if (y<0.5f) y=0.5f; if (y>N+0.5f) y=N+0.5f; j0=(int)y; j1=j0+1;
    //                 if (z<0.5f) z=0.5f; if (z>O+0.5f) z=O+0.5f; k0=(int)z; k1=k0+1;

    //                 s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1; u1 = z-k0; u0 = 1-u1;
    //                 d[IDX(i,j,k)] = s0*(t0*u0*d0[IDX(i0,j0,k0)]+t1*u0*d0[IDX(i0,j1,k0)]+t0*u1*d0[IDX(i0,j0,k1)]+t1*u1*d0[IDX(i0,j1,k1)])+
    //                     s1*(t0*u0*d0[IDX(i1,j0,k0)]+t1*u0*d0[IDX(i1,j1,k0)]+t0*u1*d0[IDX(i1,j0,k1)]+t1*u1*d0[IDX(i1,j1,k1)]);
    //             }
    //         }
    //     }
        
    //     set_bnd (M, N, O, b, d);
    // }

    void project(int M, int N, int O, double * u, double * v, double * w, double * p, double * div) {
        int i, j, k;

        for ( i=1 ; i<=M ; i++ ) { 
            for ( j=1 ; j<=N ; j++ ) { 
                for ( k=1 ; k<=O ; k++ ) {
                    div[IDX(i,j,k)] = -1.0/3.0*((u[IDX(i+1,j,k)]-u[IDX(i-1,j,k)])/M+(v[IDX(i,j+1,k)]-v[IDX(i,j-1,k)])/M+(w[IDX(i,j,k+1)]-w[IDX(i,j,k-1)])/M);
                    p[IDX(i,j,k)] = 0;
                }
            }
        }	
        
        set_bnd (M, N, O, 0, div); 
        set_bnd(M, N, O, 0, p);

        lin_solve( M, N, O, 0, p, div, 1, 6);

        for (i=1; i<=M ; i++ ) { 
            for (j=1; j<=N ; j++ ) { 
                for (k=1; k<=O ; k++) {
                    u[IDX(i,j,k)] -= 0.5f*M*(p[IDX(i+1,j,k)]-p[IDX(i-1,j,k)]);
                    v[IDX(i,j,k)] -= 0.5f*N*(p[IDX(i,j+1,k)]-p[IDX(i,j-1,k)]);
                    w[IDX(i,j,k)] -= 0.5f*O*(p[IDX(i,j,k+1)]-p[IDX(i,j,k-1)]);
                }
            }
        }
	
	    set_bnd (M, N, O, 1, u); 
        set_bnd (M, N, O, 2, v);
        set_bnd (M, N, O, 3, w);
    }

    void vorticity_confinement(double dt) {
        int i,j,k,ijk;
        int N = width;
        double *curlx = u_prev, *curly = v_prev, *curlz=w_prev, *curl=T0;		// temp buffers
        double dt0 = dt * vc_eps;
        double x,y,z;


        for (k=1; k<N; k++) {
            for (j=1; j<N; j++) {
                for (i=1; i<N; i++) {
                    ijk = IDX(i,j,k);
                        // curlx = dw/dy - dv/dz
                    x = curlx[ijk] = (w[IDX(i,j+1,k)] - w[IDX(i,j-1,k)]) * 0.5f -
                        (v[IDX(i,j,k+1)] - v[IDX(i,j,k-1)]) * 0.5f;

                        // curly = du/dz - dw/dx
                    y = curly[ijk] = (u[IDX(i,j,k+1)] - u[IDX(i,j,k-1)]) * 0.5f -
                        (w[IDX(i+1,j,k)] - w[IDX(i-1,j,k)]) * 0.5f;

                        // curlz = dv/dx - du/dy
                    z = curlz[ijk] = (v[IDX(i+1,j,k)] - v[IDX(i-1,j,k)]) * 0.5f -
                        (u[IDX(i,j+1,k)] - u[IDX(i,j-1,k)]) * 0.5f;

                        // curl = |curl|
                    curl[ijk] = sqrtf(x*x+y*y+z*z);
                }
            }
	    }

        for (k=1; k<N; k++) {
            for (j=1; j<N; j++) {
                for (i=1; i<N; i++) {
                    ijk = IDX(i,j,k);
                    double Nx = (curl[IDX(i+1,j,k)] - curl[IDX(i-1,j,k)]) * 0.5f;
                    double Ny = (curl[IDX(i,j+1,k)] - curl[IDX(i,j-1,k)]) * 0.5f;
                    double Nz = (curl[IDX(i,j,k+1)] - curl[IDX(i,j,k-1)]) * 0.5f;
                    double len1 = 1.0f/(sqrtf(Nx*Nx+Ny*Ny+Nz*Nz)+0.0000001f);
                    Nx *= len1;
                    Ny *= len1;
                    Nz *= len1;
                    u[ijk] += (Ny*curlz[ijk] - Nz*curly[ijk]) * dt0;
                    v[ijk] += (Nz*curlx[ijk] - Nx*curlz[ijk]) * dt0;
                    w[ijk] += (Nx*curly[ijk] - Ny*curlx[ijk]) * dt0;
                }
            }
        }
    }

    void add_buoyancy(double dt) {
        int N = width;
        int i, size=(N+2)*(N+2)*(N+2);

        for (i=0; i<size; i++){
            v[i] += T[i]*buoyancy*dt;
        }
    }

    void dens_step(int M, int N, int O, double * u, double * v,  double * w, double * sd, double visc, double dt) {
        add_source(M, N, O, sd, d, dt);
        SWAP(d0, d);
        diffuse(M, N, O, 0, d0, d, visc, dt);
        SWAP(d0, d);
        advect(0, d0, d, u, v, w, dt);
    }

    void vel_step(int M, int N, int O, double * u, double * v,  double * w, double * u0, double * v0, double * w0, double visc, double dt) {
        //add_source (M, N, O, u, u0, dt); add_source (M, N, O, v, v0, dt);add_source (M, N, O, w, w0, dt);
        add_buoyancy(dt);
	    vorticity_confinement(dt);
        // SWAP (u0, u); diffuse (M, N, O, 1, u, u0, visc, dt);
        // SWAP (v0, v); diffuse (M, N, O, 2, v, v0, visc, dt);
        // SWAP (w0, w); diffuse (M, N, O, 3, w, w0, visc, dt) ;
        project (M, N, O, u, v, w, u0, v0);
        SWAP (u0, u); SWAP (v0, v);SWAP (w0, w);
        advect (1, u, u0, u0, v0, w0, dt); advect (2, v, v0, u0, v0, w0, dt);advect (3, w, w0, u0, v0, w0, dt);
        project (M, N, O, u, v, w, u0, v0);
    }

    void dens_temp_step(int M, int N, int O, double * u, double * v,  double * w, double * sd, double * sT, double visc, double dt) {
        // add_source(M, N, O, sd, d, dt);
        // add_source(M, N, O, sT, T, dt);
        SWAP(d0, d);
        diffuse(M, N, O, 0, d0, d, visc, dt);
        SWAP(d0, d);
        SWAP(T0, T);
        advect_cool(0, d0, d, T0, T, u, v, w, dt);
    }



    //fluid field information
    int width; // grid x
    int height; // grid y
    int depth; // grid z
    int size;
    double cell_size;

    double *d, *d0;			// density
	double *T, *T0;			// temperature
	double *u, *u_prev;			// velocity in x direction
	double *v, *v_prev;			// velocity in y direction
	double *w, *w_prev;			// velocity in z direction

    double diffusion, viscosity, vc_eps;
    double cooling = 1.0;
    double buoyancy = 1.0;

    int relax_steps = 20;


    Vector3D origin; // Center of the grid field


    
    
    // int size;
    // double cell_size;

    // FieldCell *cells;
    // FieldCell *prev_cells;
    // double* div;
    // double* p;
    // Vector3D* vorticity;
    // Vector3D* N;

}; // class GasField

#endif // GAS_FIELD_H