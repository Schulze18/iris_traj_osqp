#include "types.h"
#include "constants.h"
#include "qdldl.h"

#include "qdldl_interface.h"
// Define data structure
c_int Pdata_i[6] = {
0,
1,
0,
2,
1,
3,
};
c_int Pdata_p[5] = {
0,
1,
2,
4,
6,
};
c_float Pdata_x[6] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000020000,
(c_float)0.44609464538381371000,
(c_float)1.00000000000000000000,
(c_float)0.38268269517676895000,
(c_float)1.00000000000000020000,
};
csc Pdata = {12, 4, 4, Pdata_p, Pdata_i, Pdata_x, -1};
c_int Adata_i[4] = {
0,
1,
2,
3,
};
c_int Adata_p[5] = {
0,
1,
2,
3,
4,
};
c_float Adata_x[4] = {
(c_float)0.99869811843404865000,
(c_float)0.99855501379556200000,
(c_float)0.99878852408893137000,
(c_float)0.99863256408359058000,
};
csc Adata = {4, 4, 4, Adata_p, Adata_i, Adata_x, -1};
c_float qdata[4] = {
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
};
c_float ldata[4] = {
(c_float)-19.45842221764716500000,
(c_float)-20.93677198540969900000,
(c_float)-18.57886375586375400000,
(c_float)-20.12218699423034400000,
};
c_float udata[4] = {
(c_float)19.45842221764716500000,
(c_float)20.93677198540969900000,
(c_float)18.57886375586375400000,
(c_float)20.12218699423034400000,
};
OSQPData data = {4, 4, &Pdata, &Adata, qdata, ldata, udata};

// Define settings structure
OSQPSettings settings = {(c_float)0.10000000000000001000, (c_float)0.00000100000000000000, 10, 4000, (c_float)0.00100000000000000000, (c_float)0.00100000000000000000, (c_float)0.00010000000000000000, (c_float)0.00010000000000000000, (c_float)1.60000000000000010000, (enum linsys_solver_type) LINSYS_SOLVER, 0, 25, 1, 
#ifdef PROFILING
(c_float)0.00000000000000000000, 
#endif  // PROFILING
};

// Define scaling structure
c_float Dscaling[4] = {
(c_float)0.51324722388247546000,
(c_float)0.47693838118475462000,
(c_float)0.53759397625901606000,
(c_float)0.49628430765002324000,
};
c_float Dinvscaling[4] = {
(c_float)1.94837878018212600000,
(c_float)2.09670691110226180000,
(c_float)1.86013989025463690000,
(c_float)2.01497404730595300000,
};
c_float Escaling[4] = {
(c_float)1.94584222176471640000,
(c_float)2.09367719854096990000,
(c_float)1.85788637558637550000,
(c_float)2.01221869942303440000,
};
c_float Einvscaling[4] = {
(c_float)0.51391628201647488000,
(c_float)0.47762854784723952000,
(c_float)0.53824604838085732000,
(c_float)0.49696387390035240000,
};
OSQPScaling scaling = {(c_float)1.00000000000000000000, Dscaling, Escaling, (c_float)1.00000000000000000000, Dinvscaling, Einvscaling};

// Define linsys_solver structure
c_int linsys_solver_L_i[6] = {
1,
3,
3,
5,
7,
7,
};
c_int linsys_solver_L_p[9] = {
0,
1,
2,
3,
3,
4,
5,
6,
6,
};
c_float linsys_solver_L_x[6] = {
(c_float)-0.09987885240889314600,
(c_float)0.40562951117949186000,
(c_float)-0.09986981184340487600,
(c_float)-0.09986325640835906600,
(c_float)0.34797950005686740000,
(c_float)-0.09985550137955620500,
};
csc linsys_solver_L = {593916600, 8, 8, linsys_solver_L_p, linsys_solver_L_i, linsys_solver_L_x, -1};
c_float linsys_solver_Dinv[8] = {
(c_float)-0.10000000000000001000,
(c_float)0.90929024900196642000,
(c_float)-0.10000000000000001000,
(c_float)1.08838604555671160000,
(c_float)-0.10000000000000001000,
(c_float)0.90931600629636145000,
(c_float)-0.10000000000000001000,
(c_float)1.03461139441377380000,
};
c_int linsys_solver_P[8] = {
6,
2,
4,
0,
7,
3,
5,
1,
};
c_float linsys_solver_bp[8];
qdldl_solver linsys_solver = {QDLDL_SOLVER, &solve_linsys_qdldl, &linsys_solver_L, linsys_solver_Dinv, linsys_solver_P, linsys_solver_bp};

// Define solution
c_float xsolution[4];
c_float ysolution[4];

OSQPSolution solution = {xsolution, ysolution};

// Define info
OSQPInfo info = {0, "Unsolved", OSQP_UNSOLVED, (c_float)0.0, (c_float)0.0, (c_float)0.0};

// Define workspace
c_float work_rho_vec[4] = {
(c_float)0.10000000000000001000,
(c_float)0.10000000000000001000,
(c_float)0.10000000000000001000,
(c_float)0.10000000000000001000,
};
c_float work_rho_inv_vec[4] = {
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
};
c_float work_x[4];
c_float work_y[4];
c_float work_z[4];
c_float work_xz_tilde[8];
c_float work_x_prev[4];
c_float work_z_prev[4];
c_float work_Ax[4];
c_float work_Px[4];
c_float work_Aty[4];
c_float work_delta_y[4];
c_float work_Atdelta_y[4];
c_float work_delta_x[4];
c_float work_Pdelta_x[4];
c_float work_Adelta_x[4];
c_float work_D_temp[4];
c_float work_D_temp_A[4];
c_float work_E_temp[4];

OSQPWorkspace workspace = {
&data, (LinSysSolver *)&linsys_solver,
work_rho_vec, work_rho_inv_vec,
work_x, work_y, work_z, work_xz_tilde,
work_x_prev, work_z_prev,
work_Ax, work_Px, work_Aty,
work_delta_y, work_Atdelta_y,
work_delta_x, work_Pdelta_x, work_Adelta_x,
work_D_temp, work_D_temp_A, work_E_temp,
&settings, &scaling, &solution, &info};

