//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// DSL.cpp
//
// Code generation for function 'DSL'
//

// Include files
#include "DSL.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <emmintrin.h>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

void DSL(const double z[22], const double k[24], const double param[4],
         const double ref[15], double t, double F1[3], double F2[3],
         double xidot[4])
{
  static const signed char iv[6]{0, 0, 1, 0, 0, 0};
  __m128d r;
  __m128d r1;
  double dv[36];
  double c_z[9];
  double h[6];
  double h_dt[6];
  double b_z[2];
  double parl_comps[2];
  double Deter;
  double Mt;
  double absxk;
  double b_h_dt_tmp;
  double b_h_dt_tmp_tmp;
  double b_ref_idx_2_tmp;
  double b_ref_idx_3_tmp;
  double b_ref_idx_3_tmp_tmp;
  double b_t;
  double c_h_dt_tmp;
  double c_h_dt_tmp_tmp;
  double c_ref_idx_2_tmp;
  double c_ref_idx_3_tmp;
  double d_h_dt_tmp;
  double d_ref_idx_3_tmp;
  double e_h_dt_tmp;
  double e_ref_idx_3_tmp;
  double f_h_dt_tmp;
  double f_ref_idx_3_tmp;
  double g_h_dt_tmp;
  double g_ref_idx_3_tmp;
  double h_dt_tmp;
  double h_dt_tmp_tmp;
  double h_h_dt_tmp;
  double i_h_dt_tmp;
  double j_h_dt_tmp;
  double k_h_dt_tmp;
  double l_h_dt_tmp;
  double m_h_dt_tmp;
  double n_h_dt_tmp;
  double o_h_dt_tmp;
  double p_h_dt_tmp;
  double q_h_dt_tmp;
  double r_h_dt_tmp;
  double ref_1_dt_tmp;
  double ref_1_tmp;
  double ref_2_dt_tmp;
  double ref_2_tmp;
  double ref_3_tmp;
  double ref_idx_2_tmp;
  double ref_idx_3_tmp;
  double ref_idx_3_tmp_tmp;
  double s_h_dt_tmp;
  double scale;
  double t_h_dt_tmp;
  double u_h_dt_tmp;
  double v_h_dt_tmp;
  double w_h_dt_tmp;
  double x_h_dt_tmp;
  double y_h_dt_tmp;
  //  Payload position
  //  Payload linear velocity
  //  Angular velocity UAV1
  //  Angular velocity UAV2
  //  DEA controller states
  //  Physical parameters
  //  Payload mass
  //  UAV mass (identical for both UAVs)
  //  Length of pendulum
  //  Gravitational acceleration
  //  Controller gains (assumed matrix with 6 rows)
  //  Gain vector for some state or error (row 1)
  //  Gain vector (row 2)
  //  Gain vector (row 3)
  //  Gain vector (first two elements of row 4)
  //  Gain vector (first two elements of row 5)
  //  Gain vector (first two elements of row 6)
  //  Reference signals for desired trajectory or orientation
  //  X-axis
  //  Y-axis
  //  Z-axis
  //  Reference values for direction vector components
  //  Desired value for q1(3)
  //  Desired value for q2(1)
  //  Desired value for q2(2)
  //     %% Vector groupings for convenience in control computation
  //  Payload position vector
  //  Direction vector from UAV1 to payload
  //  Direction vector from UAV2 to payload
  //  Payload velocity vector
  //  Angular velocity of link 1
  //  Angular velocity of link 2
  //  Output and its derivatives for tracking control
  //  Output vector
  //  Payload position x
  //  Payload position y
  //  Payload position z
  //
  //  First derivative of h (velocity and angular rates)
  //  x-velocity
  //  y-velocity
  //  z-velocity
  //  time derivative of q13
  //  time derivative of q21
  //  time derivative of q22
  //  Second derivative of h (acceleration terms)
  //  acceleration in x
  //  acceleration in y
  //  acceleration in z
  //  second derivative of q13
  //  second derivative of q21
  //  second derivative of q22
  //  Third derivative of h
  //  Break up long expressions for readability
  //
  //
  //  third derivative of q13
  //  third derivative of q21
  //  third derivative of q22
  //   %% Reference Signals and Their Derivatives (up to 4th order)
  //  Reference for payload position  (p1)
  scale = ref[1] * t + ref[3];
  ref_1_tmp = std::sin(scale);
  ref_1_dt_tmp = std::cos(scale);
  //  Reference for payload position (p2)
  scale = ref[5] * t + ref[7];
  ref_2_tmp = std::sin(scale);
  ref_2_dt_tmp = std::cos(scale);
  //  Reference for payload position (p3)
  scale = ref[9] * t + ref[11];
  ref_3_tmp = std::sin(scale);
  scale = std::cos(scale);
  //  Reference for q
  //
  //  Constant, so time derivative is zero
  //  Target value for q21
  //  Target value for q22
  //  Linear Subsystems for Reference Tracking (Error Feedback)
  //  --- Subsystem 1: Tracking x-position (p1) ---
  //  Position error
  //  Velocity error
  //  Acceleration error
  //  error
  //  Error state
  //  Virtual control input (4th order tracking)
  //  --- Subsystem 2: Tracking y-position (p2) ---
  //  --- Subsystem 3: Tracking z-position (p3) ---
  //  --- Subsystem 4: Tracking q13 (direction vector z-component of q1) ---
  //  --- Subsystem 5: Tracking q21 (direction vector x-component of q2) ---
  //  --- Subsystem 6: Tracking q22 (direction vector y-component of q2) ---
  //  Combined Virtual Control Input (Used later for force computation)
  //  Combined vector for all 6 subsystems
  //  Decoupling Matrix & Input Matrix % the_friend;
  //  Linearizing Input
  absxk = -z[19] * z[8];
  ref_idx_2_tmp = absxk + z[18];
  b_ref_idx_2_tmp = z[5] * z[6];
  ref_idx_3_tmp_tmp = z[8] * z[19];
  ref_idx_3_tmp = ref_idx_3_tmp_tmp - z[18];
  b_ref_idx_3_tmp = z[5] * z[5];
  b_t = z[7] * z[15];
  c_ref_idx_3_tmp = z[8] * z[21];
  d_ref_idx_3_tmp = z[4] * z[12];
  c_ref_idx_2_tmp = z[5] * z[7];
  e_ref_idx_3_tmp = z[6] * z[17] - z[8] * z[15];
  f_ref_idx_3_tmp = -z[6] * z[16] + b_t;
  g_ref_idx_3_tmp = z[3] * z[3];
  b_ref_idx_3_tmp_tmp = z[3] * z[4];
  Deter = (g_ref_idx_3_tmp * z[12] + b_ref_idx_3_tmp_tmp * z[13]) - z[12];
  h_dt_tmp = z[12] * z[14];
  Mt = z[16] * z[16];
  b_h_dt_tmp = z[12] * z[12];
  h_dt_tmp_tmp = z[13] * z[13];
  c_h_dt_tmp = 2.0 * h_dt_tmp_tmp;
  d_h_dt_tmp = z[14] * z[14];
  e_h_dt_tmp = z[12] * z[13];
  f_h_dt_tmp = 3.0 * z[12] * z[14];
  g_h_dt_tmp = 2.0 * z[20] * z[13];
  h_h_dt_tmp = 1.5 * z[13] * z[14];
  i_h_dt_tmp = 2.0 * z[12] * z[14];
  j_h_dt_tmp = rt_powd_snf(z[5], 3.0);
  k_h_dt_tmp = z[15] * z[15];
  l_h_dt_tmp = z[17] * z[17];
  m_h_dt_tmp = z[15] * z[16];
  n_h_dt_tmp = 2.0 * z[21] * z[17];
  o_h_dt_tmp = 2.0 * z[14] * f_ref_idx_3_tmp;
  b_h_dt_tmp_tmp = z[7] * z[16];
  p_h_dt_tmp = z[6] * z[15] + b_h_dt_tmp_tmp;
  q_h_dt_tmp = z[19] * ref_idx_3_tmp;
  r_h_dt_tmp = z[3] * z[6];
  s_h_dt_tmp = z[6] * z[6];
  t_h_dt_tmp = z[8] * z[8];
  u_h_dt_tmp = b_h_dt_tmp - h_dt_tmp_tmp;
  v_h_dt_tmp = 2.0 * z[12] * z[13];
  c_h_dt_tmp_tmp = z[4] * z[7];
  w_h_dt_tmp = r_h_dt_tmp + c_h_dt_tmp_tmp;
  x_h_dt_tmp = z[3] * z[8];
  y_h_dt_tmp = z[4] * z[6];
  h_dt[0] = -(
      ((((((((((((h_dt_tmp + z[15] * z[17]) * z[8] +
                 (2.0 * z[6] * z[16] - 2.0 * z[7] * z[15]) * z[13]) +
                (-Mt - l_h_dt_tmp) * z[6]) +
               m_h_dt_tmp * z[7]) *
                  z[19] -
              2.0 * z[21] * (z[13] - z[16]) * z[8]) -
             n_h_dt_tmp * z[7]) -
            h_dt_tmp * z[18]) +
           g_h_dt_tmp) *
              param[2] +
          z[6] * z[8] * z[19] * ref_idx_3_tmp) *
             j_h_dt_tmp +
         (((((((((b_h_dt_tmp - c_h_dt_tmp) + d_h_dt_tmp) + k_h_dt_tmp) + Mt) *
                  z[8] -
              z[17] * p_h_dt_tmp) *
                 z[3] +
             z[4] * (e_h_dt_tmp * z[8] + o_h_dt_tmp)) *
                z[19] +
            ((-b_h_dt_tmp * z[18] + (c_h_dt_tmp - d_h_dt_tmp) * z[18]) -
             2.0 * z[21] * f_ref_idx_3_tmp) *
                z[3]) -
           2.0 *
               ((-(z[14] * z[21] * z[8]) + e_h_dt_tmp * z[18] / 2.0) +
                z[14] * z[20]) *
               z[4]) *
              param[2] +
          q_h_dt_tmp *
              (((z[3] * s_h_dt_tmp - z[3] * t_h_dt_tmp) + y_h_dt_tmp * z[7]) -
               z[6])) *
             b_ref_idx_3_tmp) +
        (((((((f_h_dt_tmp * z[8] - 2.0 * z[13] * f_ref_idx_3_tmp) *
                  g_ref_idx_3_tmp +
              2.0 * (h_h_dt_tmp * z[8] + z[12] * f_ref_idx_3_tmp) * z[4] *
                  z[3]) -
             i_h_dt_tmp * z[8]) *
                z[19] +
            ((-2.0 * z[8] * z[21] * z[13] - f_h_dt_tmp * z[18]) + g_h_dt_tmp) *
                g_ref_idx_3_tmp) -
           2.0 * z[4] *
               ((-(z[12] * z[21] * z[8]) + h_h_dt_tmp * z[18]) +
                z[12] * z[20]) *
               z[3]) +
          i_h_dt_tmp * z[18]) *
             param[2] -
         x_h_dt_tmp * z[19] * ref_idx_3_tmp * (w_h_dt_tmp - 1.0)) *
            z[5]) +
       2.0 * z[3] * ref_idx_3_tmp *
           ((u_h_dt_tmp * g_ref_idx_3_tmp + v_h_dt_tmp * z[4] * z[3]) -
            b_h_dt_tmp) *
           param[2]) /
      j_h_dt_tmp / param[2]);
  h_dt_tmp = 2.0 * z[20] * z[12];
  c_h_dt_tmp = z[3] * z[12];
  e_h_dt_tmp = m_h_dt_tmp * z[6];
  f_h_dt_tmp = r_h_dt_tmp * z[7] - z[4] * s_h_dt_tmp;
  h_dt[1] = -(
      ((((((((((-2.0 * z[13] * z[14] + z[16] * z[17]) * z[8] +
               (-k_h_dt_tmp - l_h_dt_tmp) * z[7]) +
              e_h_dt_tmp) *
                 z[19] -
             2.0 * z[21] * z[15] * z[8]) +
            n_h_dt_tmp * z[6]) +
           2.0 * z[13] * z[14] * z[18]) *
              param[2] +
          2.0 * z[7] * z[8] * z[19] * ref_idx_3_tmp) *
             j_h_dt_tmp +
         (((((((-3.0 * z[12] * z[13] * z[3] -
                z[4] * (((h_dt_tmp_tmp - d_h_dt_tmp) - k_h_dt_tmp) - Mt)) *
                   z[8] -
               o_h_dt_tmp * z[3]) -
              z[4] * z[17] * p_h_dt_tmp) *
                 z[19] -
             2.0 * z[21] * z[14] * z[3] * z[8]) +
            (3.0 * z[12] * z[13] * z[18] + 2.0 * z[20] * z[14]) * z[3]) -
           2.0 *
               ((-h_dt_tmp_tmp + d_h_dt_tmp) * z[18] / 2.0 +
                z[21] * f_ref_idx_3_tmp) *
               z[4]) *
              param[2] +
          q_h_dt_tmp *
              (((f_h_dt_tmp - 2.0 * z[4] * t_h_dt_tmp) + z[4]) - z[7])) *
             b_ref_idx_3_tmp) +
        (((((((3.0 * z[14] *
                   ((c_h_dt_tmp * z[4] - z[13] * g_ref_idx_3_tmp) +
                    z[13] / 3.0) *
                   z[8] -
               2.0 * f_ref_idx_3_tmp * Deter) *
                  z[19] -
              2.0 * z[21] * Deter * z[8]) +
             (3.0 * z[13] * z[14] * z[18] + h_dt_tmp) * g_ref_idx_3_tmp) +
            z[4] * (-3.0 * z[12] * z[14] * z[18] + g_h_dt_tmp) * z[3]) -
           z[13] * z[14] * z[18]) -
          h_dt_tmp) *
             param[2] +
         ref_idx_3_tmp_tmp * ref_idx_3_tmp *
             (((g_ref_idx_3_tmp * z[7] - b_ref_idx_3_tmp_tmp * z[6]) + z[4]) -
              z[7])) *
            z[5]) +
       2.0 *
           (((-2.0 * z[12] * z[13] * rt_powd_snf(z[3], 3.0) +
              z[4] * u_h_dt_tmp * g_ref_idx_3_tmp) +
             v_h_dt_tmp * z[3]) -
            b_h_dt_tmp * z[4]) *
           ref_idx_3_tmp * param[2]) /
      j_h_dt_tmp / param[2]);
  h_dt[2] = -0.0;
  h_dt[3] =
      -((((absxk * b_ref_idx_3_tmp +
           ((-b_h_dt_tmp - h_dt_tmp_tmp) * param[2] - z[19] * w_h_dt_tmp) *
               z[5]) +
          z[14] * (c_h_dt_tmp + z[4] * z[13]) * param[2]) +
         ref_idx_3_tmp_tmp) /
        param[2]);
  h_dt_tmp = z[3] * ref_idx_3_tmp;
  h_dt[4] = -((((h_dt_tmp * s_h_dt_tmp + (((t_h_dt_tmp * z[19] - z[8] * z[18]) -
                                           param[2] * (Mt + l_h_dt_tmp)) *
                                              z[5] +
                                          c_h_dt_tmp_tmp * ref_idx_3_tmp) *
                                             z[6]) +
                param[2] * z[15] * (b_h_dt_tmp_tmp + z[8] * z[17]) * z[5]) -
               h_dt_tmp) /
              param[2] / z[5]);
  h_dt_tmp = f_h_dt_tmp - z[4] * t_h_dt_tmp;
  h_dt[5] = -((((z[7] * t_h_dt_tmp * z[19] +
                 (param[2] * z[16] * z[17] - z[7] * z[18]) * z[8]) -
                ((k_h_dt_tmp + l_h_dt_tmp) * z[7] - e_h_dt_tmp) * param[2]) *
                   z[5] +
               ref_idx_3_tmp * h_dt_tmp) /
              param[2] / z[5]);
  h[0] =
      (((k[0] * ((ref[0] * ref_1_tmp + ref[2]) - z[0]) +
         k[6] * (ref[0] * ref[1] * ref_1_dt_tmp - z[9])) +
        k[12] * (-ref[0] * (ref[1] * ref[1]) * ref_1_tmp -
                 (ref_idx_2_tmp * z[3] + b_ref_idx_2_tmp * z[19]) / z[5])) +
       k[18] * (-ref[0] * rt_powd_snf(ref[1], 3.0) * ref_1_dt_tmp -
                ((((((-z[13] + z[16]) * z[8] - z[7] * z[17]) * z[19] +
                    z[6] * z[21]) +
                   z[13] * z[18]) *
                      b_ref_idx_3_tmp +
                  ((((z[6] * z[16] - b_t) * z[19] - c_ref_idx_3_tmp) + z[20]) *
                       z[3] +
                   z[4] * z[14] * ref_idx_3_tmp) *
                      z[5]) -
                 (z[3] * z[13] - d_ref_idx_3_tmp) * z[3] * ref_idx_3_tmp) /
                    b_ref_idx_3_tmp)) +
      ref[0] * rt_powd_snf(ref[1], 4.0) * ref_1_tmp;
  h[1] =
      (((k[1] * ((ref[4] * ref_2_tmp + ref[6]) - z[1]) +
         k[7] * (ref[4] * ref[5] * ref_2_dt_tmp - z[10])) +
        k[13] * (-ref[4] * (ref[5] * ref[5]) * ref_2_tmp -
                 (ref_idx_2_tmp * z[4] + c_ref_idx_2_tmp * z[19]) / z[5])) +
       k[19] * (-ref[4] * rt_powd_snf(ref[5], 3.0) * ref_2_dt_tmp -
                (((e_ref_idx_3_tmp * z[19] + z[7] * z[21]) * b_ref_idx_3_tmp +
                  ((-z[3] * z[14] * z[8] - z[4] * f_ref_idx_3_tmp) * z[19] +
                   (z[3] * z[14] * z[18] - z[4] * (c_ref_idx_3_tmp - z[20]))) *
                      z[5]) -
                 ref_idx_3_tmp * Deter) /
                    b_ref_idx_3_tmp)) +
      ref[4] * rt_powd_snf(ref[5], 4.0) * ref_2_tmp;
  h[2] = (((k[2] * ((ref[8] * ref_3_tmp + ref[10]) - z[2]) +
            k[8] * (ref[8] * ref[9] * scale - z[11])) +
           k[14] *
               (-ref[8] * (ref[9] * ref[9]) * ref_3_tmp - (param[3] + z[18]))) +
          k[20] * (-ref[8] * rt_powd_snf(ref[9], 3.0) * scale - z[20])) +
         ref[8] * rt_powd_snf(ref[9], 4.0) * ref_3_tmp;
  h[3] = (ref[12] - z[5]) * k[3] +
         (0.0 - (-z[3] * z[13] + d_ref_idx_3_tmp)) * k[9];
  h[4] =
      (ref[13] - z[6]) * k[4] + (0.0 - (-z[7] * z[17] + z[8] * z[16])) * k[10];
  h[5] = (ref[14] - z[7]) * k[5] + (0.0 - e_ref_idx_3_tmp) * k[11];
  c_h_dt_tmp = z[4] * z[5];
  e_ref_idx_3_tmp = c_h_dt_tmp * z[7];
  f_ref_idx_3_tmp = b_ref_idx_3_tmp * z[8];
  Deter = z[3] * z[5];
  Mt = ((Deter * z[6] + e_ref_idx_3_tmp) + f_ref_idx_3_tmp) - z[8];
  b_h_dt_tmp = z[4] * z[8] - c_ref_idx_2_tmp;
  dv[1] = b_ref_idx_3_tmp * param[2] * param[1] * b_h_dt_tmp * z[3] / Mt /
          ref_idx_3_tmp;
  d_ref_idx_3_tmp = x_h_dt_tmp - b_ref_idx_2_tmp;
  dv[7] = -z[3] * d_ref_idx_3_tmp * param[1] * param[2] * b_ref_idx_3_tmp / Mt /
          ref_idx_3_tmp;
  dv[13] = (z[3] * z[7] - y_h_dt_tmp) * z[3] * param[1] * param[2] *
           b_ref_idx_3_tmp / Mt / ref_idx_3_tmp;
  dv[19] = -1.0 / Mt * param[2] * param[1] * b_h_dt_tmp;
  b_h_dt_tmp = w_h_dt_tmp * z[5];
  c_ref_idx_3_tmp = (b_ref_idx_3_tmp - 1.0) * z[8] + b_h_dt_tmp;
  dv[25] = param[1] * param[2] * z[3] * b_ref_idx_3_tmp * z[19] *
           (h_dt_tmp + c_ref_idx_2_tmp * z[8]) / c_ref_idx_3_tmp /
           ref_idx_3_tmp / z[8];
  b_t = z[5] * z[8];
  dv[31] = -param[1] * z[3] *
           (z[3] * (s_h_dt_tmp - 1.0) + z[6] * (c_h_dt_tmp_tmp + b_t)) *
           b_ref_idx_3_tmp * param[2] * z[19] / c_ref_idx_3_tmp /
           ref_idx_3_tmp / z[8];
  h_dt_tmp_tmp = -param[1] * param[2];
  dv[2] =
      h_dt_tmp_tmp * b_ref_idx_3_tmp *
      (((g_ref_idx_3_tmp * z[8] + e_ref_idx_3_tmp) + f_ref_idx_3_tmp) - z[8]) /
      Mt / ref_idx_3_tmp;
  dv[8] = -d_ref_idx_3_tmp * z[4] * b_ref_idx_3_tmp * param[1] * param[2] / Mt /
          ref_idx_3_tmp;
  scale = (g_ref_idx_3_tmp + b_ref_idx_3_tmp) - 1.0;
  absxk = b_ref_idx_3_tmp_tmp * z[7];
  f_ref_idx_3_tmp = (f_ref_idx_3_tmp + b_h_dt_tmp) - z[8];
  dv[14] = param[1] * b_ref_idx_3_tmp * param[2] * (scale * z[6] + absxk) /
           f_ref_idx_3_tmp / ref_idx_3_tmp;
  dv[20] = 1.0 / Mt * d_ref_idx_3_tmp * param[1] * param[2];
  b_h_dt_tmp = b_ref_idx_3_tmp * z[6];
  dv[26] = param[1] *
           ((scale * t_h_dt_tmp + e_ref_idx_3_tmp * z[8]) +
            z[6] * (((g_ref_idx_3_tmp * z[6] + absxk) + b_h_dt_tmp) - z[6])) *
           b_ref_idx_3_tmp * param[2] * z[19] / c_ref_idx_3_tmp /
           ref_idx_3_tmp / z[8];
  dv[32] = param[1] *
           ((((b_h_dt_tmp * z[7] - c_h_dt_tmp * z[6] * z[8]) -
              b_ref_idx_3_tmp_tmp * s_h_dt_tmp) +
             z[7] * (g_ref_idx_3_tmp - 1.0) * z[6]) +
            b_ref_idx_3_tmp_tmp) *
           b_ref_idx_3_tmp * param[2] * z[19] / f_ref_idx_3_tmp /
           ref_idx_3_tmp / z[8];
  dv[3] = Deter / Mt;
  dv[9] = c_h_dt_tmp / Mt;
  dv[15] = (b_ref_idx_3_tmp - 1.0) / Mt;
  dv[21] = ref_idx_2_tmp / z[5] / Mt;
  e_ref_idx_3_tmp = -1.0 / z[8] / Mt * z[19];
  dv[27] = e_ref_idx_3_tmp * ((Deter * z[8] - b_h_dt_tmp) + z[6]);
  dv[33] =
      e_ref_idx_3_tmp * ((c_h_dt_tmp * z[8] - b_ref_idx_3_tmp * z[7]) + z[7]);
  dv[4] = 0.0;
  dv[10] = 0.0;
  dv[16] = 0.0;
  dv[22] = 0.0;
  dv[28] = -z[6] * z[7] * param[1] * param[2] / z[8];
  dv[34] = 1.0 / z[8] * (s_h_dt_tmp - 1.0) * param[1] * param[2];
  dv[5] = 0.0;
  dv[11] = 0.0;
  dv[17] = 0.0;
  dv[23] = 0.0;
  dv[29] = 1.0 / z[8] * (s_h_dt_tmp + t_h_dt_tmp) * param[1] * param[2];
  c_h_dt_tmp = z[6] * z[7];
  dv[35] = c_h_dt_tmp * param[1] * param[2] / z[8];
  for (int i{0}; i < 6; i++) {
    dv[6 * i] = iv[i];
    h_dt[i] += h[i];
  }
  for (int i{0}; i < 6; i++) {
    e_ref_idx_3_tmp = 0.0;
    for (int i1{0}; i1 < 6; i1++) {
      e_ref_idx_3_tmp += dv[i + 6 * i1] * h_dt[i1];
    }
    h[i] = e_ref_idx_3_tmp;
  }
  //  Physical Controller, u
  //  Input Transformation
  //     %% Input Transformation (Force Computation for UAVs)
  //  Total mass (payload + one UAV)
  Mt = param[0] + param[1];
  //  Determinant-like expression for matrix inversion (used in transformation)
  f_ref_idx_3_tmp = w_h_dt_tmp + b_t;
  d_ref_idx_3_tmp = param[1] * param[1];
  Deter = Mt * Mt - d_ref_idx_3_tmp * (f_ref_idx_3_tmp * f_ref_idx_3_tmp);
  //  Inverse of the coupling matrix between UAVs (symmetric 2x2 matrix)
  //  Compute parallel components of control inputs (2 elements: one per UAV)
  //  These are transformed versions of u11 and u21 to decouple the system
  scale = 3.3121686421112381E-170;
  absxk = std::abs(z[12]);
  if (absxk > 3.3121686421112381E-170) {
    e_ref_idx_3_tmp = 1.0;
    scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    e_ref_idx_3_tmp = b_t * b_t;
  }
  absxk = std::abs(z[13]);
  if (absxk > scale) {
    b_t = scale / absxk;
    e_ref_idx_3_tmp = e_ref_idx_3_tmp * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    e_ref_idx_3_tmp += b_t * b_t;
  }
  absxk = std::abs(z[14]);
  if (absxk > scale) {
    b_t = scale / absxk;
    e_ref_idx_3_tmp = e_ref_idx_3_tmp * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    e_ref_idx_3_tmp += b_t * b_t;
  }
  e_ref_idx_3_tmp = scale * std::sqrt(e_ref_idx_3_tmp);
  scale = 3.3121686421112381E-170;
  absxk = std::abs(z[15]);
  if (absxk > 3.3121686421112381E-170) {
    c_ref_idx_3_tmp = 1.0;
    scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    c_ref_idx_3_tmp = b_t * b_t;
  }
  absxk = std::abs(z[16]);
  if (absxk > scale) {
    b_t = scale / absxk;
    c_ref_idx_3_tmp = c_ref_idx_3_tmp * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    c_ref_idx_3_tmp += b_t * b_t;
  }
  absxk = std::abs(z[17]);
  if (absxk > scale) {
    b_t = scale / absxk;
    c_ref_idx_3_tmp = c_ref_idx_3_tmp * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    c_ref_idx_3_tmp += b_t * b_t;
  }
  c_ref_idx_3_tmp = scale * std::sqrt(c_ref_idx_3_tmp);
  b_z[0] = (z[18] - ref_idx_3_tmp_tmp) / z[5];
  b_z[1] = z[19];
  c_ref_idx_3_tmp *= c_ref_idx_3_tmp;
  b_t = d_ref_idx_3_tmp * param[2];
  scale = e_ref_idx_3_tmp * e_ref_idx_3_tmp;
  parl_comps[0] = h_dt_tmp_tmp * scale * Mt / Deter +
                  b_t * c_ref_idx_3_tmp * f_ref_idx_3_tmp / Deter;
  parl_comps[1] = h_dt_tmp_tmp * c_ref_idx_3_tmp * Mt / Deter +
                  b_t * scale * f_ref_idx_3_tmp / Deter;
  absxk = f_ref_idx_3_tmp * param[1];
  r = _mm_loadu_pd(&b_z[0]);
  r1 = _mm_loadu_pd(&parl_comps[0]);
  _mm_storeu_pd(&b_z[0], _mm_sub_pd(r, r1));
  //  Compute total force for UAV1 in 3D space
  //  Transformation matrix maps input components (parallel, lateral,
  //  tangential)
  c_z[0] = z[3];
  c_z[3] = -z[4] * z[3] / z[5];
  c_z[6] = (g_ref_idx_3_tmp - 1.0) / z[5];
  c_z[1] = z[4];
  c_z[4] = (1.0 - z[4] * z[4]) / z[5];
  c_z[7] = b_ref_idx_3_tmp_tmp / z[5];
  c_z[2] = z[5];
  c_z[5] = -z[4];
  c_z[8] = z[3];
  scale = Mt * b_z[0] + b_z[1] * absxk;
  b_t = h[1];
  c_ref_idx_3_tmp = h[2];
  r = _mm_loadu_pd(&c_z[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(scale));
  r1 = _mm_loadu_pd(&c_z[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(b_t));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&c_z[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c_ref_idx_3_tmp));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&F1[0], r);
  F1[2] = (c_z[2] * scale + c_z[5] * b_t) + c_z[8] * c_ref_idx_3_tmp;
  //  Compute total force for UAV2 in 3D space
  c_z[0] = z[6];
  c_z[3] = -z[7] * z[6] / z[8];
  c_z[6] = (s_h_dt_tmp - 1.0) / z[8];
  c_z[1] = z[7];
  c_z[4] = (1.0 - z[7] * z[7]) / z[8];
  c_z[7] = c_h_dt_tmp / z[8];
  c_z[2] = z[8];
  c_z[5] = -z[7];
  c_z[8] = z[6];
  scale = b_z[0] * absxk + b_z[1] * Mt;
  b_t = h[4];
  c_ref_idx_3_tmp = h[5];
  r = _mm_loadu_pd(&c_z[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(scale));
  r1 = _mm_loadu_pd(&c_z[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(b_t));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&c_z[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c_ref_idx_3_tmp));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&F2[0], r);
  F2[2] = (c_z[2] * scale + c_z[5] * b_t) + c_z[8] * c_ref_idx_3_tmp;
  //     %% Controller2Dyn Output Interface
  //  Pack all state derivatives into output vector
  xidot[0] = z[20];
  xidot[1] = z[21];
  xidot[2] = h[0];
  xidot[3] = h[3];
}

// End of code generation (DSL.cpp)
