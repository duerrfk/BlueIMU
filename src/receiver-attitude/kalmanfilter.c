/**
 * This file is part of BlueIMU.
 *
 * Copyright 2016 Frank Duerr
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "kalmanfilter.h"
#include <stdio.h>

void kf_init(struct kf *filter, float phi, float phidot, float bias_phidot,
	     float sigma_phi_measurement, float sigma_phidot_measurement, 
	     float sigma_phidotdot, float sigma_bias)
{
     filter->x[0] = phi;
     filter->x[1] = phidot;
     filter->x[2] = bias_phidot;

     // Assume perfectly accurate initial angle, angular velocity, and bias.
     filter->P[0][0] = 0.0f;
     filter->P[0][1] = 0.0f;
     filter->P[0][2] = 0.0f;
     filter->P[1][0] = 0.0f;
     filter->P[1][1] = 0.0f;
     filter->P[1][2] = 0.0f;
     filter->P[2][0] = 0.0f;
     filter->P[2][1] = 0.0f;
     filter->P[2][2] = 0.0f;

     filter->R[0][0] = sigma_phi_measurement*sigma_phi_measurement;
     filter->R[0][1] = 0.0f;
     filter->R[1][0] = 0.0f;
     filter->R[1][1] = sigma_phidot_measurement*sigma_phidot_measurement;

     filter->sigma_phidotdot_square = sigma_phidotdot*sigma_phidotdot;

     filter->sigma_bias_square = sigma_bias*sigma_bias;
}

void kf_update(struct kf *filter, float phi_m, float phidot_m, float t_delta)
{
     float x_pred[3];
     float P_pred[3][3];
     float y[2];
     float S[2][2];
     float Sinv[2][2];
     float K[3][2];
     float temp[2][2];

     kf_predict(filter, t_delta, x_pred, P_pred);

     // Measurement residual: y(k) = z(k) - H*x_pred(k) 
     //
     //    [1 0 0]
     // H = 
     //    [0 1 0] 
     y[0] = phi_m - x_pred[0];
     y[1] = phidot_m - x_pred[1];

     // Residual covariance: S(k) = H*P_pred(k)*HT + R(k)
     S[0][0] = P_pred[0][0] + filter->R[0][0];
     S[0][1] = P_pred[0][1] + filter->R[0][1];
     S[1][0] = P_pred[1][0] + filter->R[1][0];
     S[1][1] = P_pred[1][1] + filter->R[1][1];

     // To calculate the Kalman gain below, we need the inverse of matrix S
     // (inv(S) = Sinv). We use the following formula to calculate the inverse:
     //
     //   inv(S) = 1/det(S) * adj(S)
     //    
     // For a 2x2 matrix S, the adjugate matrix adj(S) can be calculated as:
     //
     //          [S11  -S01]
     // adj(S) = 
     //          [-S10  S00]
     float detS = S[0][0]*S[1][1] - S[1][0]*S[0][1];
     float f = 1.0f/detS;
     Sinv[0][0] = f*S[1][1];
     Sinv[0][1] = -f*S[0][1];
     Sinv[1][0] = -f*S[1][0];
     Sinv[1][1] = f*S[0][0];

     // Kalman gain: K(k) = P_pred(k)*HT*Sinv(k)
     K[0][0] = P_pred[0][0]*Sinv[0][0] + P_pred[0][1]*Sinv[1][0];
     K[1][0] = P_pred[1][0]*Sinv[0][0] + P_pred[1][1]*Sinv[1][0];
     K[2][0] = P_pred[2][0]*Sinv[0][0] + P_pred[2][1]*Sinv[1][0];

     K[0][1] = P_pred[0][0]*Sinv[0][1] + P_pred[0][1]*Sinv[1][1];
     K[1][1] = P_pred[1][0]*Sinv[0][1] + P_pred[1][1]*Sinv[1][1];
     K[2][1] = P_pred[2][0]*Sinv[0][1] + P_pred[2][1]*Sinv[1][1];

     // Updated state: x(k) = x_pred(k) + K(k)*y(k)
     filter->x[0] = x_pred[0] + (K[0][0]*y[0] + K[0][1]*y[1]);
     filter->x[1] = x_pred[1] + (K[1][0]*y[0] + K[1][1]*y[1]);
     filter->x[2] = x_pred[2] + (K[2][0]*y[0] + K[2][1]*y[1]);

     // Updated error estimation: P(k) = (I - K(k)*H) * P_pred(k)
     temp[0][0] = 1.0f - K[0][0];
     temp[1][0] = 0.0f - K[1][0];
     temp[2][0] = 0.0f - K[2][0];

     temp[0][1] = 0.0f - K[0][1];
     temp[1][1] = 1.0f - K[1][1];
     temp[2][1] = 0.0f - K[2][1];

     temp[0][2] = 0.0f;
     temp[1][2] = 0.0f;
     temp[2][2] = 1.0f;

     filter->P[0][0] = temp[0][0]*P_pred[0][0] + temp[0][1]*P_pred[1][0] +
	  temp[0][2]*P_pred[2][0];
     filter->P[1][0] = temp[1][0]*P_pred[0][0] + temp[1][1]*P_pred[1][0] + 
	  temp[1][2]*P_pred[2][0];
     filter->P[2][0] = temp[2][0]*P_pred[0][0] + temp[2][1]*P_pred[1][0] + 
	  temp[2][2]*P_pred[2][0];

     filter->P[0][1] = temp[0][0]*P_pred[0][1] + temp[0][1]*P_pred[1][1] +
	  temp[0][2]*P_pred[2][1];
     filter->P[1][1] = temp[1][0]*P_pred[0][1] + temp[1][1]*P_pred[1][1] + 
	  temp[1][2]*P_pred[2][1];
     filter->P[2][1] = temp[2][0]*P_pred[0][1] + temp[2][1]*P_pred[1][1] + 
	  temp[2][2]*P_pred[2][1];

     filter->P[0][2] = temp[0][0]*P_pred[0][2] + temp[0][1]*P_pred[1][2] +
	  temp[0][2]*P_pred[2][2];
     filter->P[1][2] = temp[1][0]*P_pred[0][2] + temp[1][1]*P_pred[1][2] + 
	  temp[1][2]*P_pred[2][2];
     filter->P[2][2] = temp[2][0]*P_pred[0][2] + temp[2][1]*P_pred[1][2] + 
	  temp[2][2]*P_pred[2][2];
}

void kf_predict(const struct kf *filter, float t_delta, float x_pred[3], 
		float P_pred[3][3])
{
     float F[3][3];
     float FT[3][3];
     float Q[3][3];
     float temp[3][3];

     //        [1 t_delta -t_delta]
     // F(k) = [0    1        0   ]
     //        [0    0        1   ]
     F[0][0] = 1.0f;
     F[0][1] = t_delta;
     F[0][2] = -t_delta;
     F[1][0] = 0.0f;
     F[1][1] = 1.0f;
     F[1][2] = 0.0f;
     F[2][0] = 0.0f;
     F[2][1] = 0.0f;
     F[2][2] = 1.0f;

     // Transposed F(k)
     FT[0][0] = F[0][0];
     FT[0][1] = F[1][0];
     FT[0][2] = F[2][0];
     FT[1][0] = F[0][1];
     FT[1][1] = F[1][1];
     FT[1][2] = F[2][1];
     FT[2][0] = F[0][2];
     FT[2][1] = F[1][2];
     FT[2][2] = F[2][2];

     //         [t_delta**4/4  t_delta**3/2  0]
     //  Q(k) = [t_delta**3/2   t_delta**2   0] * sigma_phidotdot**2 +
     //         [      0             0       0]
     //
     //         [ 0 0 0 ]
     //         [ 0 0 0 ] * sigma_bias**2
     //         [ 0 0 1 ]
     float t_delta_square = t_delta*t_delta; 
     Q[0][0] = t_delta_square*t_delta_square*
	  filter->sigma_phidotdot_square/4.0f;
     Q[0][1] = t_delta_square*t_delta*filter->sigma_phidotdot_square/2.0f;
     Q[0][2] = 0.0f;
     Q[1][0] = Q[0][1];
     Q[1][1] = t_delta_square*filter->sigma_phidotdot_square;
     Q[1][2] = 0.0f;
     Q[2][0] = 0.0f;
     Q[2][1] = 0.0f;
     Q[2][2] = filter->sigma_bias_square;

     // Since we do not know the control model (B), the new state x(k)
     // is simply predicted by: x(k) = F(k)*x(k-1)     
     x_pred[0] = F[0][0]*filter->x[0] + F[0][1]*filter->x[1] + 
	  F[0][2]*filter->x[2];
     x_pred[1] = F[1][0]*filter->x[0] + F[1][1]*filter->x[1] + 
	  F[1][2]*filter->x[2];
     x_pred[2] = F[2][0]*filter->x[0] + F[2][1]*filter->x[1] + 
	  F[2][2]*filter->x[2];

     // P(k) = F(k)*P(k-1)*FT(k) + Q(k)
     temp[0][0] = F[0][0]*filter->P[0][0] + F[0][1]*filter->P[1][0] + 
	  F[0][2]*filter->P[2][0];
     temp[1][0] = F[1][0]*filter->P[0][0] + F[1][1]*filter->P[1][0] +
	  F[1][2]*filter->P[2][0];
     temp[2][0] = F[2][0]*filter->P[0][0] + F[2][1]*filter->P[1][0] +
	  F[2][2]*filter->P[2][0];

     temp[0][1] = F[0][0]*filter->P[0][1] + F[0][1]*filter->P[1][1] + 
	  F[0][2]*filter->P[2][1];
     temp[1][1] = F[1][0]*filter->P[0][1] + F[1][1]*filter->P[1][1] +
	  F[1][2]*filter->P[2][1];
     temp[2][1] = F[2][0]*filter->P[0][1] + F[2][1]*filter->P[1][1] +
	  F[2][2]*filter->P[2][1];

     temp[0][2] = F[0][0]*filter->P[0][2] + F[0][1]*filter->P[1][2] + 
	  F[0][2]*filter->P[2][2];
     temp[1][2] = F[1][0]*filter->P[0][2] + F[1][1]*filter->P[1][2] +
	  F[1][2]*filter->P[2][2];
     temp[2][2] = F[2][0]*filter->P[0][2] + F[2][1]*filter->P[1][2] +
	  F[2][2]*filter->P[2][2];

     P_pred[0][0] = temp[0][0]*FT[0][0] + temp[0][1]*FT[1][0] +
	  temp[0][2]*FT[2][0];
     P_pred[1][0] = temp[1][0]*FT[0][0] + temp[1][1]*FT[1][0] +
	  temp[1][2]*FT[2][0];
     P_pred[2][0] = temp[2][0]*FT[0][0] + temp[2][1]*FT[1][0] +
	  temp[2][2]*FT[2][0];

     P_pred[0][1] = temp[0][0]*FT[0][1] + temp[0][1]*FT[1][1] +
	  temp[0][2]*FT[2][1];
     P_pred[1][1] = temp[1][0]*FT[0][1] + temp[1][1]*FT[1][1] +
	  temp[1][2]*FT[2][1];
     P_pred[2][1] = temp[2][0]*FT[0][1] + temp[2][1]*FT[1][1] +
	  temp[2][2]*FT[2][1];

     P_pred[0][2] = temp[0][0]*FT[0][2] + temp[0][1]*FT[1][2] +
	  temp[0][2]*FT[2][2];
     P_pred[1][2] = temp[1][0]*FT[0][2] + temp[1][1]*FT[1][2] +
	  temp[1][2]*FT[2][2];
     P_pred[2][2] = temp[2][0]*FT[0][2] + temp[2][1]*FT[1][2] +
	  temp[2][2]*FT[2][2];

     P_pred[0][0] += Q[0][0];
     P_pred[1][0] += Q[1][0];
     P_pred[2][0] += Q[2][0];

     P_pred[0][1] += Q[0][1];
     P_pred[1][1] += Q[1][1];
     P_pred[2][1] += Q[2][1];

     P_pred[0][2] += Q[0][2];
     P_pred[1][2] += Q[1][2];
     P_pred[2][2] += Q[2][2];
}

