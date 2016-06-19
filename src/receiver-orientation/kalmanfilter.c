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

void kf_init(struct kf *filter, float phi, float phidot, 
	     float sigma_phi_measurement, float sigma_phidot_measurement, 
	     float sigma_a)
{
     filter->x[0] = phi;
     filter->x[1] = phidot;

     // Assume perfectly accurate initial angle and angular velocity.
     filter->P[0][0] = 0.0f;
     filter->P[0][1] = 0.0f;
     filter->P[1][0] = 0.0f;
     filter->P[1][1] = 0.0f;

     filter->R[0][0] = sigma_phi_measurement*sigma_phi_measurement;
     filter->R[0][1] = 0.0f;
     filter->R[1][0] = 0.0f;
     filter->R[1][1] = sigma_phidot_measurement*sigma_phidot_measurement;

     filter->sigma_a_square = sigma_a*sigma_a;
}

void kf_update(struct kf *filter, float phi_m, float phidot_m, float t_delta)
{
     float x_pred[2];
     float P_pred[2][2];
     float y[2];
     float S[2][2];
     float Sinv[2][2];
     float K[2][2];
     float temp[2][2];

     kf_predict(filter, t_delta, x_pred, P_pred);

     // Measurement residual: y(k) = z(k) - H*x_pred(k) 
     y[0] = phi_m - x_pred[0];
     y[1] = phidot_m - x_pred[1];

     // Residual covariance: S(k) = H*P_pred(k)*HT + R(k)
     // (HT is the transposed matrix of H)
     // Here, H is the identity matrix. So this simplifies to: 
     // S(k) = P_pred(k) + R(k)
     S[0][0] = P_pred[0][0] + filter->R[0][0];
     S[0][1] = P_pred[0][1] + filter->R[0][1];
     S[1][0] = P_pred[1][0] + filter->R[1][0];
     S[1][1] = P_pred[1][1] + filter->R[1][1];

     // To calculate the Kalman gain below, we need the inverse of matrix S
     // (Sinv = S**-1). We use the following formula to calculate the
     // inverse of S: inv(S) = 1/det(S) * adj(S)
     // For a 2x2 matrix S, the adjugate matrix adj(S) can be calculated as:
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
     // Here, H is the identity matrix. So this simplifies to: 
     // K(k) = P_pred(k)*Sinv(k)
     K[0][0] = P_pred[0][0]*Sinv[0][0] + P_pred[0][1]*Sinv[1][0];
     K[1][0] = P_pred[1][0]*Sinv[0][0] + P_pred[1][1]*Sinv[1][0];
     K[0][1] = P_pred[0][0]*Sinv[0][1] + P_pred[0][1]*Sinv[1][1];
     K[1][1] = P_pred[1][0]*Sinv[0][1] + P_pred[1][1]*Sinv[1][1];

     // Updated state: x(k) = x_pred(k) + K(k)*y(k)
     filter->x[0] = x_pred[0] + (K[0][0]*y[0] + K[0][1]*y[1]);
     filter->x[1] = x_pred[1] + (K[1][0]*y[0] + K[1][1]*y[1]);

     // Updated error estimation: P(k) = (I - K(k)*H) * P_pred(k)
     // Here, H is the identity matrix. So this simplifies to:
     // P(k) = (I - K(k)) * P_pred(k)
     
     temp[0][0] = 1.0f - K[0][0];
     temp[0][1] = 0.0f - K[0][1];
     temp[1][0] = 0.0f - K[1][0];
     temp[1][1] = 1.0f - K[1][1];

     filter->P[0][0] = temp[0][0]*P_pred[0][0] + temp[0][1]*P_pred[1][0];
     filter->P[1][0] = temp[1][0]*P_pred[0][0] + temp[1][1]*P_pred[1][0];
     filter->P[0][1] = temp[0][0]*P_pred[0][1] + temp[0][1]*P_pred[1][1];
     filter->P[1][1] = temp[1][0]*P_pred[0][1] + temp[1][1]*P_pred[1][1];
}

void kf_predict(const struct kf *filter, float t_delta, float x_pred[2], 
		float P_pred[2][2])
{
     float F[2][2];
     float FT[2][2];
     float Q[2][2];
     float temp[2][2];

     //        [1 t_delta]
     // F(k) = 
     //        [0   1    ]
     F[0][0] = 1.0f;
     F[0][1] = t_delta;
     F[1][0] = 0.0f;
     F[1][1] = 1.0f;

     // Transposed F(k)
     FT[0][0] = F[0][0];
     FT[0][1] = F[1][0];
     FT[1][0] = F[0][1];
     FT[1][1] = F[1][1];

     //        [t_delta**4/4 t_delta**3/2]
     // Q(k) =                             * sigma_a**2
     //        [t_delta**3/2  tdelta_**2 ]
     float t_delta_square = t_delta*t_delta; 
     Q[0][0] = t_delta_square*t_delta_square/4.0f*filter->sigma_a_square;
     Q[0][1] = t_delta_square*t_delta/2.0f*filter->sigma_a_square;
     Q[1][0] = Q[0][1];
     Q[1][1] = t_delta_square*filter->sigma_a_square;

     // Since we do not know the control model (B), the new state x(k)
     // is predicted by: x(k) = F(k)*x(k-1)
     
     x_pred[0] = F[0][0]*filter->x[0] * F[0][1]*filter->x[1];
     x_pred[1] = F[1][0]*filter->x[0] * F[1][1]*filter->x[1];

     // P(k) = F(k)*P(k-1)*FT(k) + Q(k)
     temp[0][0] = F[0][0]*filter->P[0][0] + F[0][1]*filter->P[1][0];
     temp[1][0] = F[1][0]*filter->P[0][0] + F[1][1]*filter->P[1][0];
     temp[0][1] = F[0][0]*filter->P[0][1] + F[0][1]*filter->P[1][1];
     temp[1][1] = F[1][0]*filter->P[0][1] + F[1][1]*filter->P[1][1];

     P_pred[0][0] = temp[0][0]*FT[0][0] + temp[0][1]*FT[1][0];
     P_pred[1][0] = temp[1][0]*FT[0][0] + temp[1][1]*FT[1][0];
     P_pred[0][1] = temp[0][0]*FT[0][1] + temp[0][1]*FT[1][1];
     P_pred[1][1] = temp[1][0]*FT[0][1] + temp[1][1]*FT[1][1];

     P_pred[0][0] += Q[0][0];
     P_pred[1][0] += Q[1][0];
     P_pred[0][1] += Q[0][1];
     P_pred[1][1] += Q[1][1];
}

