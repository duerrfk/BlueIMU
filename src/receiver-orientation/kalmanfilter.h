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

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

/*
  A Kalman filter modelling the orientation (phi) and angular velocity (phidot)
  of an IMU.

  State vector: 

        [ phi  ] 
    x = 
        [phidot] 

  State equation:

    x(k) = F(k)*x(k-1) + omega(k)

    with:

           [1 t_delta]
    F(k) = 
           [0   1    ]

               [t_delta**2/2]
    omega(k) = 
               [  t_delta   ]

  This implementation assumes no control input model (B). Instead, a normally
  distributed random angular acceleration (a) is introduced modelling the effect
  of external forces like motors as part of the random process noise (omega).

  Covariance matrix Q of normally distributed random process noise 
  omega ~ N(0,Q):

                                   [t_delta**4/4  t_delta**3/2]
    Q = G*Gtransposed*sigma_a**2 =                              * sigma_a**2
                                   [t_delta**3/2   tdelta_**2 ]

  Covariance matrix R of normally distributed random measuring noise
  v ~ N(0,R):

      [sigma_phi_m**2         0         ]
  R = 
      [     0          sigma_phidot_m**2]

*/

/**
 * All data of the Kalman filter.
 */
struct kf {
     // State vector x.
     // x[0]: angle (phi) [rad]
     // x[1]: angular velocity (phidot) [rad/s]
     float x[2];
     // Covariance matrix of random state error.
     float P[2][2];
     // Covariance matrix of random measurement noise.
     float R[2][2];
     // Variance of normally distributed random angular acceleration 
     // as part of process noise.
     float sigma_a_square;
};

/**
 * Initialization a new Kalman filter instance.
 *
 * Time is initialized to zero.
 *
 * The initial state is assumed to be perfectly accurate
 * (zero covariance matrix P).
 * 
 * @param filter pointer to filter instance to be initialized.
 * @param phi initial angle.
 * @param phidot initial angular velocity.
 * @param sigma_phi_m standard deviation of measurement noise for 
 * measurements of the angle.
 * @param sigma_phidot_m standard deviation of measurement noise for 
 * measurements of the angular velocity.
 * @param sigma_a standard deviation of random angular acceleration due 
 * to uncontrolled external forces.
 */
void kf_init(struct kf *filter, float phi, float phidot, 
	     float sigma_phi_m, float sigma_phidot_m, 
	     float sigma_a);

/**
 * State prediction.
 * 
 * Given the time after the last state update, the state (phi, phidot) and 
 * error (P) are predicted. 
 *
 * @param filter pointer to the filter instance.
 * @param t_delta elapsed time since filter update.
 * @param x_pred predicted state (angle and angular velocity).
 * @param P_pred predicted error covariance matrix.
 */
void kf_predict(const struct kf *filter, float t_delta, float x_pred[2], 
		float P_pred[2][2]);

/**
 * State update.
 *
 * Given a new measurement, the filter state is updated with a new 
 * angle, angulare velocity, and error covariance matrix.
 *
 * @param filter pointer to the filter instance to be updated.
 * @param phi_m measured angle.
 * @param phidot_m measured angular velocity.
 * @param t_delta elapsed time since last state update.
 */
void kf_update(struct kf *filter, float phi_m, float phidot_m, float t_delta);

#endif
