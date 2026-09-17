# -*- coding: utf-8 -*-
"""
Created on Fri Mar 22 10:12:10 2024

@author: Z0030883
"""

import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt


def generate_measured_data():
    """Generate the ground truth and sensor measurement signals:
    ground truth: x = t^2
    measurement: z = t^2 + normal(0, 0.01)
    """
    t = np.arange(start=0, stop=100, step=0.01)  # time sequence
    x = []  # used to store the ground truth
    z = []  # used to store the measurement

    for i in range(len(t)):
        temp = t[i]*1
        x.append(temp)
        z.append(temp + np.random.normal(loc=0.0, scale=0.1))

    # Visualize the generated signals
    plt.figure(figsize=(10, 8))
    plt.xlabel('t')
    plt.ylabel('value')
    plt.plot(t, x, label='Ground truth')
    plt.plot(t, z, 'o', t, z, label='Measurement')
    plt.legend()
    plt.show()
    return t, x, z


class KMFilter:
    def __init__(self, x_init):
        self.X_fused_2 = []  # used to sotre the filtered value
        self.P_fused_2 = []  # used to store the filtered variance
        # Setup the initial value
        # self.X_posterior_prev = np.matlib.zeros(shape=(3, 1), dtype=float)
        self.X_posterior_prev = np.matrix([x_init, 0., 0.], dtype=float).T
        self.P_posterior_prev = np.matrix([[0.1, 0., 0.],
                                           [0., 0.1, 0.],
                                           [0., 0., 0.0001]],
                                          dtype=float)
        self.X_fused_2.append(self.X_posterior_prev)
        self.P_fused_2.append(self.P_posterior_prev)

    def km_filter(self, dt, sig):
        """
        Prediction func:
        x(k)   = x(k-1)   + x'(k-1)*dt + x''(k-1)*(dt)^2 * (1/2!)  + Q1
        x'(k)  = 0*x(k-1) + x'(k-1)    + x''(k-1)*dt               + Q2
        x''(k) = 0*x(k-1) + 0*x'(k-1)  + x''(k-1)                  + Q3
        State variable:
        X = [x(k), x'(k), x''(k)].T

        F = [[1, dt, 0.5*dt^2],
             [0,  1,    dt   ],
             [0,  0,     1   ]]

        Q = [[Q1, 0,  0],
             [ 0, Q2, 0],
             [ 0, 0,  Q3]]

        Q1 ~ N(0, 1); Q2 ~ N(0, 0.01); Q3 ~ N(0, 0.001)
        X(k) = F * X(k-1) + Q

        Measurement func:
        Z(k) = H * X(k) + R
        R ~ N(0, 1)
        H = [1, 0, 0]
        """

        F2 = np.matrix([[1., dt, 0.5*dt**2],
                        [0., 1., dt],
                        [0., 0., 1.]], dtype=float)
        H2 = np.matrix([1., 0., 0.], dtype=float)
        Q2 = np.matrix([[1., 0., 0.],
                        [0., 1, 0.],
                        [0., 0., 0.001]], dtype=float)
        R2 = np.matrix([10.], dtype=float)

        X_prior = F2 * self.X_posterior_prev  # 3&3 * 3&1 = (3&1)
        P_prior = F2 * self.P_posterior_prev * F2.T + Q2

        K = (P_prior * H2.T) * np.linalg.inv(H2 * P_prior * H2.T + R2)  # 3&1
        # 3&1 + 3&1 * (1 - 1&3 * 3&1)
        X_posterior1 = X_prior + K * (sig - H2 * X_prior)
        P_posterior1 = (np.matlib.identity(n=3, dtype=float) - K*H2) * P_prior

        self.X_posterior_prev = X_posterior1
        self.P_posterior_prev = P_posterior1

        self.X_fused_2.append(self.X_posterior_prev)
        self.P_fused_2.append(self.P_posterior_prev)
        
        return self.X_posterior_prev[0, 0], self.X_posterior_prev[1, 0]

    # def km_extrapolate(self, dt):
    #     """
    #     This extrapolation method does not update the global state, which
    #     only output the extrapolated x, but the dt is not identified with
    #     the one in km_km_filter. This dt is monotonically increasing.

    #     Parameters
    #     ----------
    #     dt : TYPE
    #         This dt is the delta time from the latest calculation and last
    #         exsistent object. e.g. dt = 0.01 if 1 image is lost, dt = 0.02 if 2
    #         images are lost.

    #     Returns
    #     -------
    #     X_extra : np.ndarray
    #         DESCRIPTION.

    #     """
    #     F2 = np.matrix([[1., dt, 0.5*dt**2],
    #                     [0.,  1., dt],
    #                     [0.,  0., 1.]], dtype=float)
    #     X_extra = F2 * self.X_posterior_prev
    #     return X_extra

    def km_extrapolate(self, dt):
        """
        This extrapolation method updates the global state, instead of
        only outputing the extrapolated x, and the dt is identified with
        the one in km_km_filter. This dt is the diff between the adjacent two
        images. 
        However, the P update differs a little from the km_filter(), because
        the Q is a constant element. Since the Q is added once in km_filter()
        when the dt is 3s, but will be added 2 more times at 1s and 2s in
        km_extra(), which cause the difference of P at 3s after switching back
        from km_extra() to km_filter().

        Parameters
        ----------
        dt : TYPE
            This dt is the delta time from two adjacent objects.
        Returns
        -------
        X_extra : np.ndarray
            DESCRIPTION.

        """
        F2 = np.matrix([[1., dt, 0.5*dt**2],
                        [0., 1., dt],
                        [0., 0., 1.]], dtype=float)
        self.X_posterior_prev = F2 * self.X_posterior_prev
        self.P_posterior_prev = F2 * self.P_posterior_prev * F2.T
        return self.X_posterior_prev[0, 0], self.X_posterior_prev[1, 0]


def main():
        t, x, z = generate_measured_data()
        kmf = KMFilter(z[1])

        dt = t[1] - t[0]
        # Start the kalman filter process
        for i in range(1, len(t)):
            kmf.km_filter(dt, z[i])

        # Visualize the kalman filter result
        plt.figure(figsize=(14, 8))
        plt.title('Kalman filter')
        plt.xlabel('t')
        plt.ylabel('value')
        # plt.plot(t, x, label='Ground truth')
        plt.plot(t, z, label='Measurement')
        # plt.plot(self.t, self.X_fused, label='Solution 1')
        plt.plot(t, np.asarray(kmf.X_fused_2)[:, 0, 0], label='Solution 2')
        
        # Visualize the kalman filter v-result
        plt.figure(figsize=(14, 8))
        plt.title('Kalman filter')
        plt.xlabel('t')
        plt.ylabel('value')
        plt.plot(t, np.asarray(kmf.X_fused_2)[:, 1, 0], label='Solution 2_v')
        # a = np.asarray(kmf.X_fused_2)[:, 2, 0]
        # plt.plot(t, a, label='Solution 2_a')
        
        
        plt.legend()
        plt.show()
        return kmf


if __name__ == "__main__":
    kmf = main()