Kalman filter

See pose_estimation_using_kalman_filter_korean.pdf for detailed information.

Organimization

- notebook: ipython notebooks for studying basics of Kalman filter
- test_kalman_euler.m: Matlab test file

Simple description

- Why do we need sensor fusion? Since gyroscope sensor suffers from drift error, acceleration sensor can help reduce this undesirable effect.
- From accelerometer sensor, we obtain pitch and roll Euler angle. These values are used as observation in Kalman filter.
- From gyroscope sensor, we obtain angular velocity, which is used in A where A is the linear operator between previous state vector and current state vector. At this stage, we use the relation between quternion (q_1,...,q_4) and angular velocity (p,q,r) from gyroscope, we derive system model for Kalman filter.
- When it is hard to express the relation between the state vectors in the linear form, we approximate it by Taylor expansion. This methos is called Extended Kalman filter.

![](fig/figPhone10.png)
