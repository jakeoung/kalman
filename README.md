Kalman filter

See pose_estimation_using_kalman_filter_korean.pdf for detailed information.

Organimization

- notebook: ipython notebooks for studying basics of Kalman filter
- test_kalman_euler.m: Matlab test file

Simple description

- Why do we need sensor fusion? Since gyroscope sensor suffers from drift error, acceleration sensor can help reduce this undesirable effect.
- From accelerometer sensor, we obtain pitch and roll Euler angle. These values are used as observation in Kalman filter.
$$
\begin{align*} \begin{bmatrix} \dot \phi\\ \dot \theta\\ \dot \psi \end{bmatrix}&=
\begin{bmatrix} 1&\sin\phi\tan\theta & \cos\phi\tan\theta \\
0&\cos\phi & -\sin\phi \\ 
0&\sin\phi\sec\theta & \cos\phi\sec\theta \end{bmatrix} \begin{bmatrix}F_{gx}\\ F_{gy} \\ F_{gz} \end{bmatrix}
\end{align*}
$$
- From gyroscope sensor, we obtain angular velocity, which is used in $A$ where $A$ is the linear operator between previous state vector and current state vector. At this stage, we use the relation between quternion $(q_1,...,q_4)$ and angular velocity $(p,q,r)$ from gyroscope, we derive system model for Kalman filter.
$$
\begin{bmatrix}\dot q_1\\\dot q_2\\\dot q_3\\\dot q_4\end{bmatrix}=\frac 12 \begin{bmatrix} 0&-p&-q&-r\\p&0&r&-q\\q&-r&0&p\\r&q&-p&0 \end{bmatrix}     
\begin{bmatrix} q_1\\ q_2\\ q_3\\ q_4\end{bmatrix}
$$
- When it is hard to express the relation between the state vectors in the linear form, we approximate it by Taylor expansion. This methos is called Extended Kalman filter.
$$
\begin{align*}x_{k+1} &= f(x_k) + w_k \\
z_k &= h(x_k) + \nu_k\\
Q &: \text{ covariance matrix for }w_k\\
R &: \text{ covariance matrix for }\nu_k
\end{align*}
$$
$$
A = \left.\frac{\partial f}{\partial x}\right|_{\hat x_k} ,\quad H = \left.\frac{\partial h}{\partial x}\right|_{\hat x_k^-}
$$
$$
\begin{align*} \begin{bmatrix} \dot \phi\\ \dot \theta\\ \dot \psi \end{bmatrix}&=
\begin{bmatrix} 1&\sin\phi\tan\theta & \cos\phi\tan\theta \\
0&\cos\phi & -\sin\phi \\ 
0&\sin\phi\sec\theta & \cos\phi\sec\theta \end{bmatrix} \begin{bmatrix}\phi\\ \theta\\  \psi \end{bmatrix} \\
&= \begin{bmatrix} p+q\sin\phi\tan\theta+r\cos\phi\tan\theta \\
q\cos\phi-r\sin\phi \\ 
q\sin\phi\sec\theta + r\cos\phi\sec\theta \end{bmatrix} \\
&= f(x)+w
\end{align*}
$$
$$
z = \begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}\begin{bmatrix}\phi\\\theta\\\psi\end{bmatrix}+\nu = Hx + \nu
$$
$$
A = \begin{bmatrix} \frac{\partial f_1}{\partial\phi} & \frac{\partial f_1}{\partial\theta} & \frac{\partial f_1}{\partial\psi} \\
\frac{\partial f_2}{\partial\phi} & \frac{\partial f_2}{\partial\theta} & \frac{\partial f_2}{\partial\psi} \\
\frac{\partial f_3}{\partial\phi} & \frac{\partial f_3}{\partial\theta} & \frac{\partial f_3}{\partial\psi}
\end{bmatrix}
$$

- Because gyroscope suffer from drift errors, we have to compensate 

![](fig/figPhone10.png)
