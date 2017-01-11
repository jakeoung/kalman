clear all; close all;

if ~exist('data.mat')
    connector on;
    try 
        clear m;
        m = mobiledev;
        m.AngularVelocitySensorEnabled = 1;
        m.AccelerationSensorEnabled = 1;
        m.OrientationSensorEnabled = 1;
        pause(0.3);
        m.Logging = 1;
        pause(20);
        m.Logging = 0;
        m.AngularVelocitySensorEnabled = 0;
        m.AccelerationSensorEnabled = 0;
        m.OrientationSensorEnabled = 0;
    catch exception
        disp('error while recording sensor data');
    end

    %% get sensor data of gyro
    [av, tav] = angvellog(m);

    for k = 2:length(tav)
        p = av(k,1); q = av(k,2); r = av(k,3);
    end

    %% get sensor data of accel
    [acc, tacc] = accellog(m);

    for k = 2:length(tacc)
        ax = acc(k,1); ay = acc(k,2); az = acc(k,3);
    end
    
    %% get sensor data of orientation
    [ori, tori] = orientlog(m);
else
    load('data.mat');
end

%% initialization
nSamples = min([length(tav), length(tacc), length(tori)]);

gyroEulerSaved = zeros(nSamples, 3);
accelEulerSaved = zeros(nSamples, 3);
kalmanEulerSaved = zeros(nSamples, 3);
ekfEulerSaved = zeros(nSamples, 3);
ukfEulerSaved = zeros(nSamples, 3);
tkalman = zeros(nSamples, 1);

%% preparation for visualization
figWindow = figure(1);
subplot(1,3,1);
view1 = OrientationView('a', gca);
subplot(1,3,2);
view2 = OrientationView('b', gca);
subplot(1,3,3);
view3 = OrientationView('c', gca);
% subplot(2,2,4);
% view4 = OrientationView('d', gca);

%% kalman filter
for k = 1:nSamples
    % get gyro value
    p = av(k,1); q = av(k,2); r = av(k,3);
    if k == 1 
        dt = tav(1);
    else
        dt = tav(k) - tav(k-1);
    end
    A = eye(4) + dt * 1/2 * [0 -p -q -r;
                            p 0 r -q;
                            q -r 0 p;
                            r q -p 0
                            ];
    % get accel value
    % find the nearest time stamp in tacc
    [minValue,accIdx] = min( abs(tav(k) - tacc));
    if minValue > 10
        accIdx = k;
    end
    tkalman(k) = tacc(accIdx);
    
    [phi, theta, psi] = convertGyro2Euler(p, q, r, dt);
    gyroEulerSaved(k, :) = [phi theta psi];
    
    ax = acc(accIdx,1); ay = acc(accIdx,2); az = acc(accIdx,3);
    [phi_a, theta_a] = convertAccel2Euler(ax, ay, az);
    quatAccel = euler2quaternion(phi_a, theta_a, 0, 'xyz'); % saved only for visualization
    accelEulerSaved(k, :) = [phi_a theta_a 0];
    
    [phi, theta, psi] = kalman_euler(A, quatAccel);
    quatGyro = euler2quaternion(phi, theta, psi, 'xyz');
    kalmanEulerSaved(k, :) = [phi theta psi];
    
    [phi, theta, psi] = ekf_euler([phi_a theta_a]', [p q r], dt);
    quatEKF = euler2quaternion(phi, theta, psi, 'xzy');
    ekfEulerSaved(k, :) = [phi theta psi];
    
%     [phi, theta, psi] = ukf_euler([phi_a theta_a]', [p q r], dt);
%     quatUKF = euler2quaternion(phi, theta, psi, 'xyz');
%     ukfEulerSaved(k, :) = [phi theta psi];
    
    %% visualization 
    
    setOrientation(view1, quatAccel);
    title(view1, 'Accel', 'FontSize', 16);
    
    setOrientation(view2, quatGyro);
    title(view2, 'Kalman(Accel+Gyro)', 'FontSize', 16);
    
    setOrientation(view3, quatEKF);
    title(view3, 'EKF', 'FontSize', 16);
    
%     setOrientation(view4, quatUKF);
%     title(view4, 'UKF', 'FontSize', 16);
    
%     % for googleview (added)
%     [minValue,oriIdx] = min( abs(tav(k) - tori));
%     if minValue > 10
%         oriIdx = k;
%     end
%     % angle to radian
%     phi = -ori(oriIdx,2) * pi / 180;
%     theta = ori(oriIdx,3) * pi / 180;
%     psi = -ori(oriIdx,1) * pi / 180;
%     
%     quatOrientation = euler2quaternion(phi, theta, psi, 'xyz');
%  
%     setOrientation(view4, quatOrientation);
%     title(view4, 'GOOGLE', 'FontSize', 16);
    
    % input key
    input('a');
    
%    if want to save figure
    saveas(gca, ['fig/figPhone', int2str(k), '.png'],'png');
end

%% visualize results
gyroPhiSaved = gyroEulerSaved(:,1)*180/pi;
gyroThetaSaved = gyroEulerSaved(:,2)*180/pi;
gyroPsiSaved = gyroEulerSaved(:,3)*180/pi;

accelPhiSaved = accelEulerSaved(:,1) * 180/pi;
accelThetaSaved = accelEulerSaved(:,2) * 180/pi;
accelPsiSaved = accelEulerSaved(:,3) * 180/pi;

kalmanPhiSaved = kalmanEulerSaved(:,1)*180/pi;
kalmanThetaSaved = kalmanEulerSaved(:,2)*180/pi;
kalmanPsiSaved = kalmanEulerSaved(:,3)*180/pi;

ekfPhiSaved = ekfEulerSaved(:,1)*180/pi;
ekfThetaSaved = ekfEulerSaved(:,2)*180/pi;
ekfPsiSaved = ekfEulerSaved(:,3)*180/pi;

% figure
% plot(tkalman, gyroPhiSaved, tkalman, accelPhiSaved, tkalman, kalmanPhiSaved);
% legend('Gyroscope', 'Accleration', 'Kalman'); ylabel('Phi');
% 
% figure
% plot(tkalman, gyroThetaSaved, tkalman, accelThetaSaved, tkalman, kalmanThetaSaved);
% legend('Gyroscope', 'Accleration', 'Kalman'); ylabel('Theta');
% 
% figure
% plot(tkalman, gyroPsiSaved, tkalman, kalmanPsiSaved);
% legend('Gyroscope', 'Kalman'); ylabel('Psi');

figure
plot(tkalman, gyroPhiSaved, tkalman, accelPhiSaved, tkalman, kalmanPhiSaved, tkalman, ...
    ekfPhiSaved);
legend('Gyroscope', 'Accleration', 'Kalman', 'EKF'); ylabel('Phi');

figure
plot(tkalman, gyroThetaSaved, tkalman, accelThetaSaved, tkalman, kalmanThetaSaved, ...
    tkalman, ekfThetaSaved);
legend('Gyroscope', 'Accleration', 'Kalman', 'EKF'); ylabel('Theta');

figure
plot(tkalman, gyroPsiSaved, tkalman, kalmanPsiSaved, tkalman, ...
    ekfPsiSaved);
legend('Gyroscope', 'Kalman', 'EKF'); ylabel('Psi');


clear m;