de = out.lateral_offset;
delta = out.steering_angle;

Ts   = 0.01;
IAE  = Ts*sum(abs(de))          % integral absolute error
eRMS = sqrt(mean(de.^2))        % RMS lateral error (m)
emax = max(abs(de))             % max lateral error (m)
dmax = max(abs(rad2deg(delta))) % max steering angle (deg)