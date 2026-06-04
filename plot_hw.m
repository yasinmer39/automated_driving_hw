t = out.time;
data = out.vehicle_pos;

x_vehicle = data(:,1);
y_vehicle = data(:,2);

ds  = 0.1;
xs1 = 0:ds:50;            y1 = zeros(1, numel(xs1));  
R   = 20;  th = ds/R : ds/R : pi/2;                   
x2  = 50 + R*sin(th);     y2 = R - R*cos(th);
x_d = [xs1, x2];          y_d = [y1, y2];

figure;
plot(x_d, y_d, 'g', 'LineWidth', 1.5); hold on;
plot(x_vehicle, y_vehicle, 'r', 'LineWidth', 1.5);
xlabel('X(m)'); ylabel('Y(m)');
title('lqr');
legend('Road', 'Vehiclepath');
grid on;
xlim([0 100]); ylim([0 60]);