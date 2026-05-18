t = out.time;
data = out.vehicle_pos;

x_vehicle = data(:,1);
y_vehicle = data(:,2);

x_d = 0:1:100;
y_d = zeros(1,101);
for i = 1:length(x_d)
    if x_d(i) <= 50
        y_d(i) = x_d(i);
    else
        y_d(i) = 50;
    end
end

figure;
plot(x_d, y_d, 'b', 'LineWidth', 1.5); hold on;
plot(x_vehicle, y_vehicle, 'r', 'LineWidth', 1.5);
xlabel('X(m)'); ylabel('Y(m)');
title('MPC');
legend('Road', 'Vehiclepath');
grid on;
xlim([0 100]); ylim([0 60]);