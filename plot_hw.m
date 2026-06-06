t = out.time;
data = out.vehicle_pos;

x_vehicle = data(:,1);
y_vehicle = data(:,2);

ds   = 0.1;
x_d  = 0:ds:140;
xc   = [25 55 85 115];       % geçiş merkezleri
drop = [8  8  8  8];         % her geçişteki düşüş (işareti değiştir -> yılan)
w    = 6;                    % geçiş genişliği (büyük = daha yumuşak)
y_d  = zeros(1, numel(x_d));
for kk = 1:numel(xc)
    y_d = y_d - (drop(kk)/2).*(1 + tanh((x_d - xc(kk))/w));
end

figure;
plot(x_d, y_d, 'g', 'LineWidth', 1.5); hold on;
plot(x_vehicle, y_vehicle, 'r', 'LineWidth', 1.5);
xlabel('X(m)'); ylabel('Y(m)');
title('Adaptive Stanley');
legend('Road', 'Vehiclepath');
grid on;
xlim([0 100]); ylim([0 60]);