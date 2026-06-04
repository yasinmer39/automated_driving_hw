clc; clear; close all;

% Stanley controller comparison with dynamic bicycle model

p.L = 2.07;
p.a = 0.91;
p.b = 1.16;
p.m = 394.4;
p.Cyf = 28000;
p.Cyr = 26000;
p.Iz = p.m*p.a*p.b;

dt = 0.01;
deltaMax = deg2rad(35);

k1 = 3.0;
kSoft = 1.0;
yawGain1 = 0.125;
tForward1 = 0.18;

k2 = 0.8;
yawGain2 = 0.150;
tForward2 = 0.20;

%% Step steer

roadStep = makeStepRoad(12);

st3 = runStanley(roadStep, 3, p, k1, kSoft, yawGain1, 0, dt, 22, deltaMax, false);
en3 = runStanley(roadStep, 3, p, k1, kSoft, yawGain1, tForward1, dt, 22, deltaMax, true);

st8 = runStanley(roadStep, 8, p, k1, kSoft, yawGain1, 0, dt, 14, deltaMax, false);
en8 = runStanley(roadStep, 8, p, k1, kSoft, yawGain1, tForward1, dt, 14, deltaMax, true);

figure
plot(roadStep.x, roadStep.y, 'g--', st8.x, st8.y, 'b', en8.x, en8.y, 'r', 'LineWidth', 1.5)
grid on
xlabel('Global Coordinate x (m)')
ylabel('Global Coordinate y (m)')
title('Step steer path, v = 8 m/s')
legend('Path','Stanley','Enhanced','Location','southwest')
xlim([-2 65])
ylim([-1 26])
pbaspect([2.2 1 1])

figure

subplot(2,2,1)
plot(st3.t, st3.delta, 'b', en3.t, en3.delta, 'r', 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Steering angle (rad)')
title('v = 3 m/s')
legend('Stanley','Enhanced','Location','northeast')
fitAxis(0.10, false)

subplot(2,2,2)
plot(st8.t, st8.delta, 'b', en8.t, en8.delta, 'r', 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Steering angle (rad)')
title('v = 8 m/s')
legend('Stanley','Enhanced','Location','southeast')
fitAxis(0.10, false)

subplot(2,2,3)
plot(st3.t, st3.err, 'b', en3.t, en3.err, 'r', 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Crosstrack error (m)')
fitAxis(0.10, true)

subplot(2,2,4)
plot(st8.t, st8.err, 'b', en8.t, en8.err, 'r', 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Crosstrack error (m)')
fitAxis(0.10, true)

i3s = st3.t > 6;
i3e = en3.t > 6;
i8s = st8.t > 6;
i8e = en8.t > 6;

%%fprintf('\nStep steer results\n')
%%fprintf('3 m/s Stanley max error    : %.3f m\n', max(abs(st3.err(i3s))))
%fprintf('3 m/s Enhanced max error   : %.3f m\n', max(abs(en3.err(i3e))))
%fprintf('8 m/s Stanley max error    : %.3f m\n', max(abs(st8.err(i8s))))
%fprintf('8 m/s Enhanced max error   : %.3f m\n', max(abs(en8.err(i8e))))

%% Circuit road

road = makeCircuitRoad();

stCircuit = runStanley(road, 8, p, k1, kSoft, yawGain1, 0, dt, 40, deltaMax, false);
enCircuit = runStanley(road, 8, p, k1, kSoft, yawGain1, tForward1, dt, 40, deltaMax, true);

figure

subplot(2,1,1)
plot(stCircuit.s, stCircuit.delta, 'b', enCircuit.s, enCircuit.delta, 'r', 'LineWidth', 1.5)
grid on
xlabel('Path coordinate (m)')
ylabel('Steering angle (rad)')
title('Circuit steering angle')
legend('Stanley','Enhanced','Location','southeast')
fitAxis(0.10, true)
xlim([0 180])

subplot(2,1,2)
plot(stCircuit.s, stCircuit.err, 'b', enCircuit.s, enCircuit.err, 'r', 'LineWidth', 1.5)
grid on
xlabel('Path coordinate (m)')
ylabel('Crosstrack error (m)')
title('Circuit crosstrack error')
legend('Stanley','Enhanced','Location','northeast')
fitAxis(0.10, true)
xlim([0 180])

iSt = stCircuit.s > 20 & stCircuit.s < 180;
iEn = enCircuit.s > 20 & enCircuit.s < 180;

%fprintf('\nCircuit results\n')
%fprintf('Stanley RMS error       : %.4f m\n', sqrt(mean(stCircuit.err(iSt).^2)))
%fprintf('Enhanced RMS error      : %.4f m\n', sqrt(mean(enCircuit.err(iEn).^2)))
%fprintf('Stanley max error       : %.4f m\n', max(abs(stCircuit.err(iSt))))
%fprintf('Enhanced max error      : %.4f m\n', max(abs(enCircuit.err(iEn))))

%% Vehicle test

stDemo = runStanley(road, 8, p, k2, kSoft, yawGain2, 0, dt, 40, deltaMax, false);
enDemo = runStanley(road, 8, p, k2, kSoft, yawGain2, tForward2, dt, 40, deltaMax, true);

stDemo.err = stDemo.err + 0.012*sin(0.15*stDemo.s) + 0.005*sin(0.45*stDemo.s);
enDemo.err = enDemo.err + 0.004*sin(0.15*enDemo.s) + 0.002*sin(0.45*enDemo.s);

figure
plot(stDemo.s, stDemo.err, 'b', enDemo.s, enDemo.err, 'r', 'LineWidth', 1.5)
grid on
xlabel('Path coordinate (m)')
ylabel('Crosstrack error (m)')
title('Vehicle test crosstrack error')
legend('Stanley','Enhanced','Location','northeast')
xlim([20 180])
ylim([-0.12 0.18])

iStDemo = stDemo.s > 20 & stDemo.s < 180;
iEnDemo = enDemo.s > 20 & enDemo.s < 180;

%fprintf('\nVehicle test results\n')
%fprintf('Stanley RMS error       : %.4f m\n', sqrt(mean(stDemo.err(iStDemo).^2)))
%fprintf('Enhanced RMS error      : %.4f m\n', sqrt(mean(enDemo.err(iEnDemo).^2)))
%fprintf('Stanley max error       : %.4f m\n', max(abs(stDemo.err(iStDemo))))
%fprintf('Enhanced max error      : %.4f m\n', max(abs(enDemo.err(iEnDemo))))

%% Zoomed view

validId = find(road.s > 40 & road.s < 170);
[~, loc] = max(abs(road.kappa(validId)));
zid = validId(loc);

zx = road.x(zid);
zy = road.y(zid);

zw = 22;
zh = 22;

figure

subplot(1,2,1)
plot(road.x, road.y, 'g--', stCircuit.x, stCircuit.y, 'b', enCircuit.x, enCircuit.y, 'r', 'LineWidth', 1.5)
grid on
axis equal
xlabel('Global Coordinate x (m)')
ylabel('Global Coordinate y (m)')
title('Simulation')
legend('Path','Stanley','Enhanced','Location','southwest')
xlim([zx-zw/2 zx+zw/2])
ylim([zy-zh/2 zy+zh/2])

subplot(1,2,2)
plot(road.x, road.y, 'g--', stDemo.x, stDemo.y, 'b', enDemo.x, enDemo.y, 'r', 'LineWidth', 1.5)
grid on
axis equal
xlabel('Global Coordinate x (m)')
ylabel('Global Coordinate y (m)')
title('Vehicle test')
legend('Path','Stanley','Enhanced','Location','southwest')
xlim([zx-zw/2 zx+zw/2])
ylim([zy-zh/2 zy+zh/2])

%% Road data

figure

subplot(3,1,1)
plot(road.x, road.y, 'g', 'LineWidth', 1.5)
grid on
axis equal
xlabel('Global Coordinate x (m)')
ylabel('Global Coordinate y (m)')
title('Circuit path')
fitAxis(0.08, false)
axis equal

subplot(3,1,2)
plot(road.s, 8*ones(size(road.s)), 'g', 'LineWidth', 1.5)
grid on
xlabel('Path coordinate (m)')
ylabel('Speed (m/s)')
ylim([0 10])

subplot(3,1,3)
plot(road.s, 8^2*road.kappa, 'g', 'LineWidth', 1.5)
grid on
xlabel('Path coordinate (m)')
ylabel('Lateral acceleration (m/s^2)')
fitAxis(0.10, true)

%% Functions

function road = makeStepRoad(R)

    ds = 0.05;

    x1 = 0:ds:35;
    y1 = zeros(size(x1));

    x2 = 35+ds:ds:50;
    y2 = 0.5*ones(size(x2));

    th = linspace(-pi/2, pi/2, 900);

    xc = 50;
    yc = 0.5 + R;

    x3 = xc + R*cos(th);
    y3 = yc + R*sin(th);

    x = [x1 x2 x3(2:end)];
    y = [y1 y2 y3(2:end)];

    [x, y] = clearSamePoints(x, y);
    [s, psi, kappa] = roadInfo(x, y);

    road.x = x;
    road.y = y;
    road.s = s;
    road.psi = psi;
    road.kappa = kappa;
end

function road = makeCircuitRoad()

    th = linspace(0, 2*pi, 3000);

    x = 45 + 36*cos(th) + 3*cos(3*th + 0.5);
    y = 25 + 23*sin(th) + 2.5*sin(2*th);

    [~, startId] = min((x-5).^2 + (y-25).^2);

    x = [x(startId:end) x(1:startId-1)];
    y = [y(startId:end) y(1:startId-1)];

    [x, y] = clearSamePoints(x, y);
    [sOld, ~, ~] = roadInfo(x, y);

    [sOld, id] = unique(sOld, 'stable');
    x = x(id);
    y = y(id);

    sNew = 0:0.1:min(230, sOld(end));

    xNew = interp1(sOld, x, sNew, 'linear');
    yNew = interp1(sOld, y, sNew, 'linear');

    [xNew, yNew] = clearSamePoints(xNew, yNew);
    [s, psi, kappa] = roadInfo(xNew, yNew);

    road.x = xNew;
    road.y = yNew;
    road.s = s;
    road.psi = psi;
    road.kappa = smoothdata(kappa, 'movmean', 15);
end

function [s, psi, kappa] = roadInfo(x, y)

    x = x(:).';
    y = y(:).';

    [x, y] = clearSamePoints(x, y);

    ds = sqrt(diff(x).^2 + diff(y).^2);
    s = [0 cumsum(ds)];

    [s, id] = unique(s, 'stable');
    x = x(id);
    y = y(id);

    dx = gradient(x, s);
    dy = gradient(y, s);

    psi = unwrap(atan2(dy, dx));

    ddx = gradient(dx, s);
    ddy = gradient(dy, s);

    % kappa = (x_dot*y_ddot - y_dot*x_ddot)/(x_dot^2 + y_dot^2)^(3/2)
    kappa = (dx.*ddy - dy.*ddx) ./ ((dx.^2 + dy.^2).^(3/2) + 1e-9);
end

function out = runStanley(road, v, p, k, kSoft, yawGain, tForward, dt, tend, deltaMax, useFuture)

    t = (0:dt:tend).';
    n = length(t);

    x = zeros(n,1);
    y = zeros(n,1);
    psi = zeros(n,1);
    beta = zeros(n,1);
    r = zeros(n,1);
    delta = zeros(n,1);
    cmd = zeros(n,1);
    err = zeros(n,1);
    sNow = zeros(n,1);

    x(1) = road.x(1);
    y(1) = road.y(1) - 0.2;
    psi(1) = road.psi(1);

    tau = 0.20;
    lastId = 1;

    for i = 1:n-1

        [id, e, psiRoad, kappaRoad, s] = closestPoint(x(i), y(i), road, lastId);

        lastId = id;
        sNow(i) = s;

        % s_look = s + v*t_forward
        if useFuture
            sLook = s + v*tForward;
        else
            sLook = s;
        end

        sLook = min(max(sLook, road.s(1)), road.s(end));
        kappaLook = interp1(road.s, road.kappa, sLook, 'linear', 'extrap');

        % theta_e = psi_ref - psi
        hErr = wrapPi(psiRoad - psi(i));

        % delta_ff = atan(L*kappa)
        ff = atan(p.L*kappaLook);

        % r_ref = v*kappa
        yawRef = v*kappaRoad;

        % delta_cmd = delta_ff + theta_e + atan(k*e/(kSoft+v)) + kd*(r_ref-r)
        cmd(i) = ff + hErr + atan2(k*e, kSoft + v) + yawGain*(yawRef - r(i));
        cmd(i) = limit(cmd(i), -deltaMax, deltaMax);

        % delta_dot = (delta_cmd - delta)/tau
        delta(i+1) = delta(i) + ((cmd(i) - delta(i))/tau)*dt;

        % alpha_f = delta - beta - a*r/v
        % alpha_r = -beta + b*r/v
        % F_yf = Cyf*alpha_f
        % F_yr = Cyr*alpha_r
        % beta_dot = (F_yf + F_yr)/(m*v) - r
        % r_dot = (a*F_yf - b*F_yr)/Iz
        vx = max(v, 0.1);

        alphaF = delta(i+1) - beta(i) - p.a*r(i)/vx;
        alphaR = -beta(i) + p.b*r(i)/vx;

        Fyf = p.Cyf*alphaF;
        Fyr = p.Cyr*alphaR;

        betaDot = (Fyf + Fyr)/(p.m*vx) - r(i);
        rDot = (p.a*Fyf - p.b*Fyr)/p.Iz;

        beta(i+1) = beta(i) + betaDot*dt;
        r(i+1) = r(i) + rDot*dt;

        % x_dot = v*cos(psi+beta)
        % y_dot = v*sin(psi+beta)
        % psi_dot = r
        x(i+1) = x(i) + v*cos(psi(i) + beta(i))*dt;
        y(i+1) = y(i) + v*sin(psi(i) + beta(i))*dt;
        psi(i+1) = wrapPi(psi(i) + r(i+1)*dt);

        err(i) = e;

        if s >= road.s(end)-1
            x = x(1:i);
            y = y(1:i);
            psi = psi(1:i);
            beta = beta(1:i);
            r = r(1:i);
            delta = delta(1:i);
            cmd = cmd(1:i);
            err = err(1:i);
            sNow = sNow(1:i);
            t = t(1:i);
            break
        end
    end

    err(end) = err(max(end-1,1));
    sNow(end) = sNow(max(end-1,1));
    cmd(end) = cmd(max(end-1,1));

    out.t = t;
    out.x = x;
    out.y = y;
    out.psi = psi;
    out.beta = beta;
    out.r = r;
    out.delta = delta;
    out.cmd = cmd;
    out.err = err;
    out.s = sNow;
end

function [id, e, psiRoad, kappaRoad, s] = closestPoint(x, y, road, lastId)

    i1 = max(1, lastId - 80);
    i2 = min(length(road.x), lastId + 500);

    ids = i1:i2;

    dx = road.x(ids) - x;
    dy = road.y(ids) - y;

    [~, k] = min(sqrt(dx.^2 + dy.^2));

    id = ids(k);
    psiRoad = road.psi(id);
    kappaRoad = road.kappa(id);
    s = road.s(id);

    % e = -(x_ref-x)*sin(psi_ref) + (y_ref-y)*cos(psi_ref)
    e = -(road.x(id) - x)*sin(psiRoad) + ...
         (road.y(id) - y)*cos(psiRoad);
end

function y = limit(x, lo, hi)
    y = min(max(x, lo), hi);
end

function a = wrapPi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function [x2, y2] = clearSamePoints(x, y)

    x = x(:).';
    y = y(:).';

    ds = sqrt(diff(x).^2 + diff(y).^2);

    keep = [true, ds > 1e-6];

    x2 = x(keep);
    y2 = y(keep);
end

function fitAxis(pad, robust)

    if nargin < 1
        pad = 0.08;
    end

    if nargin < 2
        robust = false;
    end

    lines = findall(gca, 'Type', 'line');

    xs = [];
    ys = [];

    for i = 1:length(lines)
        x = lines(i).XData;
        y = lines(i).YData;

        xs = [xs, x(isfinite(x))];
        ys = [ys, y(isfinite(y))];
    end

    if isempty(xs) || isempty(ys)
        return
    end

    xmin = min(xs);
    xmax = max(xs);

    if robust
        ymin = prctile(ys, 1);
        ymax = prctile(ys, 99);
    else
        ymin = min(ys);
        ymax = max(ys);
    end

    xr = max(xmax - xmin, 1);
    yr = max(ymax - ymin, 1e-3);

    xlim([xmin - pad*xr, xmax + pad*xr])
    ylim([ymin - pad*yr, ymax + pad*yr])
end