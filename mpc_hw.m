L  = 2.65;
V  = 8.33;
Ts = 0.01;
A = [0  V;
     0  0];
B = [0;
     V/L];
C = eye(2);
Q = diag([1, 0.5]);
R = 0.5;
Np = 20;
Nc = 5;

sys_d = c2d(ss(A,B,C,0), Ts);
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;

ny = size(Cd,1);
n = size(Ad,1);
nu = size(Bd,2);
F = zeros(Np*ny, n);
Phi = zeros(Np*ny, Nc*nu);

A_power = Ad;
for i = 1:Np
    F((i-1)*ny+1 : i*ny, :)=Cd*A_power;
    for j = 1:min(i,Nc)
        Phi((i-1)*ny+1 : i*ny, (j-1)*nu+1 : j*nu)=Cd*Ad^(i-j)*Bd;
    end
    A_power = A_power*Ad;
end

Qbar = kron(eye(Np), Q);
Rbar = R*eye(Nc*nu);

K_full = (Phi'*Qbar*Phi +Rbar)\(Phi'*Qbar*F);
K_mpc = K_full(1, :);