L  = 2.65;
V  = 8.33;
Ts = 0.01;
A = [0  V;
     0  0];
B = [0;
     V/L];
Q = diag([1, 0.5]);
R = 0.5;

sys_d = c2d(ss(A,B,eye(2),0), Ts);
Ad = sys_d.A;  Bd = sys_d.B;

K = dlqr(Ad, Bd, Q, R);