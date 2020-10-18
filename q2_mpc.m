clear
clc
close all

%% Init variables
m_c = 0.493; m_p = 0.312; I_p = 0.00024;
l = 0.04; f = 0.01; k_t = 0.11; R_lqr = 10; r = 0.0335; g=9.81;
sample_time = 0.005;

dt_mpc = 0.2;
U_max = 500;

%% setup matrices
% X_ref = [0,0,0.0,0]';

A_c = [0,1,0,0;
    0, -(I_p + m_p*l^2)*f/( I_p*(m_c+m_p) + m_c*m_p*l^2 ), m_p^2 * g* l^2/(I_p*(m_c+m_p) + m_c*m_p*l^2), 0;
    0,0,0,1;
    0, -m_p*l*f/( I_p*(m_c+m_p) + m_c*m_p*l^2 ), m_p*g*l*(m_c + m_p)/( I_p*(m_c+m_p) + m_c*m_p*l^2 ), 0];
B_c = [0;
    (I_p + m_p*l^2)/( I_p*(m_c+m_p) + m_c*m_p*l^2 );
    0;
    m_p*l/( I_p*(m_c+m_p) + m_c*m_p*l^2 )] * 2*k_t/(R_lqr*r);

C_c = eye(4);
D = zeros(size(C_c,1), size(B_c,2));

sys_c = ss(A_c, B_c, C_c, D);
sys_d = c2d(sys_c, dt_mpc);
[A_d, B_d, ~, ~] = ssdata(sys_d);
% 
% A_d = [1, dt_mpc + A_c(1,1)*dt_mpc^2/2, A_c(1,2)*dt_mpc^2/2, 0;
%        0, 1+A_c(1,1)*dt_mpc, A_c(1,2)*dt_mpc, 0;
%        0, A_c(2,1)*dt_mpc^2/2, 1+A_c(2,2)*dt_mpc^2/2, dt_mpc;
%        0, A_c(2,1)*dt_mpc, A_c(2,2)*dt_mpc, 1]
%    
% B_d = [ B_c(1,1) * [dt_mpc^2, dt_mpc]';
%         B_c(2,1) * [dt_mpc^2, dt_mpc]']

    
%% lqr matrices
Q_lqr = diag([0.001, 0.1, 5000, 1]);
R_lqr = 0.2;

[k_lqr,S,CLP] = lqr(A_c,B_c,Q_lqr,R_lqr);
k_lqr(4) = 4.0; % reduce gains slightly to account for gyro noise
% k_lqr = -k_lqr;
%% mpc matrices
A = A_d - B_d*k_lqr;
B = B_d*k_lqr;
C = eye(4);
R = 0.1*diag([1,1,1,1]);
RD = 0.0*diag([1,1,100,1]);  %Weight the slew rate - respect actuation bandwidths
Q = 100*diag([1000,0.01,100,0.001]);

Ns = size(A,1); % number of states

%% build the more complicated matrices
N = 3;  %This is the horizon for MPC
Qbar = [];
Rbar = [];
RbarD = [];
Sx = [];
Su = [];
CAB = [];

for ii = 1:N
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
    RbarD = blkdiag(RbarD,RD);
    Sx = [Sx;C*A^ii];
    CAB = [CAB;C*A^(ii-1)*B];
end
for ii = 1:N
    for jj = 1:ii
%         Su(ii,jj) = sum(CAB(1:ii-jj+1));
        sumTemp = zeros(Ns);
        for kk = 1:ii-jj+1
            sumTemp = sumTemp + CAB( Ns*(kk-1) +1 : Ns*kk, : );
        end
        Su(Ns*(ii-1) +1 : Ns*ii, Ns*(jj-1) +1 : Ns*jj) = sumTemp;
    end
end
Su1=  Su(:,1:Ns);

%% convert into form used by QP
% LL = tril(ones(N));
LL = zeros(N*Ns);
for i=1:N
    for j=1:N
        LL(Ns*(i-1) +1 : Ns*i, Ns*(j-1) +1 : Ns*j) = eye(Ns);
    end
end
LL = tril(LL);

H = 2*(LL'*Rbar*LL + RbarD + Su'*Qbar*Su);

diagMat = [];
temp1 = diag(LL'*Rbar');
for i = 1:N    
    for j = 1:Ns
        diagMat(Ns*(i-1) +j, j) = temp1( Ns*(i-1) + j )* (N-i+1) ;
    end
end
    

% Fu = 2*(diag(LL'*Rbar')'+Su1'*Qbar*Su)';  %Note the trick on Rbar - u(-1) is really a scalar
Fu = 2*(diagMat'+Su1'*Qbar*Su)'; 
Fr = -2*(Qbar*Su)';
Fx = 2*(Sx'*Qbar*Su)';

kbar = zeros(N, N*Ns);
ktilda = zeros(N,Ns);
for i = 1:N
    kbar(i, Ns*(i-1) +1 : Ns*i) = k_lqr;
    ktilda( i,: ) = k_lqr;
end

% G = [tril(ones(N));-tril(ones(N))];
% W0 = ones(2*N,1);
% S = zeros(2*N,2);

G = [ kbar*LL - kbar*Su; -kbar*LL + kbar*Su ];
W0 = U_max * ones(2*N,1);
W_x = [kbar*Sx; -kbar*Sx];
W_u = [-(ktilda - kbar*Su1); ktilda - kbar*Su1];

%% set up QP
X = [0; 0; 0; 0];
T = 100;
signal = 2*square([1:T+N+1]/60);
r = zeros(Ns * size(signal, 2), 1);
for i = 1:size(signal, 2)
    r( Ns*(i-1) +1, 1 ) = signal(1, i);
end
    
% r = [signal; zeros(size(signal)); zeros(size(signal)); zeros(size(signal))];
Z = zeros(Ns,1);
U = [0;0;0;0];
options = optimoptions('quadprog');
options.Display = 'none';
for ii = 1:T-1
    Xact(Ns*(ii-1)+1: Ns*ii,:) = X; %For graphing
    f = Fx*X + Fu*U + Fr*r( Ns*(ii-1) +1 :  Ns*(ii+N-1) );  %Sometimes people hold r(ii) here
%     W = W0+[ones(N,1)*-U;ones(N,1)*U];
    W = W0 + W_x * X + W_u * U;

    % Solve the QP.
    Z = quadprog(H,f,G,W,[],[],[],[],[],options);  %Here is the magic!
    Uopt(Ns*(ii-1)+1: Ns*ii) = U + Z(1:Ns,1);  %Just use the first item   
    U = Uopt(Ns*(ii-1)+1: Ns*ii)';
    %Now I'll apply the optimal control to the system.
    X = A*X+B*U;
end
Xact(Ns*(ii)+1: Ns*(ii+1),:) = X;

%% plotting
xout = Xact(1:Ns:end,1);
rin = r(1:Ns:Ns*T);
plot( [1:T],xout, [1:T], rin);

figure(2)
plot(Uopt(1:Ns:end))