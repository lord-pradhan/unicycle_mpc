%%
%   FileName: Lab4Q2_2.m
%   Description: Implements a tracking MPC controller using a QP
%   formulation
%   Author(s): Roberto Shu
%   Last Edit: 10/15/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clear Workspace
clear all; close all; clc;

% Parameters
sim = true;

% Load Robot model 
[A,B,C,D] = tumbller_model();

% Continuous Time LQR design
Q = diag([1,1,8000,100]);
R = 1;
[Kc,~,~] = lqr(A,B,Q,R);

% Closed loop model (Plant + LQR)
Abar = (A-B*Kc);
Bbar = B;
Cbar = [1, 0, 0, 0];
Dbar = zeros(2,1);


%% BUILD QP
%%% Parameters to change by user!!

% MPC Horizon steps
Nsteps = 10;

% Cost function weights 
Noutput = size(C,1);
Nstate = size(A,2);

% Optimization variables is $ Z^* = [x, \dot{x}, \theta, \dot{\theta}] $
Q = diag([10,0]);
R = 1;
Rd = 1;

% Construct matrices required for QP  
% Standard QP Form 
%       min_z = z^T H z + F^T z
%
%        s.t.  G z <= W + Sx
Noutput = size(C,1);
Nstate = size(A,2);

Sx = [];
Su1 = [];
Su1_k = zeros(Noutput,1);
Su = zeros(Noutput*Nsteps, Nsteps);
Qs = {};
for i = 1:Nsteps
    
    % Build Sx matrix
    Sx = [Sx;C*A^i];
    
    % Build Su1 matrix
    Su1_k = Su1_k + C*A^(i-1)*B;
    Su1 = [Su1;Su1_k];
    
    % Build Su matrix 
    iRows = 1+(i-1)*Noutput:2+(i-1)*Noutput;
    Su(iRows,i) = C*B;
    Su(iRows,1) = Su1_k;
    
    % Collect Cost function weights into cell array
    Qs{i} = Q;
    Rs{i} = R;
    Rds{i} = Rd;
end

% Construct block diagonal cost weight matrices
Q_bar = blkdiag(Qs{:});
R_bar = blkdiag(Rs{:});
Rd_bar = blkdiag(Rds{:});

% Standard QP Form 
%       min_z = z^T H z + F^T z
%
%        s.t.  G z <= W + Sx

% Construct the H matrix for standard QP form
L = tril(R_bar);
H = 2*(Su'*Q_bar*Su + L'*R_bar*L + Rd_bar);

% Construct the F matrix for standard QP form
Fr = -2*Su'*Q_bar';
Fx = 2*Su'*Q_bar'*Sx;
Fu = 2*(Su'*Q_bar'*Su1 + diag(L'*R_bar'));

%% RUN MPC 

% Initial condition
X0 = [0.0; 0.0; 0.0; 0.0];
X = X0;

% Simulation/Run time
N = 40;

% Reference
stepTime = 10;
Xref = zeros(Noutput,2*N);     % Reference trajectory over time [Noutputs x N]
Xref(1,1:stepTime) = 0;           % Set position reference zero before stepTime
Xref(1,stepTime:end) = 0.1;       % Set position reference desired value after stepTime

pos_idx = 1:Noutput:Noutput*Nsteps;

Xref_i = zeros(Noutput*Nsteps,1);  % Initialize size of horizon reference, this is the query values from Xref 

% Initial control
Uopt = 0;

% Quadprop options
options = optimoptions('quadprog');
options.Display = 'none';

% Main control loop
for i = 1:N-1
    
    % Store state evolution
    Xact(i,:) = X;
    
    % Query reference trajectory
    Xref_i(pos_idx,1) = Xref(1,i:(i+Nsteps-1));
    
    % Solve the QP
    u_prev = Uopt(i);
    f = Fr*Xref_i + Fx*X0 + Fu*u_prev;
    
    Z = quadprog(H,f,[],[],[],[],[],[],[],options);

    % Extract the first control input
    Uopt = [Uopt; Z(1)];

    % Send the control input to robot
    if(sim)
        X = A*X + B*Z(1);
    else
        % Add code to send commands to tumbller.
    end
end


%% Plot results 

% Reference trajectory
figure
plot(Xref(1,:),'LineWidth',2,'DisplayName','Ref. Position [m]');
title('Reference signal');

% Control input
figure
plot(Uopt,'LineWidth',2,'DisplayName','Control') ;
title('Control input');

% State evolution
figure
subplot(2,2,1)
plot(Xact(:,1),'LineWidth',2,'DisplayName','Position [m]');
hold on;
plot(Xref(1,:),'LineWidth',2,'DisplayName','Ref. Position [m]');
ylabel('Position [m]');

subplot(2,2,2)
plot(Xact(:,2),'LineWidth',2,'DisplayName','Lin. Velocity [m/s]');
ylabel('Lin. Velocity [m/s]');

subplot(2,2,3)
plot(Xact(:,3),'LineWidth',2,'DisplayName','Angle [deg]');
ylabel('Angle [deg]');

subplot(2,2,4)
plot(Xact(:,4),'LineWidth',2,'DisplayName','Ang. Velocity [deg/s]');
ylabel('Ang. Velocity [deg/s]');


