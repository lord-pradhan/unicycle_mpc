%% Lab 4 - Michael Turski

%% Clear variables in necessary

clc
clear all
close all

%% Motor Control MPC Parametes

% MPC Parameters
N = 10;
Q = [10];
R = [1/10];
RD = [1/10];

Ts = 0.200; % s

% For data capture
simTime = 30; % s
squareWavePeriod = 5; % s
squareWaveOmega = 2*pi/squareWavePeriod;

%% Motor Control MPC Setup

% Motor Model
K = 2.9073;
Tau = 0.0850;

s = tf('s');
G = K/(Tau*s + 1);

A = -1/Tau;
B = K/Tau;
C = 1;
Gss = ss(A,B,C,0);

Gd = c2d(Gss,Ts);

A = Gd.A;
B = Gd.B;
C = Gd.C;

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
        Su(ii,jj) = sum(CAB(1:ii-jj+1));
    end
end
Su1=  Su(:,1);

LL = tril(ones(N));
H = 2*(LL'*Rbar*LL+RbarD+Su'*Qbar*Su);
Fu = 2*(diag(LL'*Rbar')'+Su1'*Qbar*Su)';
Fr = -2*(Qbar*Su)';
Fx = 2*(Sx'*Qbar*Su)';

G = [tril(ones(N));-tril(ones(N))];
W0 = 8*ones(2*N,1);
S = zeros(2*N,1);

%% Run Simulation

% % Time setup
% Tf = 1; % s
% T = ceil(Tf/Ts)+1;
% t = linspace(0,T*Ts,T+1);
% t = t(1:end-1);
% 
% % Initial condition/reference trajectory
% X = 0;
% U = 0;
% 
% r = 15*ones(1,N+T+1); %15*square([1:T+N+1]/6);
% 
% Xvec = [];
% Uvec = [];
% 
% warning('off')
% options = optimoptions('quadprog');
% options.Display = 'none';
% for ii = 1:T-1
%     Xvec = [Xvec X];
%     
%     ref = r(ii:ii+N-1)';
%     f = Fx*X + Fu*U + Fr*ref;
%     W = W0+[ones(N,1)*-U; ones(N,1)*U];
% 
%     Z = quadprog(H,f,G,W+S*X,[],[],[],[],[],options);
%     
%     U = U + Z(1);
%     Uvec = [Uvec U];
% 
%     X = A*X+B*U; %(0.5 - rand)*0.1 % add noise here
% end
% Xvec = [Xvec X];
% Uvec = [Uvec U]; % last input is not used here and will not show up on stair plot
% 
% % Plot Results
% close all
% 
% figure
% subplot(211)
% hold on
% plot(t,Xvec,'b-')
% plot(t,r(1:T),'r-')
% 
% subplot(212)
% stairs(t,Uvec,'g-')

%% Connect/Disconnect to/from the HC-06 module
clc
bt_connected = 0; % change it to 1 after connecting to bluetooth, 2 to disconnect

if (bt_connected == 0)
    bt = Bluetooth("HC-06",1); % connects to the bluetooth module as a serial communication object
    fopen(bt); % initialize the serial object
elseif bt_connected == 2
    fclose(bt);
end

%% Stream and send data to HC-06

stepAmp = 15;

stepAmpHigh = 20;
stepAmpLow = 10;

t = [];
theta = [];
thetaDot = [];
x = [];
xDot = [];
mLDot = [];
mRDot = [];
mLV = [];
mRV = [];
U = [];
refVec = [];
Upre = 0;

warning('off')
options = optimoptions('quadprog');
options.Display = 'none';

% endless loop to keep streaming data from the robot
flushinput(bt); % flush the buffer stored till now
flushoutput(bt); % flush output buffer

tic
while (1)
    
    currentTime = toc;
    if currentTime > simTime
        break
    end
    
%     flushinput(bt); % flush the buffer stored till now
%     fscanf(bt); % flushing buffer clips the data stream so have to discard the next input
    rawdata = fscanf(bt); % Store the current data streamed from the robot
    IN = parsedata(rawdata); % parse the raw data and store in a numeric array
    
    % Standard step input
%     reference = stepAmp*ones(N,1);

    % Sqaure wave
    tCur = currentTime;
    
    % No look ahead
%     val = sin(squareWaveOmega*tCur);
%     if val >= 0
%         val = stepAmpHigh;
%     else
%         val = stepAmpLow;
%     end
%     reference = val*ones(N,1);
    
    % Look ahead
    tLookAhead = linspace(tCur, tCur + N*Ts, N + 1);
    tLookAhead = tLookAhead(1:end-1);
    reference = sin(squareWaveOmega*tLookAhead);
    for ee = 1:N
        if reference(ee) >= 0
            reference(ee) = stepAmpHigh;
        else
            reference(ee) = stepAmpLow;
        end
    end
    reference = reference';
    
    state = IN(6);
    controlPre = Upre;

    f = Fx*state + Fu*controlPre + Fr*reference;
    W = W0+[ones(N,1)*-controlPre; ones(N,1)*controlPre];

    Z = quadprog(H,f,G,W+S*state,[],[],[],[],[],options);

    OUT = controlPre + Z(1); 
    
%     flushoutput(bt); % flush output buffer
    fprintf(bt,num2str(OUT)); % send the output variables to the robot in a string format
    
    % Save data after sending new command
    t = [t IN(1)/1000];
    theta = [theta IN(2)];
    thetaDot = [thetaDot IN(3)];
    x = [x IN(4)];
    xDot = [xDot IN(5)];
    mLDot = [mLDot IN(6)];
    mRDot = [mRDot IN(7)];
    mLV = [mLV IN(8)];
    mRV = [mRV IN(9)];
    
    refVec = [refVec reference(1)];
    
    U = [U OUT];
    Upre = OUT;
end

%% Save variables

fileName = 'MotorControl_Q3_N_10_LookAhead.mat';
t = t - t(1);
save(fileName,'t','theta','thetaDot','x','xDot','mLDot','mRDot','mLV','mRV','U','refVec')

%% Q1 Plots

close all

fileNames = {'MotorControl_Q1_N_2.mat','MotorControl_Q1_N_5.mat','MotorControl_Q1_N_10.mat'};
colorList = {'r-','b-','g-'};

for jj = 1:length(fileNames)
    clear t theta thetaDot x xDot mLDot mRDot mLV mRV U
    
    load(fileNames{jj});
    
    figure(1)
    subplot(121)
    hold on
    plot(t,mLDot,colorList{jj})
    
    subplot(122)
    hold on
    stairs(t,U,colorList{jj})
end

subplot(121)
plot([0 5],[15 15],'k--')
axis([0 5 0 18])
subplot(122)
axis([0 5 0 6])

%% Q2 Plots

close all

fileNames = {'MotorControl_Q2_N_10.mat'};
colorList = {'r-','b-','g-'};

for jj = 1:length(fileNames)
    clear t theta thetaDot x xDot mLDot mRDot mLV mRV U
    
    load(fileNames{jj});
    
    figure(1)
    subplot(121)
    hold on
    plot(t,mLDot,colorList{jj})
    
    subplot(122)
    hold on
    stairs(t,U,colorList{jj})
end

subplot(121)
plot([0 15],[15 15],'k--')
axis([0 15 0 18])
subplot(122)
axis([0 15 0 6])

%% Q3 Plots

close all

fileNames = {'MotorControl_Q3_N_10_lookAhead.mat','MotorControl_Q3_N_10_noLookAhead.mat'};
colorList = {'r-','b-','g-'};

for jj = 1:length(fileNames)
    clear t theta thetaDot x xDot mLDot mRDot mLV mRV U refVec
    
    load(fileNames{jj});
    
    figure(1)
    subplot(121)
    hold on
    plot(t,mLDot,colorList{jj})
    
    subplot(122)
    hold on
    stairs(t,U,colorList{jj})
end

subplot(121)
plot(t,refVec,'k--')
axis([0 30 0 25])
subplot(122)
axis([0 30 2 8])
