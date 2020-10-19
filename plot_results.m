function plot_results(Umpc,Xref,Xact,Umotor)
%% Plot results from QP-MPC for trajectory tracking of tumbller
%  
% @param[in] Umpc       History of the MPC output applied to low level
%                       control
% @param[in] Xref       The desireed reference state/trajectory to track
% @param[in] Xact       The actual state evolution after control applied
% @param[in] Umotor     The command send to the motors

%% -------------------------------------------------
% MPC output 
figure
subplot(2,2,1)
plot(Umpc(:,1),'LineWidth',2,'DisplayName','Position [m]');
hold on;
plot(Xref(:,1),'LineWidth',2,'DisplayName','Ref. Position [m]');
ylabel('Position [m]');

subplot(2,2,2)
plot(Umpc(:,2),'LineWidth',2,'DisplayName','Lin. Velocity [m/s]');
hold on;
plot(Xref(:,2),'LineWidth',2,'DisplayName','Ref. Lin. Velocity [m/s]');
ylabel('Lin. Velocity [m/s]');

subplot(2,2,3)
plot(rad2deg(Umpc(:,3)),'LineWidth',2,'DisplayName','Angle [deg]');
hold on;
plot(Xref(:,3),'LineWidth',2,'DisplayName','Ref. Angle [deg]');
ylabel('Angle [deg]');

subplot(2,2,4)
plot(rad2deg(Umpc(:,4)),'LineWidth',2,'DisplayName','Ang. Velocity [deg/s]');
hold on;
plot(Xref(:,4),'LineWidth',2,'DisplayName','Ref. Ang. Velocity [deg/s]');
ylabel('Ang. Velocity [deg/s]');

sgtitle('Control Output of QP-MPC')

%% -------------------------------------------------
% State evolution
figure
subplot(2,2,1)
plot(Xact(:,1),'LineWidth',2,'DisplayName','Position [m]');
hold on;
plot(Xref(:,1),'LineWidth',2,'DisplayName','Ref. Position [m]');
ylabel('Position [m]');

subplot(2,2,2)
plot(Xact(:,2),'LineWidth',2,'DisplayName','Lin. Velocity [m/s]');
ylabel('Lin. Velocity [m/s]');

subplot(2,2,3)
plot(rad2deg(Xact(:,3)),'LineWidth',2,'DisplayName','Angle [deg]');
ylabel('Angle [deg]');

subplot(2,2,4)
plot(rad2deg(Xact(:,4)),'LineWidth',2,'DisplayName','Ang. Velocity [deg/s]');
ylabel('Ang. Velocity [deg/s]');

sgtitle('State Evolution after applying QP-MPC output')

% Reference trajectory
%figure
%plot(Xref(1,:),'LineWidth',2,'DisplayName','Ref. Position [m]');
%title('Reference signal');

% Control input
% figure
% plot(Umotor,'LineWidth',2,'DisplayName','Control') ;
% title('Control input');


end

