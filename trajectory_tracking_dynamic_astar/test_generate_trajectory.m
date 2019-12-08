%%
clc; clear; close all
load('path.mat')

%%
resolution = 5;
x_length = 15/resolution;      % grid size / resolution
y_length = 15/resolution;

globalx = OptimalPath(:,1)./resolution - x_length/2;
globaly = OptimalPath(:,2)./resolution - y_length/2;

tf = 30;
num_samp = length(OptimalPath);
time_samp = 0:tf/(num_samp-1):tf;
order = 5;
A = zeros(2*num_samp,order+1);
A1 = zeros(num_samp,order+1); 
A2 = zeros(num_samp,order+1); 
for i = 1:num_samp
    for j = 1:order+1
        A1(i,j) = time_samp(i)^(j-1);
    end
end
for i = 1:num_samp
    for j = 2:order+1
        A2(i,j) = (j-1)*time_samp(i)^(j-2);
    end
end
A(1:2:2*num_samp-1,:) = A1;
A(2:2:2*num_samp,:) = A2;

vd_x = 0.5;
vd_y = 0.5;

bx = vd_x*ones(2*num_samp,order+1);
bx(1:2:2*num_samp-1) = globalx.*ones(num_samp,1);
bx(2) = 0; bx(end) = 0;

by = vd_y*ones(2*num_samp,order+1);
by(1:2:2*num_samp-1) = globaly.*ones(num_samp,1);
by(2) = 0; by(end) = 0;

x = A\bx;
y = A\by;

t = 0:0.01:tf;
x_traj = x(1) + x(2)*t + x(3)*t.^2 + x(4)*t.^3 + x(5)*t.^4 + x(6)*t.^5;
y_traj = y(1) + y(2)*t + y(3)*t.^2 + y(4)*t.^3 + y(5)*t.^4 + y(6)*t.^5;
dx_traj = x(2) + 2*x(3)*t + 3*x(4)*t.^2 + 4*x(5)*t.^3 + 5*x(6)*t.^4;
dy_traj = y(2) + 2*y(3)*t + 3*y(4)*t.^2 + 4*y(5)*t.^3 + 5*y(6)*t.^4;
ddx_traj = 2*x(3) + 6*x(4)*t + 12*x(5)*t.^2 + 20*x(6)*t.^3;
ddy_traj = 2*y(3) + 6*y(4)*t + 12*y(5)*t.^2 + 20*y(6)*t.^3;

% t0 = 0;
% tf = 30;
% x0 = globalx(1);
% xf = globalx(end);
% y0 = globaly(1);
% yf = globaly(end);
% 
% A = [1, t0, t0^2, t0^3;
%      0, 1, 2*t0, 3*t0^2;
%      1, tf, tf^2, tf^3;
%      0, 1, 2*tf, 3*tf^2];
% 
% x = A\[x0; 0; xf; 0];
% y = A\[y0; 0; yf; 0];
% 
% t = 0:0.01:tf;
% x_traj = x(1) + x(2)*t + x(3)*t.^2 + x(4)*t.^3;
% y_traj = y(1) + y(2)*t + y(3)*t.^2 + y(4)*t.^3;
% dx_traj = x(2) + 2*x(3)*t + 3*x(4)*t.^2;
% dy_traj = y(2) + 2*y(3)*t + 3*y(4)*t.^2;
% ddx_traj = 2*x(3) + 6*x(4)*t;
% ddy_traj = 2*y(3) + 6*y(4)*t;
% theta_traj = atan2(dy_traj,dx_traj);

vd = sqrt(dx_traj.^2 + dy_traj.^2);
wd = (ddy_traj.*dx_traj-ddx_traj.*dy_traj)./(dx_traj.^2+dy_traj.^2);

figure
hold on
plot(globalx, globaly, '-r');
plot(globalx(1), globaly(1),'xk')
plot(globalx(end),globaly(end),'xb')
plot(x_traj,y_traj,'-m')
xlabel('x [m]')
ylabel('y [m]')
axis equal
grid on
title('motion plan')
legend('path plan','start','goal','trajectory plan','Location','southeast')

% figure
% subplot(2,1,1)
% plot(t,x_traj,'m','LineWidth',1);
% xlabel('time [s]')
% ylabel('x [m]')
% title('desired position VS. time')
% subplot(2,1,2)
% plot(t,y_traj,'m','LineWidth',1);
% xlabel('time [s]')
% ylabel('y [m]')

figure
subplot(2,1,1)
plot(t,dx_traj,'m','LineWidth',1);
xlabel('time [s]')
ylabel('dx [m/s]')
title('desired velocity VS. time')
subplot(2,1,2)
plot(t,dy_traj,'m','LineWidth',1);
xlabel('time [s]')
ylabel('dy [m/s]')

figure
subplot(2,1,1)
plot(t,vd,'m','LineWidth',1);
xlabel('time [s]')
ylabel('linear velocity [m/s]')
title('desired velocity VS. time')
subplot(2,1,2)
plot(t,wd,'m','LineWidth',1);
xlabel('time [s]')
ylabel('angular velocity [rad/s]')

