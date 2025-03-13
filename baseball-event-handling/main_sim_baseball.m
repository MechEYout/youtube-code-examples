clear all
close all
clc

figure
set(0, 'DefaultLineLineWidth', 1.5);
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(gcf,'renderer','Painters')
set(gca,'LooseInset',get(gca,'TightInset'))
close

% Create a folder to save the plots to (commented out):
% folder_plots = 'plots_static';
% if ~exist(folder_plots, 'dir')
%    mkdir(folder_plots)
% end

%% Simulate throw/hit - trajectory of baseball
% Baseball is thrown/hit and travels at a defined speed through the air,
% while air resistance and gravity are acting on it

%% Define the required parameters
p.rho_air = 1.36;       % [kg/m^3] density of air at sea level
p.d_ball = 74*1e-3;     % [m] ball diameter
p.cD = 0.14;            % [-] air friction coefficient of sphere
p.A_ball = (p.d_ball*p.d_ball*pi)/4; % [m^2] Area for air resistance
p.m_ball = 142*1e-3;    % [kg] Mass of ball
p.g = 9.81;             % [m/s^2] gravity

p.start_height = -10;
p.end_height = -5;

% To demonstrate the effect of an incorrect implementation of the angle
% calculation (angle, the air resistance acts), this can be turned on:
p.use_incorrect_tan = 0;

%% Define starting parameters
v0 = 40;                % [m/s] - Speed at the start
phi_0 = deg2rad(55);    % [rad] - Angle in radians, at the start

xp_0 = cos(phi_0)*v0;
yp_0 = sin(phi_0)*v0;
x0 = 0;
y0 = p.start_height;

x0_vec = [x0, xp_0, y0, yp_0];

%% Call simulation
% Here, a trick is used to stop the simulation, if the ball hits y=-5
%   -> The maximum time for the integration was set to 100s, but the
%      simulation is stopped, as soon, as y hits p.end_height
%   -> Remove the "odeOptions" from ode45 and compare the results to see
%      the effect
%   -> Event-handling is used for this

odeOptions = odeset('Events',@(t,x) haltIntegrationAtGround(t,x,p));

[t_sim, state_sim] =...
    ode45(@(t,x) dgl_ball_movement(t, x, p), 0:1e-4:100, x0_vec, odeOptions);

% Split results out:
x_sim = state_sim(:,1);
xp_sim = state_sim(:,2);
y_sim = state_sim(:,3);
yp_sim = state_sim(:,4);

%% Reconstruct acceleration in x and y and all the Forces
% Create Initial vectors to fill
xpp_sim = zeros(size(t_sim));
ypp_sim = zeros(size(t_sim));

F_air_ges_vec = zeros(size(t_sim));
F_air_x_vec = zeros(size(t_sim));
F_air_y_vec = zeros(size(t_sim));
angle_vec = zeros(size(t_sim));

% Fill the vectors
for k=1:length(t_sim)
    [dx_ind, F_air_ind, F_air_x_ind, F_air_y_ind, angle_air_ind] =...
        dgl_ball_movement(t_sim(k), state_sim(k,:), p);
    
    xpp_sim(k) = dx_ind(2);
    ypp_sim(k) = dx_ind(4);
    
    F_air_ges_vec(k) = F_air_ind;
    F_air_x_vec(k) = F_air_x_ind;
    F_air_y_vec(k) = F_air_y_ind;
    angle_vec(k) = angle_air_ind;
end


%% Plot the result of the simulaion: Trajectory in xy-coordinates
% Last line commented out - this way, you can export a plot as vectorgrafic
% If you want to use it in latex: exporting as eps is usually easier

figure
set(gcf,'units','inch','position',[1, 3, 12, 6])
set(gca,'LooseInset',get(gca,'TightInset'))
set(gcf,'renderer','Painters')
   
plot(x_sim, y_sim)
hold on
grid on
plot(x_sim(end), y_sim(end), 'rx', 'markersize', 15)
plot([-20, 50, 50, 200], [-10, -10, -5, -5], 'k', 'linewidth', 2)
hatch_below([-20, 50, 200], [-10, -5], 4.564)
legend('flight path', 'endpoint')
axis equal % Set axis equal -> Result is easier to interpret
xlabel('$x \ \left[\rm{m}\right]$')
ylabel('$y \ \left[\rm{m}\right]$')
set(gca,'FontSize',16)
xlim([-5, x_sim(end)+5])
ylim([-16.5, 41.5])
title([{strcat('Throw with $v_0 = ',num2str(v0),...
    '\frac{\rm{m}}{\rm{s}}$ and $\varphi_0 = ', num2str(rad2deg(phi_0)),...
    '^\circ $;', '\qquad Endpoint: $x_{\rm{f}}\!\left(t_{\rm{f}}=',...
    num2str(t_sim(end), 2), ' \rm{s}\right)$=', num2str(x_sim(end), 4),...
    '\rm{m}; \ $y_{\rm{f}}\!\left(t_{\rm{f}}=',num2str(t_sim(end), 2),...
    ' \rm{s}\right)$=', num2str(y_sim(end), 4), '$\rm{m}$')}])

% print(strcat(folder_plots,'/throw_static_picture'),'-dsvg')


%% Plot results
% Show velocity and acceleration in both axis

% Find the point, where yp becomes negative to mark it
idx_cross_zero = find(yp_sim < 0, 1);

% Prepare labels to use in plot
labels_y = [{'$\dot{x} \ \left[\frac{\rm{m}}{\rm{s}} \right]$'},...
    {'$\dot{y} \ \left[\frac{\rm{m}}{\rm{s}} \right]$'},...
    {'$\ddot{x} \ \left[\frac{\rm{m}}{\rm{s}^2} \right]$'},...
    {'$\ddot{y} \ \left[\frac{\rm{m}}{\rm{s}^2} \right]$'}];
title_str=[{'Velocity x-Component'},...
    {'Velocity y-Component'},...
    {'Acceleration x-Component'},...
    {'Acceleration y-Component'}];

% Actually assemble the plot
figure
set(gcf,'units','inch','position',[4, 4, 12, 6])
set(gcf,'renderer','Painters')

subplot(2,2,1)
plot(t_sim, xp_sim)

subplot(2,2,2)
plot(t_sim, yp_sim)
hold on
ylim_get = ylim();
ylim(ylim_get)
plot(ones(size(ylim_get+[-50, 50]))*t_sim(idx_cross_zero),...
    ylim_get+[-50, 50], 'm:')
legend('$\dot{y} \! \left( t \right)$', strcat('$\dot{y}\!\left(t = ',...
    num2str(ceil(t_sim(idx_cross_zero)*1e2)/1e2),'\rm{s}\right)=0 \frac{\rm{m}}{\rm{s}}$'),...
    'location', [0.775, 0.81, 0.1, 0.1])

subplot(2,2,3)
plot(t_sim, xpp_sim)

subplot(2,2,4)
plot(t_sim, ypp_sim)
hold on
ylim_get = ylim();
ylim(ylim_get)
plot(ones(size(ylim_get+[-50, 50]))*t_sim(idx_cross_zero), ylim_get+[-50, 50], 'm:')
plot(t_sim, -p.g*ones(size(t_sim)), 'r--')
legend('$\ddot{y} \! \left( t \right)$', strcat('$\ddot{y}\!\left(t = ',...
    num2str(ceil(t_sim(idx_cross_zero)*1e2)/1e2),'\rm{s}\right)=',...
    num2str(floor(ypp_sim(idx_cross_zero)*1e2)/1e2),' \frac{\rm{m}}{\rm{s}^2}$'),...
    '$g = -9.81 \, \frac{\rm{m}}{\rm{s}^2}$',...
    'location', [0.7534, 0.15, 0.1, 0.1])

% Add stuff, that is simmilar in all plots
for k=1:4
    subplot(2,2,k)
    ylabel(labels_y{k})
    xlabel('time $t \ \left[\rm{s}\right]$')
    set(gca,'FontSize',16)
    title(title_str{k})
    grid on
end

% print(strcat(folder_plots,'/velocity_and_acceleration_xy'),'-dsvg')

%% Plot of tangential speed and forces
% Those plots were not shown in the Video, so they are not refined to the
% same level as the plots above

figure
plot(t_sim, sqrt(xp_sim.^2 + yp_sim.^2))
hold on
ylabel('Tangential velocity [m/s]')
xlabel('time t [s]')


% Plot air resistances as well
figure
subplot(2,2,1)
plot(t_sim, F_air_ges_vec)
ylabel('F - air ges [N]')
xlabel('time t [s]')

subplot(2,2,2)
plot(t_sim, rad2deg(angle_vec))
ylabel('angle air [deg]')
xlabel('time t [s]')

subplot(2,2,3)
plot(t_sim, F_air_x_vec)
ylabel('Fx - air [N]')
xlabel('time t [s]')

subplot(2,2,4)
plot(t_sim, F_air_y_vec)
ylabel('Fy - air [N]')
xlabel('time t [s]')

