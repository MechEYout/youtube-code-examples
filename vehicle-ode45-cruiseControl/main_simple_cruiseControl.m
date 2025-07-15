clear all
close all
clc

% Set plotting of axis lable to use latex interpreter
figure
set(0, 'DefaultLineLineWidth', 1.5);
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(gcf,'renderer','Painters')
set(gca,'LooseInset',get(gca,'TightInset'))
close

% % Create a folder to save the plots to (commented out):
% folder_plots_stills = 'plots_res_stills';
% if ~exist(folder_plots_stills, 'dir')
%    mkdir(folder_plots_stills)
% end

%% Simulate acceleration of vehicle

% A vehicle is driven by a constant force and the acceleration is
% simulated, where air resistance is taken into account

% The input power changes according to velocity to keep the velocity inside
% of a defined boundary; Two ways are shown, how something like this can be
% simulated;

% The following should be simulated:
%   -> If the speed is above 40 m/s -> switch to deceleration
%   -> If the speed is below 30 m/s -> switch to acceleration

%% Define the required parameters
% Source for cd and A_car:
% https://newsroom.porsche.com/en/products/taycan/aerodynamics-18554.html

% Some more information on the powertrain:
% https://newsroom.porsche.com/en_US/products/taycan/powertrain-18555.html

p.rho_air = 1.36; % [kg/m^3]
p.cD = 0.75; % [-] Air friction coefficient
p.A_car = 2.5; % [-m^2] Surface area of vehicle
p.m_car = 1200; % [kg] Mass of the vehicle
p.g = 9.81; % [m/s^2]
p.d_wheel = 20*25.4*1e-3; % [m] Wheel diameter in meter (20 inch wheel)

% Maximum power and torque with Launch control
p.Power_Max = 100*1e3; % [W]: - 760kW: Equivalent to approx. 1034 HP
p.Torque_Max = 1000; % [Nm]: Maximum torque

% Define the two limits, where acceleration and deceleration should occur
lim_xp_decel = 40;
lim_xp_accel = 30;

%% Define starting parameters
x0 = 0; % [m] - Position at the start
v0 = 0; % [m/s] - Speed at the start
sim0_vec = [x0, v0];
sim_tf = 35;

%% Solution 1: Discrete calls of integrator
% Always let ode45 run for 1ms - then check, if power has to be adjusted:
%   - Yes -> Make adjustment to power
%   - No  -> Just hold power constant as it is currently

% Initialize power for first step (so that vehicle starts to accelerate at
% t=0 already)
power_system = 1;

% Initialize simulation (timestep for subsim, prepare vectors for results)
t_step = 1e-3;
t_sim_disc = 0:t_step:sim_tf;
state_disc = sim0_vec;

% Prepare vectors for additional exports of simulation
xpp_calc = zeros(size(t_sim_disc));
pwr_actual = zeros(size(t_sim_disc));
torque_actual = zeros(size(t_sim_disc));

% Run loop in 1ms intervalls to simulate (is equivalent to a controller
% only being able to act every 1ms; input signal is then held constant for
% this interval)
tic % tic-toc routines for timing of calculation time
for k_disc = 1:(length(t_sim_disc)-1)
    [~, state_step] = ode45(@(t,x) ode_vehicle_powerDist(t, x, p, power_system),...
        [0, t_step], state_disc(end,:));
    
    state_disc = [state_disc; state_step(end,:)];
    
    % Reconstruct acceleration and save (together with actual power and
    % torque acting in the system)
    [xpp_this, ~, ~, M_this, P_this] =...
        ode_vehicle_powerDist(t_sim_disc(k_disc), state_disc(end-1,:), p, power_system);
    xpp_calc(k_disc) = xpp_this(2);
    torque_actual(k_disc) =M_this;
    pwr_actual(k_disc) = P_this;
    
    % Adjust power if required (is the "controller" part of this version)
    if state_step(end,2) >= lim_xp_decel
        power_system = -1;
    elseif state_step(end,2) <= lim_xp_accel
        power_system = 1;
    end
end
sim_time_disc = toc;

% Acceleration, power and torque for last point
[xpp_this, ~, ~, M_this, P_this] =...
    ode_vehicle_powerDist(t_sim_disc(end), state_disc(end,:), p, power_system);
xpp_calc(end) = xpp_this(2);
torque_actual(end)= M_this;
pwr_actual(end)= P_this;

% Split results out into individual variables (personal preference):
x_sim_disc = state_disc(:,1);
xp_sim_disc = state_disc(:,2);

% Get indizes, where we are accelerating (set others to nan to allow for
% nicer looking plots)
idx_accel_this = (xpp_calc > 0);
idx_decel_this = (xpp_calc < 0);
idx_accel = zeros(size(t_sim_disc))*nan;
idx_decel = zeros(size(t_sim_disc))*nan;
idx_accel(idx_accel_this > 0.5) = 1;
idx_decel(idx_decel_this > 0.5) = 1;

%% Plot results for first analysis
% Show detected acceleration and deceleration sections

figure
subplot(4,1,1)
plot(t_sim_disc, xp_sim_disc)
hold on
grid on
plot(t_sim_disc(xp_sim_disc > 40), xp_sim_disc(xp_sim_disc > 40), 'rx')
ylabel('Velocity $\left[\rm{m}/\rm{s}\right]$')
title('Velocity of discrete implementation')

subplot(4,1,2)
plot(t_sim_disc, xpp_calc)
hold on
grid on
plot(t_sim_disc.*idx_accel, xpp_calc.*idx_accel, ':')
plot(t_sim_disc.*idx_decel, xpp_calc.*idx_decel, '--')
legend({'Full $\ddot{x}\!\left(t\right)$', 'acceleration sections',...
    'deceleration sections'})
ylabel('Acceleration $\left[\rm{m}/\rm{s}^2\right]$')

subplot(4,1,3)
plot(t_sim_disc, torque_actual)
hold on
grid on
ylabel('Torque [Nm]')

subplot(4,1,4)
plot(t_sim_disc, pwr_actual)
hold on
grid on
plot([t_sim_disc, nan, t_sim_disc],...
    [ones(size(t_sim_disc)), nan, -ones(size(t_sim_disc))]*p.Power_Max,...
    'r--', 'linewidth', 1)
legend('Actual power', 'Maximum Power')
ylabel('Power [W]')

% Demonstrating, that only one point is above 40 m/s when we switch to dec:
format long % Set to display more values after the "."
idx_last = find(xp_sim_disc > 40, 1, 'last');
xp_sim_disc(idx_last+[-2, -1, 0, 1, 2])

%% Plot showing the acceleration and deceleration sections
% Showing, when we accelerate and when we decelerate

figure
set(gcf,'units','inch','position',[1,1,20,10])
set(gca,'LooseInset',get(gca,'TightInset'))
set(gcf,'renderer','Painters')

plot(t_sim_disc, xp_sim_disc)
hold on
grid on
plot(t_sim_disc, 20 + torque_actual./150, 'r:')
plot(t_sim_disc.*idx_accel, xp_sim_disc'.*idx_accel, ':')
plot(t_sim_disc.*idx_decel, xp_sim_disc'.*idx_decel, '--')
plot([0, sim_tf], [1, 1]*lim_xp_decel, '-.', 'linewidth', 1)
plot([0, sim_tf], [1, 1]*lim_xp_accel, '-.', 'linewidth', 1)
legend({'$\dot{x}$ Simulation', 'Motor torque in (scaled)',...
    'Acceleration sections', 'Deceleration sections',...
    'Limit start deceleration', 'Limit start acceleration'},...
    'location', 'best')
xlim([0, sim_tf])
title('Simulation result: Changing $u\!\left(t\right)$ depending on $\dot{x}\!\left(t\right)$')
xlabel('time $t \ \left[\rm{s}\right]$')
ylabel('Speed $\dot{x} \ \left[\frac{\rm{m}}{\rm{s}} \right]$')
set(gca,'FontSize',20)

% Export plot as vectorgraphics
% print(strcat(folder_plots_stills,'/simres_with_torque'),'-dsvg')


%% Solution 2: Event handling for quicker calculation
% Define event handling in odeset for simulation
vel_disc = [40, 30];
odeOptions = odeset('Events',@(t,x) detectVelocityChangeRequired(t,x,vel_disc));

% Initialize power and vectors
power_system = 1;
t_end_total_now = 0; % To reduce total simulation time after each step
t_sim_full = 0;
state_sim_full = sim0_vec;

% Run simulation calls in while() loop, until endpoint is reached - this
% version is equivalent to a controller that can act "infinitely fast"
tic
while(t_end_total_now < sim_tf)
    % Run simulation
    solThis = ode45(@(t,x) ode_vehicle_powerDist(t, x, p, power_system),...
        [0, sim_tf-t_end_total_now], state_sim_full(end,:), odeOptions);

    % Interpolate to calculate state with better resolution
    t_sim_this_step = unique([0:1e-3:solThis.x(end), solThis.x(end)]);
    state_interp = deval(solThis, t_sim_this_step);

    % Update endtime (max runtime of simulation)
    t_end_total_now = t_end_total_now + solThis.x(end);
    
    % Append values to total simulation time
    t_sim_full = [t_sim_full, t_sim_full(end) + t_sim_this_step(2:end)];
    state_sim_full = [state_sim_full; state_interp(:,2:end)'];

    % Switch power depending which event got triggered
    if solThis.ie == 1
        power_system = -1;
    elseif solThis.ie == 2
        power_system = 1;
    end
end
sim_time_event = toc;

% Extract values for easier plotting of results
xp_sim_event = state_sim_full(:,2);


%% Create plot to show result
% Plotting of error does not make a lot of sense here, since there is a
% shift in time due to the different methods - so the error does not really
% show anything relevant

legend_calcTime_disc = strcat('$t_{\rm{cpu}} = ',...
    num2str(sim_time_disc), '\rm{s}$');
legend_calcTime_event = strcat('$t_{\rm{cpu}} = ',...
    num2str(sim_time_event), '\rm{s}$');

str_simDisc = strcat('Simulation: Discrete intervals;{ }', legend_calcTime_disc);
str_simevent = strcat('Simulation: Event handling;{ }', legend_calcTime_event);

leg_struct = [{str_simDisc}, {str_simevent}, {'Detail'}];

% Make plot showing results
figure
set(gcf,'units','inch','position',[1,1,20,10])
set(gca,'LooseInset',get(gca,'TightInset'))
set(gcf,'renderer','Painters')
plot(t_sim_disc, xp_sim_disc)
hold on
grid on
plot(t_sim_full, xp_sim_event, '--')
legend(leg_struct{[1,2]},'location', 'northwest')
% xlim([0, sim_tf])
title('Comparison: Velocity discrete and event handling')
xlabel('time $t \ \left[\rm{s}\right]$')
ylabel('Speed $\dot{x} \ \left[\frac{\rm{m}}{\rm{s}} \right]$')
set(gca,'FontSize',20)

% Export plot as vectorgraphics
% print(strcat(folder_plots_stills,'/simres_comp_bothMethods'),'-dsvg')