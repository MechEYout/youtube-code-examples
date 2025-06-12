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
% folder_plots = 'plots_res';
% if ~exist(folder_plots, 'dir')
%    mkdir(folder_plots)
% end


%% Simulate acceleration of vehicle

% A vehicle is driven by a constant force and the acceleration is
% simulated, where air resistance is taken into account

% The input power can change, leading to potential errors in the
% simulation; Event handling is used to fix those issues

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

%% Define power acting over time

% Engine power - 1: give full power; 0: no power
t_switch_power = 25; % Time, when the power is switched

t_pwr = [0, t_switch_power, t_switch_power+3, t_switch_power+6];
p_pwr = [1, -1, 1, -1];

pwr_in_t = @(t) interp1(t_pwr, p_pwr, t, 'previous', 'extrap');

t_pwr_show = 0:1e-3:50;


figure
plot(t_pwr_show, pwr_in_t(t_pwr_show)*100)
ylabel('\% power')
grid on
xlabel('time $t \ \left[\rm{s} \right]$')
title('Power in \% over time')

%% Define starting parameters and my power by the engine
% Run simulation as is to show the issues with this simulation

% Starting paramters
x0 = 0; % [m] - Position at the start
v0 = 0; % [m/s] - Speed at the start
sim0_vec = [x0, v0];
sim_tf = 35;

% Call simulator
[t_sim, state_sim] = ode45(@(t,x) ode_vehicle(t, x, p, pwr_in_t), 0:1e-3:sim_tf, sim0_vec);

% Split results out into individual variables (personal preference):
x_sim = state_sim(:,1);
xp_sim = state_sim(:,2);

%% Run simulation with fix - Using event handling

% Setup odeOptions to have event handling
odeOptions = odeset('Events',@(t,x) detectSwitchInPwr(t,x,pwr_in_t));

% First simulation run to find, when the event gets triggered
sol = ode45(@(t,x) ode_vehicle(t, x, p, pwr_in_t),...
    [0, sim_tf], sim0_vec, odeOptions);

% Get start and endtimes for simulation with events
if isempty(length(sol.xe))
    t0_sim_this = 0;
    tf_sim_this = sim_tf;
    
    number_sub_simulations = 1;
else
    t0_sim_this = zeros(1+length(sol.xe), 1);
    tf_sim_this = ones(1+length(sol.xe), 1)*sim_tf;
    
    for k=1:length(sol.xe)
        t0_sim_this(k+1) = sol.xe(k);
        tf_sim_this(k) = sol.xe(k);
    end
    
    number_sub_simulations = 1 + length(sol.xe); 
end

% Create vectors to store simulation result
t_sim_total = [0];
x_sim_total = sim0_vec;

% Iterate over simulation points - run simulation step by step
for k_sec = 1:number_sub_simulations
    t0_this = t0_sim_this(k_sec);
    tf_this = tf_sim_this(k_sec)-1e-16;
    
    t_this_rel = sort(unique([t0_this:1e-3:tf_this, t0_this, tf_this]))-t0_this;
    
    [~, state_sim] = ode45(@(t,x) ode_vehicle(t+t0_this, x, p, pwr_in_t),...
        t_this_rel, x_sim_total(end,:));
    
    t_sim_total = [t_sim_total, t_sim_total(end) + t_this_rel(2:end)];
    x_sim_total = [x_sim_total; state_sim(2:end,:)];
end

% Export to vectors, that are easier to use
t_sim_corr = t_sim_total;
x_sim_corr = x_sim_total(:,1);
xp_sim_corr = x_sim_total(:,2);

%% Plot: What we expect vs. what we get
% Comparison of "bad" and "good" simulation

figure
set(gcf,'units','inch','position',[1,1,20,10])
set(gca,'LooseInset',get(gca,'TightInset'))
set(gcf,'renderer','Painters')

plot(t_sim, xp_sim, '-.')
hold on
grid on
plot(t_sim_corr, xp_sim_corr)
plot(t_pwr_show, pwr_in_t(t_pwr_show)*12+30, 'r:', 'linewidth', 1.5)
plot(sol.x, sol.y(2,:), 'rx')
legend('Simulation', 'Simulation corrected', 'Input (Scaled for visualization)',...
    'Ode nodes')
xlim([0, 35])
title('Expectation vs. what we get')
xlabel('time $t \ \left[\rm{s}\right]$')
ylabel('Speed $\dot{x} \ \left[\frac{\rm{m}}{\rm{s}} \right]$')
set(gca,'FontSize',20)

% print(strcat(folder_plots,'/show_issue_simulation'),'-dsvg');

return

%% Quick and dirty version - just decrease timestep of ode
% Use t_sim_corr from above to show error

odeOptions = odeset('MaxStep', 1e-3);

t_interp = 0:1e-3:sim_tf; % DOES NOT effect accuracy - only interpolation!

% We use t_sim_corr here, so that we can plot the error below (this way we
% get the result on the same timeline (interpolated) as the first simulation
% above)
[t_QAD, x_vec_QAD] = ode45(@(t,x) ode_vehicle(t, x, p, pwr_in_t),...
    t_sim_corr, sim0_vec, odeOptions);

xp_sim_QAD = x_vec_QAD(:,2);


figure
subplot(2,1,1)
plot(t_sim, xp_sim, '-.')
hold on
grid on
plot(t_sim_corr, xp_sim_corr)
plot(t_QAD, xp_sim_QAD, '--')
legend('Simulation', 'Simulation corrected', 'Simulation (Quick and dirty fix)')

subplot(2,1,2)
plot(t_sim_corr, xp_sim_corr-xp_sim_QAD)
hold on
grid on
title('Error between both versions')















