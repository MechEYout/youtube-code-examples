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

% Create a folder to save the plots to:
export_plots_to_folder = 0; % Set to one to create folder and exp. plots
if export_plots_to_folder
    folder_plots = 'comp_PID_contDisc';
    if ~exist(folder_plots, 'dir')
       mkdir(folder_plots)
    end
end

% Get color def of plot for later
pl_colors = cell(8,1);
figure
hold on
grid on
for k=1:8
    ind = plot([0,1], [0,1]);
    pl_colors{k} = ind.Color;
end
close

%% Simulation: Cart on Wheels with PID controller
% Simulating sytsem with PID controller; Comparing:
%   - continuous
%   - discret
% implementation of the PID-controller; Discrete implementation can
% consider quantization in measurement to showcase influence of controller
% cycle time

%% Define system and controller parameters
% Cart pulled back by spring with viscously acting damping element with a
% force pulling in front

% System parameters
sys.m = 1;      % [kg]   ... Mass of cart
sys.k = 200;    % [N/m]  ... Spring constant holding the mass back
sys.d = 5;      % [kg/s] ... Damping (speed dependend, viscous damping)

% Controller parameters
ctrl_cont.kp = 500;
ctrl_cont.Ti = 0.3;
ctrl_cont.Td = 0.05;
ctrl_cont.uLim = 250;

ctrl_disc = ctrl_cont;
ctrl_disc.quant_use = 1;
ctrl_disc.quant_accur = 0.001;

delta_t_disc = 1e-3; % [s]  ... Cycle time of controller

%% Run continuous and discrete simulation
% Simulation setup: endtime, target point and start-state
tf_sim = 3; % [s] ... Endtime of simulation
x_goal = 1; % [m] ... Target point
x0_sim_vec = [0,0,0];   % [x(0), xp(0), eInt(0)]

% Run continuous simulation
[t_sim, state_sim_cont] = ode45(@(t,x) sys_with_PID(t, x, sys,...
    ctrl_cont, x_goal, ctrl_cont.uLim), [0, tf_sim], x0_sim_vec);

% Recreate u_continuous
u_cont_vec = zeros(size(t_sim));
for k_steps = 1:length(t_sim)
    [~, u_PID_ind, ~] =...
        sys_with_PID(t_sim(k_steps), state_sim_cont(k_steps,:)', sys,...
        ctrl_cont, x_goal, ctrl_cont.uLim);
    u_cont_vec(k_steps) = u_PID_ind;
end

% Run discrete simulation
tSim_disc = 0:delta_t_disc:t_sim(end);
x0_vec = x0_sim_vec(1:2);   % Startvalues for states of simulation
u_FF_t = @(t) 200*0 + 0*t;  % u_FF set to zero
[state_sim_disc, u_disc_vec] = simSys_discrete(ctrl_disc, tSim_disc,...
    x0_vec, x_goal, u_FF_t, ctrl_disc.uLim);

%% Plot results for comparison of continuous and discrete implementation

figure
set(gcf,'units','inch','position',[1,1,20,10])
set(gca,'LooseInset',get(gca,'TightInset'))
set(gcf,'renderer','Painters')

subplot(2,1,1)
plot(t_sim, state_sim_cont(:,1))
hold on
grid on
plot(tSim_disc, state_sim_disc(1,:), '--')
legend('continuous', 'discrete')
title('Comparison continuous and discrete simulation')
ylabel('position $x \ \left[\rm{m}\right]$')
xlabel('time $t \ \left[\rm{s}\right]$')
set(gca,'FontSize',20)
ylim([0, 1.025])
title('Comparison: Continuous and discrete PID-controller')

subplot(2,1,2)
stairs(tSim_disc, u_disc_vec, 'color', pl_colors{2}, 'linewidth', 1.5)
hold on
grid on
plot(t_sim, u_cont_vec, 'color', pl_colors{1})
legend({'$u_{\rm{sys,cont}}$', '$u_{\rm{sys,disc}}$'}, 'location', 'east')
ylim([0, ctrl_cont.uLim])
ylabel('ctrl input $F \ \left[\rm{N}\right]$')
xlabel('time t [s]')
set(gca,'FontSize',20)

if export_plots_to_folder
    plot_name = 'comp_discCont_fullRange';
    print(strcat(folder_plots, '\', plot_name),'-dsvg');
end

%% Showing detail to show actual position vs. measurement
% Recreate measurement based on quantization
x_actSim_disc = state_sim_disc(1,:);
x_meas_reconstr = floor(x_actSim_disc./ctrl_disc.quant_accur)*...
    ctrl_disc.quant_accur;

% Define window to show in plot
xlim_show = [1.8, 1.9];


% Create plot
figure
set(gcf,'units','inch','position',[1,1,20,10])
set(gca,'LooseInset',get(gca,'TightInset'))
set(gcf,'renderer','Painters')

subplot(2,1,1)
plot(tSim_disc, x_actSim_disc)
hold on
grid on
stairs(tSim_disc, x_meas_reconstr, 'linewidth', 1.5)
plot(tSim_disc, x_meas_reconstr, 'rx', 'markersize', 4)
set(gca,'FontSize',20)
xlim(xlim_show)
ylim([0.99775, 0.99925])
xlabel('time t [s]')
ylabel('position $x \ \left[\rm{m}\right]$')
legend({'$x_{\rm{sim}} \! \left( \rm{t} \right)$',...
    '$x_{\rm{meas}} \! \left( \rm{t} \right)$', 'time of measurement'},...
    'location', 'east')
title('Comparison: Actual and measured position')

subplot(2,1,2)
ylabel('ctrl input $F \ \left[\rm{N}\right]$')
xlabel('time t [s]')
set(gca,'FontSize',20)
stairs(tSim_disc, u_disc_vec, 'color', pl_colors{2}, 'linewidth', 1.5)
hold on
grid on
xlim(xlim_show)
set(gca,'FontSize',20)
xlabel('time t [s]')
ylabel('ctrl input $F \ \left[\rm{N}\right]$')
title(strcat('Controller action (Controller cycle time: ',...
    '$t_{\rm{ctrl}} = ', num2str(delta_t_disc*1e3), '\rm{ms}$)'))

if export_plots_to_folder
    plot_name = 'comp_discCont_detail';
    print(strcat(folder_plots, '\', plot_name),'-dsvg');
end
