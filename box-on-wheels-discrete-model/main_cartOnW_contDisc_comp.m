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
    folder_plots = 'demo_error_contDisc';
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

%% Simulation: Cart on Wheels with Different Methods
% Simulating the system using different methods and comparing:
%   - Analytical solution of the differential equation
%   - Continuous solution calculated with ode45 and a very small step size
%   - Discrete solution; two versions:
%     -> Concatenated: computed step by step, from one point to the next
%     -> Direct: computed directly from the initial point to the current
%        one

%% Define system and controller parameters
% Cart pulled back by spring with a viscous damping element with a force
% pulling in front

% System parameters
sys.m = 1;      % [kg]   ... Mass of cart
sys.k = 200;    % [N/m]  ... Spring constant holding the mass back
sys.d = 5;      % [kg/s] ... Damping (speed dependend, viscous damping)

%% Define continuous system and discrete for t_ctrl = 1ms
A_cont = [0, 1; -sys.k/sys.m, -sys.d/sys.m];
B_cont = [0; 1/sys.m];
C_cont = [1, 0];
D_cont = [];

ss_cont = ss(A_cont, B_cont, C_cont, D_cont);

% Timestep for discrete simulation
t_ctrl = 1e-3;

% Calculate discrete matrix
ss_disc = c2d(ss_cont, t_ctrl);

%% Define and run different versions of the simulation
x0_vec = [0.354, 1.5640]; % [m, m/s]	... Initial state for simulation
u_input = 150;            % [N] 		... Input to use in simulation
tf_sim = 3; 	          % [s] 		... Endtime of simulation

% Define time vector for discrete simulation (used for interpolation in
% continuous ode45 simulation as well and is used to calculate the
% analytical solution to the differential equation)
t_sim = 0:t_ctrl:tf_sim;

% Calculate analytical solution (only the position x(t))
[x_analyt] =...
    analyticalSol_CartOnWheels_Adbd(t_sim, u_input, x0_vec, sys);

% Run continuous simulation (remember from the video of the ode45
% integrator, that providing t_sim only changes the interpolation - to
% change the simulation stepsize for accuracy, you must provide the
% options using: odeset() - currently commented out
% opt_cont = odeset('MaxStep', 1e-4);

[~, x_cont] =...
    ode45(@(t,x) sys_cont(t, x, sys, u_input), t_sim, x0_vec);%, opt_cont);

% Run different versions of discrete calculation; Initialize arrays:
x_disc_concat = zeros(2, length(t_sim));
x_disc_direct = zeros(2, length(t_sim));
x_disc_concat(:,1) = x0_vec;
x_disc_direct(:,1) = x0_vec;
for k = 1:(length(t_sim)-1)
    % Concatenated version, where we compute from point to point
    x_disc_concat(:, k+1) =...
        ss_disc.A*x_disc_concat(:, k) + ss_disc.B*u_input;
    
    % Direct version, where we compute each point from the first one
    t_stepThis = t_sim(k+1);
    ss_disc_this = c2d(ss_cont, t_stepThis); % This is comp. expensive
    x_disc_direct(:, k+1) =...
        ss_disc_this.A*x0_vec' + ss_disc_this.B*u_input;
    
    % Print the progress of the calculation to the screen
    fprintf('Step %6d of %6d done \n', k, (length(t_sim)-1))
end

%% Plot and compare the results
% Plot with two different limits for the error and export two separate
% plots

figure
set(gcf,'units','inch','position',[1,1,20,10])
set(gca,'LooseInset',get(gca,'TightInset'))
set(gcf,'renderer','Painters')

subplot(2,1,1)
plot(t_sim, x_analyt, 'linewidth', 3)
hold on
grid on
plot(t_sim, x_cont(:,1), '--', 'linewidth', 2)
plot(t_sim, x_disc_concat(1,:), ':', 'linewidth', 1.5)
plot(t_sim, x_disc_direct(1,:), '-.', 'linewidth', 1)
legend('analytical', 'ode45()', 'discrete concat', 'discrete direct')
set(gca,'FontSize',20)
xlabel('time $t \ \left[\rm{s}\right]$')
ylabel('position $x \ \left[\rm{m}\right]$')
title('Movement of mass: Different simulation methods')

subplot(2,1,2)
plot(t_sim, x_analyt-x_cont(:,1)', 'color', pl_colors{2})
hold on
grid on
plot(t_sim, x_analyt-x_disc_concat(1,:), 'color', pl_colors{3})
plot(t_sim, x_analyt-x_disc_direct(1,:), 'color', pl_colors{4})
set(gca,'FontSize',20)
xlabel('time $t \ \left[\rm{s}\right]$')
ylabel('position error $\tilde{x} \ \left[\rm{m}\right]$')
title('Error in position: Different simulation methods')

for k=1:2
    if k==1
        plot_name = 'comp_discCont_bigYlim';
%         ylim(10*[-1, 1]*1e-14)
    else
        plot_name = 'comp_discCont_smallYlim';
        ylim(1*[-1, 1]*1e-14)
    end
    if export_plots_to_folder
        print(strcat(folder_plots, '\', plot_name),'-dsvg');
    end
end
