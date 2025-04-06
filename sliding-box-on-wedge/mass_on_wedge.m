clear all
close all
clc

% Set plots to have labels rendered with latex-font & symbols
figure
set(0, 'DefaultLineLineWidth', 1.5);
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(gcf,'renderer','Painters')
set(gca,'LooseInset',get(gca,'TightInset'))
close

% % Create a folder to save the plots to (commented out):
% folder_plots = 'plots_stills';
% if ~exist(folder_plots, 'dir')
%    mkdir(folder_plots)
% end

%% Simulation example: Sliding wedge

% A small box is sliding down on a wedge and there is no friction acting
% anywhere in the system, so the wedge is also pushed back.

%% Define the required parameters
% Select parameter set
select_parameter_set = 1;
m_box_poss = [1, 10, 1*1e3, 10*1e6];
m_length_mass = [0.1, 0.1, 0.15, 0.2]; % Drawing box in different sizes
m_height_mass = [0.05, 0.075, 0.1, 0.2]; % Drawing box in different sizes

% Define parameter struct to be used
p.alpha = deg2rad(30);      % [rad] ... incline angle of the wedge
p.m_box = m_box_poss(select_parameter_set);% [kg]  ... mass of the box
p.m_wedge = 10;             % [kg]  ... mass of the wedge
p.g = 9.81;                 % [m/s^2].. gravity

p.ell_slide = 1;            % [m]   ... Total length of sliding motion
p.ell_extra = 0.1;          % [m]   ... Extra length of wedge on top and
                            %           bottom for plots
p.x0_mass = 0;              % [m]   ... Startpoint of mass
            
%% Call simulation
% Defining starting values
x0_vec = [p.x0_mass, 0, 0, 0];

% Define event handling to halt simulation, when box hits: ell_slider = q1
odeOptions = odeset('Events',@(t,x) haltSimulationAtLenghEll(t,x,p));

% Run simulation with interpolation
[t_sim, state_sim] =...
    ode45(@(t,x) ode_system_full(t, x, p), 0:1e-3:10, x0_vec, odeOptions);

% Run simulation to extract the number of ode45 steps as well
ode_res = ode45(@(t,x) ode_system_full(t, x, p), 0:1e-3:10, x0_vec, odeOptions);

% Split results out:
q_mass = state_sim(:,1);
qp_mass = state_sim(:,2);
q_wedge = state_sim(:,3);
qp_wedge = state_sim(:,4);

%% Calculating the analytical solution of the problem
x_mass_anal = ((p.g*sin(p.alpha))/...
    (1-((p.m_box*cos(p.alpha)*cos(p.alpha))/(p.m_box+p.m_wedge))))*...
    (t_sim.*t_sim)./2;

t_bottom = sqrt((2*(p.ell_slide - p.x0_mass)*...
    (1-((p.m_box*cos(p.alpha)*cos(p.alpha))/(p.m_box+p.m_wedge))))/...
    (p.g*sin(p.alpha)));

%% Plot and comparison
% Create strings
str_endt_simulation = strcat('$t_{\rm{bottom,sim}}=', num2str(t_sim(end)*1e3),'\rm{ms}$');
str_endt_analytical = strcat('$t_{\rm{bottom,anal}}=', num2str(t_bottom*1e3),'\rm{ms}$');
str_int_steps = strcat('ode45 steps: $', num2str(length(ode_res.x)), '$');

figure
set(gcf,'units','inch','position',[4, 4, 12, 6])
set(gca,'LooseInset',get(gca,'TightInset'))
set(gcf,'renderer','Painters')

subplot(2,2,1)
plot(t_sim, q_mass)
hold on
grid on
plot(t_sim, x_mass_anal, '--')
plot(t_sim(end), q_mass(end), 'bo')
plot(t_sim(end), x_mass_anal(end), 'rx')
plot(ode_res.x, ode_res.y(1,:), 'm+')
legend({'$q_{1,\rm{sim}}$', '$q_{1,\rm{analytical}}$', str_endt_simulation,...
    str_endt_analytical, str_int_steps}, 'fontsize', 14', 'location', 'best')
title('Position of box: $q_1$');
set(gca,'FontSize',12)
xlabel('time $t \ \left[\rm{s} \right]$')
ylabel('$q_{1} \ \left[\rm{m} \right]$')


subplot(2,2,3)
plot(t_sim, q_mass-x_mass_anal)
grid on
title('$\tilde{q}_1 = q_{1,\rm{sim}} - q_{1,\rm{analytical}} $')
set(gca,'FontSize',12)
xlabel('time $t \ \left[\rm{s} \right]$')
ylabel('$\tilde{q}_{1} \ \left[\rm{m} \right]$')

subplot(2,2,4)
plot(t_sim, q_wedge)
hold on
grid on
plot(ode_res.x, ode_res.y(3,:), 'm+')
legend({'Position wedge', str_int_steps}, 'fontsize', 14', 'location', 'best')
set(gca,'FontSize',12)
title('Position of wedge: $q_2$');
xlabel('time $t \ \left[\rm{s} \right]$')
ylabel('$q_{2} \ \left[\rm{m} \right]$')

% subplot (2,2,2) is reserved for the picture of the system with markings
% on it, as shown in the video to make clear, where q1, etc. is - so it is
% not filled in this plot

% Save plot to folder:
% print(strcat(folder_plots, '\simulation_result'), '-dsvg')
