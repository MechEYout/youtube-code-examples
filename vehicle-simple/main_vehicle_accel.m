clear all
close all
clc

% Set plotting of axis lables and title to use latex interpreter
figure
set(0, 'DefaultLineLineWidth', 1.5);
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(gcf,'renderer','Painters')
set(gca,'LooseInset',get(gca,'TightInset'))
close

%% Content of script: Simulate acceleration of vehicle

% A vehicle is driven: Power and torque are limited

% Acceleration up to maximum velocity is simulated and results are plotted

%% Define the required parameters
% No particular vehicle
p.rho_air = 1.36;   % [kg/m^3]
p.cD = 0.75;        % [-] Air friction coefficient
p.A_car = 2.5;      % [m^2] Surface area of vehicle
p.m_car = 1200;     % [kg] Mass of the vehicle
p.d_wheel = 20*25.4*1e-3;   % [m] Wheel diameter in meter (20 inch wheel)

% Maximum power and torque:
p.Power_Max = 100*1e3;      % [W]: - 100kW: Equivalent to approx. 134.1 HP
p.Torque_Max = 1000;        % [Nm]: Maximum torque

%% Define starting parameters, run simulation and export forces
% Starting paramters
x0 = 0; % [m] - Position at the start
v0 = 0; % [m/s] - Speed at the start
sim0_vec = [x0, v0]; % Put in vector for simulator

% Call simulator (first line - commented out: no interpolation of results)
% [t_sim, state_sim] = ode45(@(t,x) ode_vehicle(t, x, p), [0, 50], sim0_vec);
[t_sim, state_sim] = ode45(@(t,x) ode_vehicle(t, x, p), 0:1e-3:50, sim0_vec);

% Split results out into individual variables (personal preference):
x_sim = state_sim(:,1);
xp_sim = state_sim(:,2);

% Reconstruct acceleration and forces ------------------------------------
% Reconstruction of acceleration is also possible with the "deval()"-
% function (see comment in code)
xpp_sim = zeros(size(x_sim));
F_air_sim = zeros(size(x_sim));

F_act_sim = zeros(size(x_sim));
M_act_sim = zeros(size(x_sim));
P_act_sim = zeros(size(x_sim));
for k=1:length(t_sim)
    [dx_ind, F_air_ind, F_act_ind, M_act_ind, P_act_ind] =...
        ode_vehicle(t_sim(k), state_sim(k,:), p);
    
    % Fill vectors to plot later
    xpp_sim(k) = dx_ind(2);
    F_air_sim(k) = F_air_ind;
    
    F_act_sim(k) = F_act_ind;
    M_act_sim(k) = M_act_ind;
    P_act_sim(k) = P_act_ind;
end


%% Simulation compared to "dummy measurement"
% Since I do not have any actual measurements, I just created a few points
% here to highlight the importance of shownig everything in one plot

% Instead of adding random points - load your actual measurements here!
t_comparison = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50];
xp_comparison = [0, 13.5, 27, 37, 45, 50, 53.3, 55, 56.6, 57.5, 58];

% Make the plot
figure
set(gcf,'units','inch','position',[1,3,10,6]) % set position on screen
set(gca,'LooseInset',get(gca,'TightInset')) % Reduce borders around graph
set(gcf,'renderer','Painters') % Fix renderer (for vector graphics exports)

plot(t_sim, xp_sim)
hold on
grid on
plot(t_comparison, xp_comparison, 'rx', 'markersize', 10)
set(gca,'FontSize',14) % Set fontsize of everything larger
legend({'Simulation', 'Comparison points'}, 'location', 'east')
ylabel('Speed $\dot{x} \ \left[\frac{\rm{m}}{\rm{s}} \right]$')
xlabel('time $t \ \left[\rm{s} \right]$')
title('Velocity: Simulation with comparison points')



%% Bigger plot of simulation results
% Get estimate for v_terminal (more accurate would be to solve
% analytically, but since this is an easy calculation, just run simulation)
[t_sTerm, x_sTerm] = ode45(@(t,x) ode_vehicle(t, x, p), [0, 1000], sim0_vec);
v_terminal = x_sTerm(end,2);

% Find times, when we cross 10% and 50% of v_terminal
velocity_mark_per = [10, 50, 90];
velocities_mark = (velocity_mark_per/100)*v_terminal;
t_mark_vTerm(1) = t_sim(find(xp_sim > velocities_mark(1), 1));
t_mark_vTerm(2) = t_sim(find(xp_sim > velocities_mark(2), 1));
t_mark_vTerm(3) = t_sim(find(xp_sim > velocities_mark(3), 1));

% Create entries for legends
leg_vTerm = strcat('$v_{\rm{term}}=', num2str(v_terminal, 3), '\frac{\rm{m}}{\rm{m}}$');
leg_mark1_vTerm = strcat('$',num2str(velocity_mark_per(1)),...
    '\%$ of $v_{\rm{term}} \left(',num2str(velocities_mark(1),3),...
    '\frac{\rm{m}}{\rm{s}}\right)$ after $', num2str(t_mark_vTerm(1), 3), '\rm{s}$');
leg_mark2_vTerm = strcat('$',num2str(velocity_mark_per(2)),...
    '\%$ of $v_{\rm{term}} \left(',num2str(velocities_mark(2),3),...
    '\frac{\rm{m}}{\rm{s}}\right)$ after $', num2str(t_mark_vTerm(2), 3), '\rm{s}$');
leg_mark3_vTerm = strcat('$',num2str(velocity_mark_per(3)),...
    '\%$ of $v_{\rm{term}} \left(',num2str(velocities_mark(3),3),...
    '\frac{\rm{m}}{\rm{s}}\right)$ after $', num2str(t_mark_vTerm(3), 3), '\rm{s}$');


% Make the plot
figure
set(gcf,'units','inch','position',[1,1,20,10])

% You do not really see anything in the position, so I decided to remove it
% and show you how to make the "2 plots on one side and 3 on the other"
% subplot(3,2,1)
% plot(t_sim, x_sim)
% hold on
% grid on
% ylabel('Position $x \ \left[\rm{m} \right]$')

subplot(2,2,1)
plot(t_sim, xp_sim) % Multiply with 3.6 to show in km/h instead of m/s
hold on
grid on
plot(t_sim, ones(size(t_sim))*v_terminal, 'r--') % You can use yline() as
%                                                  well, if you have a
%                                                  newer version of matlab
plot(t_mark_vTerm(1), velocities_mark(1), 'bo') % Just markings in plot
plot(t_mark_vTerm(2), velocities_mark(2), 'rd')
plot(t_mark_vTerm(3), velocities_mark(3), 'kx')
ylabel('Speed $\dot{x} \ \left[\frac{\rm{m}}{\rm{s}} \right]$')
xlabel('time $t \ \left[\rm{s}\right]$')
title('Velocity of the vehicle')
set(gca,'FontSize',14)
% Mark times and velocities - Start
for k=1:length(velocities_mark)
    plot([0, 1, 1]*t_mark_vTerm(k), [1, 1, 0]*velocities_mark(k),...
        'k:', 'linewidth', 1)
    text(t_mark_vTerm(k)+1, 3, strcat('$', num2str(t_mark_vTerm(k),3),...
        '\rm{s}$'),'FontSize',14)
end
% Mark times and velocities - End
legend({'Simulated velocity', leg_vTerm, leg_mark1_vTerm, leg_mark2_vTerm,...
    leg_mark3_vTerm}, 'location', 'east', 'fontsize', 14)

subplot(2,2,3)
plot(t_sim, xpp_sim)
hold on
grid on
ylabel('Acceleration $\ddot{x} \ \left[\frac{\rm{m}}{\rm{s}^2} \right]$')
xlabel('time $t \ \left[\rm{s}\right]$')
title('Acceleraiton of the vehicle')
set(gca,'FontSize',14)

subplot(3,2,2)
plot(t_sim, F_act_sim)
hold on
grid on
plot(t_sim, -F_air_sim)
legend({'$F_{\rm{motor}}$', '$-F_{\rm{air}}$'},...
    'location', 'east', 'fontsize', 14)
xlabel('time $t \ \left[\rm{s}\right]$')
ylabel('Force $F_{\rm{motor}} \ \left[\rm{N} \right]$')
title('Forces acting on the vehicle')
set(gca,'FontSize',14)

subplot(3,2,4)
plot(t_sim, M_act_sim)
hold on
xlabel('time $t \ \left[\rm{s}\right]$')
grid on
ylabel('Torque $M_{\rm{motor}} \ \left[\rm{Nm} \right]$')
title('Wheel torque of the vehicle')
set(gca,'FontSize',14)

subplot(3,2,6)
plot(t_sim, P_act_sim*1e-3) % Divide [W] by 1000 to show in [kW]
hold on
grid on
ylabel('Power $P_{\rm{motor}} \ \left[\rm{kW} \right]$')
xlabel('time $t \ \left[\rm{s}\right]$')
title('Power required for the vehicle')
set(gca,'FontSize',14)

return

%% Demonstration: Providing more timesteps does not change the result
% In here it is demonstrated, that [0, tf] and [0:1e-3:tf] leads to the
% same result in the end, since the ode45 figures out the simulation
% timestep himself and then just interpolates on the time-interval provided
%
% You can type "open ode45" into the command window to see the code used in
% ode45 - here you see, that the loop that actually adjusts the timestep
% starts at around line 250 - in deval, which is then used to interpolate
% the results passed out in the structure sol, the result is simply
% interpolated

% Here we run the same simulation as above, but only provide [0, endtime]
sol = ode45(@(t,x) ode_vehicle(t, x, p), [0, 50], sim0_vec);

% Here we overrite the ode-function with some nonsense - so, when we call
% deval() afterwards, the function deval() has zero information of our
% system, because we do not pass any information to it - but the
% interpolated result is still the same
sol.extdata.odefun = @(t,x) [x(1), 1-x(2)]; % Add nonsense

% Call deval to interpolate the points of the solution (note, that deval
% has no information of our actual system dynamics, since it does not know,
% about ode_vehicle, because we removed that with the line above):
[Sxint,Spxint] = deval(sol,t_sim);

% Plot the result to show, 
figure
subplot(2,1,1)
plot(t_sim, Sxint(2,:)-xp_sim')
title('Error between deval() result and original $\dot{x}$')

subplot(2,1,2)
plot(t_sim, Spxint(2,:)-xpp_sim')
title({'Error regarding  $\ddot{x}$: See comment in code undernath',...
    'the plot to see, where this error comes from'})

% A note here on the error of the reconstructed acceleration:
%   The acceleration, when reconstructed like this is not the exact same.
%   The reason for it is most likely the following (I need to go deeper
%   into the code to confirm this however): If we calculate xpp with the
%   deval, the calculation is based on interpolation directly - if we
%   reconstruct with the for-loop shown above, we calculate xpp from an
%   interpolated result (interpolation of position and velocity) and that
%   is why it is different - will update this comment, when I go further
%   into how the ode45 works
