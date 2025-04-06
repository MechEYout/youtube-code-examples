function [dx] = ode_system_full(t, x, param)
%ode_system_full(t, x, param) Equation describing the movement of the
%box and wedge; Uses "\" to solve the linear system required to calculate
%the acceleration of the two masses (box and wedge)
%   INPUT
%       t, x -> Required inputs for ode45 - see documentation of ode45
%       param ... Parameters of the system
%   OUTPUT
%       dx -> Required output for ode45 - see documentation of ode45

%% Divide state-vector, so it is easier to work with
pos_mass = x(1);
vel_mass = x(2);
pos_wedge = x(3);
vel_wedge = x(4);

% Split parameters out in more convenient to use variables
m_box = param.m_box;
m_wedge = param.m_wedge;
alpha = param.alpha;
g = param.g;

%% Calculation of dynamic movement
% Define mass matrix and vector
M_mass = [m_box, m_box*cos(alpha); m_box*cos(alpha), m_box+m_wedge];
b_vec = [m_box*g*sin(alpha); 0];

% Solve ddot{vec{q}}:
q_pp_vec = M_mass\b_vec;

%% Calculate dynamic movement
% Use the two entries from the solution of M_mass\b_vec and the two states

dx = zeros(4,1);

dx(1) = vel_mass;
dx(2) = q_pp_vec(1);
dx(3) = vel_wedge;
dx(4) = q_pp_vec(2);

end
