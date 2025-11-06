function [xp_vec_sys] = sys_cont(t, x, sys, u_use)
%[xp_vec_sys] = sys_cont(t, x, sys, u_use)
%       -> Function for simulating the cart on wheels system with
%          continuous simulation (using ode45)
%
%   INPUT
%       t, x -> Required inputs for ode45 - see documentation of ode45
%       sys.  ... Struct containing parameters of the system
%        .m [kg]   ... Mass of the system
%        .k [N/m]  ... Spring constant of spring holding the mass back
%        .d [kg/s] ... Damping of the damper (viscous, speed dependent
%                      damping)
%       u_use ... Input force to be used in simulation
%
%   OUTPUT
%       xp_vec_sys  ... Change of state for ode45;
%           -> First entry: Change of position
%           -> Second:      Change of speed

%% Divide ode-state into workable components
x_sys = x(1);
xp_sys = x(2);

%% Calculate movement of system
% Initialize reuturn vector
xp_vec_sys = zeros(2,1);

% Implement differential equation directly
xp_vec_sys(1) = xp_sys;
xp_vec_sys(2) = -(sys.k/sys.m)*x_sys -(sys.d/sys.m)*xp_sys + ...
    (1/sys.m)*u_use;

end