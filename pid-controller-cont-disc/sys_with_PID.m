function [xp_vec_sys, u_use, u_parts] =...
    sys_with_PID(t, x, sys, ctrl, x_goal, uLim)
%[xp_vec_sys, u_use, u_parts] = sys_with_PID(t, x, sys, ctrl, x_goal, uLim)
%       -> Function for simulating the cart on wheels system with
%          continuous PID controller; Function calculates dx to be used
%          inside of ode45 and also returns the PID-parts and controller
%          action used on the system, so it can be reconstructed outside
%          and plotted
%
%   INPUT
%       t, x -> Required inputs for ode45 - see documentation of ode45
%       sys.  ... Struct containing parameters of the system
%        .m [kg]   ... Mass of the system
%        .k [N/m]  ... Spring constant of spring holding the mass back
%        .d [kg/s] ... Damping of the damper (viscous, speed dependent
%                      damping)
%       ctrl. ... Struct containing the controller gains and (optionally)
%                 the resolution of the measurement to take quantization
%                 into account
%         .kp ... proportional gain (P-part of PID-controller)
%         .Ti ... integral time (I-part of PID-controller)
%         .Td ... derivative time (D-part of PID-controller)
%       x_goal  ... [m] Target endpoint for controller
%       uLim    ... [N] Minimal and maximal foce acting from the controller
%                       -> Limit provided symmetrical:
%                               -uLim <= u(t) <= uLim
%
%   OUTPUT
%       xp_vec_sys  ... Change of state for ode45;
%           -> First entry: Change of position
%           -> Second:      Change of speed
%           -> Third:       Error (because this is the "change of the
%                           integral error" and this way the integral error
%                           is the third state in x directly)
%       u_use       ... [N] -> Actual force acting on the mass
%       u_parts.    ... Struct containing the different parts
%           .P [N] ... Proportional part
%           .I [N] ... Integral part
%           .D [N] ... Derivative part

%% Divide ode-state into workable components
% First two are from the system, the third is for the integral part of the
% controller (I-part)
x_sys = x(1);
xp_sys = x(2);
eI_sys = x(3);

%% Calcualte error and control input
% Error of system and derivative error (the derivative of the error is the
% same as the derivative of the components: ep_ctrl = xp_goal - xp_sys)
err = x_goal - x_sys;
err_p = 0 - xp_sys; % Equivalent to time derivative of error (xp_goal set
                    % to zero, becase x_goal is constant here; if a
                    % trajectory is provided for x_goal, xp_goal has to be
                    % provided accordingly)

% P-Part
u_P = err*ctrl.kp;

% D-Part
u_D = err_p*ctrl.kp*ctrl.Td;

% I-Part
if ctrl.Ti ~= 0
    u_I = eI_sys*ctrl.kp*(1/ctrl.Ti);
else
    u_I = 0;
end

% Store parts in struct for individual export
u_parts.P = u_P;
u_parts.I = u_I;
u_parts.D = u_D;

% Calculate total controller input and limit u, if uLim is provided
u_PID = u_P + u_I + u_D;

if nargin > 5
    u_use = min(max(u_PID, -uLim), uLim);
else
    u_use = u_PID;
end

%% Setup system
% Setup matrixes to calculate "xp = A*x + b*u" below;
% Alternatively, xpp can be calculated directly as well, like in the other
% videos - this is just a different version of doing it, but the result is
% identical in the end

sys.A = [0, 1; -sys.k/sys.m, -sys.d/sys.m];
sys.B = [0; 1/sys.m];

%% Calculate dx based on state and input to simulate system movement
xp_vec_sys = zeros(3,1);
xp_vec_sys(1:2) = sys.A*x(1:2) + sys.B*u_use; % Movement of system
xp_vec_sys(3) = err;                          % For integral of error

end