function [dx, F_air, F_air_x, F_air_y, angle_F_air] = dgl_ball_movement(t, x, param)
%[dx, F_air, F_air_x, F_air_y, angle_F_air] = dgl_ball_movement(t, x, param)
% Equation describing the movement of the ball, when throwen through the
% air
%
% INPUT
%   t, x -> Required inputs for ode45 - see documentation of ode45
%   param ... Parameters of the system
%       .p.rho_air     % [kg/m^3] density of air at sea level
%       .cD            % [-] air friction coefficient of object
%       .A_ball        % [m^2] Surface area facing air
%       .m_ball        % [kg] Mass of object
%       .g             % [m/s^2] gravitational acceleration
%       .use_incorrect_tan     ALWAYS set to = 0! - reason:
%                              If this is set to =1, the calculation
%                              returns inccorect results, for some areas,
%                              because the quadrant is not taken into
%                              consideration - added, so the effect could
%                              be visualized
%
% OUTPUT
%       dx -> Required output for ode45 - see documentation of ode45
%       F_air, F_air_x, F_air_y ... Force due to air resistance to plot
%                                   later
%       angle_F_air ... angle, that the air resistance is acting
%
% CHANGELOG
%   2025.02.06 - Initial creation of function

%% Divide state-vector, so it is easier to work with
x_sys = x(1);
xp_sys = x(2);
y_sys = x(3);
yp_sys = x(4);

%% Calculate air resistance and split into x and y
% Easy mistake to make here: DO NOT calculate F_{air,x} directly from xp:
%   -> The relationship is nonlinear: so first calculate the actual
%      velocity, then split up into x and y components using sin() and
%      cos()

v_ball_act_square = (xp_sys.^2 + yp_sys.^2);
F_air = (1/2)*param.rho_air*param.A_ball*param.cD*v_ball_act_square;

% Using the additional parameter, it is possible to activate the incorrect
% calculation of the angle, the air acts to visualize the error
if param.use_incorrect_tan
    angle_F_air = atan(yp_sys/xp_sys); % This is incorrect!
else
    angle_F_air = atan2(yp_sys, xp_sys); % This is correct!
end

% Information of direction is in the angle:
F_air_x = -cos(angle_F_air)*F_air; 
F_air_y = -sin(angle_F_air)*F_air;

%% Evaluate equations of motion
F_grav = param.m_ball*param.g; % If x is very high (rocket), the actual
%                                grav. force could be implemented here

xpp_sys = (1/param.m_ball)*F_air_x;
ypp_sys = (1/param.m_ball)*(F_air_y - F_grav);

%% Put into dx - vector

dx = zeros(4,1);
dx(1) = xp_sys;
dx(2) = xpp_sys;
dx(3) = yp_sys;
dx(4) = ypp_sys;

end