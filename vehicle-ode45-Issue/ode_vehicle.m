function [dx, F_air, F_act, M_act, P_act] = ode_vehicle(t, x, param, pwr_in_t)
%dgl_ball_movement(t, x, param) Equation describing the movement of the
%ball, when throwen through the air
%   INPUT
%       t, x -> Required inputs for ode45 - see documentation of ode45
%       param ... Parameters of the system
%       pwr_in_t ... Power in defined as function (here we also use "t"
%                    from the manadatory inputs)
%   OUTPUT
%       dx -> Required output for ode45 - see documentation of ode45
%       F_air, F_air_x, F_air_y ... Force due to air resistance (not used
%                                   or shown in the plots)

%% Divide state-vector, so it is easier to work with
x_sys = x(1);
xp_sys = x(2);

%% Calculate torque
% Power provided
pwr_in = pwr_in_t(t);

P_in_theoretical = pwr_in*param.Power_Max;
M_theretical = P_in_theoretical*(param.d_wheel/xp_sys);
M_act = min(param.Torque_Max, max(-param.Torque_Max, M_theretical));
P_act = (xp_sys/param.d_wheel)*M_act;

% Calculate Force pushing the vehicle depending on wheel diameter
F_act = (M_act/(param.d_wheel/2)); % {N] - Force pushing the vehicle forward

%% Calculate air resistance
F_air = -(1/2)*param.rho_air*param.A_car*param.cD*(xp_sys^2); % "-", because it acts against the movement direction

%% Evaluate equations of motion
xpp_sys = (1/param.m_car)*(F_act + F_air);

%% Put into dx - vector

dx = zeros(2,1);
dx(1) = xp_sys;
dx(2) = xpp_sys;


end