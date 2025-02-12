function [dx, F_air, F_act, M_act, P_act] = ode_vehicle(t, x, param)
%[dx, F_air, F_act, M_act, P_act] = ode_vehicle(t, x, param)
% Equation describing the movement of a vehicle, air resistance, maximum
% power and torque are taken into account (to be called by ode45());
%
%   INPUT
%       t, x -> Required inputs for ode45 - see documentation of ode45
%       param ... Parameters of the system (struct)
%        .rho_air   ... Density of air; at sea level: 1.36; % [kg/m^3]
%        .cD        ... Air friction coefficient
%        .A_car     ... Surface area of vehicle
%        .m_car     ... Mass of the vehicle
%        .d_wheel   ... Wheel diameter in meter
%        .Power_Max ... Maximum power of engine
%        .Torque_Max... Maximum torque of engine
%
%   OUTPUT
%       dx -> Required output for ode45() - see documentation of ode45()
%       F_air   ... Force of the air pushing against the vehicle
%       F_act   ... Actual force propelling the vehicle, when taking
%                   maximum power and maximum torque into account
%       M_act   ... Actual torque of the vehicle, when taking maximum power
%                   and maximum torque into account
%       P_act   ... Actual power, when taking maximum torque and movement
%                   speed into account
%
%   CHANGELOG:
%       22.01.2025  ... Initial creation of function

%% Divide state-vector, so it is easier to work with
x_sys = x(1); % Not used in calculation of dx
xp_sys = x(2);

%% Calculate torque and force, while taking maximum power into account

pwr_in = 1; % Always try to give 100% power
P_in_theoretical = pwr_in*param.Power_Max;

M_theretical = P_in_theoretical*(param.d_wheel/xp_sys);
M_act = min(param.Torque_Max, max(-param.Torque_Max, M_theretical));
P_act = (xp_sys/param.d_wheel)*M_act; % Calculate for export for plots

% Calculate Force pushing the vehicle depending on wheel diameter
F_act = (M_act/(param.d_wheel/2)); % [N] Force pushing the vehicle forward

%% Calculate air resistance
% "-" as sign, because it acts against the movement direction
F_air = -(1/2)*param.rho_air*param.A_car*param.cD*(xp_sys^2);

%% Evaluate equations of motion
xpp_sys = (1/param.m_car)*(F_act + F_air);

%% Put into dx - vector for export
dx = zeros(2,1);
dx(1) = xp_sys;
dx(2) = xpp_sys;

end