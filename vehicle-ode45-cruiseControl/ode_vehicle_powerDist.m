function [dx, F_air, F_act, M_act, P_act] = ode_vehicle_powerDist(t, x, param, pwr_in)
%[dx, F_air, F_act, M_act, P_act] = ode_vehicle_powerDist(t, x, param, pwr_in)
% Equation describing the movement of the vehicle, when the motor is acting
% on it (vehicle effected by air resistance)
%
%   INPUT
%       t, x -> Required inputs for ode45 - see documentation of ode45
%       param  ... Parameters of the system
%       pwr_in ... Desired power input [0...1] from the motor (is limited
%                  by the actual torque and velocity)
%   OUTPUT
%       dx -> Required output for ode45 - see documentation of ode45
%       F_air, F_act ... Forces due to air resistance and actual force
%                        acting on the vehicle and pushing it forward
%       M_act, P_act ... Actual torque and power acting in the vehicle

%% Divide state-vector, so it is easier to work with
x_sys = x(1);
xp_sys = x(2);

%% Calculate torque
% Power provided depends on the distance traveled
P_in_theoretical = pwr_in*param.Power_Max;
M_theretical = P_in_theoretical*(param.d_wheel/xp_sys);
M_act = min(param.Torque_Max, max(-param.Torque_Max, M_theretical));
P_act = (xp_sys/param.d_wheel)*M_act;

% Calculate Force pushing the vehicle depending on wheel diameter
F_act = (M_act/(param.d_wheel/2)); % [N] - Force pushing the vehicle forward

%% Calculate air resistance
% "-", because it acts against the movement direction
F_air = -(1/2)*param.rho_air*param.A_car*param.cD*(xp_sys^2); 

%% Evaluate equations of motion
xpp_sys = (1/param.m_car)*(F_act + F_air);

%% Put into dx - vector
dx = zeros(2,1);
dx(1) = xp_sys;
dx(2) = xpp_sys;

end