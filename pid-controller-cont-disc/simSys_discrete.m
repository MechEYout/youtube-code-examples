function [state_sim, u_ctrl_vec] =...
    simSys_discrete(ctrl, tSim, x0_vec, x_goal, u_FF, uLim)
%[state_sim, u_ctrl_vec] =...
%   simSys_discrete(ctrl, tSim, x0_vec, x_goal, u_FF, uLim)
%       -> Function for simulating the cart on wheels system with discrete
%          PID controller (using the analytical solution, where the input
%          is kept constant over one controller cycle; calculated for our
%          specifc parameter set, but time Ta from step to step can be set
%          freely)
%
%   INPUT
%       ctrl. ... Struct containing the controller gains and (optionally)
%                 the resolution of the measurement to take quantization
%                 into account
%         .kp ... proportional gain (P-part of PID-controller)
%         .Ti ... integral time (I-part of PID-controller)
%         .Td ... derivative time (D-part of PID-controller)
%         .quant_use    ... (optional) -> 1 = limit resolution of
%                                         "measurement" used inside of the
%                                         controller to the value specified
%                                         in quant_accur; 0 or not provided
%                                         at all to not use it
%         .quant_accur  ... (optional) -> [m] resolution the "measurement"
%                                         is limited to in order to show
%                                         quantization effects
%       tSim  ... Timeline for the discrete simulation; timestep used is
%                 the mean timestep provided in tSim -> so provide tSim in
%                 the form of: tsim = [0:timestep:t_end]; Input u_FF and
%                 u_ctrl are held on constant level over each timestep
%       x0_vec  ... Startvales for simulation [x0, xp0] in [m, m/s]
%       x_goal  ... [m] Target endpoint for controller
%       u_FF    ... [N] Feedforward force  u_FF(t) provided as anonymous
%                       function, so the time dependency can be passed in;
%                       Example: "u_FF_t = @(t) 200 + 10*t;"
%       uLim    ... [N] Minimal and maximal foce acting from the controller
%                       -> Limit provided symmetrical:
%                               -uLim <= u(t) <= uLim
%
%   OUTPUT
%       state_sim   ... State as (2 x length(tSim)) array
%       u_ctrl_vec  ... Force acting on the system (PID + uFF; limited to
%                       "-uLim <= u(t) <= uLim" - this is the actual force)

%% Preparation
% Calculation of timestep from tSim
Ta = mean(diff(tSim));

% Preparation of state_sim for return values and u_ctrl_vec
state_sim = zeros(2, length(tSim));
state_sim(1,1) = x0_vec(1);
state_sim(2,1) = x0_vec(2);

u_ctrl_vec = zeros(1, length(tSim));

% If quantization is provided, set variable
if isfield(ctrl, 'quant_use')
    if ctrl.quant_use
        use_quant_input = 1;
    else
        use_quant_input = 0;
    end
else
    use_quant_input = 0;
end

%% Calculate matrixes Ad, bd
% With those, the analytical solution:
%       x_{k+1} = Ad*x_k + bd*u_k
% with u_k being constant over one controller cycle can be calculated

% Calculate Ad for variable Ta with parameters: m=1, k=200, d=5
Ad__1_1 = exp(-(5/2)*Ta)*cos((5*sqrt(31)*Ta)/2) + ...
    (sqrt(31)*exp(-(5/2)*Ta)*sin((5*sqrt(31)*Ta)/2))/31;
Ad__1_2 = (2*sqrt(31)*exp(-(5/2)*Ta)*sin((5*sqrt(31)*Ta)/2))/155;
Ad__2_1 = -(80*sqrt(31)*exp(-(5/2)*Ta)*sin((5*sqrt(31)*Ta)/2))/31;
Ad__2_2 = exp(-(5/2)*Ta)*cos((5*sqrt(31)*Ta)/2) - ...
    (sqrt(31)*exp(-(5/2)*Ta)*sin((5*sqrt(31)*Ta)/2))/31;

Ad = [Ad__1_1, Ad__1_2; Ad__2_1, Ad__2_2];

% Calculate bd for variable Ta with parameters: m=1, k=200, d=5
bd_1 = (1/200) - (exp(-(5/2)*Ta)*cos((5*sqrt(31)*Ta)/2))/200 - ...
    (sqrt(31)*exp(-(5/2)*Ta)*sin((5*sqrt(31)*Ta)/2))/6200;
bd_2 = Ad__1_2;

bd = [bd_1; bd_2];

%% Run through timeline and simulate system responce with controller
% Create variable for error (required to calculate derivative of error and
% euler-approximation for integral)
e_ctrl_km1 = 0;
ei_ctrl = 0;

% Loop through timeline
for k=1:(length(tSim) - 1)
    % Calculate required controller input
    x_vec_now = state_sim(:,k);
    x_sys = x_vec_now(1);
    
    % Take quantization of measurement into account (if required)
    if use_quant_input
        x_sys = floor(x_sys/ctrl.quant_accur)*ctrl.quant_accur;
    end
    
    % Calculate error, integral (for I-Part) and derivative (for D-part)
    e_ctrl = x_goal - x_sys;
    ei_ctrl = ei_ctrl + e_ctrl*Ta;
    ed_ctrl = (e_ctrl - e_ctrl_km1)/Ta;
    e_ctrl_km1 = e_ctrl;

    % Calculate controller input (if Ti=0; deactivate I-part of PID)
    u_P = ctrl.kp*e_ctrl;
    if ctrl.Ti ~= 0
        u_I = ei_ctrl*ctrl.kp*(1/ctrl.Ti);
    else
        u_I = 0;
        ei_ctrl = 0;
    end
    u_D = ctrl.kp*ctrl.Td*ed_ctrl;

    % Total controller action with feed-forward; Saturate to limits and
    % store in vector for export
    u_ctrl = u_P + u_I + u_D + u_FF(tSim(k));
    u_ctrl = min(max(u_ctrl, -uLim), uLim);
    
    u_ctrl_vec(k) = u_ctrl;

    % Calculate system response to controller input held constant over one
    % cycle
    state_sim(:,k+1) = Ad*x_vec_now + bd*u_ctrl;
end

% Set last input equal to (end-1) to show in plot correctly
u_ctrl_vec(end) = u_ctrl_vec(end-1);

end