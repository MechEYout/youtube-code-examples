function [value,isterminal,direction] = haltSimulationAtLenghEll(t, x, param)
%[value,isterminal,direction] = haltSimulationAtLenghEll(t, x, param)
% Event handling for ode45 to halt the simulation, after the box reaches
% the defined bottom point of the wedge as specified in param.ell_slide
%
%       See Matlab documentation for general information of event handling:
%   	https://de.mathworks.com/help/matlab/math/ode-event-location.html
%
% INPUT
%   t, x -> Required inputs for ode45 - see documentation of ode45
%   param ... Parameters of the system
%       .ell_slide  ... If the box reaches this length, the simulation
%                       should be stopped, because we hit the bottom of our
%                       wedge (the endpoint, that we defined)
%
% OUTPUT
%   value, isterminal, direction ... Required outputs for event handling

%% Setup of event handling

value = x(1)-param.ell_slide;   % The value that we want to be zero
isterminal = 1;                 % Halt integration 
direction = 0;                  % Direction does not matter (considering
%                                 what the system looks like, the velocity
%                                 will always be positive though)

end