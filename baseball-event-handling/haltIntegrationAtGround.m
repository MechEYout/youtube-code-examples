function [value,isterminal,direction] = haltIntegrationAtGround(t, x, param)
%[value,isterminal,direction] = haltIntegrationAtGround(t, x, param)
% Event handling for ode45 to halt the simulation, after the ball hits the
% height specified in param.end_height from above
%
%       See Matlab documentation for general information of event handling:
%   	https://de.mathworks.com/help/matlab/math/ode-event-location.html
%
% INPUT
%   t, x -> Required inputs for ode45 - see documentation of ode45
%   param ... Parameters of the system
%       .end_height  ... If the object reached this heigh with negative
%                        speed, the calculation should be stopped
%
% OUTPUT
%   value, isterminal, direction ... Required outputs for event handling

%% Setup of event handling

value = x(3)-param.end_height;  % The value that we want to be zero
isterminal = 1;                 % Halt integration 
direction = -1;                 % Only, when approached with yp < 0

end