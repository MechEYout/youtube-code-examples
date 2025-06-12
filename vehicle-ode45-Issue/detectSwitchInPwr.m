function [value,isterminal,direction] = detectSwitchInPwr(t, x, pwr_in_t)
%[value,isterminal,direction] = haltIntegrationAtGround(t, x, param)
% Event handling for ode45 to halt the simulation, after the ball hits the
% height specified in param.end_height from above
%
%       See Matlab documentation for general information of event handling:
%   	https://de.mathworks.com/help/matlab/math/ode-event-location.html
%
% INPUT
%   t, x -> Required inputs for ode45 - see documentation of ode45
%           ... We do not use the state here, we only use our
%               input-function pwr_in_t to determine, when the switches are
%   param ... Parameters of the system
%       .pwr_in_t  ... Function describing the power put into our system;
%                      The way it is setup here only allows to detect, when
%                      the power crosses 0 (from positive to negative or
%                      from negative to positive), but jumps (eg. 50% to
%                      60% are not detected)
%
% OUTPUT
%   value, isterminal, direction ... Required outputs for event handling

%% Setup of event handling

value = pwr_in_t(t);    % The value where we want to detect zero-crossings
isterminal = 0;         % DO NOT stop integration - only detect!
direction =  0;         % Both directions

end