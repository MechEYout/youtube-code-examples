function [value,isterminal,direction] = detectVelocityChangeRequired(t, x, vel_disc)
%[value,isterminal,direction] = detectVelocityChangeRequired(t, x, vel_disc)
% Event handling for ode45 to halt the simulation, if one of our events
% gets triggered (velocity going above vel_disc(1), or falling below
% vel_disc(2));
%
%       See Matlab documentation for general information of event handling:
%   	https://de.mathworks.com/help/matlab/math/ode-event-location.html
%
% INPUT
%   t, x -> Required inputs for ode45 - see documentation of ode45
%            ... We use the state here to determine the current velocity
%   vel_disc ... Velocity limits to trigger our event-detection; Vector
%                with two entries:
% vel_disc(1) ... If xp bigger than this - halt sim and change to decel
% vel_disc(2) ... If xp smaller than this - halt sim and change to accel
%
% OUTPUT
%   value, isterminal, direction ... Required outputs for event handling

xp_vehicle = x(2);

%% Setup of event handling
value = [xp_vehicle - vel_disc(1);      % The value where we want to detect
         xp_vehicle - vel_disc(2)];     %    zero-crossings
isterminal = [1;  1];   % Stop simulation for both if detected!
direction =  [1; -1];   % First: Positive direction, Second, negative dir.

end