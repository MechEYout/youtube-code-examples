function [x_analyt] = analyticalSol_CartOnWheels_Adbd(t, uk, x0_vec, p)
%[x_analyt] = analyticalSol_CartOnWheels_Adbd(t, uk, x0_vec, p)
%   Calculate the analytical solution of the cart on wheels example: Mass
%   held back by a spring and damper and a force F = uk pulles in the
%   front; Analytical solution calculated based on Ad and bd, multiplied
%   with x0 and xp0 (state at the start at t=0); Input uk HAS to be
%   constant over the entire time (is checked)
%
%   INPUT
%       t       ... time, where the analytical solution is be calculated
%       uk      ... input signal (has to be constant over all timepoints,
%                   since the analytical solution assumes constant u(t)=uk;
%                   can be provided as vector with same length as t or as
%                   single variable)
%       x0_vec  ... initial points for position and velocity
%                   x0_vec = [x_0, xp_0]
%       p.      ... struct containing the parameters
%        .m     ... [kg]   ... Mass of the cart
%        .k     ... [N/m]  ... Spring constant holding the mass back
%        .m     ... [kg/s] ... Damping constant of the damper
%
%   OUTPUT
%       x_analyt ... Analytical solution calculated based on the parameters
%                    and initial values provided

%% Check provided input uk for dimension and if provided correctly
% Check, if uk is a vector and return error, if entries are different
if length(uk) > 1
    if length(t) ~= length(uk)
        error('uk and t are a different length')
    end
    if any(abs(diff(uk)) > 1e-8)
        error('all entries of uk have to be equal')
    end
end

%% Calculate analytical solution
% Calculate intermediary values
delta = p.d/(2*p.m);
wd = sqrt(p.k/p.m - delta*delta);

% Extract starting position and velocity
x0 = x0_vec(1);
xp0 = x0_vec(2);

% Precalculate exp() part
e_part = exp(-delta*t);

% Calcualte parts effected by x0 and xp0 (this is the Ad*x multiplication)
p_x0 = ((e_part./(wd)).*(delta*sin(wd*t) + wd*cos(wd*t)))*x0;
p_xp0 = (((e_part./(wd)).*sin(wd*t)))*xp0;

% Calculate part concerning uk (this is the bd*uk multiplication)
p_uk = ((1/(p.m*wd*(wd*wd + delta*delta)))*(wd -...
    e_part.*(delta*sin(wd*t) + wd*cos(wd*t)))).*uk;

% Sum together to calculate the resulting x(t)
x_analyt = p_x0 + p_xp0 + p_uk;

end