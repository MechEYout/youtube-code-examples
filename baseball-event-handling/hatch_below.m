function hatch_below(x_line, y_line, x_space)
%hatch_below(x_line, y_line, x_space) Function to add hatching below a
%certain line in a plot - Can only add hatched lines with one step in the
%middle, like it is used in the plots (it can only do 1 step going up)
%
%   Overall this is not a good implementation, but I needed it quickly for
%   a specific purpose - and it does the job
%
% INPUT:
%   x_line: vector with 3 entries: [startpoint, location of step, endpoint]
%   y_line: vector with 2 entries: [height at start, height after step]
%
% OUTPUT:
%   Draws lines in plot
%
% CHANGELOG:
%   2025.02.06 - Initial creation of function


number_hatchlines = ceil((max(x_line)-min(x_line))/x_space);

number_of_points_in_line = 1001;

for k=1:number_hatchlines
    % Calculate line:
    x_poss = min(x_line) + (k-1)*x_space + linspace(0,20, number_of_points_in_line);
    y_poss = max(y_line) - linspace(0,20, number_of_points_in_line);
    
    % Quick and dirty version of finding the points
    x_not = x_poss < x_line(2);
    y_not = y_poss > y_line(1);
    both_not = x_not.*y_not;
    
    idx_not = find(both_not > 0.5);
    
    x_plot = x_poss;
    x_plot(idx_not)= nan;
    y_plot = y_poss;
    y_plot(idx_not)= nan;
    
    plot(x_plot, y_plot, 'k', 'linewidth', 1)
    
end

end
