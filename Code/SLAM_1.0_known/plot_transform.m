function [ y_trans ] = plot_transform( y_vec )
% The used video and standart matlab plots have a different convention for
% th y axis of plots. This is account for with this function.


y_trans = 1088-y_vec;


end

