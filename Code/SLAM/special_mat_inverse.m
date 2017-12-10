% Function for calculating the inverse of Matrices, whose elements are
% saved in the column of an other Matrix.
% Input : Mat 4xM
% Return:
% Inverse of the column matrices saved again as a column vector.
% inv: 4xM
function inv = special_mat_inverse(Mat)
%Scaling factor
inv = zeros(4,size(Mat,2));
deter = Mat(1,:) .* Mat(4,:) - Mat(2,:).* Mat(3,:);
%Swap
det = 1./deter;
inv(1,:) = det .* Mat(4,:);
inv(2,:) = (-1) *  det .* Mat(2,:);
inv(3,:) = (-1) * det .* Mat(3,:);
inv(4,:) = det .* Mat(1,:);

end