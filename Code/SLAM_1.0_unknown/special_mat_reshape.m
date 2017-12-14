% Transforms a 1x4 vector to a 2x2 matrix
function mat_resh = special_mat_reshape(mat)
    mat_resh = zeros(2,2);
    mat_resh(1) = mat(1);
    mat_resh(2) = mat(3);
    mat_resh(3) = mat(2);
    mat_resh(4) = mat(4);
end