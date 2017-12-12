function weights = measurement_likelihoods(particle,z,Q_t)
    N = particle(5);
    M = size(z,2);
    weights = [];
    for j = 1:N
        l_idx = 6+(j-1)*7;
        z_hat = observation_model_slam(particle(1:3),particle(l_idx:l_idx+1));
        H = calculate_jacobian(particle(1:3),particle(l_idx:l_idx+1));
        H = reshape(H,2,2);
        Sigma = [particle(idx+2) particle(idx+3); 
                particle(idx+4) particle(idx+5)];
        Q = H * Sigma * H' + Q_t;
        inv_Q = Q^-1;
        weights = [weights; weight_feature(z,z_hat,Q_inv)];
    end
end