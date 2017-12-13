function p_bar = discard_function(particle,p_idx)
    p_bar = particle;
    N = particle(5);
    k_idx = 6 + (N-1)*7;
    p_bar (p_idx:p_idx+6) = particle(k_idx:k_idx+6);
    p_bar(k_idx:k_idx+6) = 0;
    p_bar(5) = particle(5) - 1;
end
