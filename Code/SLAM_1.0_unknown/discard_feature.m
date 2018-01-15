%if the log odds of the feature existence falls below a certain threshold
%(here <= 0) the feature is discarded and the memory management is adjusted
%accordingly.
function p_bar = discard_function(particle,discard)
    p_bar = zeros(size(particle,1),1)-1;
    p_bar(1:4) = particle(1:4);
    N = particle(5);
    drop = size(discard,2);
    p_bar(5) = N - drop;
    %iterate over particcle and save the particles that should not be
    %discarded
    p_idx = 6; %idx for the save postition for the new particle
    for i=1:N
        d_idx = 6 + (i-1)*7;
        if(ismember(d_idx,discard))
            continue
        else
            p_bar(p_idx:p_idx+6) = particle(d_idx:d_idx+6);
            p_idx = p_idx + 7;
        end
    end
end
