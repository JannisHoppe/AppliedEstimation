% H : 4xM
% Each row refers to one entry of the Jacobian of the observation model.
function H = calculate_jacobian(S,L)
z_hat = observation_model_slam(S,L);
z_hat = z_hat(1,:);
H1 = (L(1,:)-S(1,:))./z_hat;
H2 = (L(2,:)-S(2,:))./z_hat;
H3 = -((L(2,:)-S(2,:))./(z_hat).^2);
H4 = -((L(1,:)-S(1,:))./(z_hat).^2);
H = [H1;H2;H3;H4];
end