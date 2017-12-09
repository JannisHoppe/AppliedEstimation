% H : 4xM
% Each row refers to one entry of the Jacobian of the observation model.
function H = calculate_jacobian(S_bar,idx)
z_hat = observation_model_slam(S_bar(1:3,:),S_bar(idx+1:idx+2,:));
z_hat = z_hat(1,:);
H1 = (S_bar(idx+1,:)-S_bar(1,:))./z_hat;
H2 = (S_bar(idx+2,:)-S_bar(2,:))./z_hat;
H3 = -((S_bar(idx+2,:)-S_bar(2,:))./(z_hat).^2);
H4 = -((S_bar(idx+1,:)-S_bar(1,:))./(z_hat).^2);
H = [H1;H2;H3;H4];
end