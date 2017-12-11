clear all;
% syms Mx xx My xy z x_theta z1 z2 real

% mu = [Mx; My];
% pos = [xx; xy; x_theta];
% z = [z1 ;z2];
% h = [sqrt((Mx-xx)^2+(My-xy)^2); atan2(My-xy,Mx-xx)-x_theta];
% H = jacobian(h,mu)

%h_inv = solve(z==h,mu)

syms H11 H12 H21 H22 Sig11 Sig12 Sig21 Sig22 Q11 Q12 Q21 Q22 K1 K2 K3 K4 mu1 mu2 zd1 zd2 real

H = [H11 H12;H21 H22];
Sig = [Sig11 Sig12;Sig21 Sig22];
H_T = transpose(H);
Q = [Q11 Q12;Q21 Q22];
invQ = inv(Q)




