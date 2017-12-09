clear all;
syms Mx xx My xy z x_theta z1 z2 real

mu = [Mx; My];
pos = [xx; xy; x_theta];
z = [z1 ;z2];
h = [sqrt((Mx-xx)^2+(My-xy)^2); atan2(My-xy,Mx-xx)-x_theta];
H = jacobian(h,mu)

%h_inv = solve(z==h,mu)

