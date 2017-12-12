syms

z_hat = [1 2 3 4;5 6 7 8];
z = [3.5;5];

delta = z-z_hat;

Q = [1,2,3,4;5,6,7,8;9,10,11,12;13,14,15,16]';

%zd1*(Q11*zd1 + Q21*zd2) + zd2*(Q12*zd1 + Q22*zd2)

exponent = delta(1,:).*(Q(1,:).*delta(1,:) + Q(3,:).*delta(2,:)) + delta(2,:).*(Q(2,:).*delta(1,:) + Q(4,:).*delta(2,:))

determinat = (Q(1,:).*Q(4,:)-Q(3,:).*Q(2,:)).^0.5