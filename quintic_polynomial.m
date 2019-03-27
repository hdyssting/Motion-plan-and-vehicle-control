function coff = quintic_polynomial(d0,d0_dot,d0_ddot,di,di_dot,di_ddot,T)

coff(1) = d0;
coff(2) = d0_dot;
coff(3) = d0_ddot/2;

A = [T^3,     T^4,       T^5;
     3*T^2,   4*T^3,     5*T^4;
     6*T^1,   12*T^2,    20*T^3];
 
b = [di-(d0+d0_dot*T+1/2*d0_ddot*T^2);
     di_dot-(d0_dot+d0_ddot*T);
     di_ddot-d0_ddot];

temp = A\b;
coff(4:6) = temp';