function coff = quartic_polynomial(s0,s0_dot,s0_ddot,si,si_dot,si_ddot,T)

coff(1) = s0;
coff(2) = s0_dot;
coff(3) = s0_ddot;

A = [3*T^2,   4*T^3;
     6*T^1,   12*T^2];
b = [si_dot-s0_dot-s0_ddot*T;
     si_ddot-s0_ddot];
 
temp  = A\b;
coff(4:5) = temp';

