% This function will take input as current state, current velocity and
% current acceleration given as x0, x0_d, x0_dd respectively. 
% Final  state, velocity and acceleration given as xf, xf_d, xf_dd. Tau
% will be time to go

function [c0, c1, c2, c3, c4, c5] = coefficient(x, xd, xdd, tf, tf_d, tf_dd, tau)

c0 = x;
c1 = xd;
c2 = xdd/2;

c3 = -(20*x - 20*tf + 12*tau*xd + 8*tau*tf_d + 3*xdd*tau^2 - tf_dd*tau^2)/(2*tau^3);

c4 = (30*x - 30*tf + 16*tau*xd + 14*tau*tf_d + 3*xdd*tau^2 - 2*tf_dd*tau^2)/(2*tau^4);

c5 = -(12*x - 12*tf + 6*tau*xd + 6*tau*tf_d + xdd*tau^2 - tf_dd*tau^2)/(2*tau^5);

%X = sprintf('%d %d %d %d %d %d at tfinal = %d.\n',c0, c1, c2, c3, c4, c5, tfinal);




