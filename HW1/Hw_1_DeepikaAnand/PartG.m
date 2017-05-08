%x(t) = c0 + c1 * t + c2 * t^2 + c3 * t^3 + c4 * t^4 + c5 * t^5
% The first three are start position triplet, then xf triplet is final
% position paramters and tf is the time for which the simulation will run
% Initally set it to 1 or 10. xf = 1 xf_d = 0. x0 = 0 and x0_d = 0

function PartG(func, x0, x0_d, x0_dd, xf, xf_d, xf_dd, t0, tf)
dt = 0.001;
t=t0:dt:tf;

syms c0 c1 c2 c3 c4 c5 x
f(x) = c0 + c1 * x + c2 * x^2 + c3 * x^3 + c4 * x^4 + c5 * x^5
df = diff(f, x);
dff = diff (df, x);

%Start position
distance(1) = x0;
velocity(1) = x0_d; %Becuase values of constants are not available yet. 
acc(1) = x0_dd;

time_to_go = tf;

for i = 1:length(t)-1
   
   fprintf('Executed i = %d and time_to_go = %d \n', i, time_to_go)
   
   [o_c0, o_c1, o_c2, o_c3, o_c4, o_c5] = feval(func, distance(i),velocity(i),acc(i),xf, xf_d, xf_dd, time_to_go);
 
   distance(i+1) = vpa(subs(f, {c0, c1, c2, c3, c4, c5, x}, {o_c0, o_c1, o_c2, o_c3, o_c4, o_c5, dt}));
   velocity(i+1) = vpa(subs(df, {c1, c2, c3, c4, c5, x}, {o_c1, o_c2, o_c3, o_c4, o_c5, dt}));
   acc(i+1) = vpa(subs(dff, {c2, c3, c4, c5, x}, {o_c2, o_c3, o_c4, o_c5, dt}));

   time_to_go = time_to_go - dt;
end

figure
subplot(3,1,1)
plot(t, distance)
xlabel('Time')
ylabel('Distance')
title('Distance plot')

subplot(3,1,2)
plot(t, velocity)
xlabel('Time')
ylabel('velocity')
title('velocity plot')

subplot(3,1,3)
plot(t, acc)
xlabel('Time')
ylabel('Acceleration')
title('Acceleration plot')
