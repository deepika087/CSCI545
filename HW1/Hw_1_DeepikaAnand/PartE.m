% y0 - initial position
% dt - Time steps
% u0 - initial velocity = 0
% t0 - start time = 0
% tf - Final time = 1/10 or till the robot reaches the y = 1

function PartE(func, y0, dt, u0, t0, tf)

t=t0:dt:tf;
                            
y(1)=y0;
u(1)=u0;

for i = 1:length(t)-1
    y(i+1)=y(i) + dt * u(i);
    u(i+1)=u(i) + dt * (feval(func, t(i), y(i), u(i)))
end

figure
subplot(2,1,1)
plot(t, y)
xlabel('Time')
ylabel('Velocity')
title('Velocity plot')

subplot(2,1,2)
plot(t, u)
xlabel('Time')
ylabel('Acceleration')
title('Acceleration plot')



