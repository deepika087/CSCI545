function euler(func, y0, dt, t0, tf)

t=t0:dt:tf;

y(1)=y0;

for i = 1:length(t)-1
    y(i+1)=y(i) + dt * (feval(func, t(i), y(i)));
end

t=t';
y=y';

plot(t, y)

xlabel('Time')
ylabel('Distance')



