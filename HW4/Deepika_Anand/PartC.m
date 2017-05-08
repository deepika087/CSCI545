% y0 - initial position
% dt - Time steps
% u0 - initial velocity = 0
% t0 - start time = 0
% tf - Final time = 1/10 or till the robot reaches the y = 1
% PartC(@PartC_funct, 0, 0.001, 0, 0, 1)
t=0:0.001:1;

x(1)=1;
xdot(1)=0;
alpha_x = 8;

y(1)=0;
ydot(1)=0;
zdot(1)=0;
y_d(1)=0;
y_dd(1)=0;

c = [ 1.0000 0.6294 0.3962 0.2494 0.1569 0.0988 0.0622 0.0391 0.0246 0.0155];
sigmaSquare = [ 41.6667 16.3934 6.5359 2.5840 1.0235 0.4054 0.1606 0.0636 0.0252 0.0252];
sigmaSquare = (1/1000).*sigmaSquare;
%w =  [ -100 -100 -100 1000 100000 1000 1 1 1 1];
w=[ 0 0 0 0 0 0 0 0 0 0]
%w= [ -305.1741 -435.7856 -686.6491 -840.2790 -627.4147   37.7373  761.4741  896.6861  406.3107  174.0274];
%w = [-246.1932594	-462.5872036	-759.5941195	-931.9358686	-642.6412049	198.350185	1030.307108	991.5533221	238.9686813	33.24762351]
%w=[ 10 10 10 10 10 10 10 10 10 10]
%w=[ -10 -10 -10 -10 -10 -10 -10 -10 -10 -10]
%w=[ 1 1 1 100 1000 100 1 1 1 1];

si(1)=0;
phi(1)=0;

finalmatrix = [];
for i = 1:length(t)-1
    phi=[];
    si=[];
    for j = 1:10
        si(j)= exp((-1/(2 * sigmaSquare(j))) * ( (x(i) - c(j)) * (x(i) - c(j)) ));
    end
    for j = 1:10
        phi(j) = (si(j) * x(i))/sum(si);
    end   
    finalmatrix = [finalmatrix; si];
    force = phi * transpose(w);	
  
    zdot(i+1) = 25*(6*(1-y(i))-ydot(i))+force;
    ydot(i+1) = ydot(i) + zdot(i)*dt;
    y(i+1) = y(i) + ydot(i+1)*dt;
    x(i+1) = x(i) - alpha_x*x(i)*dt;

end
figure
subplot(4,1,1);
plot(t, y)
title('Position plot')

subplot(4,1,2)
plot(t, ydot)
title('Velocity plot')

subplot(4,1,3)
plot(t, zdot)
title('Acceleration plot')

subplot(4, 1, 4)
x(1002)=[];
plot(t, x)
title('x plot')

y1=transpose(finalmatrix(:,1));
y2=transpose(finalmatrix(:,2));
y3=transpose(finalmatrix(:,3));
y4=transpose(finalmatrix(:,4));
y5=transpose(finalmatrix(:,5));
y6=transpose(finalmatrix(:,6));
y7=transpose(finalmatrix(:,7));
y8=transpose(finalmatrix(:,8));
y9=transpose(finalmatrix(:,9));
y10=transpose(finalmatrix(:,10));
size(y1)
t(1001) = [];
%figure
%plot(t, y1, t, y2, t, y3, t, y4, t, y5, t, y6, t, y7, t, y8, t, y9, t, y10)




