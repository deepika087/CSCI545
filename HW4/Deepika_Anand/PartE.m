dt=0.001;
x(1)=1;
alpha_x = 8;
c = [ 1.0000 0.6294 0.3962 0.2494 0.1569 0.0988 0.0622 0.0391 0.0246 0.0155];
sigmaSquare = [ 41.6667 16.3934 6.5359 2.5840 1.0235 0.4054 0.1606 0.0636 0.0252 0.0252];
sigmaSquare = (1/1000).*sigmaSquare;
w(1)=0;	
filename = 'imitation.data';
delimiter=' ';
data=importdata(filename,delimiter);
yn=data(:,1);
y_dn=data(:,2);
y_ddn=data(:,3);

g = ones(1001);
g = g(:,1);
f_d = y_ddn - 25*(6*(g - yn) - y_dn);

t=0:dt:1;
for i = 1:length(t)-1
    x(i+1) = x(i) - alpha_x*x(i)*dt;
end
s=x; % 1000 X 1

for i=1:2
    psi=zeros(1001);
    for j=1:1001
        psi(j,j) = exp((-1/(2 * sigmaSquare(i))) * ( (x(j) - c(i)) * (x(j) - c(i)) ));
    end
    w(i) = (transpose(s) * psi * f_d)/(transpose(s) * psi * s);
end
disp(w)












