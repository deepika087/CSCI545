filename = 'noisy.data';
delimiter=' ';
data=importdata(filename,delimiter);
yn=data(:,1);
xn=data(:,2);
un=data(:,3);

Wn=5/(100/2);

[B,A]=butter(2,Wn);
% to Do find coefficients.
disp(B)
disp(A)

xF=filter(B,A,yn); %Xf = Transmitted
t = 1:1000;

figure;
plot(t,xF,t,xn,'g'); %X1,Y1,LineSpec1,...,Xn,Yn,LineSpecn
xlabel('Time');
ylabel('Output');
title('Part B: Butterworth Filter');
legend('Filtered values','Unfiltered values');

td = finddelay(xn, xF);

%Kalman filter
lengthi=1000;

%Initializations
Q=0.001;
R=0.01;
A=0.9;
B=2;
P=zeros(1,lengthi);
X=zeros(1,lengthi);
P(1)=1;
K_arr=zeros(1,lengthi);
P_arr=zeros(1,lengthi);
K_arr(1)=0
P_arr(1)=1

for i=2:lengthi
    X(i) = A*X(i-1)+B*un(i-1);
    Ptemp = P(i-1);
    newP = (A*Ptemp*A')+Q;
    K=newP./(newP+R);
    X(i)=X(i)+(K*(yn(i)-X(i)));
    P(i) = (1-K).* newP;
    K_arr(i) = K;
    P_arr(i) = P(i);
end

figure;
plot(t,X,'r', t,xn,'g');
xlabel('Time');
ylabel('Output');
title('Part C: Kalman Filter');
legend('Filtered values','Unfiltered values');


subplot(2,1,1)
plot(1:lengthi, K_arr, 'b')
title('Gain plotted wrt time/iterations')

subplot(2,1,2)
plot(1:lengthi, P_arr, 'g')
title('Posterior covariance matrix plotted wrt time/iterations')

td = finddelay(xn, X)
disp(td)
disp(P_arr(lengthi - 1))