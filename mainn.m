clc;clear
N=501;
T=0.1;
alpha=1/60;
% X=[0 0 0 47.8109 18.1173 47.8109 0 0 0]';%%%X初值
X=[10 10 10 47.8109 18.1173 47.8109 1 1 1]';%状态向量初值
P=diag([1000;1000;1000;10;10;10;10;10;10]).^2;%%%%P初值
n=size(X,1);
% % % % 
target=load('target.txt');
% % % % % % 
UAV1=load('UAV1.txt');
UAV2=load('UAV2.txt');
Xs1=UAV1(:,2);
Ys1=UAV1(:,3);
Zs1=UAV1(:,4);
Xs2=UAV2(:,2);
Ys2=UAV2(:,3);
Zs2=UAV2(:,4);
% % % % % % 量测信息
measurement=load('measurement.txt');
gama1=measurement(:,2);
eta1=measurement(:,3);
gama2=measurement(:,4);
eta2=measurement(:,5);
% % % % % 
v=0.3*pi/180*randn;
R=v^2*eye(4);
% Q=diag([10;10;10;5;5;5;1;1;1]).^2;
O=zeros(3,3);
o=zeros(1,6);
I=eye(3);
E=eye(n);
% % % % % % % 
%过程噪声矩阵
Q=diag([10;5;1]);
%%%
F=[I T*I (1/alpha)^2*(-1+alpha*T+exp(-alpha*T))*I;
    O  I    (1/alpha)*(1-exp(-alpha*T))*I;
    O  O    exp(-alpha*T)*I];
U=[(T^2/2-(alpha*T-1+exp(-alpha*T))/alpha^2)*I;
      (T-(1-exp(-alpha*T))/alpha)*I;
      (1-exp(-alpha*T))*I];
G=[1/alpha^2*((1-exp(-alpha*T))/alpha+alpha*T^2/2-T)*I;
       (T-(1-exp(-alpha*T))/alpha)/alpha*I;
       (1-exp(-alpha*T))/alpha*I]; 
% % % %  系统噪声  以下噪声导致不能收敛
%  q11=1/(2*alpha^5)*(1-exp(-2*alpha*T)+2*alpha*T+2*alpha^3*T^3/3-2*alpha^2*T^2-4*alpha*T*exp(-alpha*T));
%  q12=1/(2*alpha^4)*(1+exp(-2*alpha*T)-2*exp(-alpha*T)+2*alpha*T*exp(-alpha*T)-2*alpha*T+alpha^2*T^2);
%  q13=1/(2*alpha^3)*(1-2*exp(-alpha*T)+2*alpha*T*exp(-alpha*T));
%  q22=1/(2*alpha^3)*(4*exp(-alpha*T)-3-exp(-2*alpha*T)+2*alpha*T);
%  q23=1/(2*alpha^2)*(exp(-2*alpha*T)+1-2*alpha*T);
%  q33=1/(2*alpha)*(1-exp(-alpha*T));
%  Q=2*alpha*100^2*[q11 q12 q13;q12 q22 q23;q13 q23 q33];
% % % %    
for k=1:N
    %X=F*X+U*X(7:9)+G*w;
     Z=[gama1(k) eta1(k) gama2(k) eta2(k)]';
     H=cal_H(X(1),X(2),X(3),Xs1(k),Xs2(k),Ys1(k),Ys2(k),Zs1(k),Zs2(k));
     X=F*X+U*X(7:9);
     P=F*P*F'+G*Q*G';
     K=P*H'/(H*P*H'+R);
     Zpre=cal_Z(X(1),X(2),X(3),Xs1(k),Xs2(k),Ys1(k),Ys2(k),Zs1(k),Zs2(k));
     X=X+K*(Z-Zpre);   
     P=(eye(9)-K*H)*P;
     Xekf(k,:)=X;
end
for k=1:N
   error_x(:,k)=Xekf(k,1)-target(k,2);
   error_y(:,k)=Xekf(k,2)-target(k,3);
   error_z(:,k)=Xekf(k,3)-target(k,4);
   error_vx(:,k)=Xekf(k,4)-target(k,5);
   error_vy(:,k)=Xekf(k,5)-target(k,6);
   error_vz(:,k)=Xekf(k,6)-target(k,7);
   error_ax(:,k)=Xekf(k,7)-target(k,8);
   error_ay(:,k)=Xekf(k,8)-target(k,9);
   error_az(:,k)=Xekf(k,9)-target(k,10);
end
t=1:N;
subplot(3,1,1);
plot(t,error_x);
title('x轴位置EKF结果与目标值误差');
xlabel('时间(s)');ylabel('误差(m)');
grid on
subplot(3,1,2);
plot(t,error_y);
title('y轴位置EKF结果与目标值误差');
xlabel('时间(s)');ylabel('误差(m)');
grid on
subplot(3,1,3);
plot(t,error_z);
title('z轴位置EKF结果与目标值误差');
xlabel('时间(s)');ylabel('误差(m)');
grid on
% % % % % % % % % % % 
figure
plot(t,Xekf(:,1),t,target(:,2));
title('x轴运动轨迹');
xlabel('t(s)');ylabel('x(m)');
legend('滤波结果','目标值');
grid on
figure
plot(t,Xekf(:,2),t,target(:,3));
title('y轴运动轨迹');
xlabel('t(s)');ylabel('y(m)');
legend('滤波结果','目标值');
grid on
figure
plot(t,Xekf(:,3),t,target(:,4));
title('z轴运动轨迹');
xlabel('t(s)');ylabel('z(m)');
legend('滤波结果','目标值');
grid on
% % % % % % % % 
subplot(3,1,1);
plot(t,error_vx);
title('x轴速度EKF结果与目标值误差');
xlabel('时间(s)');ylabel('误差(m/s)');
grid on
subplot(3,1,2);
plot(t,error_vy);
title('y轴速度EKF结果与目标值误差');
xlabel('时间(s)');ylabel('误差(m/s)');
grid on
subplot(3,1,3);
plot(t,error_vz);
title('z轴速度EKF结果与目标值误差');
xlabel('时间(s)');ylabel('误差(m/s)');
grid on
% % % % % % % % % % % % 
figure
plot(t,Xekf(:,4),t,target(:,5));
title('x轴速度变化曲线');
xlabel('t(s)');ylabel('vx（m/s）');
legend('滤波结果','目标值');
grid on
figure
plot(t,Xekf(:,5),t,target(:,6));
title('y轴速度变化曲线');
xlabel('t(s)');ylabel('vy（m/s）');
legend('滤波结果','目标值');
grid on
figure
plot(t,Xekf(:,6),t,target(:,7));
title('z轴速度变化曲线');
xlabel('t(s)');ylabel('vz（m/s）');
legend('滤波结果','目标值');
grid on
% % % % % % % % % % % % 
subplot(3,1,1);
plot(t,error_ax);
title('x轴加速度EKF结果与目标值误差');
xlabel('时间(s)');ylabel('误差(m/s^2)');
grid on
subplot(3,1,2);
plot(t,error_ay);
title('y轴加速度EKF结果与目标值误差');
xlabel('时间(s)');ylabel('误差(m/s^2)');
grid on
subplot(3,1,3);
plot(t,error_az);
title('z轴加速度EKF结果与目标值误差');
xlabel('时间(s)');ylabel('误差(m/s^2)');
grid on
%%%%%%%
figure
plot(t,Xekf(:,7),t,target(:,8));
title('x轴加速度变化曲线');
xlabel('t(s)');ylabel('ax（m/s^2）');
legend('滤波结果','目标值');
grid on
figure
plot(t,Xekf(:,8),t,target(:,9));
title('y轴加速度变化曲线');
xlabel('t(s)');ylabel('ay（m/s^2）');
legend('滤波结果','目标值');
grid on
figure
plot(t,Xekf(:,9),t,target(:,10));
title('z轴加速度变化曲线');
xlabel('t(s)');ylabel('vz（m/s^2）');
legend('滤波结果','目标值');
grid on




