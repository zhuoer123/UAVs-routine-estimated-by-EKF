clc;clear
UAV1=load('uav1.txt');
UAV2=load('uav2.txt');
plot3(UAV1(:,2),UAV1(:,3),UAV1(:,4));
hold on
plot3(UAV2(:,2),UAV2(:,3),UAV2(:,4));
legend('无人机1','无人机2');
title('无人机轨迹');
xlabel('x轴/m');ylabel('y轴/m');zlabel('z轴/m');
grid on

