clc;clear
UAV1=load('uav1.txt');
UAV2=load('uav2.txt');
plot3(UAV1(:,2),UAV1(:,3),UAV1(:,4));
hold on
plot3(UAV2(:,2),UAV2(:,3),UAV2(:,4));
legend('���˻�1','���˻�2');
title('���˻��켣');
xlabel('x��/m');ylabel('y��/m');zlabel('z��/m');
grid on

