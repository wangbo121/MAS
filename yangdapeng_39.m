clear;
close all;
clc;

A = [ -2 2;
      -1 1];
B = [1;0];
C = [1 0];
F = [1;-1];
K = [-1 2];

L = [
    3      0    0    -1    -1    -1 ;
    -1     1    0    0     0     0;
    -1    -1    2    0    0    0;
    -1     0    0    1    0    0;
    0        0    0    -1    1    0;
    0        0    0    0    -1    1;
    ];

zeor_2d = zeros(2,2);%二维0矩阵
A_big = [A,B*K;zeor_2d,A+B*K];
H_big = [zeor_2d,zeor_2d;-F*C,F*C];

% delta_time = 1;
delta_time = 0.01;% 0.01 表示每次迭代一次的时间间隔是0.01秒钟，也就是控制频率是100hz，这个频率在实际无人机等控制中是可行而且经常用的
I_2 = eye(2);

total_time_s = 100;%仿真的时间长度，100秒
loop_tick = total_time_s/delta_time;




ksi1 = [0.4 0.3 0 0]';
ksi2 = [0.5 0.2 0 0]';
ksi3 = [0.6 0.1 0 0]';
ksi4 = [0.7 0.0 0 0]';
ksi5 = [0.8 -0.1 0 0]';
ksi6 = [0.4 -0.2 0 0]';

c_small = 0.2;
% c_small = 1.1;

H = H_big;
for i = 1:1:loop_tick
    
    ksi1_matrix(:,i) = ksi1;
    ksi1_next = (  A_big * ksi1 + c_small * (   L(1,1)*H*ksi1 + L(1,2)*H*ksi2 + L(1,3)*H*ksi3 + L(1,4)*H*ksi4 + L(1,5)*H*ksi5 + L(1,6)*H*ksi6   )  ) * delta_time + ksi1;
    
    ksi2_matrix(:,i) = ksi2;
    ksi2_next = (  A_big * ksi2 + c_small * (   L(2,1)*H*ksi1 + L(2,2)*H*ksi2 + L(2,3)*H*ksi3 + L(2,4)*H*ksi4 + L(2,5)*H*ksi5 + L(2,6)*H*ksi6   )  ) * delta_time + ksi2;
    
    ksi3_matrix(:,i) = ksi3;
    ksi3_next = (  A_big * ksi3 + c_small * (   L(3,1)*H*ksi1 + L(3,2)*H*ksi2 + L(3,3)*H*ksi3 + L(3,4)*H*ksi4 + L(3,5)*H*ksi5 + L(3,6)*H*ksi6   )  ) * delta_time + ksi3;
    
    ksi4_matrix(:,i) = ksi4;
    ksi4_next = (  A_big * ksi4 + c_small * (   L(4,1)*H*ksi1 + L(4,2)*H*ksi2 + L(4,3)*H*ksi3 + L(4,4)*H*ksi4 + L(4,5)*H*ksi5 + L(4,6)*H*ksi6   )  ) * delta_time + ksi4;
    
    ksi5_matrix(:,i) = ksi5;
    ksi5_next = (  A_big * ksi5 + c_small * (   L(5,1)*H*ksi1 + L(5,2)*H*ksi2 + L(5,3)*H*ksi3 + L(5,4)*H*ksi4 + L(5,5)*H*ksi5 + L(5,6)*H*ksi6   )  ) * delta_time + ksi5;
    
    ksi6_matrix(:,i) = ksi6;
    ksi6_next = (  A_big * ksi6 + c_small * (   L(6,1)*H*ksi1 + L(6,2)*H*ksi2 + L(6,3)*H*ksi3 + L(6,4)*H*ksi4 + L(6,5)*H*ksi5 + L(6,6)*H*ksi6   )  ) * delta_time + ksi6;
    
    ksi1 = ksi1_next;
    ksi2 = ksi2_next;
    ksi3 = ksi3_next;
    ksi4 = ksi4_next;
    ksi5 = ksi5_next;
    ksi6 = ksi6_next;
    
end

time = 1:1:loop_tick;

figure;
plot(time,ksi1_matrix(1,:),'r',time,ksi2_matrix(1,:),'g',time,ksi3_matrix(1,:),'b',time,ksi4_matrix(1,:),'c',time,ksi5_matrix(1,:),'m',time,ksi6_matrix(1,:),'k');

figure;
plot(time,ksi1_matrix(2,:),'r',time,ksi2_matrix(2,:),'g',time,ksi3_matrix(2,:),'b',time,ksi4_matrix(2,:),'c',time,ksi5_matrix(2,:),'m',time,ksi6_matrix(2,:),'k');

figure;
plot(time,ksi1_matrix(3,:),'r',time,ksi2_matrix(3,:),'g',time,ksi3_matrix(3,:),'b',time,ksi4_matrix(3,:),'c',time,ksi5_matrix(3,:),'m',time,ksi6_matrix(3,:),'k');

figure;
plot(time,ksi1_matrix(4,:),'r',time,ksi2_matrix(4,:),'g',time,ksi3_matrix(4,:),'b',time,ksi4_matrix(4,:),'c',time,ksi5_matrix(4,:),'m',time,ksi6_matrix(4,:),'k');


%虽然仿真结果不错，但是只是说明了这个公式能够使得系统收敛，控制u=k*v，但是如果u是输入受限制的呢，控制肯定不可能是无穷的呀

for i = 1:1:loop_tick
    
    %temp = [ ksi1_matrix(3,i) ksi1_matrix(4,i)];
    temp = [ ksi1_matrix(3,i);ksi1_matrix(4,i)];
    u1(i) = K * temp; %20171116 切记matlab默认是行向量，所以u1是个行向量
    
    temp = [ ksi2_matrix(3,i);ksi2_matrix(4,i)];
    u2(i) = K * temp; %20171116 切记matlab默认是行向量，所以u1是个行向量
    
    temp = [ ksi3_matrix(3,i);ksi3_matrix(4,i)];
    u3(i) = K * temp; %20171116 切记matlab默认是行向量，所以u1是个行向量
    
    temp = [ ksi4_matrix(3,i);ksi4_matrix(4,i)];
    u4(i) = K * temp; %20171116 切记matlab默认是行向量，所以u1是个行向量
    
    temp = [ ksi5_matrix(3,i);ksi5_matrix(4,i)];
    u5(i) = K * temp; %20171116 切记matlab默认是行向量，所以u1是个行向量
    
    temp = [ ksi6_matrix(3,i);ksi6_matrix(4,i)];
    u6(i) = K * temp; %20171116 切记matlab默认是行向量，所以u1是个行向量
end

figure;
plot(time,u1,'r',time,u2,'g',time,u3,'b',time,u4,'c',time,u5,'m',time,u6,'k');











































