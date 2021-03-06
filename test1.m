clear;
close all;
clc;


for i = 2:2:12
    ksi(i) = 0.2 * (i/2);
end


for i = 1:2:11
    ksi(i) = 0.2 * ((i+1)/2) - 0.1;
end

ksi=ksi';


test_matrix = [
               -1.5    1.5    0    0    0    0 ;
               2        -2    0    0    0    0;
               0.9     0      -2.8 0    1.9    0;
               0        1.2    0    -2.5    0    1.3;
               0        0    1.4    1.8    -3.2    0;
               0        0    0    0    0.7    -0.7;
              ];
Ln = - test_matrix;

% delta_time = 1;
delta_time = 0.01;% 0.01 表示每次迭代一次的时间间隔是0.01秒钟，也就是控制频率是100hz，这个频率在实际无人机等控制中是可行而且经常用的
I_2 = eye(2);

% kron(Ln,I_2);
kron(Ln,I_2)
total_time_s = 100;%仿真的时间长度，100秒
loop_tick = total_time_s/delta_time;
for i = 1:1:loop_tick
    ksi_matrix(:,i) = ksi;
    ksi = ( - kron(Ln,I_2) * ksi ) * delta_time + ksi;
end


time = 1:1:loop_tick;

figure;
plot(time,ksi_matrix(2,:),'r',time,ksi_matrix(4,:),'g',time,ksi_matrix(6,:),'b',time,ksi_matrix(8,:),'c',time,ksi_matrix(10,:),'m',time,ksi_matrix(12,:),'k');
% plot(time,ksi_matrix(2,:),'r',time,ksi_matrix(4,:),'g',time,ksi_matrix(6,:),'b');

figure;
plot(time,ksi_matrix(1,:),'r',time,ksi_matrix(3,:),'g',time,ksi_matrix(5,:),'b');