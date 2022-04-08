clc;
close all;
clear all;

N_samples = 500;
del_T = 0.05;

a = 0;
b = 14;
r = a + (b-a).*rand(6,1);
r = sort(r);

D_out = [0 0 0 0 0 0 0 0;
         0 2 0 0 0 0 0 0;
         0 0 2 0 0 0 0 0;
         0 0 0 2 0 0 0 0;
         0 0 0 0 2 0 0 0;
         0 0 0 0 0 2 0 0;
         0 0 0 0 0 0 2 0;
         0 0 0 0 0 0 0 0];

A = [0 0 0 0 0 0 0 0;
     1 0 1 0 0 0 0 0;
     0 1 0 1 0 0 0 0;
     0 0 1 0 1 0 0 0;
     0 0 0 1 0 1 0 0;
     0 0 0 0 1 0 1 0;
     0 0 0 0 0 1 0 1;
     0 0 0 0 0 0 0 0];

X(:,1) = [0;r;14];

for i = 1:1:N_samples
    i
    X(:,i+1) = X(:,i) - 1*(D_out - A)*X(:,i)*del_T;
    
    figure(1)
    scatter(X(1,i),0,'filled','g');
    hold on;
    scatter(X(2,i),0,'filled','g');
    hold on;
    scatter(X(3,i),0,'filled','g');
    hold on;
    scatter(X(4,i),0,'filled','g');
    hold on;
    scatter(X(5,i),0,'filled','g');
    hold on;
    scatter(X(6,i),0,'filled','g');
    hold on;
    scatter(X(7,i),0,'filled','g');
    hold on;
    scatter(X(8,i),0,'filled','g');
    hold off;
    grid on;
    

end

figure(2)
plot(X(1,:),'LineWidth',2)
title('Robot 1')
ylabel('Robot 1 - X')
xlabel('Time(x0.05) seconds')
grid on;

figure(3)
plot(X(2,:),'LineWidth',2)
title('Robot 2')
ylabel('Robot 2 - X')
xlabel('Time(x0.05) seconds')
grid on;

figure(4)
plot(X(3,:),'LineWidth',2)
title('Robot 3')
ylabel('Robot 3 - X')
xlabel('Time(x0.05) seconds')
grid on;

figure(5)
plot(X(4,:),'LineWidth',2)
title('Robot 4')
ylabel('Robot 4 - X')
xlabel('Time(x0.05) seconds')
grid on;

figure(6)
plot(X(5,:),'LineWidth',2)
title('Robot 5')
ylabel('Robot 5 - X')
xlabel('Time(x0.05) seconds')
grid on;

figure(7)
plot(X(6,:),'LineWidth',2)
title('Robot 6')
ylabel('Robot 6 - X')
xlabel('Time(x0.05) seconds')
grid on;

figure(8)
plot(X(7,:),'LineWidth',2)
title('Robot 7')
ylabel('Robot 7 - X')
xlabel('Time(x0.05) seconds')
grid on;

figure(9)
plot(X(8,:),'LineWidth',2)
title('Robot 8')
ylabel('Robot 8 - X')
xlabel('Time(x0.05) seconds')
grid on;



