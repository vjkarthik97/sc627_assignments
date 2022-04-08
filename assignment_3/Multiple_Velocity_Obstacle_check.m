clc;
close all;
clear all;


del_T = 0.1;
%% Environment Definitions 
Obs_R = 0.15/2;
Robot_R = 0.15/2;

Obstacles(1,:) = [2 -1 0 0.1 Obs_R+Robot_R]; %[x y V_x V_y R]
Obstacles(2,:) = [3 1 0 0.1 Obs_R+Robot_R]; %[x y V_x V_y R]
Obstacles(3,:) = [4 0 0 0 Obs_R+Robot_R]; %[x y V_x V_y R]

V_obs_1 = [Obstacles(1,3) Obstacles(1,4)];
V_obs_2 = [Obstacles(2,3) Obstacles(2,4)];
V_obs_3 = [Obstacles(3,3) Obstacles(3,4)];

Obstacle_1_State(1,:) = Obstacles(1,1:2);
Obstacle_2_State(1,:) = Obstacles(2,1:2);
Obstacle_3_State(1,:) = Obstacles(3,1:2);

Goal = [5 0];
%Robot_R = 0.15;

Robot_State(1,:) = [0 0 0];
v_robot(1) = 0.1;

Robot_x = Robot_State(1,1);
Robot_y = Robot_State(1,2);

norm([Robot_x Robot_y] - Goal)

k = 1;

while (norm([Robot_State(k,1) Robot_State(k,2)] - Goal)>0.1)


    can_theta = linspace(-pi/18,pi/18,40);
    can_mag_V = linspace(0,0.1,10);
    %can_mag_V = [0.1]
    best_value = -inf;
    best_robot_velocity = [0 0];
    for i = 1:1:length(can_theta)
        for j = 1:1:length(can_mag_V)
        
            %V_robot_cartesian = [v_robot(k)*cos(Robot_State(k,3)) v_robot(k)*sin(Robot_State(k,3))];
            
            candidate_V_robot = [can_mag_V(j)*cos(Robot_State(k,3) + can_theta(i)) can_mag_V(j)*sin(Robot_State(k,3) + can_theta(i))];
            
            collisionFlag_1 = Velocity_Obstacle_Collision_check(Obstacle_1_State(k,1),Obstacle_1_State(k,2),V_obs_1,Obstacles(1,5),Robot_State(k,1),Robot_State(k,2),candidate_V_robot);
            collisionFlag_2 = Velocity_Obstacle_Collision_check(Obstacle_2_State(k,1),Obstacle_2_State(k,2),V_obs_2,Obstacles(2,5),Robot_State(k,1),Robot_State(k,2),candidate_V_robot);
            collisionFlag_3 = Velocity_Obstacle_Collision_check(Obstacle_3_State(k,1),Obstacle_3_State(k,2),V_obs_3,Obstacles(3,5),Robot_State(k,1),Robot_State(k,2),candidate_V_robot);

            if(collisionFlag_1 == 1 || collisionFlag_2 == 1 || collisionFlag_3 == 1)
                continue;
            else
                Value = dot([-Robot_State(k,1)+Goal(1) -Robot_State(k,2)+Goal(2)], candidate_V_robot);
                if(Value>best_value)
                    best_value = Value;
                    best_robot_velocity = candidate_V_robot;
                end
            end

        end
    end

            Robot_State(k+1,1) = Robot_State(k,1) + best_robot_velocity(1)*del_T;
            Robot_State(k+1,2) = Robot_State(k,2) + best_robot_velocity(2)*del_T;
            Robot_State(k+1,3) = atan2(best_robot_velocity(2),best_robot_velocity(1));

            Obstacle_1_State(k+1,1) = Obstacle_1_State(k,1) + V_obs_1(1)*del_T;
            Obstacle_1_State(k+1,2) = Obstacle_1_State(k,2) + V_obs_1(2)*del_T;
     
            if(Obstacle_1_State(k+1,2) > 1 || Obstacle_1_State(k+1,2) < -1)
                V_obs_1 = -V_obs_1;
            end

            Obstacle_2_State(k+1,1) = Obstacle_2_State(k,1) + V_obs_2(1)*del_T;
            Obstacle_2_State(k+1,2) = Obstacle_2_State(k,2) + V_obs_2(2)*del_T;

            if(Obstacle_2_State(k+1,2) > 1 || Obstacle_2_State(k+1,2) < -1)
                V_obs_2 = -V_obs_2;
            end

            Obstacle_3_State(k+1,1) = Obstacle_3_State(k,1) + V_obs_3(1)*del_T;
            Obstacle_3_State(k+1,2) = Obstacle_3_State(k,2) + V_obs_3(2)*del_T;

            if(Obstacle_3_State(k+1,2) > 1 || Obstacle_3_State(k+1,2) < -1)
                V_obs_3 = -V_obs_3;
            end


                
            k = k +1;

            figure(1)
            scatter(Robot_State(k,1),Robot_State(k,2),'filled','g');
            hold on;

            scatter(Obstacle_1_State(k,1),Obstacle_1_State(k,2),'filled','r');
            hold on;
            viscircles([Obstacle_1_State(k,1) Obstacle_1_State(k,2)],Obs_R,'Color','r','LineWidth',0.5);
            hold on;

            scatter(Obstacle_2_State(k,1),Obstacle_2_State(k,2),'filled','r');
            hold on;
            viscircles([Obstacle_2_State(k,1) Obstacle_2_State(k,2)],Obs_R,'Color','r','LineWidth',0.5);
            hold on;

            scatter(Obstacle_3_State(k,1),Obstacle_3_State(k,2),'filled','r');
            hold on;
            viscircles([Obstacle_3_State(k,1) Obstacle_3_State(k,2)],Obs_R,'Color','r','LineWidth',0.5);
            hold on;

            viscircles([Robot_State(k,1) Robot_State(k,2)],Robot_R,'Color','g','LineWidth',0.5);
            hold on;
            quiver(Robot_State(k,1),Robot_State(k,2),3*best_robot_velocity(1),3*best_robot_velocity(2))
            hold off;
            grid on;
            xlim([0 5])
            ylim([-2.5 2.5])
end

%% Plotting Robot Trajectory

figure(1)
plot(Robot_State(:,1),Robot_State(:,2),'LineWidth',2)
xlim([-5 5])
ylim([-5 5])
grid on;

figure(2)
plot(Robot_State(:,1))
hold on;
plot(Obstacle_1_State(:,1))
hold on;
plot(Obstacle_2_State(:,1))
hold on;
plot(Obstacle_3_State(:,1))
ylabel('X')
xlabel('Time (x0.1) seconds')
grid on;
legend('Robot','Obstacle 1','Obstacle 2','Obstacle 3')

figure(3)
plot(Robot_State(:,2))
hold on;
plot(Obstacle_1_State(:,2))
hold on;
plot(Obstacle_2_State(:,2))
hold on;
plot(Obstacle_3_State(:,2))
ylabel('Y')
xlabel('Time (x0.1) seconds')
grid on;
legend('Robot','Obstacle 1','Obstacle 2','Obstacle 3')