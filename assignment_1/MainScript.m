clc;
close all;
clear all;

k = 1;

[Start,Goal,step_size,ObsList,ObsNum] = DataRetrievelText();

Robot_State(1,:) = Start;
k = 1;
%CheckingCollision = checkCollision(PolygonList,Start);

while(Euclidean_distance(Robot_State(k,1),Robot_State(k,2),Goal(1),Goal(2))>step_size)

    direction_vector_to_goal = [Goal(1)-Robot_State(k,1) Goal(2)-Robot_State(k,2)];
    unit_tangent_to_goal = direction_vector_to_goal/norm(direction_vector_to_goal);

    Candidate_next_waypoint = Robot_State(k,:) + unit_tangent_to_goal*step_size;
    
    %CollisionFlag = 1;
    count_obs = 1;
    while(count_obs<=ObsNum)

        %Polygon_check = cell2mat(struct2cell(ObsList.Ob(count_obs)));
        Polygon_check = ObsList.Ob(count_obs).SortedVertices;
        CollisionFlag = checkCollision(Polygon_check,Candidate_next_waypoint);
        if(CollisionFlag == 1)
            count_obs = count_obs+1;
            continue;
        else
            PolygonList = Polygon_check;
            break;
        end

    end    
        
    if(CollisionFlag == 1)
        Robot_State(k+1,:) = Candidate_next_waypoint;
        k = k + 1;
        continue;
    else
        
        [threshold,temp1,temp2] = closestPolygonEdgeComputer(PolygonList,Robot_State(k,:));
        temp = 1;
        Distance_To_Goal_min = inf;
        circum_navigate_start_point_x = Robot_State(k,1);
        circum_navigate_start_point_y = Robot_State(k,2);
        circum_navigate_start_edge = temp2;
        prev_temp2 = circum_navigate_start_edge;

        figure(1)
        plot(circum_navigate_start_point_x,circum_navigate_start_point_y,'-o','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6])
        hold on

        temp_goal_x = nan;
        temp_goal_y = nan;

        unit_tangent_circumnavigating = Tangent_Computer(PolygonList,Robot_State(k,:));
        Robot_State(k+1,:) = Robot_State(k,:) + step_size*unit_tangent_circumnavigating;
        k = k+1;

        threshold = step_size;

        update = 0;

        while((Euclidean_distance(Robot_State(k,1),Robot_State(k,2),circum_navigate_start_point_x,circum_navigate_start_point_y) > threshold) || (update~=length(PolygonList(:,1))))

            Distance_To_Goal(temp) = Euclidean_distance(Robot_State(k,1),Robot_State(k,2),Goal(1),Goal(2));
            if(Distance_To_Goal(temp) < Distance_To_Goal_min)
                temp_goal_x = Robot_State(k,1);
                temp_goal_y = Robot_State(k,2);
                Distance_To_Goal_min = Distance_To_Goal(temp);
                [threshold,temp1,temp_goal_edge] = closestPolygonEdgeComputer(PolygonList,Robot_State(k,:));
            end

            unit_tangent_circumnavigating = Tangent_Computer(PolygonList,Robot_State(k,:));
            Robot_State(k+1,:) = Robot_State(k,:) + step_size*unit_tangent_circumnavigating;
        
            temp = temp+1; 
            k = k+1;
            
            [threshold,temp1,temp2] = closestPolygonEdgeComputer(PolygonList,Robot_State(k,:));

            if(temp2 ~= prev_temp2)
                update = update+1;
                prev_temp2 = temp2;
            end

        end

        while(Euclidean_distance(Robot_State(k,1),Robot_State(k,2),temp_goal_x,temp_goal_y) > threshold || (temp2~=temp_goal_edge))           
             
            unit_tangent_circumnavigating = Tangent_Computer(PolygonList,Robot_State(k,:));
            Robot_State(k+1,:) = Robot_State(k,:) + step_size*unit_tangent_circumnavigating;
            k = k+1;
            [threshold,temp1,temp2] = closestPolygonEdgeComputer(PolygonList,Robot_State(k,:));
            
        end
    end
end

figure(1)
%plot(Robot_State(:,1),Robot_State(:,2),'LineWidth',2)
%hold on;
for i = 1:1:ObsNum
    
    PolygonList = ObsList.Ob(i).Vertices;
    Polygon_Plot = polyshape([PolygonList(:,1)],[PolygonList(:,2)]);
    plot(Polygon_Plot);
    hold on;
end
plot(Start(1),Start(2),'-o','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6])
hold on;
plot(Goal(1),Goal(2),'-o','MarkerSize',10,'MarkerEdgeColor','green','MarkerFaceColor',[.6 1 .6])
grid on;
%an = animatedline(Robot_State(:,1),Robot_State(:,2));
an = animatedline([0 0],[0 0],'Color','r','LineWidth',3);
%figure(2)

h = plot(nan, nan, 'ko', 'MarkerSize', 7, 'MarkerFaceColor','k');   %yellow, filled, large
for i = 1:1:length(Robot_State(:,1))
    
    addpoints(an,Robot_State(i,1),Robot_State(i,2));
    set(h, 'XData', Robot_State(i,1), 'YData', Robot_State(i,2));

    drawnow
end
%comet(Robot_State(:,1),Robot_State(:,2))

%% Writing to an output text file

fileID = fopen('output.txt','w');
formatSpec = '%f,%f\n';
for i = 1:1:length(Robot_State(:,1))
    fprintf(fileID,formatSpec,Robot_State(i,1),Robot_State(i,2));
end
