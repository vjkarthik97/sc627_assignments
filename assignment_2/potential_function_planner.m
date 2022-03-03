clc;
close all;
clear all;

[Start,Goal,step_size,ObsList,ObsNum] = DataRetrievelText();

syms x y Att_Potential_close Att_Potential_far Rep_Potential_overall
syms Rep_Potential [1 ObsNum]


Robot_State(1,:) = Start;
k = 1;

chi = 0.8;
d_star = 2;
eta = 0.8;
Q_star = 2;


Att_Potential_close = 0.5*chi*(Euclidean_distance(x,y,Goal(1),Goal(2)))^2;
Att_Potential_far = d_star*chi*Euclidean_distance(x,y,Goal(1),Goal(2)) - 0.5*chi*(d_star)^2;
%Rep_Potential = 0;
%for i = 1:1:ObsNum

    %[d_obs_temp,temp1,temp2] = closestPolygonEdgeComputer(ObsList.Ob(i).SortedVertices,[x y]);
    %Rep_Potential = Rep_Potential + 0.5*eta*((1/(d_obs_temp^2)) - (1/(Q_star^2))); 
%end

grad_Att_close = gradient(Att_Potential_close,[x y]);
grad_Att_far = gradient(Att_Potential_far,[x y]);
%grad_Rep = gradient(Rep_Potential,[x y]);

while(Euclidean_distance(Robot_State(k,1),Robot_State(k,2),Goal(1),Goal(2))>step_size)
    
    %Rep_Potential = sym(Rep_Potential,'clear');
    syms x y

    for i = 1:1:ObsNum

        [d_obs_temp,lambda_star,edge_index] = closestPolygonEdgeComputer(ObsList.Ob(i).SortedVertices,[Robot_State(k,1) Robot_State(k,2)]);
    
        if(edge_index == length(ObsList.Ob(i).SortedVertices))
            
            P_closest = (1-lambda_star)*ObsList.Ob(i).SortedVertices(edge_index,:) + lambda_star*ObsList.Ob(i).SortedVertices(1,:) ;
        
            closest_distance = Euclidean_distance(x,y,P_closest(1),P_closest(2));

        else
            
            P_closest = (1-lambda_star)*ObsList.Ob(i).SortedVertices(edge_index,:) + lambda_star*ObsList.Ob(i).SortedVertices(edge_index+1,:) ;

            closest_distance = Euclidean_distance(x,y,P_closest(1),P_closest(2));
        
        end

        if(d_obs_temp<=Q_star)
            Rep_Potential(i) = 0.5*eta*((1/closest_distance) - (1/Q_star))^2; 
        else
            Rep_Potential(i) = 0;
            %continue;
        end
    end

    Rep_Potential_overall = Rep_Potential(1);

    if(ObsNum>1)
        
        for j = 2:1:ObsNum
            Rep_Potential_overall = Rep_Potential_overall + Rep_Potential(j);
        end
    end
    
    grad_Rep = gradient(Rep_Potential_overall,[x y]);

    x = Robot_State(k,1);
    y = Robot_State(k,2);

    if(Euclidean_distance(Robot_State(k,1),Robot_State(k,2),Goal(1),Goal(2))<d_star)
        v = -double(subs(grad_Att_close) + subs(grad_Rep));
    else
        v = -double(subs(grad_Att_far) + subs(grad_Rep));
    end

    Robot_State(k+1,:) = Robot_State(k,:) + transpose(v)*step_size; 
    k = k+1;
    
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
