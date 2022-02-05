function [closest_distance,flag,edge_index] = closestPolygonEdgeComputer(PolygonList,Q)

    N = length(PolygonList(:,1));
    i = 1;
    distance = inf;

    while(i<=N)
    
        if(i<N)
            P1 = PolygonList(i,:);
            P2 = PolygonList(i+1,:);
        else
            P1 = PolygonList(N,:);
            P2 = PolygonList(1,:);
        end
        
        lambda_star = ((P2(1)-P1(1))*(Q(1) - P1(1)) + (P2(2)-P1(2))*(Q(2) - P1(2)))/(norm(P2-P1))^2;
        %lambda_star = ((P2(1)-P1(1))*(Q(1) - P1(1)) + (P2(2)-P1(2))*(Q(2) - P1(2)))/((P2(1)-P1(1))*(P2(1) - P1(1)) + (P2(2)-P1(2))*(P2(2) - P1(2)));
        
        if(lambda_star>1)
            lambda_star = 1;
        end
        if(lambda_star<0)
            lambda_star = 0;
        end

        P_closest = (1-lambda_star)*P1 + lambda_star*P2;

        closest_distance = norm(Q-P_closest);
        
        if(closest_distance<distance)
            edge_index = i;
            flag = lambda_star;
            distance = closest_distance;
        end
        i = i+1;
    end

end