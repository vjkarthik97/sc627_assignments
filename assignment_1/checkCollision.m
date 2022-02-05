function CollisionFlag = checkCollision(PolygonList,Q)

i = 1;
CollisionFlag = 0;

N = length(PolygonList(:,1));
while(i<=N)

    if(i<N)
        P1 = PolygonList(i,:);
        P2 = PolygonList(i+1,:);
    else
        P1 = PolygonList(N,:);
        P2 = PolygonList(1,:);
    end

    V1 = [P2-P1 0];
    V2 = [Q-P1 0];

    cross_product = cross(V1,V2);
    sign_cross = sign(dot(cross_product,[0 0 1]));

    if(i==1)
        prev_sign_cross = sign_cross;
        i = i+1;
    else
        if((sign_cross*prev_sign_cross) == 1)
            i = i+1;
        else
            CollisionFlag = 1;
            break;
        end
    end

end


end
