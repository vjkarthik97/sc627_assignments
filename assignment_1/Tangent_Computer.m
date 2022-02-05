function unit_tangent_vector = Tangent_Computer(PolygonList,Q)

N = length(PolygonList(:,1));

[closest_distance,lambda,edge_index] = closestPolygonEdgeComputer(PolygonList,Q);

if((lambda>0) && (lambda <1))
    if(edge_index<N)
        P1 = PolygonList(edge_index,:);
        P2 = PolygonList(edge_index+1,:);
    else
        P1 = PolygonList(N,:);
        P2 = PolygonList(1,:);
    end
    
    unit_tangent_vector = (P2-P1)/norm(P2-P1);
else
    if(lambda == 1)
        if(edge_index<N)
        point = PolygonList(edge_index+1,:);
        else
        point = PolygonList(1,:);
        end
    end

    if(lambda == 0)
        point = PolygonList(edge_index,:);
    end

        Radial_Vector = point - Q;
        
        Rotation_vector = [0 1;-1 0];

        tangent_vector = Rotation_vector*transpose(Radial_Vector);

        unit_tangent_vector = 1*transpose(tangent_vector)/norm(tangent_vector);
end