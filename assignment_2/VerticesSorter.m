function sorted_vertices = VerticesSorter(VerticesList)

angle_verticeslist = zeros(1,length(VerticesList(:,1)));
temp = 0;
temp_angle = 0;
centroid_x = sum(VerticesList(:,1))/length(VerticesList(:,1));
centroid_y = sum(VerticesList(:,2))/length(VerticesList(:,1));

for i = 1:1:length(VerticesList(:,1))
    
    VerticesList(i,:) = VerticesList(i,:) - [centroid_x centroid_y];
    angle_verticeslist(i) = atan2(VerticesList(i,2),VerticesList(i,1));

end

%angle_verticeslist = ComputeAnglesToVertices(VerticesList);

for i = 1:1:length(VerticesList(:,1))-1
    for j = 1:1:length(VerticesList(:,1))-1

        if(angle_verticeslist(j)>angle_verticeslist(j+1))
            temp = VerticesList(j,:); 
            VerticesList(j,:) = VerticesList(j+1,:);
            VerticesList(j+1,:) = temp;

            temp_angle = angle_verticeslist(j); 
            angle_verticeslist(j) = angle_verticeslist(j+1);
            angle_verticeslist(j+1) = temp_angle;
            
        end
    end
end

 for i = 1:1:length(VerticesList(:,1))
    
    VerticesList(i,:) = VerticesList(i,:) + [centroid_x centroid_y];
    %angle_verticeslist(i) = atan2(VerticesList(i,2),VerticesList(i,1));

 end

 sorted_vertices = VerticesList
end