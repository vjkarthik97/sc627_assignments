function [Start,Goal,step_size,ObsList,ObsNum] = DataRetrievelText()

struct ObsList Ob;

fullFileName = 'input.txt';
fileID = fopen(fullFileName, 'rt');
% Read the first line of the file.
%textLine = fgetl(fileID);
%A = sscanf(textLine,'%f,%f');
%textLine = 'random_initialization';

%lineCounter = 1;
count = 1;
set = 1;
set_count = 1;

while feof(fileID) == 0
	% Print out what line we're operating on.
	%fprintf('%s\n', textLine);
    
	% Read the line.
	textLine = fgetl(fileID);

        %if length(textLine) == 0
        if isempty(textLine) == 1
            count = 1;
            set = set+1;
            continue;
        end

        %if(feof(fileID) == 1)
            %break;
        %end
                     
        A = sscanf(textLine,'%f,%f');

        if(set == 1)
            if(set_count == 1)
                Start = [A(1) A(2)];              
            end
            if(set_count == 2)
                Goal = [A(1) A(2)];               
            end
            if(set_count == 3)
                step_size = A;
                %set_count = set_count+1;
            end
            set_count = set_count+1;
        else
            ObsList.Ob(set-1).Vertices(count,:) = [A(1) A(2)];
            count = count+1;
        end
	%lineCounter = lineCounter + 1;
end

ObsNum = set - 1;

for i = 1:1:ObsNum
    ObsList.Ob(i).SortedVertices = VerticesSorter(ObsList.Ob(i).Vertices);
    %ObsList.Ob(i).Vertices
end
%Start = M(1,:);
%Goal = M(2,:);

%fileID = fopen('input.txt','r');
%formatSpec = '%d,%d \n';
%A=textscan(fileID,formatSpec);


end