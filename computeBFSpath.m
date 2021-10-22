%% code was contributed by Haley Ekstrom, Class of 2021, MAE %%%%%%%%%%%%%
function [path] = computeBFSpath(AdjTable,start,goal)
% computeBFSpath takes a graph described by its adjacenecy table, a start
% node, and a goal node and gives a path from start to goal along the BFS tree rooted at start
% [path] = computeBFSpath(AdjTable,start,goal)

% AdjTable{i} = [ui_1,ui_2,...];
% ...
% AdjTable{n} = [un_i, un_i, ...]; 

% start = v_start;
% goal = v_goal;

[parents] = computeBFStree(AdjTable,start); % retrieve parent vector from computeBFStree

path = [goal]; % create path array and enqueue goal node
u = goal; % begin with goal node

while parents(u) ~= u  % continue until path reaches start node 
    u = parents(u);
    path = [u, path];  % enqueue node along path at front of array
    if parents(goal) == -1  % if start and goal node are disconnected
        error('Path between start and goal node does not exist.')
    end
end

end

