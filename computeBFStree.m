function [parents] = computeBFStree(AdjTable,start)
%computeBFStress takes a graph described by its adjacency table "AdjTable"
%and a start node "start" and gives a vector of pointers "parents"
%describing the BFS tree rooted at start
% [parents] = computeBFStree(AdjTable,start)

% AdjTable{i} = [ui_1,ui_2,...];
% ...
% AdjTable{n} = [un_i, un_i, ...]; 
% start = v_start

n = length(AdjTable);  % number of nodes 
parents(1:n,1) = -1;   % initlly label all nodes as unvisited
parents(start) = start ;  % parent of start is self
queue = [start];  % create queue and enqueue start

% BFS
for v = 1:n  % for each node v in graph G
    while isempty(queue) == 0  % continue all connected nodes are visited
        v = queue(1);  % enqueue start node
        for u = AdjTable{v}
            if parents(u) == -1  % if unvisited
                parents(u) = v;  % assign parent node
                queue = [queue, u];  % enqueue u
            end
        end
        queue(1) = [];  % dequeue parent node from start of queue
    end
end

end

