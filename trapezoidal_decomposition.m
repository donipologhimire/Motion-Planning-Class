clc;clf
clear all
close all

% workspace: polygon with polygonal holes
obs_x = [0, 0, 200, 200, NaN, 85, 100, 80, 65, NaN, 35, 60, 55, 30, NaN, 125, 150, 130, 105, NaN, 122, 110, 50, 120, 180, 165, 129, 90];
obs_y = [0, 200, 200, 0, NaN, 15, 40, 60, 35, NaN, 70, 75, 105, 100, NaN, 95, 120, 145, 120, NaN, 160, 180, 130, 40, 90, 110, 80, 130];

% plot workspace
figure
mapshow(obs_x,obs_y,'DisplayType', 'polygon', 'FaceColor', 'white')
hold on
% start = [188, 160];
start = [20, 20];
% goal = [110, 135];
goal = [188,160];

% Possible errors:
for i = 1:length(obs_x)
    if start(1) == obs_x(i) 
        error('Error: The x-coordinate of the start point is equal to that of one of the obstacles in the workspace. Please enter a different value.')
    elseif goal(1) == obs_x(i)
        error('Error: The x-coordinate of the goal point is equal to that of one of the obstacles in the workspace. Please enter a different value.')
    end
end
[in_s,on_s] = inpolygon(start(1),start(2),obs_x,obs_y);
[in_g,on_g] = inpolygon(goal(1),goal(2),obs_x,obs_y);
if ~in_s && ~on_s 
    error('Error: The start point is on an obstacle or outside of the workspace. Please enter a different point.' )
elseif ~in_g && ~on_g
    error('Error: The goal point is on an obstacle or outside of the workspace. Please enter a different point.' )
end
if start(1) > 200
    error('Error: The start point outside of the workspace. Please enter a different point.' )
elseif goal(1) > 200
    error('Error: The goal point outside of the workspace. Please enter a different point.' )
end
    

set(gca, 'Color', 'k')
plot([start(1), goal(1)], [start(2), goal(2)], "r.", 'MarkerSize', 12)
text(start(1),start(2),'Start','Color','g')
text(goal(1),goal(2),'Goal','Color','g')
xlim([0 200])
ylim([0 200])

% workspace
wksp = [];
wksp(:,1) = obs_x(1:4);
wksp(:,2) = obs_y(1:4);

% obstacle list
obs_1 = [];
obs_1(:,1) = obs_x(6:9);
obs_1(:,2) = obs_y(6:9);

obs_2 = [];
obs_2(:,1) = obs_x(11:14);
obs_2(:,2) = obs_y(11:14);

obs_3 = [];
obs_3(:,1) = obs_x(16:19);
obs_3(:,2) = obs_y(16:19);

obs_4 = [];
obs_4(:,1) = obs_x(21:28);
obs_4(:,2) = obs_y(21:28);

obs_x_list = [obs_1(:,1); obs_2(:,1); obs_3(:,1); obs_4(:,1)];
obs_y_list = [obs_1(:,2); obs_2(:,2); obs_3(:,2); obs_4(:,2)];

% order all obstacle vertices in increasing x coordinate
[obs_x_sorted, obs_x_order] = sort(obs_x_list);
obs_y_sorted = obs_y_list(obs_x_order);
v = [];
v(:,1) = obs_x_sorted;
v(:,2) = obs_y_sorted;

% initialize segments
s_w = [];
s_1 = [];
s_2 = [];
s_3 = [];
s_4 = [];

% create segment arrays
for w = 1:length(wksp)
    if w == length(wksp)
        s_w(1:4, w) = [wksp(w,1); wksp(w,2); wksp(1,1); wksp(1,2)];
    else
        s_w(1:4, w) = [wksp(w,1); wksp(w,2); wksp(w+1,1); wksp(w+1,2)];
    end
end
for i = 1:length(obs_1)
    if i == length(obs_1)
        s_1(1:4, i) = [obs_1(i,1); obs_1(i,2); obs_1(1,1); obs_1(1,2)];
    else
        s_1(1:4, i) = [obs_1(i,1); obs_1(i,2); obs_1(i+1,1); obs_1(i+1,2)];
    end
end
for j = 1:length(obs_2)
    if j == length(obs_2)
        s_2(1:4, j) = [obs_2(j,1); obs_2(j,2); obs_2(1,1); obs_2(1,2)];
    else
        s_2(1:4, j) = [obs_2(j,1); obs_2(j,2); obs_2(j+1,1); obs_2(j+1,2)];
    end
end
for k = 1:length(obs_3)
    if k == length(obs_3)
        s_3(1:4, k) = [obs_3(k,1); obs_3(k,2); obs_3(1,1); obs_3(1,2)];
    else
        s_3(1:4, k) = [obs_3(k,1); obs_3(k,2); obs_3(k+1,1); obs_3(k+1,2)];
    end
end
for l = 1:length(obs_4)
    if l == length(obs_4)
        s_4(1:4, l) = [obs_4(l,1); obs_4(l,2); obs_4(1,1); obs_4(1,2)];
    else
        s_4(1:4, l) = [obs_4(l,1); obs_4(l,2); obs_4(l+1,1); obs_4(l+1,2)];
    end
end
seg = [s_w, s_1, s_2, s_3, s_4];

% ordering segments = [left_endpoint; right_endpoint]
for t = 1:length(seg)
    if seg(1,t) > seg(3,t)
        temp1 = seg(1:2,t);
        temp2 = seg(3:4,t);
        seg(1:2,t) = temp2;
        seg(3:4,t) = temp1;
    end
end

% initialize arrays
L_x = [];
T = []; % trapezoids
int_pts = []; % intersection points
midpoints = [];
midpoints_list = [];
centers = []; % trapezoid centers
Trapezoid_list = [];

g = 3; 

% for each vertex
for i = 1:length(v)
    current_v = v(i,:);
    % update list of segments for each vertex
    S_i = [];
    int_pts = [];
    % sweeping line L
    L_p1 = [current_v(1,1) 0];
    L_p2 = [current_v(1,1) 200];
    for n = 1:length(seg)
        % determine which obstacle segments are intersected by L
        [int, p_int] = doTwoSegmentsIntersect(L_p1, L_p2, seg(1:2,n)',seg(3:4,n)');
        if int
            s_n = [seg(1:2,n);seg(3:4,n)];
            % edges intersected by L
            S_i = [S_i s_n];
            % intersection points along L
            int_pts = [int_pts; p_int];
        end
    end
    
    % Determine type of vertex:
    
    % determine pt and pb
    int_pts_y = int_pts(:,2);
    diff = int_pts(:,2) - current_v(2);
    % define pt as closest intersection above v on L
    pt_idx = find(diff == min(diff(diff > 0 & diff ~= 0)));
    if i == length(v)-1
        pt_idx = 1;
    end
    pt_y = unique(int_pts_y(pt_idx));
    pt = [current_v(1) pt_y];
    % define pb as closest intersection below v on L
    pb_idx = find(diff == max(diff(diff < 0 & diff ~= 0)));
    pb_y = unique(int_pts_y(pb_idx));
    pb = [current_v(1) pb_y];
   
    % determine if point directly above v on L is in free workspace
    [in_t, ~] = inpolygon(current_v(1), current_v(2)+1, obs_x, obs_y);
    % determine if point directly below v on L is in free workspace
    [in_b, ~] = inpolygon(current_v(1), current_v(2)-1, obs_x, obs_y);
    
    if in_t && in_b % type i and iii
        midpoints = [pt(1), (((pt(2)-current_v(2))/2)+current_v(2)); pb(1), (current_v(2)-((current_v(2)-pb(2))/2))];
        plot(midpoints(:,1),midpoints(:,2),'.', 'Color', 'k','MarkerSize', 10)
        midpoints_list = [midpoints_list; midpoints(:,1), midpoints(:,2)];
        Si_l = [];
        Si_r = [];
        for q = 1:length(S_i)
            current_vT = current_v';
            if S_i(1:2,q) == current_vT(:)  % if vertex v is the left endpoint of two segments
                Si_l = [Si_l, S_i(1:2,q)];
            elseif S_i(3:4,q) == current_vT(:) % if vertex v is the right endpoint of two segments
                Si_r = [Si_r, S_i(3:4,q)];
            end 
        end
        
        % type i: left/left convex vertex
        if length(Si_l) == 2
            if pt(1) == min(obs_x(obs_x ~= 0))  % first trapezoid
                T(1:2,:) = [pt', [0 200]', [0 0]', pb'];
                polyin = polyshape(T(1,:),T(2,:));
                plot(polyin, 'FaceColor','w')
                Trapezoid_list = [Trapezoid_list;polyin];
                [Tc_x, Tc_y] = centroid(polyin);
                centers = [centers; Tc_x, Tc_y];
                midpoints_list = [midpoints_list;Tc_x,Tc_y];
                scatter(Tc_x,Tc_y,'d','MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w')
            else
                T(g:(g+1),:) = [pt', S_i(1:2,pt_idx), S_i(1:2,pb_idx), pb'];
                polyin = polyshape(T(g,:),T(g+1,:));
                plot(polyin, 'FaceColor','w')
                Trapezoid_list = [Trapezoid_list;polyin];
                [Tc_x, Tc_y] = centroid(polyin);
                centers = [centers; Tc_x, Tc_y];
                scatter(Tc_x,Tc_y,'d','MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w')
                midpoints_list = [midpoints_list;Tc_x,Tc_y];
                g = g + 2;
            end
            % update segment endpoints
            for h = 1:length(seg)
                if seg(1:4,h) == S_i(1:4,pt_idx)
                    seg(1:2,h) = [pt'];
                elseif seg(1:4,h) == S_i(1:4,pb_idx)
                    seg(1:2,h) = [pb'];
                end
            end
            
        % type iii: right/right convex vertex
        elseif length(Si_r) == 2
            % find left endpoint of segments current_v is right endpoint of
            v_l = [];
            for r = 1:length(S_i)
                if S_i(3:4,r) == current_vT(:)
                    v_l = [v_l S_i(1:2,r)];
                end
            end
            % left endpoint of segment with greater y-component that current_v is right endpoint
            v_lt = [];
            v_lt_idx = find(v_l(2,:) == max(v_l(2,:)));
            v_lt = v_l(:,v_lt_idx);
            % left endpoint of segment with smaller y-component that current_v is right endpoint
            v_lb = [];
            v_lb_idx = find(v_l(2,:) == min(v_l(2,:)));
            v_lb = v_l(:,v_lb_idx);
            % upper trapezoid
            T(g:(g+1),:) = [pt', S_i(1:2,pt_idx), v_lt, current_v'];
            polyin = polyshape(T(g,:),T(g+1,:));
            plot(polyin, 'FaceColor','w')
            Trapezoid_list = [Trapezoid_list;polyin];
            [Tc_x, Tc_y] = centroid(polyin);
            centers = [centers; Tc_x, Tc_y];
            midpoints_list = [midpoints_list;Tc_x,Tc_y];
            scatter(Tc_x,Tc_y,'d','MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w')
            g = g + 2;
            % lower trapezoid
            T(g:(g+1),:) = [current_v', v_lb, S_i(1:2,pb_idx), pb'];
            polyin = polyshape(T(g,:),T(g+1,:));
            plot(polyin, 'FaceColor','w')
            Trapezoid_list = [Trapezoid_list;polyin];
            [Tc_x, Tc_y] = centroid(polyin);
            centers = [centers; Tc_x, Tc_y];
            scatter(Tc_x,Tc_y,'d','MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w')
            midpoints_list = [midpoints_list;Tc_x,Tc_y];
            g = g + 2;
            
            % update segment endpoints
            for h = 1:length(seg)
                if seg(1:4,h) == S_i(1:4,pt_idx)
                    seg(1:2,h) = [pt'];
                elseif seg(1:4,h) == S_i(1:4,pb_idx)
                    seg(1:2,h) = [pb'];
                end
            end
        end
    % type v: left/rigth convex vertex
    elseif ~in_t && in_b
        midpoints = [midpoints; pb(1), (current_v(2)-((current_v(2)-pb(2))/2))];
        plot(midpoints(:,1),midpoints(:,2),'.', 'Color', 'k','MarkerSize', 10)
        midpoints_list = [midpoints_list;midpoints(:,1),midpoints(:,2)];
        current_vT = current_v';
        % find left endpoint of segment current_v is the right endpoint of
        for r = 1:length(S_i)
            if S_i(3:4,r) == current_vT(:)
                v_l = S_i(1:2,r);
            end
        end
        % S_i(1:2,pb_idx) is the left endpoint of the segment pb is the
        % right endpoint of
        T(g:(g+1),:) = [current_v', v_l, S_i(1:2,pb_idx), pb'];
        polyin = polyshape(T(g,:),T(g+1,:));
        plot(polyin, 'FaceColor','w')
        [Tc_x, Tc_y] = centroid(polyin);
        centers = [centers; Tc_x, Tc_y];
        Trapezoid_list = [Trapezoid_list;polyin];
        scatter(Tc_x,Tc_y,'d','MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w')
        midpoints_list = [midpoints_list;Tc_x,Tc_y];
        g = g + 2;
        % update segment endpoints
        for h = 1:length(seg)
            if seg(1:4,h) == S_i(1:4,pb_idx)
                seg(1:2,h) = [pb'];
            end
        end
        
        % type vi: left/right non-convex vertex
    elseif in_t && ~in_b
        midpoints = [pt(1), (((pt(2)-current_v(2))/2)+current_v(2))];
        plot(midpoints(:,1),midpoints(:,2),'.', 'Color', 'k','MarkerSize', 10)
        midpoints_list = [midpoints_list;midpoints(:,1),midpoints(:,2)];
        current_vT = current_v';
        % find left endpoint of segment current_v is the right endpoint of
        for r = 1:length(S_i)
            if S_i(3:4,r) == current_vT(:)
                v_l = S_i(1:2,r);
            end
        end
        
        % S_i(1:2,pt_idx) is the left endpoint of the segment pt is the
        % right endpoint of
        T(g:(g+1),:) = [pt', S_i(1:2,pt_idx), v_l, current_v'];
        polyin = polyshape(T(g,:),T(g+1,:));
        plot(polyin, 'FaceColor','w')
        Trapezoid_list = [Trapezoid_list;polyin];
        [Tc_x, Tc_y] = centroid(polyin);
        centers = [centers; Tc_x, Tc_y];
        scatter(Tc_x,Tc_y,'d','MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w')
        midpoints_list = [midpoints_list;Tc_x,Tc_y];
        g = g + 2;
        % update segment endpoints
        for h = 1:length(seg)
            if seg(1:4,h) == S_i(1:4,pt_idx)
                seg(1:2,h) = [pt'];
            end
        end
    end
end

% last trapezoid
T(49:50,:) = [pt', [200 200]', [200 0]', pb'];
polyin = polyshape(T(49,:),T(50,:));
plot(polyin, 'FaceColor','w')
[Tc_x, Tc_y] = centroid(polyin);
scatter(Tc_x,Tc_y,'d','MarkerEdgeColor','k', 'MarkerFaceColor', 'w')

% %%%%%%%%%%%%%%%%%%%%%%%% ROADMAP CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% list of midpoints, trapezoids and midpoints of trapezoids
midpoints_list = [midpoints_list;Tc_x,Tc_y];
Trapezoid_list = [Trapezoid_list;polyin];
% removing some duplicate values
[UniXY,Index]=unique(midpoints_list,'rows');
DupIndex=setdiff(1:size(midpoints_list,1),Index);
midpoints_list(DupIndex,:)=[];
sz_trapezoid = size(Trapezoid_list);
sz_midpoint = size(midpoints_list);
points_x = [];
points_y = [];
mid_center = [];
% for every trapezoid
for i = 1:sz_trapezoid(1)
    point_1 = [];
    point_2 = [];
    [x,y] = boundary(Trapezoid_list(i));
    % to include start and goal point in the 
    [in_start,on_start] = inpolygon(start(1),start(2),x,y);
    [in_goal,on_goal] = inpolygon(goal(1),goal(2),x,y);
    % for every midpoint in the edge and center of trapezoid
    for j = 1:sz_midpoint(1)
        [in,on] = inpolygon(midpoints_list(j,1),midpoints_list(j,2),x,y);
        
        % if the point is in the center of trapezoid
        if in == 1 && on == 0
            point_1 = [point_1;midpoints_list(j,:)];
            mid_center = [mid_center; point_1];
        
        % if the point is on the edge of trapezoid
        elseif in ==1 && on == 1 
            point_2 = [point_2;midpoints_list(j,:)];
            mid_center = [mid_center; point_2];
        end
        % start_point connected the road_map
        if (in_start == 1 && on_start == 0 && in == 1 && on == 0)
            if isempty(point_1) == 0
                point_xs = [point_1(1,1),start(1,1)];
                point_ys = [point_1(1,2),start(1,2)];
                plot(point_xs,point_ys,'--','Color', 'k')
            end    
        end
        % goal_point connected to the road_map
        if in_goal == 1 && on_goal == 0 && in == 1 && on == 0
            if isempty(point_1) == 0
                point_xg = [point_1(1,1),goal(1,1)];
                point_yg = [point_1(1,2),goal(1,2)];
                plot(point_xg,point_yg, '--','Color', 'k')
            end
            
        end
        % connecting all the points together to create roadmap
        if isempty(point_1) == 0 && isempty(point_2)== 0
            sz_point2 = size(point_2);
            for k = 1:sz_point2(1)
                point_x = [point_1(1,1),point_2(k,1)];
                point_y = [point_1(1,2),point_2(k,2)];
                plot(point_x,point_y, 'Color', 'b')
                points_x = [points_x; point_x];
                points_y = [points_y; point_y];
            end
        end
     end
end


% identify which nodes correspond to the start and goal nodes
centers_sorted = unique(centers,'rows');
mid_center_sorted = unique(mid_center,'rows');
for i = 1:length(mid_center_sorted)
    if mid_center_sorted(i,1:2) == [point_xs(1), point_ys(1)] 
        [row,col] = find(mid_center_sorted == mid_center_sorted(i,1:2));
        start_node = unique(row);
        if numel(start_node) > 1
            diff_sx1 = abs(start(1) - centers_sorted(1));
            diff_sx2 = abs(start(1) - centers_sorted(end));
            if diff_sx1 < diff_sx2
                start_node = min(unique(row));
            elseif diff_sx1 > diff_sx2
                start_node = max(unique(row));
            end
        end
    elseif mid_center_sorted(i,1:2) == [point_xg(1), point_yg(1)] 
        [row,col] = find(mid_center_sorted == mid_center_sorted(i,1:2));
        goal_node = unique(row);  
        if numel(goal_node) > 1
            diff_gx1 = abs(goal(1) - centers_sorted(1));
            diff_gx2 = abs(goal(1) - centers_sorted(end));
            if diff_gx1 < diff_gx2
                goal_node = min(unique(row));
            elseif diff_gx1 > diff_gx2
                goal_node = max(unique(row));
            end
        end
    end
end

% %%%%%%%%%%%%%%%% Adjacency Tables for BFS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % numbered in order of increasing x-component of trapezoid center 
AdjTable{1} = [2,3];
AdjTable{2} = [1,4];
AdjTable{3} = [1,6];
AdjTable{4} = [2,5];
AdjTable{5} = [4,7];
AdjTable{6} = [3,8,9];
AdjTable{7} = [5,13];
AdjTable{8} = [6,10];
AdjTable{9} = [6,20];
AdjTable{10} = [8,11];
AdjTable{11} = [10,12];
AdjTable{12} = [11,14];
AdjTable{13} = [7,15];
AdjTable{14} = [12,15];
AdjTable{15} = [13,16,14,17];
AdjTable{16} = [15,19];
AdjTable{17} = [15,18];
AdjTable{18} = [17,21];
AdjTable{19} = [16,22];
AdjTable{20} = [9,31];
AdjTable{21} = [18,23];
AdjTable{22} = [19,24];
AdjTable{23} = [21,26];
AdjTable{24} = [22,25];
AdjTable{25} = [24,30];
AdjTable{26} = [23,30];
AdjTable{27} = [29,28];
AdjTable{28} = [27,33];
AdjTable{29} = [27,32];
AdjTable{30} = [25,26,35];
AdjTable{31} = [20,34];
AdjTable{32} = [29,36];
AdjTable{33} = [28,38];
AdjTable{34} = [31,37];
AdjTable{35} = [30,47];
AdjTable{36} = [32,39];
AdjTable{37} = [34,39];
AdjTable{38} = [33,40];
AdjTable{39} = [36,37,42];
AdjTable{40} = [38,41];
AdjTable{41} = [40,43];
AdjTable{42} = [39,44];
AdjTable{43} = [41,45];
AdjTable{44} = [42,46];
AdjTable{45} = [43,48];
AdjTable{46} = [44,48];
AdjTable{47} = [35,51];
AdjTable{48} = [45,49,46];
AdjTable{49} = [48,50];
AdjTable{50} = [49,52];
AdjTable{51} = [47,53];
AdjTable{52} = [50,53];
AdjTable{53} = [51,52];

[BFS_path] = computeBFSpath(AdjTable,start_node,goal_node);
path_points = [start;mid_center_sorted(BFS_path,:);goal];
h= animatedline('Color','red','Linewidth',2)
for k = 1:length(path_points)
    addpoints(h,double(path_points(k,1)), double(path_points(k,2)));
    drawnow 
    pause(0.2)
end
title ('Sweeping Trapezoid Decomposition, ROADMAP and BFS')






