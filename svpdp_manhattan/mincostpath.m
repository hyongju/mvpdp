function [path,cst] = mincostpath(G,start,goal,v_list,v_list_int)

% this function finds minimum cost path between start and goal node from a
% directed graph,
% the arc cost corresponds to the actual distance on a roadmap between the
% two GPS locations

% input:    G: directed graph (matlab)
%           start: index of the start node
%           goal: index of the goal node
%           v_list: positions of the list of vertices
%           v_list_int: positions of the list of vertices that has at least
%           three neighbors

% output:   path: the generated sequence of nodes with minimum tour cost
%           cst: the minimum tour cost

% a list of dependent functions:
%           'nearestpnt','nearestpntsub', 'lldistkm'




v_list_new = v_list(v_list_int,:);

% find the nesrest point between the a vertex in a graph G the 
[~,qi] = nearestpnt(start, v_list_new);     % start node
[~,qj] = nearestpnt(goal, v_list_new);      % goal node

qi_in = qi;
qj_in = qj;

% find the shortest distance path between start node(*) and the goal
% node(*), where (*) indicates that it is a node from G

[P,~] = shortestpath(G,v_list_int(qi),v_list_int(qj));


% post-processing of the path

visited = [];
cnt = 0;
flag = 1;
while(isempty(P))
    cnt = cnt + 1;
    visited = [visited qi qj];
    if flag ==1
        [~,qi_next] = nearestpntsub(v_list_new(qi,:), v_list_new, visited);
        qi = qi_next;
        flag = 2;

    else
        [~,qj_next] = nearestpntsub(v_list_new(qj,:), v_list_new, visited);
        qj = qj_next;
        flag = 1;
    end
    [P,~] = shortestpath(G,v_list_int(qi),v_list_int(qj));
end

% compute the cost incurred from the minimum length tour

cst = 0;
for i =1:length(P)-1
    cst = cst + lldistkm(v_list(P(i),:),v_list(P(i+1),:));
end

% sum all the incurred costs
cst = cst + lldistkm(v_list_new(qi_in,:),v_list_new(qi,:)) + lldistkm(v_list_new(qj_in,:),v_list_new(qj,:));
cst = cst + lldistkm(v_list_new(qi_in,:),start) + lldistkm(v_list_new(qj_in,:),goal);

% generate sequence of paths
path = [v_list_int(qi_in) P v_list_int(qj_in)];
