function [pout,indexU] = nearestpntsub(pnt, pos, visited)
    % given a point 'pnt' and a set of points 'pos', and the index set of 
    % visited points, find the nearest points from 'pnt' to 'pos' that has
    % not been visited points whose index is in 'visited'
    
    % input:    pnt
    %           pos
    %           visited
    
    % output:   pout: nearest point 
    %           indexU: index of the nearest point (it will be added to the
    %           'visited' at the next iteration
    
    unvisited = setdiff(1:length(pos),visited);
    posU = pos(unvisited,:);
    
    for i= 1:size(posU,1)
        postmp(i,1)=lldistkm(pnt,posU(i,:));
    end
    [~,index] = min(postmp);
    for i = 1:length(index)
        indexU(i) = unvisited(index(i));
    end
    pout = pos(indexU,:);
end