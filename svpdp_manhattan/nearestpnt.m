function [pout,index] = nearestpnt(pnt, pos)

% given a point 'pnt', and a set of points, 'pos', find a point in 'pos'
% that is nesrest to the 'pnt' and find its index

    for i= 1:size(pos,1)
        postmp(i,1)=lldistkm(pnt,pos(i,:));
    end
    
    [~,index] = min(postmp);
    pout = pos(index,:);
end