function [tour1,tcst,tpath] = mvpdp_mhn(v_pos,p_pos,d_pos,q,appx,v_list,e_list,v_list_int,k)

% this function computes the approximate solution the to single vehicle
% pickup and delivery problem where the cost matrix was constructed using
% the actual real-world travel cost, where the roadmaps was generated based
% upon the map acquired from 'openstreetmap.org'

% inputs:
%       v_pos: the position of the vehicle depot (O&D)
%       p_pos: positions of the pickup demands
%       d_pos: positions of the delivey demands
%       q:  capacity of the vehicle
%       appx: MIP relative gap
%       v_list: list of vertices (GPS locations)
%       e_list: adjacencies between vertices
%       v_list_int: list of vertices (GPS locations) who has at least 3
%       neighbors
%       k: number of vehicles


gamma = 1.01;
vert = [v_pos;p_pos;d_pos];

% total number of vertices
v = size(vert,1);        

% create an undirected graph G of Manhattan, NYC

s1 = e_list(:,1)';
t1 = e_list(:,2)';
wt1 = e_list(:,3)';

for i = 1:length(v_list)
    n1{i} = sprintf('%d',i);
end
G = graph(s1,t1,wt1,n1);

% generate the cost matrix (c)
c = zeros(v,v);
for i = 1:size(c,1)
    for j = 1:size(c,2)
        if i == j
            c(i,j) = 0;
        else
            [path{i,j},c(i,j)] = mincostpath(G,vert(i,:),vert(j,:),v_list,v_list_int);
        end
    end
end

% make c a PSD (positie semi-definite) matrix
for i = 1:size(c,1)
    sum_c(i) = 0;
    for j = 1:size(c,2)
        sum_c(i) = sum_c(i) + abs(c(i,j));
    end
    c(i,i) = c(i,i) + gamma * sum_c(i);
end

Q = zeros(v^2,v^2);
for i = 1:v
    Q((i-1) * v + 1:(i-1) * v + v,(i-1) * v + 1:(i-1) * v + v) = c;
end
for i = 1:v-((k-1)+1)
    Q(i*v+v*(k-1)+1:i*v+v*(k-1)+v,(i-1)*v+1:(i-1)*v+v) = c/2;
    Q((i-1)*v+1:(i-1)*v+v,i*v+v*(k-1)+1:i*v+v*(k-1)+v) = c/2;
end

for i = 1:k
    Q((v-(k-i+1))*v +1:(v-(k-i+1))*v +v,(i-1)*v+1:(i-1)*v+v) = c/2;
    Q((i-1)*v+1:(i-1)*v+v,(v-(k-i+1))*v +1:(v-(k-i+1))*v +v) = c/2;
end

%% interger programming....

% decision vector {0,1}
x = binvar(v^2,1);
% decision matrix {0,1}
X = reshape(x,v,v)';

xx = reshape(X,v^2,1);
% integer slack variables
y = intvar(v,1);
F = [];

% set of constraints

Lt = zeros(v,v);
for i = 1:v
    for j = 1:floor((v-i)/k)+1
        Lt(i,(j-1)*k+i) = 1;
    end
end
Lt = Lt';
for i = 1:k
    L{i} = zeros(v,v);
    for j = 1:floor((v-i)/k)+1
        i;
        L{i}((j-1)*k+i,:) = Lt((j-1)*k+i,:);
    end
end

% capacity constraint
n = size(p_pos,1);          % number of customers (n)
d = [zeros(k,1);ones(n,1);-ones(n,1)];
for i = 1:length(L)
    F = [F,L{i}*X*d <= q];
end

% degree constraint
F = [F, X * ones(v,1)==ones(v,1),X'*ones(v,1) == ones(v,1)];
for i = 1:k
    F = [F,X(v-i+1,k-i+1) == 1];
end
N = 1:1:v;

% precedence constraint
for i = 1:n
    eini = zeros(1,v);
    eini(i+k) = 1;
    eini(i+n+k) = -1;
    F = [F,N*X*eini' <= 0];
    F = [F,N*X*eini' == y(i)*k];
end

% IQP
obj =x'*Q*x;
ops = sdpsettings('solver','cplex','verbose',2,'showprogress',1,'debug',1);

% cplex specific settings:
ops.cplex.display = 'on';
ops.cplex.mip.tolerances.mipgap=appx;
optimize(F,obj,ops)

% generate the optimal tour from x
solution = reshape(value(xx),[size(c,1),size(c,2)]);
tour = round(solution*[1:v]');
tour = [1:k, tour'];

tcst = 0;

% generate:
%   the (sub) optimal tour cost (tcst)
%   a sequence of positions from the (sub) optimal tour on real road-map
for i = 1:k
    tour1{i} = [];
    tpath{i} = [];
    for j = 1:length(tour)/k
        tour1{i} = [tour1{i} tour(k*(j-1)+i)];
    end
    for j =1:length(tour1{i})-1
        tcst = tcst+ c(tour1{i}(j),tour1{i}(j+1));
        tpath{i} = [tpath{i} path{tour1{i}(j),tour1{i}(j+1)}];
    end
end


