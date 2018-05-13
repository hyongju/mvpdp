clear all;close all;clc
%%
% -------------------------------------------------------------------------
% An IQP Formulation for Multiple  Vehicle  Pickup  and  Delivery  Problem (MVPDP)
% by Hyongju Park and Jinsun Liu
% description: this program solves MVPDP using our permutation-matrix based
% formulation using branch-and-cut methods for 'random' problem instances
% -------------------------------------------------------------------------


k = 2;                      % number of vehicles (vehicle depots)
n = 5;                      % number of custumers(n) this needs to be divisible by the number k

q = 2;                      % capacity of each vehicle
rng('shuffle');             % random seed: shuffle
mip_gap = 0;                % MIP relative gap (e.g., 0.02: 2%)

gamma = 0;                  % gamma (parameter)
% Di = 0;

vert = rand(2*k+n*2+1,2);   % generate location of the vertices randomly

gamma = 1.01;               % gamma (parameter)
Di = 0;
v = size(vert,1);           % number of vertices |V| = 2n + 1

% generate the cost matrix (c)
c = zeros(v,v);
for i = 1:size(c,1)
    for j = 1:size(c,2)
        c(i,j) = norm(vert(i,:)-vert(j,:)); % arc cost is the euclidean distance inbetween
    end
end
% set diagonals of c to zero
for i = 1:k-1
    c(1+k+i,1+i+1) = 0;
end

% set other cost to edges that are always zero
c(1,2) = 0;
c(1+1+k,1) = 0;

% make c PSD (positive semi-definite)
for i = 1:size(c,1)
    sum_c(i) = 0;
    for j = 1:size(c,2)
        sum_c(i) = sum_c(i) + abs(c(i,j));
    end
    c(i,i) = c(i,i) + sum_c(i);
end

all(eig(c)>=0)              % check if c is PSD

% transform into quadratic form x'Q x
% obtain Q using 
Q = zeros(v^2,v^2);         
for i = 1:v
    Q((i-1) * v + 1:(i-1) * v + v,(i-1) * v + 1:(i-1) * v + v) = gamma*c;
end
for i = 1:v-((1-1)+1)
    Q(i*v+v*(1-1)+1:i*v+v*(1-1)+v,(i-1)*v+1:(i-1)*v+v) = c/2;
    Q((i-1)*v+1:(i-1)*v+v,i*v+v*(1-1)+1:i*v+v*(1-1)+v) = c/2;
end

for i = 1:1
    Q((v-(1-i+1))*v +1:(v-(1-i+1))*v +v,(i-1)*v+1:(i-1)*v+v) = c/2;
    Q((i-1)*v+1:(i-1)*v+v,(v-(1-i+1))*v +1:(v-(1-i+1))*v +v) = c/2;
end

for i = 1:v^2
    Di(i) = min(sum(Q(:,i)),sum(Q(:,i)));
end

Qtil = Q + diag(Di);

%% Integer Quadratic Programming (IQP) formulation

% define X as a vxv matrix with {0,1}s
X = binvar(v,v,'full');

% fill 0s in X to reduce the search space of the solver
X(1,:) = [0,1,zeros(1,v-2)];
X(2,1:k+1) = zeros(1,k+1);
X(2,1+k+2:2*k+1) = zeros(1,k-1);
X(2,v-n+1:v) = zeros(1,n);

X(v-2,1+2*k+1:v-n) = zeros(1,n);
X(v-2,1:1+k-1) = zeros(1,1+k-1);
X(v-2,k+2:1+2*k) = zeros(1,k);
X(v-2,2*k+1+1:2*k+1+n) = zeros(1,n);
X(v-1,:) = [zeros(1,2*k),1,zeros(1,2*n)];
X(v,:) = [1,zeros(1,v-1)];

% x = vec(X)
x = reshape(X',v^2,1);

% constraints
F = [];

% degree
F = [F,X*ones(v,1)==ones(v,1),X'*ones(v,1)==ones(v,1)];


% vehicle precedence
N = 1:v;
for i = 1:k
    eini = zeros(1,v);
    eini(1+i) = 1;
    eini(1+i+k) = -1;
    F = [F,N*X*eini' <= -1];
end
for i = 1:k-1
    eini = zeros(1,v);
    eini(1+k+i) = 1;
    eini(1+i+1) = -1;
    F = [F,N*X*eini' == -1];
end

% capacity and association
for j = 1:k
    for i = 1:n
        eini0 = zeros(1,v);
        eini0(2*k+1+i) = 1;
        eini0(2*k+1+n+i) = -1;
        
        eini1 = zeros(1,v);
        eini1(1+j) = 1;
        eini1(2*k+1+i) = -1;

        eini2 = zeros(1,v);
        eini2(1+k+j) = -1;
        eini2(2*k+1+i) = 1;     
        
        eini3 = zeros(1,v);
        eini3(1+k+j) = -1;
        eini3(2*k+1+n+i) = 1;          
        F = [F,implies([N*X*eini1' <= -1, N*X*eini2' <= -1],[N*X*eini0' <= -1,N*X*eini3' <= -1])];
    end
end

% customer precedence
F = [F,0<=tril(ones(v,v))*X*[zeros(k*2+1,1);ones(n,1);-ones(n,1)] <= q*ones(v,1)];

% objective function as a quadratic function
obj = x'*Qtil*x - Di*x;
ops = sdpsettings('solver','cplex','verbose',3,'showprogress',1,'debug',1,'usex0',0,'warning',1);

% cplex specific settings
ops.cplex.mip.tolerances.mipgap=mip_gap;
ops.cplex.display = 'on';

% solve IQP
optimize(F,obj,ops)


solution = value(X);

% obtain sigma (optimal tour) from sigma0 using X (optimal solution of IQP)

tour = round(solution*[1:v]');
tour = [1, tour'];

% plot solution tour

h0 = figure('position',[100 100 800 800],'Color',[1 1 1]);
plot(vert(1+2*k+1:1+2*k+1+n-1,1),vert(1+2*k+1:1+2*k+1+n-1,2),'ok','MarkerSize',10,'LineWidth',1); hold on;
plot(vert(1+2*k+1+n:end-1,1),vert(1+2*k+1+n:end-1,2),'ok','MarkerSize',10,'MarkerFaceColor',[0.85 0.85 0.85],'LineWidth',1); hold on;
hold on;
plot(vert(2:2+k-1,1),vert(2:2+k-1,2),'sk','MarkerSize',12,'LineWidth',1); hold on;
hold on;
plot(vert(2+k:2+2*k-1,1),vert(2+k:2+2*k-1,2),'sk','MarkerSize',12,'MarkerFaceColor',[0.85 0.85 0.85],'LineWidth',1); hold on;
hold on;
plot(vert(1,1),vert(1,2),'ok','MarkerSize',10,'LineWidth',1); hold on;
hold on;



for i = 1:size(vert,1)
    if (mod(i-2*k-1,n)) == 0 && i > k
        prtVal = n;
    elseif i <= k
        prtVal = i;
    else
        prtVal = mod(i-k,n);
    end
    if i <=2*k+1 && i >1
        if mod(i-1,k) == 0
            prtVal = k;
        else
            prtVal = mod(i-1,k);
        end
        if i <= k+1
            str = 'VO';
        else
            str = 'VD';
        end
        col = 'black';
    elseif i == 1
        str = 'O';
        col = 'black';
        prtVal = [];
    elseif i > 2*k+1 && i <= 2*k+1+n
        str = 'P';
        col = 'black';
        if mod(i-1-2*k,n) == 0
            prtVal = n;
        else
            prtVal = mod(i-1-2*k,n) ;
        end
    else
        str = 'D';
        col = 'black';
        if mod(i-1-2*k,n) == 0
            prtVal = n;
        else
            prtVal = mod(i-1-2*k,n) ;
        end            
    end
    text(vert(i,1)+0.02,vert(i,2)+0.02, sprintf('%.0f %s',prtVal,str),'Color',sprintf('%s',col),'FontSize',16);
end

for i = 1:n
    line([vert(1+2*k+i,1) vert(1+2*k+i+n,1)],[vert(1+2*k+i,2) vert(1+2*k+i+n,2)],'Color','k','LineStyle','--');hold on;
end
spc = [];
for i = 1:k-1
    line([vert(1+k+i,1) vert(1+i+1,1)],[vert(1+k+i,2),vert(1+i+1,2)],'Color','k','LineStyle',':');hold on;
    for j = 1:length(tour)-1
        if (1+k+i == tour(j)) && (1+i+1  == tour(j+1))
            spc = [spc j];
        end
    end
end

for i = 1:length(tour)-1
    if i == 1 || i==(length(tour)-1)
        line([vert(tour(i),1) vert(tour(i+1),1)],[vert(tour(i),2) vert(tour(i+1),2)],'LineStyle',':','Color','black');hold on; 
    elseif ismember(i,spc)
        
    else
        line([vert(tour(i),1) vert(tour(i+1),1)],[vert(tour(i),2) vert(tour(i+1),2)],'Color','black','LineWidth',2);hold on;   
    end
end

set(gca,'FontSize',16);
axis([0 1 0 1]);
axis off;
set(findall(h0, 'Type', 'Text'),'FontWeight', 'Normal')

xlabel([]);
ylabel([]);
