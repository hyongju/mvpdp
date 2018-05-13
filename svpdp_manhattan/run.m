clear all;close all;clc;

% this script generates a minimum tour, for a list of pickup and delivery
% pairs (real-world) acquired from the yellow-cab data of Manhattan acquired
% from NYC.org, the initial location of the vehicles were randomly sampled



% load data
load('./data/nyc_10min.mat');                  % NYC yellow cab data 1/1/2013, 00:00-00:10 (nyc.gov)
load('./data/ve_list_int.mat');                % list of vertices, edge from roadmap of manhattan, NYC (openstreetmap.org)

% set of parameters
k = 1;                                  % number of vehicles
n = 18;                                 % number of customers
q = 2;                                  % max. capacity of each vehicle

% location of the vehicles (uniformly sampled from the list of vertices)
vPos = [v_list(sort(randsample(size(v_list,1),k)),1) v_list(sort(randsample(size(v_list,1),k)),2)];

mip_gap = 0.02;                         % MIP relative gap
x_offset = 0.0008; 
y_offset = 0.0008;

for i = 1:n
    pPos(i,:) = randPos{i+n}(1,:);      % pickup demands
    dPos(i,:) = randPos{i+n}(2,:);      % delivery demands 
end

% conversion of directed edges to undirected edges
e_list2 = e_list;
for i = 1:size(e_list,1)
    if e_list(i,1) > e_list(i,2)
        e_list2(i,1) = e_list(i,2);
        e_list2(i,2) = e_list(i,1);
    end
end
e_list3 = unique(e_list2,'rows');

% find minimum length tour (approximate) using our formulation (IQP)
[tour,tcst,tpath] = mvpdp_mhn(vPos,pPos,dPos,q,mip_gap,v_list,e_list3,v_list_int,k);

%% prepare a sequence of paths to generate figure

recData{l,1} = vPos;
recData{l,2} = pPos;
recData{l,3} = dPos;
recData{l,4} = e_list3;
recData{l,5} = tour{1};
recData{l,6} = tcst;
recData{l,7} = tpath{1};
clear vPos pPos dPos e_list3 tour tcst tpath

l = 1;
posTot = [recData{l,1};recData{l,2};recData{l,3}];
posTotNew = [];
v_list_new = v_list(v_list_int,:);
for i = 1:size(posTot,1)
    [~,qi] = nearestPnt(posTot(i,:), v_list_new);
    posTotNew = [posTotNew;v_list_new(qi,:)];
end
tourTot = v_list(recData{l,7},:);
for i = 1:length(posTotNew)
    for j = 1:length(tourTot)
        if dist2(tourTot(j,:),posTotNew(i,:)) < eps
            upuntil(i) = j;
            break;
        end
    end
end
for i = 1:length(recData{l,5})
    newTour(i) = upuntil(recData{l,5}(i));
end
prb = [];
for i = 1:length(newTour)-2
    if newTour(i) > newTour(i+1)
        prb = [prb newTour(i+1)];
    end
end
prb_acum = prb;
while(~isempty(prb))
    tourTot = v_list(recData{l,7},:);
    for i = 1:length(posTotNew)
        for j = 1:length(tourTot)
            if dist2(tourTot(j,:),posTotNew(i,:)) < eps
                if ~ismember(j,prb_acum)
                    upuntil(i) = j;
                    break;
                end
            end
        end
    end
    for i = 1:length(recData{l,5})
        newTour(i) = upuntil(recData{l,5}(i));
    end    
    prb = [];
    for i = 1:length(newTour)-2
        if newTour(i) > newTour(i+1)
            prb = [prb newTour(i+1)];
        end
    end
end
%%
% plot results

for y1 = length(newTour):length(newTour)
    h0 = figure('position',[100 100 900 1200],'Color',[1 1 1]);
    h1 = plot(recData{l,1}(:,2),recData{l,1}(:,1),'MarkerSize',11,'Marker','square',...
        'LineStyle','none',...
        'Color','k');hold on;
    h2 = plot(recData{l,2}(:,2),recData{l,2}(:,1),'MarkerSize',9,'Marker','o',...
        'MarkerFaceColor',[0.85 0.85 0.85],'LineStyle','none',...
        'Color','k');hold on;
    h3 = plot(recData{l,3}(:,2),recData{l,3}(:,1),'MarkerSize',9,'Marker','o',...
        'LineStyle','none',...
        'Color','k');hold on;
    for i = 1:size(recData{l,2},1)
        text(recData{l,2}(i,2)+x_offset,recData{l,2}(i,1)+y_offset,strcat(num2str(i),'P'),'FontSize',14,'Color','k');hold on;
        text(recData{l,3}(i,2)+x_offset,recData{l,3}(i,1)+y_offset,strcat(num2str(i),'D'),'FontSize',14,'Color','k');hold on;
    end
    for i = 1:length(recData{l,2})
        h6 = line([recData{l,2}(i,2) recData{l,3}(i,2)],[recData{l,2}(i,1) recData{l,3}(i,1)],'LineStyle','--','Color','k');hold on;
    end

    set(gca,'FontSize',14);
    ylabel('latitude');xlabel('longitude');
    axis([min(posTotNew(:,2))-0.001 max(posTotNew(:,2))+0.001 min(posTotNew(:,1))-0.0012 max(posTotNew(:,1))+0.003]);

    if y1 == length(newTour)
        for j = 5:length(recData{l,7})-5
            h8 = line([v_list(recData{l,7}(j),2) v_list(recData{l,7}(j+1),2)],[v_list(recData{l,7}(j),1) v_list(recData{l,7}(j+1),1)],'Color',[0 0 0],'LineWidth',2); hold on;
        end
    end
    h5 = legend([h1 h2 h3 h6 h8],'vehicle depot (O&D)','pickup','delivery','association','solution tour (cycle)');
    set(h5,'Location','northwest');
end
plot_google_map('Alpha',0.23,'ShowLabels',0);
set(findall(h0, 'Type', 'Text'),'FontWeight', 'Normal')

for y1 = length(newTour):length(newTour)
    h0 = figure('position',[100 100 900 1200],'Color',[1 1 1]);
    h1 = plot(recData{l,1}(:,2),recData{l,1}(:,1),'MarkerSize',11,'Marker','square',...
        'LineStyle','none',...
        'Color','k');hold on;
    h2 = plot(recData{l,2}(:,2),recData{l,2}(:,1),'MarkerSize',9,'Marker','o',...
        'MarkerFaceColor',[0.85 0.85 0.85],'LineStyle','none',...
        'Color','k');hold on;
    h3 = plot(recData{l,3}(:,2),recData{l,3}(:,1),'MarkerSize',9,'Marker','o',...
        'LineStyle','none',...
        'Color','k');hold on;
    for i = 1:size(recData{l,2},1)
        text(recData{l,2}(i,2)+x_offset,recData{l,2}(i,1)+y_offset,strcat(num2str(i),'P'),'FontSize',14,'Color','k');hold on;
        text(recData{l,3}(i,2)+x_offset,recData{l,3}(i,1)+y_offset,strcat(num2str(i),'D'),'FontSize',14,'Color','k');hold on;
    end
    for i = 1:length(recData{l,2})
        h6 = line([recData{l,2}(i,2) recData{l,3}(i,2)],[recData{l,2}(i,1) recData{l,3}(i,1)],'LineStyle','--','Color','k');hold on;
    end
    set(gca,'FontSize',14);
    ylabel('latitude');xlabel('longitude');
    axis([min(posTotNew(:,2))-0.001 max(posTotNew(:,2))+0.001 min(posTotNew(:,1))-0.0012 max(posTotNew(:,1))+0.003]);

    h5 = legend([h1 h2 h3 h6 h8],'vehicle depot (O&D)','pickup','delivery','association','solution tour (cycle)');
    set(h5,'Location','northwest');
end
title('customers’ pickup-delivery pair');
plot_google_map('Alpha',0.23,'ShowLabels',0);
set(findall(h0, 'Type', 'Text'),'FontWeight', 'Normal')    
