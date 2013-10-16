function sol = BOACO(dg,Cdist,Ccrime,start,goal,xy,param)
% Bi-Objective Ant Colony Optimization
%

% debug?
debug1 = 0;
debug2 = 1;

% Default parameters
if nargin < 7
    param.epsilon = 0.0001;
    param.localUpdateEvapRate = 0.9;
    param.globalUpdateEvapRate= 0.9;
    param.q0 = 0.1;
    param.a = 10;
    param.b = 90;
    param.num_ants = 100;
    param.num_colonies = 20;
end

% parameters
eps = param.epsilon;
evapLocal = param.localUpdateEvapRate;
evapGlobal = param.globalUpdateEvapRate;
q0 = param.q0;
a = param.a;
b = param.b;
num_ants = param.num_ants;
num_colonies = param.num_colonies;

if a < 10; error('a should be a positive value!!!');end
if a > b; error('b should be more than a!!!');end
if b > num_ants; error('b should be less than the number of ants!!!');end

% Upper and lower limits on edge values
C1min = min(min(nonzeros(Cdist)));
C1max = max(max(Cdist));
C2min = min(min(nonzeros(Ccrime)));
C2max = max(max(Ccrime));

% Pheromone Matrices for 1st and 2nd objectives
Tau1 = dg; %sparse(row,col,ones(nnz(dg),1),N,N);
Tau2 = dg; %sparse(row,col,ones(nnz(dg),1),N,N);

% Compute Second Heuristic Parameter
n2 = n2heuristic(dg,goal);
n2 = 1./n2;

% Non-dominated solutions
sol = struct('id',{},'c1',{},'c2',{},'path',{});
sol(1) = struct('id',-1,'c1',inf,'c2',inf,'path',[]);

% Run multiple colonies
for c = 1:num_colonies
    fprintf('\t Iteration #%d\n',c);
    ants = struct('id',{},'nodeId','','c1',{},'c2',{},'path',{});
    for h = 1:num_ants
        ants(h) = CreateAnt(h,start);
    end
    NondominAnts = zeros(1,num_ants); % Ants generated non-dominated solutions
    active_flag = ones(1,num_ants); % Active ants: Active=1, at Goal=0, Got stuck=-1;
    while ~isempty(find(active_flag == 1,1))
        for ant = find(active_flag == 1)
            % Next possible nodes
            nextNodes = findAdjacent(ants(ant),dg);
            nextNodes = nonDuplicate(ants(ant).path,nextNodes);
            
            % Check to see if the ant get stuck
            if isempty(nextNodes)
                active_flag(ant) = -1;
                continue
            end
            
            % First Heuristic
            n11 = zeros(1,length(nextNodes));
            n12 = zeros(1,length(nextNodes));
            for j = 1:length(nextNodes)
                n11(j) = min(1, (C1max-Cdist(ants(ant).nodeId,nextNodes(j))) /(C1max-C1min) + eps);
                n12(j) = min(1, (C2max-Ccrime(ants(ant).nodeId,nextNodes(j)))/(C2max-C2min) + eps);
            end
            
            % Second Heuristic has been pre-calculated
            
            % Ant specific weights
            if ant <= a
                gamma = 0;
            elseif (a<ant) && (ant<b)
                gamma = ant/(b-a)-ant/(b-a);
            else
                gamma = 1;
            end
            
            % Transition probability
            q = rand();
            if q <= q0
                nextNode = -1;
                maxProb = -1;
                for j = 1:length(nextNodes)
                    pij = (Tau1(ants(ant).nodeId,nextNodes(j))*n11(j)).^(gamma)*...
                        (Tau2(ants(ant).nodeId,nextNodes(j))*n12(j)).^(1-gamma)*n2(nextNodes(j));
                    if pij > maxProb
                        maxProb = pij;
                        nextNode = nextNodes(j);
                    end
                end
            else
                pij = zeros(1,length(nextNodes));
                pij_sum = 0;
                for j = 1:length(nextNodes)
                    pij(j) = (Tau1(ants(ant).nodeId,nextNodes(j))*n11(j)).^(gamma)*...
                        (Tau2(ants(ant).nodeId,nextNodes(j))*n12(j)).^(1-gamma) * n2(nextNodes(j));
                    pij_sum = pij_sum + pij(j);
                end
                % Normalize the exit probabilities
                pij = pij/pij_sum;
                
                %Find the selected index
                p = rand();
                pij =  cumsum(pij./sum(pij));
                nextNode = nextNodes(find(pij>=p,1,'first'));
            end
            
            % Local pheromone update
            Tau1(ants(ant).nodeId,nextNode) = Tau1(ants(ant).nodeId,nextNode)*evapLocal;
            Tau2(ants(ant).nodeId,nextNode) = Tau2(ants(ant).nodeId,nextNode)*evapLocal;
            
            % Move to next node
            ants(ant).c1 = ants(ant).c1 + Cdist(ants(ant).nodeId,nextNode);
            ants(ant).c2 = ants(ant).c2 + Ccrime(ants(ant).nodeId,nextNode);
            ants(ant).nodeId = nextNode;
            ants(ant).path(end+1) = nextNode;
            
            % check if nextNode is goal and update non-dominated solutions
            if ants(ant).nodeId == goal
                active_flag(ant) = 0;
                if debug1 % debug
                    ants(ant)
                    showpath(start,goal,ants(ant),xy,sprintf('Ant: %d',ant));
                    pause(0.1)
                    close all
                end
                [sol] = UpdateSol(sol,ants(ant));
            end
        end
    end
    if debug2
        showpath(start,goal,sol,xy,sprintf('Itr %d: Non-dominated Solutions',c))
        pause()
        close all
    end
    % Global pheromone update
    tmp = [sol.id];
    NondominAnts = tmp(tmp>-1);
    
    for i = 1 : length(sol)
        sol(i).id = -1;
    end
     
    for ant = NondominAnts
        antPath = ants(ant).path;
        l = length(antPath);
        for i = 1 : l-1
            Tau1(antPath(i),antPath(i+1)) = Tau1(antPath(i),antPath(i+1))*evapGlobal;
            Tau2(antPath(i),antPath(i+1)) = Tau2(antPath(i),antPath(i+1))*evapGlobal;
            Tau1(antPath(i),antPath(i+1)) = ...
                min(1,Tau1(antPath(i),antPath(i+1))+C1min/Cdist(antPath(i),antPath(i+1)));
            Tau2(antPath(i),antPath(i+1)) = ...
                min(1,Tau2(antPath(i),antPath(i+1))+C2min/Ccrime(antPath(i),antPath(i+1)));
        end
    end
end
showpath(start,goal,sol,xy,'Non-dominated Solutions')
end

%% Find depth value of each node with respect to goal
function n2 = n2heuristic(dg,goal)
N = length(dg);
n2 = zeros(1,N);

frontier(1) = goal;
explored = zeros(1,N);

while ~isempty(frontier)
    node = frontier(1);
    frontier(1) = [];
    explored(node) = 1;
    
    nextNodes = Adjacents(dg,node);
    for next = nextNodes
        if ~nonDupExplored(explored,next)
            if ~nonDupFrontier(frontier,next)
                frontier = [frontier,next];
                n2(next) = n2(node) + 1;
            end
        end
    end
    
end

idx = find(n2);
n2(idx) = n2(idx) + 1;
n2(goal) = 1;

%%%%%%%%%%%%%
    function ids = Adjacents(dg,node)
        [~,ids,~] = find(dg(node,:));
    end
%%%%%%%%%%%%%
    function dup = nonDupExplored(list,next)
        ind = find(list);
        dup = find(ind == next);
        if isempty(dup)
            dup = 0;
        end
    end
%%%%%%%%%%%%%
    function dup = nonDupFrontier(list,next)
        dup = find(list == next);
        if isempty(dup)
            dup = 0;
        end
    end
%%%%%%%%%%%%%
end

%%
function [sol] = UpdateSol(sol,ant)
% sol: Pareto optimal solutions: to be updated
% ant: The new solution
n = length(sol);
flag = 0;

for i = n:-1:1
    if ~((ant.c1>sol(i).c1) && (ant.c2>sol(i).c2))
        flag = 1;
        if (ant.c1<sol(i).c1) && (ant.c2<sol(i).c2)
            sol(i) = [];
        end
    end
end

if flag
    tmp.id = ant.id;
    tmp.c1 = ant.c1;
    tmp.c2 = ant.c2;
    tmp.path = ant.path;
    sol(end+1) = tmp;
end

end
%% Find new ways which have not been explored before
function candidates = nonDuplicate(explored,candidates)
n = length(candidates);
for i = n : -1 : 1
    if sum(explored == candidates(i)) > 0
        candidates(i) = [];
    end
end
end
%% Find new possible ways from the current location of the ant
function ids = findAdjacent(ant,dg)
[~,ids,~] = find(dg(ant.nodeId,:));
end

%% Euclidean distance between two points
function [euclideanDistance] = EuDist(n1,n2)
euclideanDistance =  sqrt(sum((n1-n2).^2));
end
%% Create new ant
function ant = CreateAnt(id,nodeId)
ant.id   = id;
ant.nodeId = nodeId;
ant.c1    = 0;
ant.c2    = 0;
ant.path = [ant.nodeId];
end
%% Show path
function showpath(start,goal,sol,xy,text)
figure('units','normalized','outerposition',[0 0 1 1])
hold all
plot(xy(:,1),xy(:,2),'k.','Markersize',5);
plot(xy(start,1),xy(start,2),'r.','Markersize',30);
plot(xy(goal,1),xy(goal,2),'b.','Markersize',30);
n = length(sol);
for i = 1 : n
    plot(xy(sol(i).path,1),xy(sol(i).path,2));
end
title(text)
end