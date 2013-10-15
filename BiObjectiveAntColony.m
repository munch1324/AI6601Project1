function [ output_args ] = BiObjectiveAntColony(adjacency,dist,crime,start,goal,xy)
%BIOBJECTIVEANTCOLONY Summary of this function goes here
%   Detailed explanation goes here

%%Global Variables

%Equation 2
N = length(adjacency);
C1min = min(min(dist(dist > 0)));
C1max = max(max(dist));
C2min = min(min(crime(crime > 0)));
C2max = max(max(crime));
epsilon = 0.0001;
localUpdateEvapRate = 0.99;
globalUpdateEvapRate= 0.99;
q0 = 0.1;
a = 3;
b = 8;
H = 10;
C = 5;
gammaTau = 10;

%First  Pheromone Matrix
Tau1 = ones(N,N);
%Second Pheromone Matrix
Tau2 = ones(N,N);

%Outlining paper steps (Fig 2 - pg 1241)

%%TODO : compute second heuristic
%Compute Second Heuristic Parameter
%Table 1 p. 1240
n2 = ones(1,N);

%Create a Colony


%Run multiple colonies
for c = 1:C
    
    ant = [];
    %For each ant
    for h = 1:H
        ant(h).nodeId = start;
        ant(h).path   = start;
        %until you reach the sink
        while ant.nodeId ~= goal
            J = findAdjacent(ant.nodeId,adjacency);
            
            %First Heuristic - Equation 2 Page 1239
            n11 = sparse(1,max(J));
            n12 = sparse(1,max(J));
            for j = J
               n11(j) = min(1, (C1max-dist(ant(h).nodeId,j)) /(C1max-C1min) + epsilon);
               n12(j) = min(1, (C2max-crime(ant(h).nodeId,j))/(C2max-C2min) + epsilon);    
            end
            
            if h <= a
                gamma = 0;
            elseif a < h < b
                gamma = h/(b-a) - a/(b-a);
            else
                gamma = 1;
            end
            nextIndex = -1;
            
            %Transition Event - Equation 5 p. 1240
            %%TODO what is q?
            q = rand();
            if q <= q0
                %%For each of the edges find argmax over J
                maxProb = -1;
                for j = J
                    pij = (Tau1(ant(h).nodeId,j)*n11(j)).^(gamma) * (Tau2(ant(h).nodeId,j)*n12(j)).^(1-gamma) * n2(j);
                    if pij > maxProb
                       maxProb = pij;
                       nextIndex = j;
                    end
                end
            else
                %%Find Pij for each of the edges using this equation:

                pij = sparse(1,max(J));
                summedOver = 0;
                
                %Caclulating the probabilities based on the costs, the
                %visibility, and the weight (gamma)
                for j = J
                    pij(j) = (Tau1(ant(h).nodeId,j)*n11(j)).^(gamma) * (Tau2(ant(h).nodeId,j)*n12(j)).^(1-gamma) * n2(j);
                    summedOver = summedOver + pij(j);
                end
                %Normalizing the exit probabilities
                pij = pij/summedOver;
                
                %Find the selected index
                p = rand();
                for j = J
                    p = p - pij(j);
                    if p < 0
                       nextIndex = j;
                       break;
                    end
                end
            end
            
            %Equation 2 - first movement heuristic
            ant(h).path(end+1) = nextIndex;
            ant(h).nodeId = nextIndex
        end
        %Local Pheromone update - serially
        Tau1 = Tau1*localUpdateEvapRate;
        Tau2 = Tau2*localUpdateEvapRate;
        for h = 1:H
            for i = 2:length(ant(h).path)
                %Adding Pheromone -- less is added to 'longer' edges
                Tau1(ant.nodeId,nextIndex) = Tau1(ant(h).path(i-1),ant(h).path(i)) + gammaTau / dist(ant(h).path(i-1),ant(h).path(i));
                Tau2(ant.nodeId,nextIndex) = Tau2(ant(h).path(i-1),ant(h).path(i)) + gammaTau / crime(ant(h).path(i-1),ant(h).path(i));
            end
        end
    end
    
    %%TODO calculate the non-dominated solutions
    %perform a global update
    %Equation 4 p.1240
    %%TODO : only on non-dominated paths
    Tau1 = Tau1*globalUpdateEvapRate;
    Tau2 = Tau2*globalUpdateEvapRate;
    for r=1:length(Tau1(:,1))
        for c=1:length(Tau1(1,:))
            if Tau1(r,c) ~= 0
                Tau1 = min(1,Tau1(r,c)+ delta/dist(r,c));
            end
        end
    end
    for r=1:length(Tau2(:,1))
        for c=1:length(Tau2(1,:))
            if Tau1(r,c) ~= 0
                Tau2 = min(1,Tau2(r,c)+ delta/crime(r,c));
            end
        end
    end
    
    %%TODO remove dominated paths
    
end
    
%% Return paths


end
%%
function node = createNode(id,g,h,f,path)
node.id   = id;
node.g    = g;
node.h    = h;
node.f    = f;
node.path = [path;node.id];
end

%%
function ids = findAdjacent(id,dg)
[~,ids,~] = find(dg(id,:));
end

%%
function ind = cost_sort(openList)
n = length(openList);
costs = zeros(n,1);
for i = 1 : n
    costs(i) = openList(i).f;
end
[~,ind] = sort(costs);
ind = ind(1);
end

%%
function [euclideanDistance] = EuDist(n1,n2)
euclideanDistance =  sqrt(sum((n1-n2).^2));
end
