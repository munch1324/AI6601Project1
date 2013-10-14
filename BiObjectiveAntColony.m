function [ output_args ] = BiObjectiveAntColony(adjacency,dist,crime,start,goal,xy)
%BIOBJECTIVEANTCOLONY Summary of this function goes here
%   Detailed explanation goes here

%%Global Variables

%Equation 2
N = length(adjacencyMatrix);
C1min = min(min(dist));
C1max = max(max(dist));
C2min = min(min(crime));
C2max = max(max(crime));
epsilon = 0.0001;
localUpdateEvapRate = 0.99;
globalUpdateEvapRate= 0.99;
q0 = 0.9;
a = 3;
b = 8;
H = 10;
gammaTau = 10;

%First  Pheromone Matrix
Tau1 = sparse(N,N);
%Second Pheromone Matrix
Tau2 = sparse(N,N);

%Outlining paper steps (Fig 2 - pg 1241)

%Compute Second Heuristic Parameter
%Table 1 p. 1240


%Create a Colony


%Continue until end condition is satisfied
endConditionMet = false;
while ~endConditionMet
    
    %For each ant
    
    for h = 1:H
        ant(h).nodeId = start;
        %until you reach the sink
        while ant.nodeId ~= goal
            J = findAdjacent(ant.nodeId,adjacency);
            
            %First Heuristic - Equation 2 Page 1239
            n11 = sparse(1,max(J));
            n12 = sparse(1,max(J));
            for j = J
               n11(j) = min(1, (C1max-dist(i,j)) /(C1Max-C1Min) + epsilon);
               n12(j) = min(1, (C2max-crime(i,j))/(C2Max-C2Min) + epsilon);    
            end
            
            nextIndex = -1;
            
            %Transition Event - Equation 5 p. 1240
            if q <= q0
                %%For each of the edges find argmax over J
                max = -1;

                for j = J
                    pij = (Tau1(i,j)*n11(i,j)).^(gamma) * (Tau2(i,j)*n12(i,j)).^(1-gamma) * n2(j);
                    if pij > max
                       max = pij;
                       nextIndex = j;
                    end
                end
            else
                %%Find Pij for each of the edges using this equation:
                if h <= a
                    gamma = 0;
                elseif a < h < b
                    gamma = h/(b-a) - a/(b-a);
                else
                    gamma = 1;
                end
                pij = sparse(1,max(J));
                summedOver = 0;
                
                %Caclulating the probabilities based on the costs, the
                %visibility, and the weight (gamma)
                for j = J
                    pij(j) = (Tau1(i,j)*n11(i,j)).^(gamma) * (Tau2(i,j)*n12(i,j)).^(1-gamma) * n2(j);
                    summedOver = summedOver + pij(j);
                end
                %Normalizing the exit probabilities
                for j = J
                    pij(j) = pij/summedOver;
                end
                %Find the 
                p = rand();
                nextIndex = find(pij > 0.5, 1);
            end
            
            %Equation 2 - first movement heuristic
            ant(h).path(end+1) = nextIndex;
            ant(h).nodeId = nextIndex;
        end
        %Local Pheromone update - serially
        for h = 1:H
            for i = 2:length(ant(h).path)
                Tau1 = Tau1*localUpdateEvapRate;
                Tau2 = Tau2*localUpdateEvapRate;

                Tau1(ant.nodeId,nextIndex) = Tau2(ant.nodeId,nextIndex) + gammaTau / Tau1(ant.nodeId,nextIndex);
                Tau2(ant.nodeId,nextIndex) = Tau2(ant.nodeId,nextIndex) + gammaTau / Tau2(ant.nodeId,nextIndex);
            end
        end
    end
    
    
    %if ants didn't generate a solution, repeat
    if ~solution
        
    
    else%perform a global update
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
    end
    
    
end



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
function ids = findAdjacent(node,dg)
[~,ids,~] = find(dg(node.id,:));
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
