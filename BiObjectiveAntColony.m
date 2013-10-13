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


%First  Pheromone Matrix
Tau1 = sparse(N,N);
%Second Pheromone Matrix
Tau2 = sparse(N,N);

%Outlining paper steps (Fig 2 - pg 1241)

%Compute Second Heuristic Parameter

%Create a Colony


%Continue until end condition is satisfied
iii = 1;
iii = iii + 1; 
endConditionMet = false;
while ~endConditionMet
    
    %For each ant
    ants = [1:H];
    for h = ants;
        
        %until you reach the sink
        while 1 == 1
            %Transition Event - Equation 5 p. 1240
            if q <= q0
                %%TODO: for each of the edges find argmax over J
                n11 = min(1, (C1max-dist(i,j)) /(C1Max-C1Min) + epsilon);
                n12 = min(1, (C2max-crime(i,j))/(C2Max-C2Min) + epsilon);    
            else
                %%TODO: find Pij for each of the edges using this equation:
                
                if h <= a
                    gamma = 0
                elseif a < h < b
                    gamma = h/(b-a) - a/(b-a);
                else
                    gamma = 1;
                end
                %sum over all j
                summedOver = (Tau1(i,j)*n1(i,j)).^(gamma) * (Tau2(i,j)*n2(i,j)).^(1-gamma) * n2(j);
                
                (Tau1(i,j)*n1(i,j)).^(gamma) * (Tau2(i,j)*n2(i,j)).^(1-gamma) * n2(j) / summedOver;
            end
            
            %Equation 2 - first movement heuristic
            i = 1;
            j = 1;
            localUpdateEvapRate;
            
        
        
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
                Tau1 = min(1,Tau1(r,c)+ delta/dist(r,c));
            end
        end
        for r=1:length(Tau2(:,1))
            for c=1:length(Tau2(1,:))
                Tau2 = min(1,Tau2(r,c)+ delta/crime(r,c));
            end
        end
    end
    
    
end



end

