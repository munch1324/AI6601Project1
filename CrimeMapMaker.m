function [ output_args ] = CrimeMapMaker( input_args )
%CRIMEMAPMAKER Summary of this function goes here
%   Detailed explanation goes here
open Map.mat
ixy = ans.ixy;
open Crime.mat
rank_lat_long = ans.rank_lat_long;

ixy(:,3) = zeros(length(ixy(:,1)),1);
V = [0.00001, 0.0001, 0.001];
for v = V
    
    filename = sprintf('Crime_%f.mat', v);
    for i = 1:length(rank_lat_long(:,1))
        for j = 1:length(ixy(:,1))
            ixy(j,3) = ixy(j,3) + sum(normpdf( ixy(j,2:-1:1), rank_lat_long(i,2:3), [v, v]));
        end
    end
    save(filename, 'ixy');
end

axis xy
scatter3(ixy(:,1), ixy(:,2), ixy(:,3), 0.2, ixy(:,3));
 %mesh(ixy(:,1), ixy(:,2), ixy(:,3)), hold on, plot3(x,y,z,'o'), hold off

end

