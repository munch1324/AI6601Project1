function [parsed_osm, dg, dist, crime, crime_node] = osm2matlab(var)
% Prepare Connectivity, Distance, Crime Cost Matrices
% parsed_osm = Data convereted from the osm xml file to matlab struct
% dg(i,j) = Connectivity of node i to node j 
% dist(i,j) = Distance of edge(i->j)
% crime(i,j) = Crime intensity of edge(i->j) 
% crime_node(i) = Crime value at node i
% var = variance for generating the crime map and values

if exist('osm2matlab.mat','file') ~= 2
    
    % OSM to matlab
    addpath(genpath(fullfile(pwd,'openstreetmap')));
    
    openstreetmap_filename = 'atlanta.osm';
    map_osm = load_osm_xml(openstreetmap_filename);
    osm_xml = map_osm.osm;
    parsed_osm = parse_osm(osm_xml);
    [connectivity_matrix, ~] = extract_connectivity(parsed_osm);
    dg = or(connectivity_matrix, connectivity_matrix.');
    [n,m] = size(dg);
    
    % Create Distance Matrix
    [dim,~] = size(dg);
    nodexy = parsed_osm.node.xy';
    tmp = zeros(nnz(dg),3);
    
    t = 1;
    for node = 1 : dim
        ind = sparse2ind(dg,node);
        for j = 1 : length(ind)
            tmp(t,:) = [node ind(j) sqrt(sum((nodexy(node,:)-nodexy(ind(j),:)).^2))];
            t = t + 1;
        end
    end
    
    dist = sparse(tmp(:,1),tmp(:,2),tmp(:,3),n,m,nnz(dg));
    
    % Create Crime Cost Matrix
    load('Crime.mat');
    
    crime_node = zeros(size(nodexy,1),1);
    for i = 1 : size(nodexy,1)
        crime_node(i,1) = sum(mvnpdf(rank_lat_long(:,3:-1:2),nodexy(i,:),var.*eye(2)));
    end
    
    tmp = zeros(nnz(dg),3);
    t = 1;
    for node = 1 : dim
        ind = sparse2ind(dg,node);
        for j = 1 : length(ind)
            tmp(t,:) = [node ind(j) 0.5*(crime_node(node)+crime_node(j))];
            t = t + 1;
        end
    end
    
    crime = sparse(tmp(:,1),tmp(:,2),tmp(:,3),n,m,nnz(dg));
    
    % Save data
    clearvars -except parsed_osm dg dist crime_node crime
    save('osm2matlab.mat')
else
    load('osm2matlab.mat')
end

end

%% Find the non-zero indices of the row of a sparse matrix
function ind = sparse2ind(mat, row)
[~,ind,~] = find(mat(row,:));
end