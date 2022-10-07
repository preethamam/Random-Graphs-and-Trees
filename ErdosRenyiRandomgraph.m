function [G,n,m] = ErdosRenyiRandomgraph(n, p, X, Y, showPlot)

%%***********************************************************************%
%*                        Erdos-Renyi random graph                      *%
%*           Generates a Erdos-Renyi random graph by probability        *%
%*                                                                      *%
%*                                                                      *%
%* Author: Preetham Manjunatha                                          *%
%* Github link: https://github.com/preethamam                           *%
%* Date: 05/14/2022                                                     *%
%************************************************************************%
%
%************************************************************************%
%
% Usage: [G,n,m]                = ErdosRenyiRandomgraph(n,p)
%  
% Inputs:
%
%           n                   - Graph size, number of vertexes, |V|
%           p                   - Probability, p, to connect vertices of Erdos-Renyi model      
%
%
% 
% Outputs: 
%
%           G                   - Graph output
%           n                   - Number of vertexes, |V|
%           m                   - Number of edges, |E|
%
%
%--------------------------------------------------------------------------
% Example 1: Similarity weights
% n = 100;
% p = 0.25;
% [G,n,m] = ErdosRenyiRandomgraph(n,p);
%
% Example 2: Average weights
% n = 10000;
% p = 0.05;
% [G,n,m] = ErdosRenyiRandomgraph(n,p);
%
% Example 3: Dissimilarity weights
% n = 1000;
% p = 0.15;
% [G,n,m] = ErdosRenyiRandomgraph(n,p);


%------------------------------------------------------------------------------------------------------------------------
% nargin check
if nargin < 4
    error('Not enough input arguments.');
elseif nargin > 5
    error('Too many input arguments.');
end

%------------------------------------------------------------------------------------------------------------------------
% Generate random graph
G = spones(triu(sprand(n,n,p),1));
if nargout>2
    m = nnz(G);
end

%------------------------------------------------------------------------------------------------------------------------
% Generate random graph with symmetry
G = G + G';

G = graph(G);
G.Nodes.XData = X';
G.Nodes.YData = Y';

% Plot the graph
if (showPlot)    
    figure;
    p = plot(G,'XData',X,'YData',Y);
    p.MarkerSize = 3;
    title('Erdos-Renyi Graph')
    xlabel('X'); ylabel('Y');
end
end