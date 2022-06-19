function G = RRT(startpos, endpos, boundary, iter_n, stepSize, dim, obst_num, obst_r, showPlot)

%%***********************************************************************%
%*                    Rapidly-Exploring Random Tree                     *%
%*           Generates a rapidly-exploring random tree graph            *%
%*                                                                      *%
%*                                                                      *%
%* Authors: Dr. Preetham Manjunatha & Zhenghao Li                       *%
%* Github link: https://github.com/preethamam                           *%
%* Date: 05/14/2022                                                     *%
%************************************************************************%
%
%************************************************************************%
%
% Usage: G = RRT(startpos, endpos, boundary, iter_n, stepSize, dim, obst_num, obst_r, show_graph)
%
% Inputs:
%
%           startpos            - Coordinate of the starting point [x y (z)]
%           endpos              - Coordinate of the ending point [x y (z)]
%           boundary            - The boundary of the graph 
%                                 [x_min x_max y_min y_max z_min z_max]
%           iter_n              - Number of iteration (number of points)
%           stepSize            - Step size between new points and the
%                                 nearest neighbor
%           dim                 - Dimension of the graph. 2D or 3D
%           obst_num            - The number of obstacles in the graph
%           obst_r              - The obstacles' radius. If 0 then the
%                                 radius will be random 
%           showPlot            - 1 to plot the graph, 0 to skip
%
%  
% Outputs: 
%
%           G                   - Graph object of a Rapidly-Exploring Random Tree
% 
%
%--------------------------------------------------------------------------
% Example 1: 2D tree graph with 2 obstcacles with 25 points
% startpos = [0 0]
% endpos = [5 5]
% boundary = [-5 5 -5 5]
% iter_n = 25
% stepSize = 0.7
% dim = 2
% obst_num = 2
% obst_r = 0
% show_graph = 1
% G = RRT(startpos, endpos, boundary, iter_n, stepSize, dim, obst_num, obst_r, show_graph)
%
% Example 2: 3D tree graph with 2 obstcacles with 25 points
% startpos = [0 0 0]
% endpos = [5 5 5]
% boundary = [-5 5 -5 5 -5 5]
% iter_n = 25
% stepSize = 0.7
% dim = 3
% obst_num = 2
% obst_r = 0
% show_graph = 1
% G = RRT(startpos, endpos, boundary, iter_n, stepSize, dim, obst_num, obst_r, show_graph)
%
%​
%------------------------------------------------------------------------------------------------------------------------   
% nargin check
if nargin < 8
    error('Not enough input arguments.');
elseif nargin > 9
    error('Too many input arguments.');
end

if nargin == 8
    %-----------------------
    % Weight type
    showPlot = 0;
end

% check dimension consistency
if length(startpos) ~= dim
    error('startpos dimension should be the same as dim')
end

if length(endpos) ~= dim
    error('endpos dimension should be the same as dim')
end

if length(boundary) ~= dim * 2
    error('boundary dimension should be the same as dim')
end


%------------------------------------------------------------------------------------------------------------------------
% initiate the coordinates of all nodes in the graph.
s = [1];
t = [1];

switch dim
    case 2
        coor = [startpos(1); startpos(2)];
        sx = endpos(1) - startpos(1);
        yx = endpos(2) - startpos(2);
    case 3
        coor = [startpos(1); startpos(2); startpos(3)];
        sx = endpos(1) - startpos(1);
        yx = endpos(2) - startpos(2);
        zx = endpos(3) - startpos(3);
end

% initiate add_end as zero, if end point not added during graph
% generating, add_end will remain 0 and the end point will be added to
% the graph eventually
add_end = 0;

% initiate obstacles
obst = zeros(obst_num, 2*dim);
rad = 0;

% generate obstacles
if obst_num > 0
    for i = 1 : obst_num
        temp = rand(1, dim + 1);
        temp(1) = randi([startpos(1)+round(sx*0.2) endpos(1)-round(sx*0.2)]);
        temp(2) = randi([startpos(2)+round(yx*0.2) endpos(2)-round(yx*0.2)]);
        if dim == 3
            temp(3) = randi([startpos(3)+round(zx*0.2) endpos(3)-round(zx*0.2)]);
        end
        temp(dim+1) = stepSize+rand(1)*0.2*min(sx,yx);
        for d = 1 : dim
            if obst_r > 0
                obst(i,(d-1)*2+1) = temp(d) - obst_r;
                obst(i,(d-1)*2+2) = temp(d) + obst_r;
                rad = obst_r;
            else
                obst(i,(d-1)*2+1) = temp(d) - temp(dim+1);
                obst(i,(d-1)*2+2) = temp(d) + temp(dim+1);
                rad = temp(dim+1);
            end
        end
    end
end

% main loop
iter = 1;
for i = 1 : iter_n * 5
    if iter == iter_n
        break
    end
    % generate a randome point withing boundary
    switch dim
        case 2
            randnode = rand(1,2);
            randnode(1) = startpos(1) - sx/2 + randnode(1)*sx*2;
            randnode(2) = startpos(2) - yx/2 + randnode(2)*yx*2;
        case 3
            randnode = rand(1,3);
            randnode(1) = startpos(1) - sx/2 + randnode(1)*sx*2;
            randnode(2) = startpos(2) - yx/2 + randnode(2)*yx*2;
            randnode(3) = startpos(3) - zx/2 + randnode(3)*zx*2;
    end

    % Use KDTree Search to get the nearest point to the random point
    MdlKDT = KDTreeSearcher(coor');
    IdxKDT = rangesearch(MdlKDT, randnode, max([boundary(2)-boundary(1) boundary(4)-boundary(3)]));

    % if no nodes near by, continue to next iteration
    if isempty(IdxKDT{1})
        continue
    end

    % calculate coordinates for next point
    dirn = randnode' - coor(:,IdxKDT{1}(1));
    len = norm(dirn, 'fro');
    dirn = (dirn / len) * min(stepSize,len);
    newnode = coor(:,IdxKDT{1}(1)) + dirn;

    % Ignore the point if it lands out side the boundary
    if dim == 2
        if newnode(1) < boundary(1) || newnode(1) > boundary(2) || newnode(2) < boundary(3) || newnode(2) > boundary(4)
            continue
        end
    elseif dim == 3
        if newnode(1) < boundary(1) || newnode(1) > boundary(2) || newnode(2) < boundary(3) || newnode(2) > boundary(4) || newnode(3) < boundary(5) || newnode(3) > boundary(6)
            continue
        end
    end

    % Ignore the point if it lands in any of the obsticals
    in_obst = 0;
    for k = 1 : obst_num
        if dim == 2
            if newnode(1) > obst(k,1) && newnode(1) < obst(k,2) && newnode(2) > obst(k,3) && newnode(2) < obst(k,4)
                in_obst = 1;
                break
            end
        elseif dim == 3
            if newnode(1) > obst(k,1) && newnode(1) < obst(k,2) && newnode(2) > obst(k,3) && newnode(2) < obst(k,4) && newnode(3) > obst(k,5) && newnode(3) < obst(k,6)
                in_obst = 1;
                break
            end
        end
    end
    if in_obst == 1 
        continue
    end

    % adding new node to the coordinates
    new_node_idx = size(coor,2) + 1;
    coor(:,new_node_idx) = newnode;

    % adding an edge from new node to nearest neighbor in the graph
    s = [s IdxKDT{1}(1)];
    t = [t new_node_idx];
    iter = iter + 1;

    % check if the new node is close to the endpos
    % if so, add an edge from new node to end pos
    dist = sqrt(sum((newnode-endpos').^2));
    if dist < stepSize * 1.5
        add_end = 1;
        coor(:,size(coor,2)+1) = endpos';
        s = [s new_node_idx];
        t = [t size(coor,2)];
        iter = iter + 1;
    end
end

% if the end point is not added, add it to the graph
if ~add_end
    coor(:,size(coor,2)+1) = endpos';
    s = [s size(coor,2)];
    t = [t size(coor,2)];
end

% Build the final random tree graph
G = graph(s, t, ones(1, length(s)), 'omitselfloops');
G.Nodes.XData = coor(1,:)';
G.Nodes.YData = coor(2,:)';
if dim == 3
    G.Nodes.ZData = coor(3,:)';
end

%------------------------------------------------------------------------------------------------------------------------
% plot the graph if show_graph is true
if showPlot && dim == 2
    figure;
    p = plot(G,'XData',coor(1,:),'YData',coor(2,:));
    p.MarkerSize = 4;
    hold on
    % plot the obsticals
    for i = 1 : obst_num
        %rad = obst(i, 2) - obst(i, 1);
        rectangle('Position',[obst(i, 1) obst(i, 3) rad rad],'Curvature',1,'FaceColor', 'r')
    end
    hold off
    title('Rapidly Exploring Random Tree (RRT)')
    xlabel('X'); ylabel('Y');

elseif showPlot && dim == 3
    figure;
    p = plot(G,'XData',coor(1,:),'YData',coor(2,:), 'ZData',coor(3,:));
    p.MarkerSize = 4;
    hold on
    for i = 1 : obst_num
        %rad = obst(i, 2) - obst(i, 1);
        [x,y,z] = sphere;
        x = x * rad;
        y = y * rad;
        z = z * rad;
        h = surf(x + obst(i, 1) + rad,y + obst(i, 3) + rad,z + obst(i, 5) + rad);
        set(h, 'FaceAlpha', 0.5, 'FaceColor', 'r')
    end
    hold off
    title('Rapidly Exploring Random Tree (RRT)')
    xlabel('X'); ylabel('Y');
end

end

