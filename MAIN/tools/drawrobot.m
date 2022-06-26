function h = drawrobot(varargin)

pose  = varargin{1};
radius = varargin{2};

x = pose(1); y = pose(2); theta = pose(3);
T = [x; y];
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

% Draw circular contour
h1 = drawellipse(pose,radius,radius,[0.5 0.5 0.5]);
% Draw line with orientation theta with length radius
p = R*[radius;0] + T;
h2 = plot([T(1) p(1)],[T(2) p(2)],'Color',[0 0.75 1],'linewidth',2);
h = cat(1,h1,h2);

end
