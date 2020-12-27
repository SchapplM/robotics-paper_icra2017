function hdl = plot_cube2(origin,X,Y,Z,color)
% CUBE_PLOT plots a cube with dimension of X, Y, Z.
%
% INPUTS:
% origin = set origin point for the cube in the form of [x,y,z].
% X      = cube length along x direction.
% Y      = cube length along y direction.
% Z      = cube length along z direction.
% color  = STRING, the color patched for the cube.
%         List of colors
%         b blue
%         g green
%         r red
%         c cyan
%         m magenta
%         y yellow
%         k black
%         w white
% OUPUTS:
% Plot a figure in the form of cubics.
%
% EXAMPLES
% cube_plot(2,3,4,'red')
%

% ------------------------------Code Starts Here------------------------------ %
% Define the vertexes of the unit cubic

ver = [X'+Y';
    Y';
    Y'+Z';
    X'+Y'+Z';
    Z';
    X'+Z';
    X';
    0 0 0];

%  Define the faces of the unit cubic
fac = [1 2 3 4;
    4 3 5 6;
    6 7 8 5;
    1 2 8 7;
    6 7 1 4;
    2 3 5 8];

cube = [ver(:,1)+origin(1),ver(:,2)+origin(2),ver(:,3)+origin(3)];
hdl=patch('Faces',fac,'Vertices',cube,'FaceColor',color);
set(hdl,'facealpha',0.3);
end