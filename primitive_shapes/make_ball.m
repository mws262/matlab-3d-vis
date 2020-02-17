function [ball_patch, ball_verts_untransformed] = make_ball(radius)
%MAKE_BALL Make the standard soccer ball visuals.
%   It is centered around the origin initially, but moved upward by its
%   radius. It is added to the current axis.
%   Patch object is returned, along with the initial vertices for later
%   transforming relative to the origin.
%
%   MAKE_BALL(radius)
%
%   Inputs:
%       `radius` -- Radius of the ball in meters.
%   Outputs: <none>
%
%   See also MAKE_VISUALIZER_SCENE, PATCH.
%

validateattributes(radius, {'single', 'double'}, {'real', 'positive', 'scalar'});

[sphere_x,sphere_y,sphere_z] = sphere(25);
ball_patch = patch(surf2patch(radius * sphere_x, radius * sphere_y, radius * sphere_z, 100*radius * sphere_z));
ball_verts_untransformed = ball_patch.Vertices;
ball_patch.Vertices = ball_verts_untransformed + [0, 0, radius];

ball_patch.FaceColor = 'interp';
ball_patch.EdgeAlpha = 0.0;
ball_patch.FaceAlpha = 1;

cmap = flag(size(ball_patch.Vertices,1));
ball_patch.FaceVertexCData = cmap;

end

