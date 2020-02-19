function scene_fig = make_visualizer_scene()
% MAKE_VISUALIZER_SCENE Construct the a blank world scene containing the
% ground, sky, and basic keyboard/mouse interaction for camera motion.
%
%   scene_fig = MAKE_VISUALIZER_SCENE()
%
%   Inputs: <none>
%
%   Outputs:
%       scene_fig -- Figure containing all the graphics objects.
%
%   See also MAKE_BALL, MAKE_CYLINDER, MOUSE_DOWN_CALLBACK,
%   MOUSE_UP_CALLBACK, MOUSE_MOTION_CALLBACK, KEY_PRESS_CALLBACK,
%   KEY_RELEASE_CALLBACK.
%

scene_fig = figure(1);
scene_fig.NumberTitle = 'off';
scene_fig.Name = 'Matt''s Visualizer';
scene_fig.ToolBar = 'none';
scene_fig.MenuBar = 'none';
colormap(winter);
hold on;

% Floor plane representing surface that the ball is rolling on.
[floor_x, floor_y] = meshgrid(-10:0.5:10); % Generate x and y data
[floor_z] = zeros(size(floor_x, 1)); % Generate z data
floor_patch = patch(surf2patch(floor_x, floor_y, floor_z));
floor_patch.FaceColor = [0.8,0.8,0.6];
floor_patch.EdgeAlpha = 0.2;
floor_patch.FaceAlpha = 1;
floor_patch.SpecularStrength = 0.2;
floor_patch.DiffuseStrength = 0.9;

% Sky sphere. From my MatlabPlaneGraphics example set.
skymap = [linspace(1,0.8,10)',linspace(1,0.8,10)',linspace(1,0.95,10)'];

[skyX,skyY,skyZ] = sphere(100);
sky = surf(200*skyX,200*skyY,200*skyZ,'LineStyle','none','FaceColor','interp');
sky.AmbientStrength = 0.8;
colormap(skymap);

axis([-3, 3, -3, 3]);
daspect([1,1,1]);
ax = scene_fig.Children;

ax.Projection = 'perspective';
ax.Clipping = 'off';
ax.Visible = 'off';
scene_fig.Position = [0, 0, 1500, 975];
ax.CameraPosition = [-2.4, -1.6, 1.8];
ax.CameraTarget = [0, 0, 0];
scene_fig.WindowKeyPressFcn = @key_press_callback;
scene_fig.WindowKeyReleaseFcn = @key_release_callback;
scene_fig.WindowScrollWheelFcn = @mousewheel_callback;
scene_fig.WindowButtonDownFcn = @mouse_down_callback;
scene_fig.WindowButtonUpFcn = @mouse_up_callback;
camva(40);
light1 = light();
light1.Position = [10,10,40];
light1.Style = 'infinite';

shift_down = false;
previous_pos = [0, 0, 0];

    function mouse_down_callback(src,dat)
    % MOUSE_DOWN_CALLBACK Function which gets called automatically when
    % assigned as a callback for the visualizer. Triggered on mouse buttons
    % being pressed. Starts listening for dragging motion by assigning
    % MOUSE_MOTION_CALLBACK as a mouse motion callback.
    %
    %   MOUSE_DOWN_CALLBACK(src, dat)
    %
    %   Inputs:
    %       `src` -- Which graphics object triggered this callback.
    %       `dat` -- Data structure with information about the mouse event.
    %
    %   Outputs: <none>
    %
    %   See also MOUSE_MOTION_CALLBACK, MOUSE_UP_CALLBACK, MOUSE_DOWN_CALLBACK,
    %   KEY_CALLBACK, KEY_RELEASE_CALLBACK, PATCHFACEFCN, MAKE_VISUALIZER_SCENE.
    %

    if strcmp(dat.EventName, 'WindowMousePress')
        previous_pos = src.CurrentPoint;
        src.WindowButtonMotionFcn = @mouse_motion_callback;
    end
    end

    function mouse_up_callback(src,dat)
    % MOUSE_UP_CALLBACK Function which gets called automatically when
    % assigned as a callback for the visualizer. Triggered on mouse buttons
    % being released. This terminates any dragging interactions which might be
    % occuring by unassigning the WindowButtonMotionFcn.
    %
    %   MOUSE_UP_CALLBACK(src, dat)
    %
    %   Inputs:
    %       `src` -- Which graphics object triggered this callback.
    %       `dat` -- Data structure with information about the mouse event.
    %
    %   Outputs: <none>
    %
    %   See also MOUSE_MOTION_CALLBACK, MOUSE_UP_CALLBACK, MOUSE_DOWN_CALLBACK,
    %   KEY_CALLBACK, KEY_RELEASE_CALLBACK, PATCHFACEFCN, MAKE_VISUALIZER_SCENE.
    %

    if strcmp(dat.EventName, 'WindowMouseRelease')
        src.WindowButtonMotionFcn = '';
    end
    end

    function mouse_motion_callback(src, dat)
    % MOUSE_MOTION_CALLBACK Function which gets called automatically when
    % assigned as a callback for the visualizer. Triggered on mouse motion.
    % This callback is turned off when mouse buttons are not pressed. When
    % mouse is dragged, orbits camera around the camera target. If shift is
    % pressed, translates both camera and camera target relative to the plane
    % of the ground.
    %
    %   MOUSE_MOTION_CALLBACK(src, dat)
    %
    %   Inputs:
    %       `src` -- Which graphics object triggered this callback.
    %       `dat` -- Data structure with information about the mouse event.
    %
    %   Outputs: <none>
    %
    %   See also MOUSE_MOTION_CALLBACK, MOUSE_UP_CALLBACK, MOUSE_DOWN_CALLBACK,
    %   KEY_CALLBACK, KEY_RELEASE_CALLBACK, PATCHFACEFCN, MAKE_VISUALIZER_SCENE.
    %

    if isempty(previous_pos)
        previous_pos = src.CurrentPoint;
    end
    current_pos = src.CurrentPoint; % Capture current mouse position in screen coords.
    mouse_diff = current_pos - previous_pos;
    mouse_diffX = mouse_diff(1);
    mouse_diffY = mouse_diff(2);
    camvec = ax.CameraPosition - ax.CameraTarget;
    if shift_down
        forwardvec = camvec .* [1,1,0];
        forwardvec = forwardvec / norm(forwardvec);
        rightvec = cross(camvec, [0, 0, 1]);
        forwarddiff = 0.003 * mouse_diffY * forwardvec;
        rightdiff = 0.001 * mouse_diffX * rightvec;
        ax.CameraTarget = ax.CameraTarget + forwarddiff + rightdiff;
        ax.CameraPosition = ax.CameraPosition + forwarddiff + rightdiff;
    else
        ax.CameraPosition = (angle2dcm(0, 0, 0.006*mouse_diffX, 'xyz') * camvec')' + ax.CameraTarget;
        camvec = ax.CameraPosition - ax.CameraTarget;
        sidevec = cross([0,0,1], camvec);
        if mouse_diffY > 0 || dot(camvec, camvec) - camvec(3)^2 > 0.2 % Avoid getting in gimbal lock range.
            twirl_rot = axang2rotm([sidevec, mouse_diffY * 0.003]);
            ax.CameraPosition = (twirl_rot * camvec')' + ax.CameraTarget;
        end
        if ax.CameraPosition(3) < 0.1
            ax.CameraPosition(3) = 0.1;
        end
    end
    previous_pos = current_pos;
    end

    function mousewheel_callback( src, dat )
    % MOUSEWHEEL_CALLBACK Function which gets called automatically when
    % assigned as a callback for the visualizer. Triggered on mouse scroll
    % wheel. Handles zooming in/out in the direction of the camera target along
    % the line between the camera and the camera target.
    %
    %   MOUSEWHEEL_CALLBACK(src, dat)
    %
    %   Inputs:
    %       `src` -- Which graphics object triggered this callback.
    %       `dat` -- Data structure with information about the mouse wheel
    %       event.
    %
    %   Outputs: <none>
    %
    %   See also MOUSE_MOTION_CALLBACK, MOUSE_UP_CALLBACK, MOUSE_DOWN_CALLBACK,
    %   KEY_CALLBACK, KEY_RELEASE_CALLBACK, PATCHFACEFCN, MAKE_VISUALIZER_SCENE.
    %

    delta = 0.25; % Multiplier for zooming. Hand-tuned.

    curr_cam_pos = ax.CameraPosition;
    curr_cam_tar = ax.CameraTarget;

    cam_vec = curr_cam_tar - curr_cam_pos;

    if dat.VerticalScrollCount > 0 % Zoom out
        ax.CameraPosition = curr_cam_pos - cam_vec/norm(cam_vec)*delta;
    elseif dat.VerticalScrollCount < 0 && dot(cam_vec, cam_vec) > 1 % Zoom in, but only if we aren't too close to the target.
        ax.CameraPosition = curr_cam_pos + cam_vec/norm(cam_vec)*delta;
    end
    end

    function key_press_callback( src, dat )
    % KEY_PRESS_CALLBACK Function which gets called automatically when
    % assigned as a callback for the visualizer. Triggered on a keyboard key
    % being pressed down. Handles doing camera orbits if shift is not pressed.
    % Arrow keys trigger the motion. Does camera and camera target shifting
    % (panning) if shift is pressed. Works closely with key_release_callback.
    %
    %   KEY_PRESS_CALLBACK(src, dat)
    %
    %   Inputs:
    %       `src` -- Which graphics object triggered this callback.
    %       `dat` -- Data structure with information about the keyboard event.
    %
    %   Outputs: <none>
    %
    %   See also MOUSE_MOTION_CALLBACK, MOUSE_UP_CALLBACK, MOUSE_DOWN_CALLBACK,
    %   KEY_CALLBACK, KEY_RELEASE_CALLBACK, PATCHFACEFCN, MAKE_VISUALIZER_SCENE.
    %

    turn_delta = 0.25; % Multiplier for tilting.
    trans_delta = 0.1; % Multiplier for panning.

    curr_cam_pos = ax.CameraPosition;
    curr_cam_tar = ax.CameraTarget;
    cam_vec = curr_cam_tar - curr_cam_pos;
    cam_up = cross(cross(cam_vec, [0, 0, 1]), cam_vec);

    shift_down = ~isempty(dat.Modifier) && strcmp(dat.Modifier{1}, 'shift');
    switch dat.Key
        case 'uparrow'
            if shift_down % Pan forwards.
                forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
                forward_vec = forward_vec/norm(forward_vec);
                ax.CameraPosition = curr_cam_pos + trans_delta * forward_vec;
                ax.CameraTarget = curr_cam_tar + trans_delta * forward_vec;
            else % Tilt upwards.
                ax.CameraPosition = curr_cam_pos + cam_up/(norm(cam_up) + eps) * turn_delta;
            end
        case 'downarrow'
            if shift_down % Pan backwards.
                forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
                forward_vec = forward_vec/norm(forward_vec);
                ax.CameraPosition = curr_cam_pos - trans_delta * forward_vec;
                ax.CameraTarget = curr_cam_tar - trans_delta * forward_vec;
            else % Tilt downwards.
                if curr_cam_pos(3) > 0.2 % No ground-penetrating cameras
                    ax.CameraPosition = curr_cam_pos - cam_up/(norm(cam_up) + eps) * turn_delta;
                end
            end
        case 'rightarrow'
            if shift_down % Pan right.
                forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
                forward_vec = forward_vec/norm(forward_vec);
                right_vec = cross(forward_vec, [0, 0, 1]);
                ax.CameraPosition = curr_cam_pos + trans_delta * right_vec;
                ax.CameraTarget = curr_cam_tar + trans_delta * right_vec;
            else % Tilt right.
                cam_r = cross(cam_vec, cam_up);
                ax.CameraPosition = curr_cam_pos + cam_r/(norm(cam_r) + eps) * turn_delta;
            end
        case 'leftarrow'
            if shift_down % Pan left.
                forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
                forward_vec = forward_vec/norm(forward_vec);
                right_vec = cross(forward_vec, [0, 0, 1]);
                ax.CameraPosition = curr_cam_pos - trans_delta * right_vec;
                ax.CameraTarget = curr_cam_tar - trans_delta * right_vec;
            else % Tilt left.
                cam_r = cross(cam_vec, cam_up);
                ax.CameraPosition = curr_cam_pos - cam_r/(norm(cam_r) + eps) * turn_delta;
            end
        otherwise
            % Other events maybe added in the future.
    end
    end

    function key_release_callback(src,dat)
    % KEY_RELEASE_CALLBACK Function which gets called automatically when
    % assigned as a callback for the visualizer. Triggered on a keyboard key
    % being released. Currently only listens for modifier keys being released.
    %
    %   KEY_RELEASE_CALLBACK(src, dat)
    %
    %   Inputs:
    %       `src` -- Which graphics object triggered this callback.
    %       `dat` -- Data structure with information about the keyboard event.
    %
    %   Outputs: <none>
    %
    %   See also MOUSE_MOTION_CALLBACK, MOUSE_UP_CALLBACK, MOUSE_DOWN_CALLBACK,
    %   KEY_CALLBACK, KEY_RELEASE_CALLBACK, PATCHFACEFCN, MAKE_VISUALIZER_SCENE.
    %

    if strcmp(dat.Key, 'shift')
        shift_down = false;
    end
    end

end