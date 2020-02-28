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
scene_fig.Visible = false;
scene_fig.NumberTitle = 'off';
scene_fig.Name = 'Matt''s Visualizer';
scene_fig.ToolBar = 'none';
scene_fig.MenuBar = 'none';
colormap(winter);
hold on;

axis([-3, 3, -3, 3]);
daspect([1,1,1]);
ax = scene_fig.Children;

ax.Projection = 'perspective';
ax.Clipping = 'off';
ax.Visible = 'off';
scene_fig.Position = [0, 0, 1500, 975];
ax.CameraPosition = [-2.4, -1.6, 1.8];
ax.CameraTarget = [0, 0, 0];

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
skymap = [linspace(1,0.8,200)',linspace(1,0.85,200)',linspace(1,0.95,200)'];

[skyX,skyY,skyZ] = sphere(4);
sky = surf(50*skyX,50*skyY,50*skyZ,'LineStyle','none','FaceColor','interp');
sky.FaceLighting = 'gouraud';
sky.AmbientStrength = 0.8;
colormap(skymap);

makeCoordinateFrame();
scene_fig.WindowKeyPressFcn = @key_press_callback;
scene_fig.WindowKeyReleaseFcn = @key_release_callback;
scene_fig.WindowScrollWheelFcn = @mousewheel_callback;
scene_fig.WindowButtonDownFcn = @mouse_down_callback;
scene_fig.WindowButtonUpFcn = @mouse_up_callback;
camva(40);
light1 = light();
light1.Position = [0,0,20];
light1.Style = 'infinite';
% light1.Color = [0.1, 0.1, 0.1];


% light2 = light();
% light2.Position = [10,-10,5];
% light2.Style = 'local';
% light2.Color = [0.1, 0.1, 0.1];

shift_down = false;
previous_pos = [0, 0, 0];

scene_fig.Visible = true;
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
            rotvec = cross([0, 0, 1], camvec / norm(camvec)) * mouse_diffY  + [0, 0, -mouse_diffX];
            rotvec = rotvec / norm(rotvec);
            ang = sqrt((mouse_diffY * 0.003) ^ 2 + (0.006 * mouse_diffX) ^ 2);
            
            camvec = cos(ang) * camvec + sin(ang) * cross(rotvec, camvec) + (1 - cos(ang)) * dot(rotvec, camvec) * rotvec;

            if dot(camvec, camvec) - camvec(3)^2 > 0.2 % Avoid getting in gimbal lock range.
                ax.CameraPosition = camvec + ax.CameraTarget;
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

%     function rotm = axangToRotm(ax, ang)
%         % AXANGTOROTM Converts an axis-angle parameterized rotation to a
%         % rotation matix. Eliminates the one dependency on the robotics
%         % toolbox.
%         
%         cang = cos(ang);
%         skewmat = [0, -ax(3), ax(2); ax(3), 0, -ax(1); -ax(2), ax(1), 0];
%         rotm = cang * eye(3) + sin(ang) * skewmat + (1 - cang) * kron(ang, ang');
%         norm(rotm)
%         rotm = rotm / norm(rotm);
% %         [u s vt] = svd(rotm);
% %         rotm = u * vt';
%     end

    function makeCoordinateFrame()
        plot3(0, 0, 0, '.b', 'MarkerSize', 25);
        quiver3(0, 0, 0, 0.2, 0, 0, 'LineWidth', 4, 'Color', 'r');
        quiver3(0, 0, 0, 0, 0.2, 0, 'LineWidth', 4, 'Color', 'g');
        quiver3(0, 0, 0, 0, 0, 0.2, 'LineWidth', 4, 'Color', 'b');
        text(0.22, 0, 0, 'x', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        text(0, 0.22, 0, 'y', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        text(0, 0, 0.22, 'z', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end
end