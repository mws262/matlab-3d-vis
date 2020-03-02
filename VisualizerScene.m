classdef VisualizerScene < handle
    %VISUALIZERSCENE Class version of make_visualizer_scene function.
    %   Better for expanding the functionality of the basic graphics world
    %   by subclassing this.
    
    properties
        Figure % Figure object containing the axes, callbacks, etc.
        Axis % Axis on which the 3D world is drawn. There will not be visible x, y, z axes, but all the plots go on here.
    end
    
    properties(Access = private)
        mouse_pos_prev; % Previously-seen mouse position for determining changes in mouse drag.
        mouse_pos_curr % Most-recently-seen mouse position.
        shift_key_down = false % Track whether this modifier key is pressed.
    end
    
    methods
        function obj = VisualizerScene(fig_number)
            % Create a new scene in a new figure. If the figure number is
            % specified (optional), it will be created with that ID.
            if nargin == 0
                obj.Figure = figure();
            else
                obj.Figure = figure(fig_number);
            end
            
            obj.Figure.Visible = false; % Turn off the figure while the world is being constructed. Re-enable afterwards.
            
            % Kill most of the default figure UI components to make it look
            % less like a MATLAB plot and more like a game visualizer.
            obj.Figure.NumberTitle = 'off';
            obj.Figure.Name = 'Matt''s Visualizer';
            obj.Figure.ToolBar = 'none';
            obj.Figure.MenuBar = 'none';
            hold on;
            
            axis([-3, 3, -3, 3]);
            daspect([1,1,1]);
            obj.Axis = obj.Figure.Children;
            
            % Set starting camera settings.
            obj.Axis.Projection = 'perspective';
            obj.Axis.Clipping = 'off';
            obj.Axis.Visible = 'off';
            obj.Figure.Position = [0, 0, 1500, 975];
            obj.Axis.CameraPosition = [-2.4, -1.6, 1.8];
            obj.Axis.CameraTarget = [0, 0, 0];
            
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
            
            VisualizerScene.makeCoordinateFrame(); % Make the x, y, z arrows graphically.
            
            % Set keyboard and mouse callbacks.
            obj.Figure.WindowKeyPressFcn = @(src, dat)key_press_callback(obj, src, dat);
            obj.Figure.WindowKeyReleaseFcn = @(src, dat)key_release_callback(obj, src, dat);
            obj.Figure.WindowScrollWheelFcn = @(src, dat)mousewheel_callback(obj, src, dat);
            obj.Figure.WindowButtonDownFcn = @(src, dat)mouse_down_callback(obj, src, dat);
            obj.Figure.WindowButtonUpFcn = @(src, dat)mouse_up_callback(obj, src, dat);
            camva(40);
            
            % Add lighting to the scene.
            light1 = light();
            light1.Position = [0,0,20];
            light1.Style = 'infinite';
            
            obj.Figure.Visible = true;
        end
    end
    
    methods(Static)
        function makeCoordinateFrame()
            % Make the xyz coordinate frame with arrows pointing in those
            % directions.
            plot3(0, 0, 0, '.b', 'MarkerSize', 25);
            quiver3(0, 0, 0, 0.2, 0, 0, 'LineWidth', 4, 'Color', 'r');
            quiver3(0, 0, 0, 0, 0.2, 0, 'LineWidth', 4, 'Color', 'g');
            quiver3(0, 0, 0, 0, 0, 0.2, 'LineWidth', 4, 'Color', 'b');
            text(0.22, 0, 0, 'x', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            text(0, 0.22, 0, 'y', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            text(0, 0, 0.22, 'z', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        end
    end
    
    methods(Access = protected)
        function mouse_down_callback(obj, src, dat)
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
            try
                if strcmp(dat.EventName, 'WindowMousePress')
                    obj.mouse_pos_prev = src.CurrentPoint;
                    src.WindowButtonMotionFcn = @(src, dat)mouse_motion_callback(obj, src, dat);
                end
            catch ME % If the window gets closed, I'd rather the animation loop exit without error messages.
                if ~strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
                    rethrow(ME);
                end
            end
        end
        
        function mouse_up_callback(obj, src, dat)
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
        
        function mouse_motion_callback(obj, src, ~)
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
            try
                if isempty(obj.mouse_pos_prev)
                    obj.mouse_pos_prev = src.CurrentPoint;
                end
                obj.mouse_pos_curr = src.CurrentPoint; % Capture current mouse position in screen coords.
                mouse_diff = obj.mouse_pos_curr - obj.mouse_pos_prev;
                mouse_diffX = mouse_diff(1);
                mouse_diffY = mouse_diff(2);
                camvec = obj.Axis.CameraPosition - obj.Axis.CameraTarget;
                if obj.shift_key_down
                    forwardvec = camvec .* [1,1,0];
                    forwardvec = forwardvec / norm(forwardvec);
                    rightvec = cross(camvec, [0, 0, 1]);
                    forwarddiff = 0.003 * mouse_diffY * forwardvec;
                    rightdiff = 0.001 * mouse_diffX * rightvec;
                    obj.Axis.CameraTarget = obj.Axis.CameraTarget + forwarddiff + rightdiff;
                    obj.Axis.CameraPosition = obj.Axis.CameraPosition + forwarddiff + rightdiff;
                else
                    rotvec = cross([0, 0, 1], camvec / norm(camvec)) * mouse_diffY  + [0, 0, -mouse_diffX];
                    rotvec = rotvec / norm(rotvec);
                    ang = sqrt((mouse_diffY * 0.003) ^ 2 + (0.006 * mouse_diffX) ^ 2);
                    
                    camvec = cos(ang) * camvec + sin(ang) * cross(rotvec, camvec) + (1 - cos(ang)) * dot(rotvec, camvec) * rotvec;
                    
                    if dot(camvec, camvec) - camvec(3)^2 > 0.2 % Avoid getting in gimbal lock range.
                        obj.Axis.CameraPosition = camvec + obj.Axis.CameraTarget;
                    end
                    if obj.Axis.CameraPosition(3) < 0.1
                        obj.Axis.CameraPosition(3) = 0.1;
                    end
                end
                obj.mouse_pos_prev = obj.mouse_pos_curr;
            catch ME % If the window gets closed, I'd rather the animation loop exit without error messages.
                if ~strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
                    rethrow(ME);
                end
            end
        end
        
        function mousewheel_callback(obj, ~, dat)
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
            try
                delta = 0.25; % Multiplier for zooming. Hand-tuned.
                
                curr_cam_pos = obj.Axis.CameraPosition;
                curr_cam_tar = obj.Axis.CameraTarget;
                
                cam_vec = curr_cam_tar - curr_cam_pos;
                
                if dat.VerticalScrollCount > 0 % Zoom out
                    obj.Axis.CameraPosition = curr_cam_pos - cam_vec/norm(cam_vec)*delta;
                elseif dat.VerticalScrollCount < 0 && dot(cam_vec, cam_vec) > 1 % Zoom in, but only if we aren't too close to the target.
                    obj.Axis.CameraPosition = curr_cam_pos + cam_vec/norm(cam_vec)*delta;
                end
            catch ME % If the window gets closed, I'd rather the animation loop exit without error messages.
                if ~strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
                    rethrow(ME);
                end
            end
        end
        
        function key_press_callback(obj, ~, dat)
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
            try
                turn_delta = 0.25; % Multiplier for tilting.
                trans_delta = 0.1; % Multiplier for panning.
                
                curr_cam_pos = obj.Axis.CameraPosition;
                curr_cam_tar = obj.Axis.CameraTarget;
                cam_vec = curr_cam_tar - curr_cam_pos;
                cam_up = cross(cross(cam_vec, [0, 0, 1]), cam_vec);
                
                obj.shift_key_down = ~isempty(dat.Modifier) && strcmp(dat.Modifier{1}, 'shift');
                switch dat.Key
                    case 'uparrow'
                        if obj.shift_key_down % Pan forwards.
                            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
                            forward_vec = forward_vec/norm(forward_vec);
                            obj.Axis.CameraPosition = curr_cam_pos + trans_delta * forward_vec;
                            obj.Axis.CameraTarget = curr_cam_tar + trans_delta * forward_vec;
                        else % Tilt upwards.
                            obj.Axis.CameraPosition = curr_cam_pos + cam_up / (norm(cam_up) + eps) * turn_delta;
                        end
                    case 'downarrow'
                        if obj.shift_key_down % Pan backwards.
                            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
                            forward_vec = forward_vec/norm(forward_vec);
                            obj.Axis.CameraPosition = curr_cam_pos - trans_delta * forward_vec;
                            obj.Axis.CameraTarget = curr_cam_tar - trans_delta * forward_vec;
                        else % Tilt downwards.
                            if curr_cam_pos(3) > 0.2 % No ground-penetrating cameras
                                obj.Axis.CameraPosition = curr_cam_pos - cam_up / (norm(cam_up) + eps) * turn_delta;
                            end
                        end
                    case 'rightarrow'
                        if obj.shift_key_down % Pan right.
                            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
                            forward_vec = forward_vec/norm(forward_vec);
                            right_vec = cross(forward_vec, [0, 0, 1]);
                            obj.Axis.CameraPosition = curr_cam_pos + trans_delta * right_vec;
                            obj.Axis.CameraTarget = curr_cam_tar + trans_delta * right_vec;
                        else % Tilt right.
                            cam_r = cross(cam_vec, cam_up);
                            obj.Axis.CameraPosition = curr_cam_pos + cam_r / (norm(cam_r) + eps) * turn_delta;
                        end
                    case 'leftarrow'
                        if obj.shift_key_down % Pan left.
                            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
                            forward_vec = forward_vec/norm(forward_vec);
                            right_vec = cross(forward_vec, [0, 0, 1]);
                            obj.Axis.CameraPosition = curr_cam_pos - trans_delta * right_vec;
                            obj.Axis.CameraTarget = curr_cam_tar - trans_delta * right_vec;
                        else % Tilt left.
                            cam_r = cross(cam_vec, cam_up);
                            obj.Axis.CameraPosition = curr_cam_pos - cam_r / (norm(cam_r) + eps) * turn_delta;
                        end
                    otherwise
                        % Other events maybe added in the future.
                end
            catch ME % If the window gets closed, I'd rather the animation loop exit without error messages.
                if ~strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
                    rethrow(ME);
                end
            end
        end
        
        function key_release_callback(obj, ~, dat)
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
                obj.shift_key_down = false;
            end
        end
    end
end
