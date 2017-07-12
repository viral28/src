classdef suj < handle
    % Class used to interface with ROS dVRK console topics and convert to useful
    % Matlab commands and properties.  To create a robot interface:
    %   r = suj('PSM1');
    %   r
    %
    % In general, the word `cartesian` is omitted.  When using joint space,
    % add the word `joint`.  `move` stands for moves using absolute
    % positions while `dmove` are always relative to the current desired
    % position as reported by the dVRK C++ console application (i.e. last desired command).
    %   r.position_desired   % contains 4x4 homogeneous transform
    %   r.position_current   % actual 4x4 for the reported position based
    %   on encoders

    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
        ros_namespace % namespace for this arm, should contain head/tail / (default is /dvrk/SUJ)
        robot_name    % name of robot, e.g. PSM1, ECM.  Must match ROS topics namespace
        ros_name      % full ROS namespace, i.e. ros_name + robot_name (e.g. /dvrk/SUJ/PSM1)
    end

    % values set by this class, can be read by others
    properties (SetAccess = protected)
        position_desired        % Last received desired cartesian position
        position_current        % Last received current cartesian position
        position_local_desired  % Last received desired cartesian position
        position_local_current  % Last received current cartesian position
    end

    % only this class methods can view/modify
    properties (SetAccess = private)
        % subscribers
        position_desired_subscriber
        position_local_desired_subscriber
        position_current_subscriber
        position_local_current_subscriber
    end

    methods

        function self = suj(name, namespace)
            % Create a robot interface.  The name must match the suj name
            % in ROS topics (test using rostopic list).  The namespace is
            % optional, default is /dvrk/SUJ.  It is provide for
            % configurations with multiple dVRK so one could have
            % /dvrkA/SUJ/PSM1 and /dvrkB/SUJ/PSM1
            if nargin == 1
                namespace = '/dvrk/SUJ/';
            end
            self.ros_namespace = namespace;
            self.robot_name = name;
            self.ros_name = strcat(self.ros_namespace, self.robot_name);

            % ----------- subscribers

            % position cartesian desired
            self.position_desired = [];
            topic = strcat(self.ros_name, '/position_cartesian_desired');
            self.position_desired_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);
            self.position_desired_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_desired_cb(sub, data);

            % position cartesian local desired
            self.position_local_desired = [];
            topic = strcat(self.ros_name, '/position_cartesian_local_desired');
            self.position_local_desired_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);
            self.position_local_desired_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_local_desired_cb(sub, data);

            % position cartesian current
            self.position_current = [];
            topic = strcat(self.ros_name, '/position_cartesian_current');
            self.position_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);
            self.position_current_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_current_cb(sub, data);

            % position cartesian local current
            self.position_local_current = [];
            topic = strcat(self.ros_name, '/position_cartesian_local_current');
            self.position_local_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);
            self.position_local_current_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_local_current_cb(sub, data);

        end


        function delete(self)
            % hack to disable callbacks from subscribers
            % there might be a better way to remove the subscriber itself
            self.position_desired_subscriber.NewMessageFcn = @(a, b, c)[];
            self.position_local_desired_subscriber.NewMessageFcn = @(a, b, c)[];
            self.position_current_subscriber.NewMessageFcn = @(a, b, c)[];
            self.position_local_current_subscriber.NewMessageFcn = @(a, b, c)[];
        end

        function position_desired_cb(self, ~, pose) % second argument is subscriber, not used
            % Callback used to retrieve the last desired cartesian position
            % published and store as property position_desired

            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([pose.Pose.Position.X, pose.Pose.Position.Y, pose.Pose.Position.Z]);
            orientation = quat2tform([pose.Pose.Orientation.W, pose.Pose.Orientation.X, pose.Pose.Orientation.Y, pose.Pose.Orientation.Z]);
            % combine position and orientation
            self.position_desired = position * orientation;
        end

        function position_local_desired_cb(self, ~, pose) % second argument is subscriber, not used
            % Callback used to retrieve the last desired cartesian position
            % published and store as property position_local_desired

            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([pose.Pose.Position.X, pose.Pose.Position.Y, pose.Pose.Position.Z]);
            orientation = quat2tform([pose.Pose.Orientation.W, pose.Pose.Orientation.X, pose.Pose.Orientation.Y, pose.Pose.Orientation.Z]);
            % combine position and orientation
            self.position_local_desired = position * orientation;
        end

        function position_current_cb(self, ~, pose) % second argument is subscriber, not used
            % Callback used to retrieve the last measured cartesian
            % position published and store as property position_current

            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([pose.Pose.Position.X, pose.Pose.Position.Y, pose.Pose.Position.Z]);
            orientation = quat2tform([pose.Pose.Orientation.W, pose.Pose.Orientation.X, pose.Pose.Orientation.Y, pose.Pose.Orientation.Z]);
            % combine position and orientation
            self.position_current = position * orientation;
        end

        function position_local_current_cb(self, ~, pose) % second argument is subscriber, not used
            % Callback used to retrieve the last measured cartesian
            % position published and store as property position_local_current

            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([pose.Pose.Position.X, pose.Pose.Position.Y, pose.Pose.Position.Z]);
            orientation = quat2tform([pose.Pose.Orientation.W, pose.Pose.Orientation.X, pose.Pose.Orientation.Y, pose.Pose.Orientation.Z]);
            % combine position and orientation
            self.position_local_current = position * orientation;
        end

    end % methods

end % class
