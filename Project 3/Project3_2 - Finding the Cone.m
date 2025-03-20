%nComments done by Chat GPT
clc
clear all
ip = "http://192.168.237.129:11311";
rosshutdown;
rosinit(ip);
velPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
laserSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');
% Desired wall distance
desired_distance = 0.7; % Adjust based on your environment (in meters)
min_distance = 0.4; % Minimum safe distance to avoid collisions

cone_prev=false;

while true 
velMsg = rosmessage(velPub);
laserMsg = receive(laserSub, 10); % Wait up to 10 seconds for a message
ranges = laserMsg.Ranges; % Distance readings from the laser scanner
% Parameters
desired_distance = 1; % Desired distance from the wall (meters)
min_distance = 0.4; % Minimum safe distance to avoid collisions (meters)
k_p = 0.6; % Proportional gain for angular velocity (tune as needed)
k_p_linear = 0.5; % Proportional gain for linear velocity (tune as needed)

% Get distances from laser scan
front_indices = [1:45, 315:360]; % Combine indices for front cone (0 to 45 and 315 to 360)
front_distance = min(ranges(front_indices)); % Minimum distance in the front region
[left_distance, left_index] = min(ranges(225:315)); % Left sensor (225 to 315)

% Calculate the error (distance to wall deviation)
error = desired_distance - left_distance;

% Logic for wall-following with proportional control
if front_distance < min_distance
    % Obstacle ahead: Stop and turn left to avoid collision
    velMsg.Linear.X = 0.0; % Stop
    velMsg.Angular.Z = 0.5; % Turn left
elseif left_distance < desired_distance && left_distance > min_distance
    % Wall is within an acceptable range: move forward and adjust turning rate
    velMsg.Linear.X = 0.4 - k_p_linear * error; % Adjust forward speed proportionally
    velMsg.Angular.Z = -k_p * error; % Adjust turning rate proportionally
elseif left_distance >= desired_distance
    % Wall is too far: Turn left to get closer
    velMsg.Linear.X = 0.2; % Move forward
    velMsg.Angular.Z = k_p * error; % Turn left
else
    % Wall is too close: Turn right to move away
    velMsg.Linear.X = 0.2; % Move forward
    velMsg.Angular.Z = -k_p * error; % Turn right
end
send(velPub, velMsg);
cone=identify_cone(ranges(180:360), 30);
if cone_prev &&  cone
    disp('Cone Found')
end
cone_prev=cone;
pause(0.1);
end
rosshutdown;


function [isCone] = identify_cone(sensor, points_to_check)
    %Comments done by chat GPT
    isCone = false; % Default: not a cone
    % Find the index of the closest object
    [~, min_index] = min(sensor);

    % Extract region of interest (ROI)
    start_index = max(1, min_index - ceil(points_to_check / 2));
    end_index = min(length(sensor), min_index + ceil(points_to_check / 2));
    selected_distances = sensor(start_index:end_index)';

    % Remove invalid data points
    if any(isinf(selected_distances))
        return;
    end

    % Convert polar coordinates to Cartesian coordinates
    angles = deg2rad(start_index:end_index);
    x = selected_distances .* cos(angles);
    y = selected_distances .* sin(angles);
     
    % Analyze geometric properties
    % Fit a second-order polynomial (quadratic curve)
    if length(x) < 3
        % Not enough points for curvature analysis
        return;
    end
    p = polyfit(x, y, 2); % Quadratic fit
    %Plot used for debugging
   plot(x,y)
   hold on
   plot(x,polyval(p,x))
   
    % Check curvature
    curvature = abs(p(1)); % Curvature is proportional to the quadratic coefficient
    if curvature > 5 && curvature<10
        isCone = true;
        plot(x,y,"Color",[1 1 0]) 
    end
    if curvature<0.2
        pause
    end
    hold off
end




