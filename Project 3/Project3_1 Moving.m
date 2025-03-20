ip = "http://192.168.237.129:11311"; % Modify with your ROS Master URI
rosshutdown;
rosinit(ip);
velPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
laserSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');
min_distance=0.5;
while true 
velMsg = rosmessage(velPub);
laserMsg = receive(laserSub, 10); % Wait up to 10 seconds for a message
ranges = laserMsg.Ranges; % Distance readings from the laser scanner

% Filter out 'inf' values before summing
filtered_right = ranges(1:90);
filtered_right = filtered_right(~isinf(filtered_right)); % Remove 'inf' values
sum_right = sum(filtered_right);

filtered_left = ranges(270:360);
filtered_left = filtered_left(~isinf(filtered_left)); % Remove 'inf' values
sum_left = sum(filtered_left);


if ranges(1)>min_distance
    velMsg.Linear.X = 0.2; % Linear velocity in m/s
else
    velMsg.Linear.X = 0.0; % Linear velocity in m/s
end

if sum_right<sum_left
    velMsg.Angular.Z = -0.2; % Angular velocity in rad/s
elseif sum_left<sum_right
    velMsg.Angular.Z = 0.2; % Angular velocity in rad/s
else
    velMsg.Angular.Z = 0; % Angular velocity in rad/s
end
% Example: Move the robot forward

send(velPub, velMsg);
pause(0.5);
end
rosshutdown;

