function [mu_new, cov_new] = EKFslam(mu, cov, speed, rotationspeed, signal, dt)
    
    %Comments and repetitive calculations (eg. the measurement Jacobian matrix) were
    %done by chat GPT
    % Noise matrices
    pNoise = diag([0.0001, 0.0001, 0.08,repmat([0.001],1,15)]);    % Process noise covariance
    distanceNoise = 4; % Noise for distance measurements
    angleNoise = deg2rad(10); % Noise for angle measurements (in radians)
    % Construct the measurement noise covariance matrix for 5 anchors (10 measurements)
    mVar = diag([repmat(distanceNoise, 1, 5), repmat(angleNoise, 1, 5)]);
    z = reshape(signal(:, 1:2)', [], 1);  % Create the vector for the measurements from the signal input
    % Initialize EKF
    filter = trackingEKF( ...
        @(state) myTransitionModel(state, speed, rotationspeed, dt,z), ...
        @(state) myMeasurementFunction(state), ...
        'ProcessNoise', pNoise, ...
        'MeasurementNoise', mVar, ...
        'StateTransitionJacobianFcn', @(state) myStateJacobian(state, speed, rotationspeed, dt), ...
        'MeasurementJacobianFcn', @(state) myMeasurementJacobian(state));
    initialize(filter,mu,cov);

    predict(filter); % Prediction  
  
    correct(filter,z);  % Correct the predicted state using the actual measurements
    %Update
    mu_new=filter.State;
    cov_new=filter.StateCovariance;
end




function statePred = myTransitionModel(mu, speed, rotationspeed, dt,z)
    statePred=mu;
    num_anchors=5;
    robot_dim = 3;
    if mu(robot_dim+3)==-1 %Initialize the anchor positions
        for i=1:num_anchors
            statePred(robot_dim+i*3-2)=mu(1)+z(i*2-1)*cos(mu(3)+z(i*2));
            statePred(robot_dim+i*3-1)=mu(2)+z(i*2-1)*sin(mu(3)+z(i*2));
            statePred(robot_dim+i*3)=i;
        end
        
    end
    x = mu(1);
    y = mu(2);
    theta = mu(3);

    % Predict new state
    theta_new = wrapToPi(theta + rotationspeed * dt);
    x_new = x + speed * cos(theta) * dt;
    y_new = y + speed * sin(theta) * dt;

    statePred(1:3) = [x_new; y_new; theta_new];
end


function z_pred = myMeasurementFunction(state)
    num_anchors=5;
    robot_dim = 3;
    anchor_dim = 3;

    % Extract robot position and orientation
    robot_x = state(1);
    robot_y = state(2);
    robot_theta = state(3);

    % Preallocate measurement vector
    z_pred = zeros(num_anchors * 2, 1);

    % Compute relative measurements for anchors
    for i = 1:num_anchors
        anchor_idx = robot_dim + (i - 1) * anchor_dim + 1;
        anchor_x = state(anchor_idx);
        anchor_y = state(anchor_idx + 1);

        % Relative distance and orientation
        dx = anchor_x - robot_x;
        dy = anchor_y - robot_y;
        distance = sqrt(dx^2 + dy^2);
        orientation = wrapToPi(atan2(dy, dx) - robot_theta);

        % Add to measurement vector
        z_pred((i-1)*2+1:i*2) = [distance; orientation];
    end
    %z_pred
end

function F = myStateJacobian(mu, speed, rotationspeed, dt)
    theta = mu(3);
    F = eye(length(mu)); % Identity matrix for all state variables

    % Update the first three rows for robot motion
    F(1, 3) = -speed * sin(theta) * dt;
    F(2, 3) = speed * cos(theta) * dt;
end

function H = myMeasurementJacobian(state)
    num_anchors=5;
    robot_dim = 3;
    anchor_dim = 2;

    % Extract robot position and orientation
    robot_x = state(1);
    robot_y = state(2);
    robot_theta = state(3);

    % Initialize measurement Jacobian
    H = zeros(num_anchors * 2, length(state));

    for i = 1:num_anchors
        anchor_idx = robot_dim + (i - 1) * anchor_dim + 1;
        anchor_x = state(anchor_idx);
        anchor_y = state(anchor_idx + 1);

        % Relative position
        dx = anchor_x - robot_x;
        dy = anchor_y - robot_y;
        q = dx^2 + dy^2;
        sqrt_q = sqrt(q);

        % Partial derivatives for distance
        H((i-1)*2+1, 1) = -dx / sqrt_q; % ∂distance/∂robot_x
        H((i-1)*2+1, 2) = -dy / sqrt_q; % ∂distance/∂robot_y
        H((i-1)*2+1, anchor_idx) = dx / sqrt_q; % ∂distance/∂anchor_x
        H((i-1)*2+1, anchor_idx+1) = dy / sqrt_q; % ∂distance/∂anchor_y

        % Partial derivatives for orientation
        H((i-1)*2+2, 1) = dy / q; % ∂orientation/∂robot_x
        H((i-1)*2+2, 2) = -dx / q; % ∂orientation/∂robot_y
        H((i-1)*2+2, 3) = -1; % ∂orientation/∂robot_theta
        H((i-1)*2+2, anchor_idx) = -dy / q; % ∂orientation/∂anchor_x
        H((i-1)*2+2, anchor_idx+1) = dx / q; % ∂orientation/∂anchor_y
    end
end

