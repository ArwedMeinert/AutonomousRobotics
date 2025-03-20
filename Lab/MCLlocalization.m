function [state,particles] = MCLlocalization(state_prev,particles_prev,speed,rotationspeed,signal,anchors,dt)
n=1000;
if size(particles_prev,1)<100 %no particles are initialised
    particles_prev=createRandom_Particles(n);
end

particles=ones(n,3);

deviation=0.5;
z = reshape(signal(:, 1:2)', [], 1);

state_pred=zeros(n,3);
prob=zeros(n,1);
for m=1:size(particles_prev,1)
    state_pred(m,:)=sample_motion_model(particles_prev(m,:),speed,rotationspeed,dt);
    prob(m)=measurement_model(state_pred(m,:),deviation,anchors,z);
    
end
%prob_norm=ones(1000,1);
prob_norm=prob/sum(prob);

for m=1:size(particles,1)
    [particles(m,:),prob(m)]=select_particle(prob_norm,state_pred);
end
[val, max_idx] = max(prob); % Get the index of the highest probability
state = state_pred(max_idx, :)'; % Return the particle with the highest probability
end

function [selected_particle,prob] = select_particle(probabilities, particles)

    % Create the cumulative distribution function (CDF)
    cdf = cumsum(probabilities);

    % Generate a random number in the range [0, 1]
    r = rand();

    % Find the first particle whose CDF value exceeds the random number
    idx = find(cdf >= r, 1);

    % Select the corresponding particle
    selected_particle = particles(idx, :);
    prob=probabilities(idx);
end

function [particles]=createRandom_Particles(n)
    particles=ones(n,3);
    particles(:, 1:2) = rand(n, 2) * 35;
    particles(:, 3) = wrapToPi(rand(n, 1) * 2 * pi);
end

function [predicted] = sample_motion_model(state,speed,rotationspeed,dt)
    predicted=state;
    x = state(1);
    y = state(2);
    theta = state(3);

    % Predict new state
    theta_new = wrapToPi(theta + rotationspeed * dt+randn*pi*0.03);
    x_new = x + speed * cos(theta) * dt+randn*0.3;
    y_new = y + speed * sin(theta) * dt+randn*0.3;

    predicted(:) = [x_new; y_new; theta_new];
end

function [prob] = measurement_model(state,deviation,anchors,z)
    
    num_anchors=5;


    % Extract robot position and orientation
    robot_x = state(1);
    robot_y = state(2);
    robot_theta = state(3);

    % Preallocate measurement vector
    z_pred = zeros(num_anchors * 2, 1);

    % Compute relative measurements for anchors
    for i = 1:num_anchors
        % Index for anchor positions (x and y) in the anchors matrix
        anchor_x = anchors(i, 1);  % x-coordinate of the ith anchor
        anchor_y = anchors(i, 2);  % y-coordinate of the ith anchor
    
        % Calculate relative distance and orientation
        dx = anchor_x - robot_x;
        dy = anchor_y - robot_y;
        distance = sqrt(dx^2 + dy^2);
        orientation = wrapToPi(atan2(dy, dx) - robot_theta);
    
        % Add to measurement vector (z_pred)
        z_pred((i-1)*2+1:i*2) = [distance; orientation];
    end
    prob=prod((1/sqrt(2*pi*deviation^2)).*exp(-((z' - z_pred').^2) / (2 * deviation^2)));
end