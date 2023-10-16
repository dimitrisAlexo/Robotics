% Implementation of the control scheme

function [Q, G_et, Q_dot, Q_ddot, T] = control(G_0t, t_final, dt)

% The Robot model
robot = mdl_ur10e();

% Robot parameters
g_0e = @(q) double(robot.fkine(q));
Je = @(q) double(robot.jacobe(q));

degrees_freedom = robot.n;

q0 = [-0.140 -1.556 -1.359 1.425 -1.053 -1.732];

q_dot_max = deg2rad([120 120 180 180 180 180]);
q_ddot_max = 250;

% Controller parameters
kp = 300;
ki = 0.5;

% The time array
T = 0:dt:t_final;
N = length(T);

% Iteration data
Q = zeros(N, degrees_freedom);
Q(1,:) = q0;

Q_dot = zeros(N, degrees_freedom);
Q_ddot = zeros(N, degrees_freedom);

G_et = zeros(4, 4, N);

int_u = zeros(degrees_freedom, 1);

for n = 1:(N-1)
    % Target position and orientation
    g_0t = G_0t{n};

	% Calculating the difference of the tool and the target
	G_et(:,:,n) = g_0e(Q(n,:)) \ g_0t;

	% Extracting transform information
	[theta, rot_axis] = tr2angvec(G_et(:,:,n));
	p = G_et(1:3,4,n);

	% Constructing spin vector based on the transform
	Ve = [p ; sin(theta/2) * rot_axis'];

	% Calculating the controller inputs
	u = Je(Q(n,:)) \ Ve;

    % Integrator
    int_u = int_u + u * dt;

	% Implementing the controller
	q_dot = kp * u' + ki * int_u';

	% Enforcing q_dot limits using a saturation function
	q_dot = sign(q_dot) .* min(abs(q_dot), q_dot_max);
    
    % Using a low pass filter to limit the acceleration
    dQ_dot = (q_dot - Q_dot(n,:));
    dQ_dot = sign(dQ_dot) .* min(abs(dQ_dot), q_ddot_max * dt);
	
    Q_dot(n + 1,:) = Q_dot(n,:) + dQ_dot;
    
    Q_ddot(n + 1,:) = dQ_dot / dt;

	% Integrating the resulting output
	Q(n + 1,:) = Q(n,:) + dt * Q_dot(n + 1,:);

end

G_et(:,:,N) = g_0e(Q(N,:)) \ g_0t;
