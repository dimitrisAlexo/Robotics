% Extract the data of the ball's movement

function [G_cb, Wsp] = get_data(t_final)

% Extract data

Ts = 0.002;
t = 0:Ts:t_final;
N = length(t);

Wsp = Wspace();
P_cb = zeros(3, N);
V_cb = zeros(3, N);
W_cb = zeros(3, N);

for n = 1:N+13
    [p_cb, v_cb, w_cb] = Wsp.sim_ball(Ts);
    
    if n > 13
        
        k = n - 13;
        
        P_cb(:, k) = p_cb;
        V_cb(:, k) = v_cb;
        W_cb(:, k) = w_cb;
        
    end
    
end

G_cb = cell(1, N); % Initialize cell array

for i = 1:N
    % Extract the i-th values from p_cb, v_cb, and w_cb
    p = P_cb(:, i);
    v = V_cb(:, i);
    w = W_cb(:, i);
    
    % Compute the rotation matrix
    theta = acos(abs(v(2)) / norm(v));
    if v(3) * v(2) < 0
        theta = -theta;
    end
    R = rotx(rad2deg(theta));
    
    % Create the homogeneous matrix using p, v, and R
    G_cb{i} = [R, p; 0 0 0 1];
end
