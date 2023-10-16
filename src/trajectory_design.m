% Desired trajectory for catching the ball

function trajectory_design
    %% Trajectory Planning Variables
    
    tic

    global z0 zf tf V tb a; 
    z0 = 0.45;
    zf = 0.06;
    tf = 6;
    V = 2*(zf - z0) / tf;
    tb = tf - (zf - z0)/V;
    a = V / tb;

    %% Homogenous Transforms
    
    robot = mdl_ur10e();

    % The time array
    global dt;
    t_final = 10;
    dt = 2e-3;

    % Set g_0c
    g_0c = [eye(3) [0.4; 0; 0.2]; 0 0 0 1];

    % Get G_cb cell array
    [G_cb, Wsp] = get_data(t_final);

    % Set g_bt
    g_bt = @(t) [angvec2r(pi, [0; 1; 0]) [0; 0; z(t)]; 0 0 0 1];

    % Set G_ot
    N = size(G_cb, 2);
    G_0t = cell(1, N);
    for i = 1:N
        G_0t{i} = g_0c * G_cb{i} * g_bt(i);
    end
    
    
    %% Control

    [Q, G_et, Q_dot, Q_ddot, T] = control(G_0t, t_final, dt);

    G_be = zeros(4, 4, N);

    for i = 1:N
        G_be(:, :, i) = g_bt(i) / G_et(:, :, i);
    end

    %% P_be

    figure
    title("P_{be} (m) - Time (sec)");
    hold on
    plot(T, squeeze(G_be(1, 4, :)));
    plot(T, squeeze(G_be(2, 4, :)));
    plot(T, squeeze(G_be(3, 4, :)));
    legend("x", "y", "z");
    xlabel("t");
    ylabel("P_{be}");
    hold off


    %% Angle, Rotation Axis

    [Theta, Rot_Axis] = tr2angvec(G_be);

    figure
    subplot(2,1,1)
    plot(T, mod(Theta .* sign(Rot_Axis(:, 2)), 2*pi));
    title("Theta (rad) - Time (sec)");
    xlabel("t");
    ylabel("theta");

    subplot(2,1,2)
    title("k_{be} - Time (sec)");
    hold on
    plot(T, Rot_Axis(:, 1) .* sign(Rot_Axis(:, 2)));
    plot(T, Rot_Axis(:, 2) .* sign(Rot_Axis(:, 2)));
    plot(T, Rot_Axis(:, 3) .* sign(Rot_Axis(:, 2)));
    legend("x", "y", "z");
    xlabel("t");
    ylabel("k_{be}");
    hold off


    %% Q, Q_dot, Q_ddot

    figure
    plot(T, Q(:, 1), T, Q(:, 2), T, Q(:, 3), ...
         T, Q(:, 4), T, Q(:, 5), T, Q(:, 6));
    title("Q (rad) - Time (sec)");
    legend("q_1", "q_2", "q_3", "q_4", "q_5", "q_6")
    xlabel("t");
    ylabel("Q");

    figure
    plot(T, Q_dot(:, 1), T, Q_dot(:, 2), T, Q_dot(:, 3), ...
         T, Q_dot(:, 4), T, Q_dot(:, 5), T, Q_dot(:, 6));
    title("Q' (rad/s) - Time (sec)");
    legend("q'_1", "q'_2", "q'_3", "q'_4", "q'_5", "q'_6")
    xlabel("t");
    ylabel("Q'");

    figure
    plot(T, Q_ddot(:, 1), T, Q_ddot(:, 2), T, Q_ddot(:, 3), ...
         T, Q_ddot(:, 4), T, Q_ddot(:, 5), T, Q_ddot(:, 6));
    title("Q'' (rad/s^2) - Time (sec)");
    legend("q''_1", "q''_2", "q''_3", "q''_4", "q''_5", "q''_6")
    xlabel("t");
    ylabel("Q''");
    
    toc
    
    
    %% Visualize

    try
        figure
        Wsp.visualize(robot , Q', [90, 0]);
    catch ME
        % Check if the error is related to closing the window
        if strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
            disp('Animation window closed.');
        else
            rethrow(ME);
        end
    end


end

function z = z(t)

    global dt z0 zf tf V tb a; 

    t = t * dt;

    if t >= 0 && t <= tb
        z = z0 + a/2*t^2;
    elseif t > tb && t <= tf - tb
        z = (zf + z0 - V*tf)/2 + V*t;
    elseif t > tf - tb && t <= tf
        z = zf - (a*tf^2)/2 + a*tf*t - a/2*t^2;
    else
        z = zf;
    end

end

