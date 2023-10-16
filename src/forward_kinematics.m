% Forward kinematics for the joints' initial state

function forward_kinematics

robot = mdl_ur10e();

q0 = [-0.140 -1.556 -1.359 1.425 -1.053 -1.732];

g0e = @(q) robot.fkine(q);
g0e_0 = double(g0e(q0));
R_0 = g0e_0(1:3, 1:3);
p_0 = g0e_0(1:3, 4:4);

disp('Initial Rotation Matrix (R_0):');
disp(R_0);

disp('Initial Position Vector (p_0):');
disp(p_0);
