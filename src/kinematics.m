% Kinematics: [U, V, W] -> joint angles theta0, theta1, theta2.
% - rasticraneIk (from homework); elbow angle theta2 is absolute from horizontal
% - Tool length differs: magnet tip when empty, stamp face when loaded
% - Reject unreachable targets and targets outside the joint limits (don't clamp)
