function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================


Kd_x = 45;

Kd_y = 35;

Kd_z = 35;

Kp_x = 210;

Kp_y = 200;

Kp_z = 190;

Kd_phi = 3;

Kd_theta = 3;

Kd_psi = 3;

Kp_phi = 110;

Kp_theta = 110;

Kp_psi = 110 ;

R1 = des_state.acc(1) + Kd_x * (des_state.vel(1) - state.vel(1)) + Kp_x * (des_state.pos(1) - state.pos(1));

R2 = des_state.acc(2) + Kd_y * (des_state.vel(2) - state.vel(2)) + Kp_y * (des_state.pos(2) - state.pos(2));

R3 = des_state.acc(3) + Kd_z * (des_state.vel(3) - state.vel(3)) + Kp_z * (des_state.pos(3) - state.pos(3));

phi_des = (1/params.gravity) * (R1 * sin(des_state.yaw) - R2 * cos(des_state.yaw));

theta_des = (1/params.gravity) * (R1 * cos(des_state.yaw) + R2 * sin(des_state.yaw));

psi_des = des_state.yaw;

K1 = Kp_phi * (phi_des - state.rot(1)) + Kd_phi * (0 - state.omega(1));

K2 = Kp_theta * (theta_des - state.rot(2)) + Kd_theta * (0 - state.omega(2));

K3 = Kp_psi * (psi_des - state.rot(3)) + Kd_psi * (des_state.yawdot - state.omega(3));

F = params.mass * (params.gravity + R3);

M = [K1; K2; K3];

% =================== Your code ends here ===================

end
