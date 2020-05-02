function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% FILL IN YOUR CODE HERE

kp_y = 20;
kp_z = 80;
kp_phi = 150;
kv_y = 5;
kv_z = 20;
kv_phi = 10;




phi_c = (-1/params.gravity)*(des_state.acc(1) + kv_y*(des_state.vel(1)-state.vel(1)) + kp_y*(des_state.pos(1)-state.pos(1)));

u1 = params.mass*(params.gravity + des_state.acc(2) + kv_z*(des_state.vel(2)-state.vel(2)) + kp_z*(des_state.pos(2)- state.pos(2)));

u2 = params.Ixx*(0 + kv_phi*(0-state.omega) + kp_phi*(phi_c - state.rot));



end

