function [jointPosSmoothed, jointVels, jointAccs, sgolayJointsTime, sgolayjointsVel] = estimateJointVelocitiesAndAccelerations(time, jointPos, polyOrder, winSize)
 
%   sg_pos_smoother = sgolay(polyOrder, winSize, 0, mean(diff(time)));
%   sg_vel_filter = sgolay(polyOrder, winSize, 1, mean(diff(time)));
%   sg_acc_filter = sgolay(polyOrder, winSize, 2, mean(diff(time)));
% 
%   n_joints = size(jointPos, 2);
% 
%   for i =  1:n_joints
%     jointPosSmoothed(:, i) = sgolayfilt(jointPos(:, i), sg_pos_smoother);
%   end
%   
%    for i =  1:n_joints
%     jointVels(:, i) = sgolayfilt(jointPosSmoothed(:, i), sg_vel_filter);
%     jointAccs(:, i) = sgolayfilt(jointPosSmoothed(:, i), sg_acc_filter);
%   end

    % sgolay joint vel for comparison of lpf joint vel

    nr_joints_est = size(jointPos, 1);
    dt = mean(diff(time));
    [~, g] = sgolay(polyOrder, winSize);

    jointPosSmoothed = zeros(length(time), nr_joints_est)';
    jointVels = zeros(length(time), nr_joints_est)';
    jointAccs = zeros(length(time), nr_joints_est)';

    for idx = 1:nr_joints_est
        jointPosSmoothed(idx, :) = conv(jointPos(idx, :), factorial(0)/(-dt)^0 * g(:, 1), 'same');
        jointVels(idx, :) = conv(jointPos(idx, :), factorial(1)/(-dt)^1 * g(:, 2), 'same');
        jointAccs(idx, :) = conv(jointPos(idx, :), factorial(2)/(-dt)^2 * g(:, 3), 'same');
    end

    sgolayJointsTime = time(:, winSize:end-winSize);
    sgolayjointsVel = jointVels(:, winSize:end-winSize);

end