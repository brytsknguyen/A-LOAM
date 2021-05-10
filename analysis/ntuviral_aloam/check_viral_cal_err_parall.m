function [P_ate, rot_rmse] = check_viral_cal_err_parall(test_id, test_fullname)


% clearvars('-except', vars_to_keep{:});
close all;

% addpath('/home/tmn/MATLAB_WS');

t_shift = 0;

myorange = [0.96, 0.96, 0.00];
mycyan   = [0.00, 0.75, 1.00];

fighd = [];

%% get the exp name number
exp_name =  test_fullname.name;
exp_path = [test_fullname.folder '/' test_fullname.name '/'];


gndtr_pos_fn         = [exp_path 'leica_pose.csv'];
gndtr_dji_imu_fn     = [exp_path 'dji_sdk_imu.csv'];
gndtr_vn100_imu_fn   = [exp_path 'vn100_imu.csv'];
pose_horz_est_fn     = [exp_path 'opt_odom_horz.csv'];
pose_vert_est_fn     = [exp_path 'opt_odom_vert.csv'];
tf_B2Bhloam_fn       = [exp_path '../tf_B2Bhloam.csv'];
tf_B2Bvloam_fn       = [exp_path '../tf_B2Bvloam.csv'];
rot_B2vn100_fn       = [exp_path '../rot_B2vn100.csv'];
rot_B2djiimu_fn      = [exp_path '../rot_B2djiimu.csv'];
trans_B2prism_fn     = [exp_path '../trans_B2prism.csv'];


%% Read the data from log and start processing


%% Read the gndtr data from logs

% Position groundtr
gndtr_pos_data = csvread(gndtr_pos_fn,  1, 0);

% Orientation groundtr
% Check if file size is 0
dji_file   = dir(gndtr_dji_imu_fn);
vn100_file = dir(gndtr_vn100_imu_fn);

% Orientation is in z upward frame, convert it to body frame
rot_B_Beimu = eye(3);

dji_present   = (dji_file.bytes ~= 0);
vn100_present = (vn100_file.bytes ~= 0);

if vn100_present
    gndtr_vn100_data = csvread([exp_path 'vn100_imu.csv'], 1, 0);
end

if dji_present
    gndtr_Beimu_data = csvread(gndtr_dji_imu_fn, 1, 0);
    rot_B_Beimu    = csvread(rot_B2djiimu_fn, 0, 0);
elseif ~dji_present && vn100_present
    gndtr_Beimu_data = csvread([exp_path 'vn100_imu.csv'], 1, 0);
    rot_B_Beimu    = csvread(rot_B2vn100_fn,  0, 0);
end

t0_ns = gndtr_pos_data(1, 1);

% pos groundtruthdata
t_pos = (gndtr_pos_data(:, 1) - t0_ns)/1e9 + t_shift;
P     = gndtr_pos_data(:, 4:6);

% ori groundtruthdata
t_ori = (gndtr_Beimu_data(:, 1)- t0_ns)/1e9;
Q     =  quatnormalize(gndtr_Beimu_data(:, [7, 4:6]));
Q0    =  quatnormalize(Q(1, :));

% Delete the duplicate in position groundtruth data
[~, Px_unq_idx] = unique(P(:, 1));
[~, Py_unq_idx] = unique(P(:, 2));
[~, Pz_unq_idx] = unique(P(:, 3));

P_unq_idx = union(union(Px_unq_idx, Py_unq_idx), Pz_unq_idx);
P = P(P_unq_idx, :);
t_pos = t_pos(P_unq_idx, :);

% Delete the duplicate in orientation groundtruth data
[~, Qx_unq_idx] = unique(Q(:, 1));
[~, Qy_unq_idx] = unique(Q(:, 2));
[~, Qz_unq_idx] = unique(Q(:, 3));
[~, Qw_unq_idx] = unique(Q(:, 4));

Q_unq_idx = union(union(union(Qx_unq_idx, Qy_unq_idx), Qz_unq_idx),...
                        Qw_unq_idx);
Q     = Q(Q_unq_idx, :);
t_ori = t_ori(Q_unq_idx, :);


%% Calculate transform from lidars to the prism


% Position of prism in the body frame
trans_B2prism = (csvread(trans_B2prism_fn, 0, 0))';


% Horizontal lidar transform
tf_B2Bhloam    = csvread(tf_B2Bhloam_fn, 0, 0);
rot_B2Bhloam   = tf_B2Bhloam(1:3, 1:3);
trans_B2Bhloam = tf_B2Bhloam(1:3, 4);
% Displacement from the horz lidar to the prism
trans_Bhloam2prism = rot_B2Bhloam'*(trans_B2prism - trans_B2Bhloam);
rot_Bhloam2Beimu   = rot_B2Bhloam'*rot_B_Beimu;


% Vertical lidar transform
tf_B2Bvloam    = csvread(tf_B2Bvloam_fn, 0, 0);
rot_B2Bvloam   = tf_B2Bvloam(1:3, 1:3);
trans_B2Bvloam = tf_B2Bvloam(1:3, 4);
% Displacement from the horz lidar to the prism
trans_Bvloam2prism = rot_B2Bvloam'*(trans_B2prism - trans_B2Bvloam);
rot_Bvloam2Beimu   = rot_B2Bvloam'*rot_B_Beimu;



%% Read the hloam estimate data from logs

hloam_data = csvread(pose_horz_est_fn, 1, 0);
t_hloam = (hloam_data(:, 1) - t0_ns)/1e9;
P_hloam = hloam_data(:, 4:6);
Q_hloam = quatnormalize(hloam_data(:, [10, 7:9]));
V_hloam = hloam_data(:, 11:13);

% Compensate the position estimate with the prism displacement
P_hloam = P_hloam + quatconv(Q_hloam, trans_Bhloam2prism');
% Compensate the orientation estimate with the extrinsic
Q_hloam = quatmultiply(Q_hloam, rotm2quat(rot_Bhloam2Beimu));


%% Read the hloam estimate data from logs

vloam_data = csvread(pose_vert_est_fn, 1, 0);
t_vloam = (vloam_data(:, 1) - t0_ns)/1e9;
P_vloam = vloam_data(:, 4:6);
Q_vloam = quatnormalize(vloam_data(:, [10, 7:9]));
V_vloam = vloam_data(:, 11:13);

% Compensate the position estimate with the prism displacement
P_vloam = P_vloam + quatconv(Q_vloam, trans_Bvloam2prism');
% Compensate the orientation estimate with the extrinsic
Q_vloam = quatmultiply(Q_vloam, rotm2quat(rot_Bvloam2Beimu));


%% Resample gndtruth by estimates' timestamp


% Horizontal lidar
for dummy = 1
    
% Find the interpolated time stamps
[rshloam_pos_itp_idx(:, 1), rshloam_pos_itp_idx(:, 2)]...
    = combteeth(t_hloam, t_pos);
[rshloam_ori_itp_idx(:, 1), rshloam_ori_itp_idx(:, 2)]...
    = combteeth(t_hloam, t_ori);

% Remove the un-associatable samples
rshloam_nan_idx = find(isnan(rshloam_pos_itp_idx(:, 1))...
                        | isnan(rshloam_pos_itp_idx(:, 2))...
                        | isnan(rshloam_ori_itp_idx(:, 1))...
                        | isnan(rshloam_ori_itp_idx(:, 2))...
                        );

t_hloam_full = t_hloam;
P_hloam_full = P_hloam;
Q_hloam_full = Q_hloam;
V_hloam_full = V_hloam;

rshloam_pos_itp_idx(rshloam_nan_idx, :) = [];
rshloam_ori_itp_idx(rshloam_nan_idx, :) = [];
t_hloam(rshloam_nan_idx, :)     = [];
P_hloam(rshloam_nan_idx, :)     = [];
Q_hloam(rshloam_nan_idx, :)     = [];
V_hloam(rshloam_nan_idx, :)     = [];

% interpolate the pos gndtr state
P_rshloam = vecitp(P, t_pos, t_hloam, rshloam_pos_itp_idx);

[rot_align_hloam, trans_align_hloam] = traj_align(P_rshloam, P_hloam);

% Align the position estimate
P_hloam      = (rot_align_hloam*P_hloam'      + trans_align_hloam)';
P_hloam_full = (rot_align_hloam*P_hloam_full' + trans_align_hloam)';

% Align the velocity estimate
V_hloam      = (rot_align_hloam*V_hloam')';
V_hloam_full = (rot_align_hloam*V_hloam_full')';

% interpolate the ori gndtr state
Q_rshloam = quatitp(Q, t_ori, t_hloam, rshloam_ori_itp_idx);


% Find the optimized rotation between the groundtruth and the estimate
rot_rshloam = quat2rotm(Q_rshloam);
rot_hloam   = quat2rotm(Q_hloam);

rot_rshloam2hloam_opt = (rot_opt(rot_rshloam, rot_hloam))';

% Align the ori estimate
Q_rshloam = quatmultiply(rotm2quat(rot_rshloam2hloam_opt), Q_rshloam);

end


% Vertical lidar
for dummy = 1
    
% Find the interpolated time stamps
[rsvloam_pos_itp_idx(:, 1), rsvloam_pos_itp_idx(:, 2)]...
    = combteeth(t_vloam, t_pos);
[rsvloam_ori_itp_idx(:, 1), rsvloam_ori_itp_idx(:, 2)]...
    = combteeth(t_vloam, t_ori);

% Remove the un-associatable samples
rsvloam_nan_idx = find(isnan(rsvloam_pos_itp_idx(:, 1))...
                        | isnan(rsvloam_pos_itp_idx(:, 2))...
                        | isnan(rsvloam_ori_itp_idx(:, 1))...
                        | isnan(rsvloam_ori_itp_idx(:, 2))...
                        );

t_vloam_full = t_vloam;
P_vloam_full = P_vloam;
Q_vloam_full = Q_vloam;
V_vloam_full = V_vloam;

rsvloam_pos_itp_idx(rsvloam_nan_idx, :) = [];
rsvloam_ori_itp_idx(rsvloam_nan_idx, :) = [];
t_vloam(rsvloam_nan_idx, :)     = [];
P_vloam(rsvloam_nan_idx, :)     = [];
Q_vloam(rsvloam_nan_idx, :)     = [];
V_vloam(rsvloam_nan_idx, :)     = [];

% interpolate the pos gndtr state
P_rsvloam = vecitp(P, t_pos, t_vloam, rsvloam_pos_itp_idx);

[rot_align_vloam, trans_align_vloam] = traj_align(P_rsvloam, P_vloam);

% Align the position estimate
P_vloam      = (rot_align_vloam*P_vloam'      + trans_align_vloam)';
P_vloam_full = (rot_align_vloam*P_vloam_full' + trans_align_vloam)';

% Align the velocity estimate
V_vloam      = (rot_align_vloam*V_vloam')';
V_vloam_full = (rot_align_vloam*V_vloam_full')';

% interpolate the ori gndtr state
Q_rsvloam = quatitp(Q, t_ori, t_vloam, rsvloam_ori_itp_idx);


% Find the optimized rotation between the groundtruth and the estimate
rot_rsvloam = quat2rotm(Q_rsvloam);
rot_vloam   = quat2rotm(Q_vloam);

rot_rsvloam2vloam_opt = (rot_opt(rot_rsvloam, rot_vloam))';

% Align the ori estimate
Q_rsvloam = quatmultiply(rotm2quat(rot_rsvloam2vloam_opt), Q_rsvloam);

end



%% Calculate the position and rotation errors


%% Calculate the position and rotation errors of the estimates


% Horizontal lidar
for dummy = 1
    
P_hloam_err     = P_rshloam - P_hloam;
P_hloam_rmse    = rms(P_hloam_err);
P_hloam_ate     = norm(P_hloam_rmse);
P_hloam_err_nrm = sqrt(dot(P_hloam_err, P_hloam_err, 2));

Q_hloam_err       = quatmultiply(quatinv(Q_hloam), Q_rshloam);
YPR_hloam_err	  = wrapToPi(quat2eul(Q_hloam_err));
rot_hloam_ang_err = quat2axang(Q_hloam_err);
% Wrap this error to -pi to pi;
rot_hloam_ang_err(:, end) = wrapToPi(rot_hloam_ang_err(:, end));
% Find the outliers
olrhloam_idx = find(isoutlier(rot_hloam_ang_err(:, end), 'mean'));
% Extract the inlier errors
rot_hloam_ang_err_nolr = rot_hloam_ang_err(:, end);
t_hloam_nolr           = t_hloam;
rot_hloam_ang_err_nolr(olrhloam_idx, :) = [];
t_hloam_nolr(olrhloam_idx)              = [];
% Calculate the error
rot_hloam_rmse         = rms(rot_hloam_ang_err_nolr(:, end))/pi*180;
% rot_h_ang_err_norm = abs(rot_h_ang_err(:, end));

end


% Vertical lidar
for dummy = 1
    
P_vloam_err     = P_rsvloam - P_vloam;
P_vloam_rmse    = rms(P_vloam_err);
P_vloam_ate     = norm(P_vloam_rmse);
P_vloam_err_nrm = sqrt(dot(P_vloam_err, P_vloam_err, 2));

Q_vloam_err       = quatmultiply(quatinv(Q_vloam), Q_rsvloam);
YPR_vloam_err	  = wrapToPi(quat2eul(Q_vloam_err));
rot_vloam_ang_err = quat2axang(Q_vloam_err);
% Wrap this error to -pi to pi;
rot_vloam_ang_err(:, end) = wrapToPi(rot_vloam_ang_err(:, end));
% Find the outliers
olrvloam_idx = find(isoutlier(rot_vloam_ang_err(:, end), 'mean'));
% Extract the inlier errors
rot_vloam_ang_err_nolr = rot_vloam_ang_err(:, end);
t_vloam_nolr           = t_vloam;
rot_vloam_ang_err_nolr(olrvloam_idx, :) = [];
t_vloam_nolr(olrvloam_idx)              = [];
% Calculate the error
rot_vloam_rmse         = rms(rot_vloam_ang_err_nolr(:, end))/pi*180;
% rot_h_ang_err_norm = abs(rot_h_ang_err(:, end));

end


%% Save the important variables for later analyses
save([exp_path exp_name '_poses.mat'],...
     't_hloam', 'P_rshloam', 'Q_rshloam',...
     'P_hloam', 'Q_hloam',...
     'rot_align_hloam', 'trans_align_hloam');

% Save the errors calculated
save([exp_path exp_name '_rmse.mat'],...
     'P_hloam_ate', 'rot_hloam_rmse',...
     'P_vloam_ate', 'rot_vloam_rmse');


%% Print the result
fprintf(['test: %2d. %s.\n'...
         'Error:\tPos [m]\t Rot [deg]\n'...
         'HORZ:\t%6.4f\t %7.4f.\n'...
         'VERT:\t%6.4f\t %7.4f.\n\n'],...
          test_id, exp_name(8:end),...
          P_hloam_ate, rot_hloam_rmse,... 
          P_vloam_ate, rot_vloam_rmse);
      
P_ate    = [P_hloam_ate,    P_vloam_ate];
rot_rmse = [rot_hloam_rmse, rot_vloam_rmse];



%% Calculate the maximum time
t_max = max([t_pos; t_hloam]);



%% Plot the 3D trajectory
figpos = [1920 0 0 0] + [0, 480, 630, 400];
figure('position', figpos, 'color', 'w', 'paperpositionmode', 'auto');
fighd = [fighd gcf];
hold on;
plot3(P(1:2, 1), P(1:2, 2), P(1:2, 3),...
    'r', 'linewidth', 3);
plot3(P_hloam(1:2, 1),  P_hloam(1:2, 2),  P_hloam(1:2, 3),...
    'b', 'linewidth', 3);
plot3(P(:, 1), P(:, 2), P(:, 3),...
    '.r', 'markersize', 6);
plot3(P_hloam_full(:, 1),  P_hloam_full(:, 2),  P_hloam_full(:, 3),...
    '.b', 'markersize', 6);

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;
daspect([1 1 1]);
view([-21 15]);
tightfig;
set(gca, 'fontsize', 13);
% lg_hd = legend('Leica', 'LOAM (H)', 'LOAM (V)', 'hloam');
lg_hd = legend('Leica', 'H.LOAM');
set(lg_hd, 'position', [0.69925297449761,...
                        0.694882822813015,...
                        0.233789057870308,...
                        0.26499999254942]);
saveas(gcf, [exp_path exp_name '_traj.fig']);
% saveas(gcf, [exp_path exp_name '_traj.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_traj.png']);



%% Plot the time evolution of position
figpos = [1920 0 0 0] + [0, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
axgnd = plot(t_pos, P(:, 1),   'r', 'linewidth', 4);
axh   = plot(t_hloam,   P_hloam(:, 1), 'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('X [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
axgnd = plot(t_pos, P(:, 2),   'r', 'linewidth', 4);
axh   = plot(t_hloam,   P_hloam(:, 2), 'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('Y [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
axgnd = plot(t_pos, P(:, 3),   'r', 'linewidth', 3);
axh   = plot(t_hloam,   P_hloam(:, 3), 'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
xlabel('Time [s]');
ylabel('Z [m]');
% ylim([-1.5, 1.5]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);


tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyzt.fig']);
% saveas(gcf, [exp_path exp_name '_xyzt.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyzt.png']);



%% Plot the time evolution of orientation of hloam

% Calculate the yaw pitch rol relative to the initial position
YPR_Bhloam = quat2eul(quatmultiply(quatinv(Q(1, :)), Q));
YPR_hloam  = quat2eul(quatmultiply(quatinv(Q_hloam(1, :)), Q_hloam));


figpos = [1920 0 0 0] + [630, 480, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
axgnd = plot(t_ori,   YPR_Bhloam(:, 1)*180/pi, 'r', 'linewidth', 4);
axh   = plot(t_hloam, YPR_hloam(:, 1)*180/pi,  'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('Yaw [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
axgnd = plot(t_ori,   YPR_Bhloam(:, 2)*180/pi, 'r', 'linewidth', 4);
axh   = plot(t_hloam, YPR_hloam(:, 2)*180/pi,  'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('Pitch [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
axgnd = plot(t_ori,   YPR_Bhloam(:, 3)*180/pi, 'r', 'linewidth', 4);
axh   = plot(t_hloam, YPR_hloam(:, 3)*180/pi,  'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
xlabel('Time [s]');
ylabel('Roll [deg]');
% ylim([-1.5, 1.5]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Groundtruth', 'HLOAM');
set(lg_hd, 'position', [0.740750745700882,...
                        0.249333251622547,...
                        0.214062495995313,...
                        0.239999993294478]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_yprt_horz.fig']);
% saveas(gcf, [exp_path exp_name '_yprt.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_yprt_vert.png']);



%% Plot the time evolution of orientation of vloam

% Calculate the yaw pitch rol relative to the initial position
YPR_Bvloam = quat2eul(quatmultiply(quatinv(Q(1, :)), Q));
YPR_vloam  = quat2eul(quatmultiply(quatinv(Q_vloam(1, :)), Q_vloam));


figpos = [1920 0 0 0] + [630, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
axgnd = plot(t_ori,   YPR_Bvloam(:, 1)*180/pi, 'r', 'linewidth', 4);
axh   = plot(t_vloam, YPR_vloam(:, 1)*180/pi,  'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('Yaw [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
axgnd = plot(t_ori,   YPR_Bvloam(:, 2)*180/pi, 'r', 'linewidth', 4);
axh   = plot(t_vloam, YPR_vloam(:, 2)*180/pi,  'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('Pitch [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
axgnd = plot(t_ori,   YPR_Bvloam(:, 3)*180/pi, 'r', 'linewidth', 4);
axh   = plot(t_vloam, YPR_vloam(:, 3)*180/pi,  'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
xlabel('Time [s]');
ylabel('Roll [deg]');
% ylim([-1.5, 1.5]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Groundtruth', 'VLOAM');
set(lg_hd, 'position', [0.740750745700882,...
                        0.249333251622547,...
                        0.214062495995313,...
                        0.239999993294478]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_yprt_vert.fig']);
% saveas(gcf, [exp_path exp_name '_yprt.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_yprt_vert.png']);




%% Plot the time evolution of position error
figpos = [1920 0 0 0] + [630*2, 480, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_hloam,  P_hloam_err(:, 1),  'b', 'linewidth', 2);
ylabel('X Err. [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_hloam,  P_hloam_err(:, 2),  'b', 'linewidth', 2);
% xlabel('Time [s]');
ylabel('Y Err [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_hloam,  P_hloam_err(:, 3),  'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Z Err [m]');
% ylim([-1.5, 1.5]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('H.LOAM');
set(lg_hd, 'position', [0.138344824380857,...
                        0.353488372093023,...
                        0.22,...
                        0.15]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyz_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_xyz_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyz_err_t.png']);



%% Plot the time evolution of orientation err
figpos = [1920 0 0 0] + [630*2, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_hloam,  YPR_hloam_err(:, 1)*180/pi,  'b', 'linewidth', 2);
ylabel('Yaw Err. [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_hloam, YPR_hloam_err(:, 2)*180/pi,    'b', 'linewidth', 2);
ylabel('Pitch Err. [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_hloam,  YPR_hloam_err(:, 3)*180/pi,    'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Roll Err. [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('H.LOAM');

set(lg_hd, 'position', [0.138344824380857,...
                        0.353488372093023,...
                        0.22,...
                        0.15]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_ypr_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_ypr_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_ypr_err_t.png']);



%% Plot the combined time evolution of position estimation error
figpos = [1920 0 0 0] + [930, 750, 630, 200];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_hloam, P_hloam_err(:, 1), 'r', 'linewidth', 2);
plot(t_hloam, P_hloam_err(:, 2), 'g', 'linewidth', 2);
plot(t_hloam, P_hloam_err(:, 3), 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Error [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Px error', 'Py error', 'Pz error');

set(lg_hd, 'orientation', 'horizontal',...
    'position', [0.3 0.85 0.4 0.1000]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyz_h_err_t.fig']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyz_h_err_t.png']);



%% Plot the combined time evolution of orientation estimation error
figpos = [1920 0 0 0] + [300, 750, 630, 200];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_hloam, YPR_hloam_err(:, 1)/pi*180, 'r', 'linewidth', 2);
plot(t_hloam, YPR_hloam_err(:, 2)/pi*180, 'g', 'linewidth', 2);
plot(t_hloam, YPR_hloam_err(:, 3)/pi*180, 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Error [deg]');
ylim([-8 8]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Yaw error', 'Pitch error', 'Roll error');

set(lg_hd, 'orientation', 'horizontal',...
           'position', [0.3 0.85 0.4 0.1000]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_ypr_h_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_ypr_h_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_ypr_h_err_t.png']);

end