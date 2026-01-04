% planar manipulator 
function planar_actuated_manipulator_line_demo_with_video()

% 3R planar manipulator with synthetic actuator stroke mapping
% following a straight line segment with fixed slope angle
%
% Task:
%   End-effector tracks p(t) = p0 + d(t) * [cos(alpha), sin(alpha)]
%   where d(t) goes from 0 to Len with a smooth cubic profile
%
% Outputs:
%   On-screen animation of a 3-link planar arm
%   Video file
%
% Notes:
%   All parameters are synthetic

clc; 
close all;

%% Synthetic manipulator parameters -----------------------
% Link lenghts (unit)
L1 = 0.9;
L2 = 0.7;
L3 = 0.4;

% Joint limits (rad)
th_lim = [-pi,  pi;
          -pi,  pi;
          -pi,  pi];

% Synthetic actuator stroke mapping (non-proprietary)
act(1) = struct('s0', 0.40, 'a', 0.12, 'b', 0.05, 'smin', 0.35, 'smax', 0.60);
act(2) = struct('s0', 0.45, 'a', 0.15, 'b', 0.06, 'smin', 0.38, 'smax', 0.70);
act(3) = struct('s0', 0.30, 'a', 0.10, 'b', 0.04, 'smin', 0.25, 'smax', 0.50);

%% Line task definition -----------------------
alpha_deg = -50;                 % fixed slope angle (deg)
alpha = deg2rad(alpha_deg);

Len = 0.5;                      % line length (units) 
p0  = [0.9, 0.8];               % start point 
dir = [cos(alpha), sin(alpha)]; 

% Timing
dt = 0.03;
T  = 5.0;
[t, d, d_dot, d_ddot] = cubic_scalar_profile(0, Len, T, dt);

% Desired path
p_des = p0 + d(:) * dir;

% Desired end-effector orientation:
phi_des = alpha * ones(size(t));

N = numel(t);

%% Inverse Kinematic solve per step -----------------------
theta  = nan(N,3);
stroke = nan(N,3);
pose   = nan(N,3);
status = strings(N,1);

theta_seed = [0.2, 0.7, -0.4];

for k = 1:N

    xd = p_des(k,1); 
    yd = p_des(k,2); 
    phid = phi_des(k);

    [th, ok, msg] = ik_3R_planar(xd, yd, phid, L1, L2, L3, th_lim, theta_seed);
    status(k) = msg;

    if ok
        theta(k,:) = th;
        theta_seed = th; % continuity
        pose(k,:)  = fk_3R_planar(th, L1, L2, L3);

        [s, s_ok, s_msg] = map_theta_to_stroke(th, act);
        if s_ok
            stroke(k,:) = s;
        else
            status(k) = status(k) + " | " + s_msg;
        end
    else
        if k > 1 && all(isfinite(theta(k-1,:)))
            theta_seed = theta(k-1,:);
        end
    end
end

fprintf("Solved %d/%d IK steps successfully.\n", sum(all(isfinite(theta),2)), N);

%% Auto-scale (ensure full arm visible) -----------------------
Xall = []; Yall = [];
for k = 1:N
    if ~all(isfinite(theta(k,:))), continue; end
    th = theta(k,:);
    [x1,y1,x2,y2,x3,y3] = joint_positions(th, L1, L2, L3);
    Xall = [Xall, 0, x1, x2, x3];
    Yall = [Yall, 0, y1, y2, y3];
end

if isempty(Xall)
    error('No feasible IK solutions. Try smaller Len, different p0, or change link lengths.');
end

pad = 0.25*(L1+L2+L3);
xmin = min([Xall, p_des(:,1)']) - pad;
xmax = max([Xall, p_des(:,1)']) + pad;
ymin = min([Yall, p_des(:,2)']) - pad;
ymax = max([Yall, p_des(:,2)']) + pad;

%% Animation and Video export -----------------------
fig = figure('Name','Planar Manipulator Line Tracking (Generic)','Color','w');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlim(ax, [xmin xmax]);
ylim(ax, [ymin ymax]);

% Desired path
plot(ax, p_des(:,1), p_des(:,2), 'k--', 'LineWidth', 1.2);

% Start/End markers
plot(ax, p_des(1,1), p_des(1,2), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 6);
plot(ax, p_des(end,1), p_des(end,2), 'ks', 'MarkerFaceColor','k', 'MarkerSize', 7);

% Links and joints
hLinks  = plot(ax, [0 0], [0 0], 'LineWidth', 3);
hJoints = plot(ax, [0 0 0], [0 0 0], 'ko', 'MarkerFaceColor','k', 'MarkerSize', 5);
hEE     = plot(ax, 0, 0, 'ks', 'MarkerFaceColor','k', 'MarkerSize', 6);

% Trace of actual end-effector
hTrace = plot(ax, nan, nan, 'k-', 'LineWidth', 1);

title(ax, sprintf('3R Planar Manipulator | Line Tracking (alpha=%g° , Len=%g)', alpha_deg, Len));
xlabel(ax, 'X'); ylabel(ax, 'Y');

% Video writer
videoFileMP4 = 'demo_line_motion.mp4';
videoFileAVI = 'demo_line_motion.avi';
try
    vw = VideoWriter(videoFileMP4, 'MPEG-4');
catch
    vw = VideoWriter(videoFileAVI, 'Motion JPEG AVI');
end
vw.FrameRate = round(1/dt);
open(vw);

txtHandle = text(ax, 0.02, 0.98, '', 'Units','normalized', 'VerticalAlignment','top');

traceX = [];
traceY = [];

for k = 1:N
    if ~all(isfinite(theta(k,:)))
        continue;
    end
    th = theta(k,:);
    [x1,y1,x2,y2,x3,y3] = joint_positions(th, L1, L2, L3);

    set(hLinks,  'XData', [0 x1 x2 x3], 'YData', [0 y1 y2 y3]);
    set(hJoints, 'XData', [0 x1 x2],    'YData', [0 y1 y2]);
    set(hEE,     'XData', x3,           'YData', y3);

    traceX(end+1) = x3; %#ok<AGROW>
    traceY(end+1) = y3; %#ok<AGROW>
    set(hTrace, 'XData', traceX, 'YData', traceY);

    set(txtHandle, 'String', sprintf('t=%.2fs | d=%.2f | th=[%.2f %.2f %.2f] | s=[%.3f %.3f %.3f]', ...
        t(k), d(k), th(1), th(2), th(3), stroke(k,1), stroke(k,2), stroke(k,3)));

    drawnow;
    writeVideo(vw, getframe(fig));
end

close(vw);

if exist(videoFileMP4,'file')
    fprintf("Video saved: %s\n", videoFileMP4);
elseif exist(videoFileAVI,'file')
    fprintf("Video saved: %s\n", videoFileAVI);
end

end

%% Helper functions -----------------------

function [t, d, d_dot, d_ddot] = cubic_scalar_profile(d0, df, tf, dt)
% Smooth cubic scalar profile with zero start/end velocity
t = 0:dt:tf;
a0 = d0;
a1 = 0;
a2 = 3*(df - d0)/tf^2;
a3 = -2*(df - d0)/tf^3;
d = a0 + a1*t + a2*t.^2 + a3*t.^3;
d_dot = a1 + 2*a2*t + 3*a3*t.^2;
d_ddot = 2*a2 + 6*a3*t;
end

function pose = fk_3R_planar(th, L1, L2, L3)
t1 = th(1); t2 = th(2); t3 = th(3);
x = L1*cos(t1) + L2*cos(t1+t2) + L3*cos(t1+t2+t3);
y = L1*sin(t1) + L2*sin(t1+t2) + L3*sin(t1+t2+t3);
phi = wrapToPi(t1+t2+t3);
pose = [x, y, phi];
end

function [x1,y1,x2,y2,x3,y3] = joint_positions(th, L1, L2, L3)
t1 = th(1); t2 = th(2); t3 = th(3);
x1 = L1*cos(t1);               y1 = L1*sin(t1);
x2 = x1 + L2*cos(t1+t2);       y2 = y1 + L2*sin(t1+t2);
x3 = x2 + L3*cos(t1+t2+t3);    y3 = y2 + L3*sin(t1+t2+t3);
end

function [th, ok, msg] = ik_3R_planar(xd, yd, phid, L1, L2, L3, th_lim, th_seed)
% 3R planar IK: reduce to 2R on wrist, then set theta3 from phi
ok = false; th = [nan nan nan]; msg = "IK failed";

xw = xd - L3*cos(phid);
yw = yd - L3*sin(phid);

r2 = xw^2 + yw^2;
c2 = (r2 - L1^2 - L2^2) / (2*L1*L2);
if abs(c2) > 1
    msg = "Unreachable (2R)";
    return;
end

s2_pos =  sqrt(max(0,1-c2^2));
s2_neg = -sqrt(max(0,1-c2^2));

cand = zeros(2,3);
t2a = atan2(s2_pos, c2);
t1a = atan2(yw, xw) - atan2(L2*sin(t2a), L1 + L2*cos(t2a));
t3a = wrapToPi(phid - (t1a + t2a));
cand(1,:) = [wrapToPi(t1a), wrapToPi(t2a), t3a];

t2b = atan2(s2_neg, c2);
t1b = atan2(yw, xw) - atan2(L2*sin(t2b), L1 + L2*cos(t2b));
t3b = wrapToPi(phid - (t1b + t2b));
cand(2,:) = [wrapToPi(t1b), wrapToPi(t2b), t3b];

valid = false(2,1);
for i=1:2
    valid(i) = all(cand(i,:) >= th_lim(:,1)' & cand(i,:) <= th_lim(:,2)');
end
if ~any(valid)
    msg = "Joint limits";
    return;
end

seed = th_seed(:)';
dist = sum((wrapToPi(cand - seed)).^2, 2);
dist(~valid) = inf;
[~, idx] = min(dist);
th = cand(idx,:);
ok = true;
msg = "OK";
end

function [s, ok, msg] = map_theta_to_stroke(th, act)
% Synthetic stroke mapping; checks stroke limits
ok = false; msg = "Stroke mapping failed";
s = nan(1,3);
for i=1:3
    ti = th(i);
    si = act(i).s0 + act(i).a*(1 - cos(ti)) + act(i).b*sin(ti);
    s(i) = si;
    if si < act(i).smin || si > act(i).smax
        msg = sprintf("Stroke limit violated on actuator %d", i);
        return;
    end
end
ok = true; msg = "OK";
end

