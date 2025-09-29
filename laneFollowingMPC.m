
%% Lane-following with MPC (pixel-based demo) - corrected version
% - Uses a discrete linearized lateral-error plant
% - Creates an mpcstate object once and updates it each frame
% - Falls back to a simple proportional controller if MPC toolbox not available

videoFile = "F:\Downloads\80400-572395752_small.mp4";
videoReader = VideoReader(videoFile);
videoPlayer = vision.VideoPlayer('Position',[100 100 960 540]);

% --- MPC design parameters (tune these) ---
Ts = 0.1;          % sampling period (s)
P = 12;            % prediction horizon (steps)
M = 3;             % control horizon (steps)
L = 200;           % effective wheelbase (in pixels) - tune for your image
v_pix = 40;        % forward speed in pixels/sec (tune)
u_min = -0.6; u_max = 0.6;   % steering bounds (radians, small angle)

% --- Check for MPC availability ---
hasMPC = (exist('mpc','file') == 2) || (exist('mpc','class') == 8);
% alternative robust check: try-catch when creating mpc object

if hasMPC
    % Continuous linearized model (small angles)
    A_cont = [0, v_pix; 0, 0];
    B_cont = [0; v_pix / L];
    C = eye(2); D = zeros(2,1);

    % Discretize
    sysd = c2d(ss(A_cont, B_cont, C, D), Ts);
    Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;

    % Create MPC object (discrete plant)
    mpcobj = mpc(ss(Ad,Bd,C,D,Ts), Ts, P, M);

    % Set weights (tune these)
    mpcobj.Weights.OutputVariables = [1.0 1.0];   % [weight_e_y, weight_e_psi]
    mpcobj.Weights.ManipulatedVariables = 0.1;     % penalize big steering
    % Optionally penalize rate of change:
    mpcobj.Weights.ManipulatedVariablesRate = 0.01;

    % Set MV limits correctly
    mpcobj.MV.Min = u_min;
    mpcobj.MV.Max = u_max;

    % Create initial MPC state object (important!)
    xmpc = mpcstate(mpcobj);

    % Reference we want (zero lateral & heading error)
    r_ref = [0; 0]; % desired e_y = 0, e_psi = 0 (column vector)
    disp('Using MPC controller');
else
    % fallback linear controller
    Kp_psi = 0.8;    % gain on heading error
    Kp_y   = 0.002;  % gain on lateral pixel error
    disp('MPC toolbox not found — using simple proportional controller');
end

% --- main processing loop ---
while hasFrame(videoReader)
    frame = readFrame(videoReader);
    [Himg, Wimg, ~] = size(frame);
    x0 = Wimg/2; y0 = Himg;    % vehicle bottom-center

    grayFrame = rgb2gray(frame);
    blurred = imgaussfilt(grayFrame, 7);
    edges = edge(blurred, 'Canny');

    % ROI mask (adjust trapezoid if needed)
    mask = poly2mask([324 1248 1100 883], [1080 1080 620 620], size(edges,1), size(edges,2));
    roi = edges & mask;

    % Hough & lines
    [H,T,R] = hough(roi);
    peaks = houghpeaks(H, 1);
    lines = houghlines(roi, T, R, peaks, 'FillGap', 3, 'MinLength', 40);

    % Collect centers of detected dashed segments
    x_centers = [];
    y_centers = [];
    for k = 1:length(lines)
        pts = [lines(k).point1; lines(k).point2]; % [x1 y1; x2 y2]
        x1 = pts(1,1); y1 = pts(1,2);
        x2 = pts(2,1); y2 = pts(2,2);
        xc = (x1 + x2)/2;
        yc = (y1 + y2)/2;
        x_centers(end+1) = xc;
        y_centers(end+1) = yc;
    end

    % Not enough segments -> skip control
    if numel(x_centers) < 2
        outFrame = insertText(frame, [10 10], 'No centerline detected', 'FontSize', 14, 'BoxColor','red');
        videoPlayer(outFrame);
        pause(0.01);
        continue;
    end

    % Fit 1st-degree polynomial: x = m*y + b (so we can sample ahead by y)
    p_center = polyfit(y_centers, x_centers, 1);
    y_ahead = linspace(Himg*0.95, Himg*0.5, 10); % sample ahead
    x_ahead = polyval(p_center, y_ahead);

    % Pick a waypoint a few steps ahead
    idx_way = 3;
    x_wp = x_ahead(idx_way);
    y_wp = y_ahead(idx_way);

    % Compute errors
    e_y = (x_wp - x0);                      % lateral pixel error (+ right)
    dx = x_wp - x0;
    dy = y0 - y_wp;                         % forward distance (pixels)
    psi_target = atan2(dx, dy);             % heading to waypoint
    e_psi = psi_target;                     % heading error

    % state vector
    xk = [e_y; e_psi];

    if hasMPC
        % Preparing measured output and reference
        ymeas = xk;          % measured outputs (we treat states as outputs)
        r = r_ref;           % desired output (0,0)
        % Call mpcmove with mpcstate object (xmpc). This updates xmpc internally.
        % Return uk (control), Info, and updated xmpc
        [uk, Info] = mpcmove(mpcobj, xmpc, xk, r, ymeas);
        disp(uk);
        delta = uk;  % steering (rad)
    else
        % fallback controller
        delta = Kp_psi * e_psi + Kp_y * e_y;
        delta = max(min(delta, u_max), u_min);
    end
    
    % Visualization
    steering_deg = rad2deg(delta);
    infoStr = sprintf('e_y=%.1f px, e_psi=%.3f rad, steer=%.1f°', e_y, e_psi, steering_deg);

    outFrame = frame;
    for i=1:length(x_ahead)-1
        outFrame = insertShape(outFrame, 'Line', [x_ahead(i) y_ahead(i) x_ahead(i+1) y_ahead(i+1)], ...
                               'Color', 'yellow','LineWidth', 3);
    end
    outFrame = insertMarker(outFrame, [x_wp, y_wp], 'o', 'Color', 'red', 'Size', 8);
    outFrame = insertText(outFrame, [10 10], infoStr, 'FontSize', 14, 'BoxColor', 'black', 'TextColor', 'white');

    videoPlayer(outFrame);

    pause(Ts*0.9);
end

release(videoPlayer);
