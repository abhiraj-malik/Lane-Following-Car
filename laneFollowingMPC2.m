

videoFile = "F:\Downloads\80400-572395752_small.mp4";
videoReader = VideoReader(videoFile);
videoPlayer = vision.VideoPlayer('Position',[100 100 960 540]);

% MPC design parameters (tune these) 
Ts = 0.1;          % sampling period (s)
P = 12;            % prediction horizon (steps)
M = 3;             % control horizon (steps)
L = 200;           % effective wheelbase (in pixels)
v_pix = 40;        % forward speed in pixels/sec 
u_min = -0.6; u_max = 0.6;   % steering limits (radians)


hasMPC = (exist('mpc','file') == 2) || (exist('mpc','class') == 8);

if hasMPC
    
    A_cont = [0, v_pix; 0, 0];
    B_cont = [0; v_pix / L];
    C = eye(2); D = zeros(2,1);

    % discretize
    sysd = c2d(ss(A_cont, B_cont, C, D), Ts);
    Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;

    mpcobj = mpc(ss(Ad,Bd,C,D,Ts), Ts, P, M);

   
    mpcobj.Weights.OutputVariables = [1.0 1.0];   % [weight_e_y, weight_e_psi]
    mpcobj.Weights.ManipulatedVariables = 0.1;     % penalize big steering
    mpcobj.Weights.ManipulatedVariablesRate = 0.01;

   
    mpcobj.MV.Min = u_min;
    mpcobj.MV.Max = u_max;

    
    xmpc = mpcstate(mpcobj);

   
    r_ref = [0; 0]; % desired e_y = 0, e_psi = 0 (column vector)
    disp('Using MPC controller');
else
    
    Kp_psi = 0.8;   
    Kp_y   = 0.002; 
    disp('MPC toolbox not found — using simple proportional controller');
end


while hasFrame(videoReader)
    frame = readFrame(videoReader);
    [Himg, Wimg, ~] = size(frame);
    x0 = Wimg/2; y0 = Himg;    
    % creating canny frames
    grayFrame = rgb2gray(frame);
    blurred = imgaussfilt(grayFrame, 7);
    edges = edge(blurred, 'Canny');

    
    mask = poly2mask([324 1248 1100 883], [1080 1080 620 620], size(edges,1), size(edges,2));
    roi = edges & mask;

    % hough lines for car to follow
    [H,T,R] = hough(roi);
    peaks = houghpeaks(H, 1);
    lines = houghlines(roi, T, R, peaks, 'FillGap', 3, 'MinLength', 40);

   
    x_centers = [];
    y_centers = [];
    for k = 1:length(lines)
        pts = [lines(k).point1; lines(k).point2]; % [x1 y1; x2 y2]
        % lines points
        x1 = pts(1,1); y1 = pts(1,2);
        x2 = pts(2,1); y2 = pts(2,2);
        xc = (x1 + x2)/2;
        yc = (y1 + y2)/2;
        x_centers(end+1) = xc;
        y_centers(end+1) = yc;
    end

   
    if numel(x_centers) < 2
        outFrame = insertText(frame, [10 10], 'No centerline detected', 'FontSize', 14, 'BoxColor','red');
        videoPlayer(outFrame);
        pause(0.01);
        continue;
    end

    % fit 1st-degree polynomial: x = m*y + b (so we can sample ahead by y)
    p_center = polyfit(y_centers, x_centers, 1);
    y_ahead = linspace(Himg*0.95, Himg*0.5, 10); 
    x_ahead = polyval(p_center, y_ahead);

    % pick a waypoint that is few steps ahead of Car
    idx_way = 3;
    x_wp = x_ahead(idx_way);
    y_wp = y_ahead(idx_way);

    e_y = (x_wp - x0);                      
    dx = x_wp - x0;
    dy = y0 - y_wp;
    % target steer angle or reference
    psi_target = atan2(dx, dy);            
    e_psi = psi_target;                     

    % state vector
    xk = [e_y; e_psi];

    if hasMPC
        ymeas = xk;          % measured outputs (we treat states as outputs)
        r = r_ref;           % desired output (0,0)
        
        [uk, Info] = mpcmove(mpcobj, xmpc, xk, r, ymeas);
        disp(uk);
        delta = uk;  % steering (rad)
    else
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
