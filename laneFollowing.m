% videoFile = "F:\Downloads\80400-572395752_small.mp4"; 
% videoReader = VideoReader(videoFile);
% videoPlayer = vision.VideoPlayer;
% 
% 
% while hasFrame(videoReader)
%    frame = readFrame(videoReader);
%    grayFrame = rgb2gray(frame);
%    blurred = imgaussfilt(grayFrame, 15);
%    edges = edge(blurred, 'Canny');
%    mask = poly2mask([324 1248 1100 883], [1080 1080 620 620], size(edges,1), size(edges,2));
%    roi = edges & mask;
%    [H,T,R] = hough(roi);
%    peaks = houghpeaks(H,1);
%    lines = houghlines(roi,T,R,peaks, 'FillGap', 3,'MinLength', 10);
% 
%    outFrame = frame;
%    for k = 1:length(lines)
% 
%        xy = [lines(k).point1; lines(k).point2];
%        outFrame = insertShape(outFrame, 'Line', xy, 'LineWidth', 4, 'Color', 'green');
% 
% 
%    end
%    videoPlayer(roi); 
% end

videoFile = "F:\Downloads\80400-572395752_small.mp4"; 
videoReader = VideoReader(videoFile);
videoPlayer = vision.VideoPlayer;

while hasFrame(videoReader)
    % Read frame
    frame = readFrame(videoReader);
    grayFrame = rgb2gray(frame);

    % Preprocessing
    blurred = imgaussfilt(grayFrame, 15);
    edges = edge(blurred, 'Canny');

    % Region of interest (polygon mask)
    mask = poly2mask([324 1248 1100 883], [1080 1080 620 620], size(edges,1), size(edges,2));
    roi = edges & mask;

    % Hough transform
    [H,T,R] = hough(roi);
    peaks = houghpeaks(H, 1);   % detect up to 2 lane lines
    lines = houghlines(roi, T, R, peaks, 'FillGap', 3, 'MinLength', 40);

    % Overlay detected lines on original frame
    outFrame = frame;   % copy original frame
    for k = 1:length(lines)
        xy = [lines(k).point1, lines(k).point2]; % [x1 y1 x2 y2]
        x1 = xy(:,1); y1 = xy(:,2); x2 = xy(:,3); y2 = xy(:,4);
        x_center = (x1 + x2)/2; 
        y_center = (y1 + y2)/2;
        disp(x_center);
        disp(y_center);
        outFrame = insertShape(outFrame, 'Line', xy, 'LineWidth', 4, 'Color', 'green');
    end

    % Display
    videoPlayer(outFrame);
end


