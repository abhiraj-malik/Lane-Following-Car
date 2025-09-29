img = imread("C:\Users\ABHIRAJ\Pictures\Screenshots\Screenshot 2025-09-21 134718.png");
figure;
grayFrame = rgb2gray(img);
gaussImg = imgaussfilt(grayFrame, 5);
edges = edge(gaussImg, 'Canny');
mask = poly2mask([180 700 600 490], [559 559 310 310], size(edges,1), size(edges,2));
roi = edges & mask;
[H,T,R] = hough(roi);
imshow(imadjust(rescale(H)),'XData',T,'YData',R,...
    'InitialMagnification','fit');
peaks = houghpeaks(H,1);
lines = houghlines(roi,T,R,peaks, 'FillGap', 3,'MinLength', 40);
figure, imshow(img), hold on;
disp(length(lines));
for k = 1:length(lines)
    % Get endpoints
    xy = [lines(k).point1; lines(k).point2];  % 2x2 matrix [x1 y1; x2 y2]
    
    % Plot line
    plot(xy(:,1), xy(:,2), 'LineWidth', 2, 'Color', 'green');
    
    % Optionally plot endpoints
    % plot(xy(1,1), xy(1,2), 'x', 'Color', 'yellow');
    % plot(xy(2,1), xy(2,2), 'x', 'Color', 'red');
end