clear all
close all
clc

%%
% vis
r = 5;

% img size
H = 125;
W = 400;

% pixel loaction
X = 200;
Y = 5;

figure('Position', [1,1,1080,1920])

% angle
for alpha = -90:2:90 % [deg]
    
    %% create image
    img = true(H,W);
    img(Y - 1:Y + 1,X - 1:X + 1) = false;
    
    imgRot = imrotate(img,alpha);
    [h, w] = size(imgRot);
    
    % find location in rotated image
    CC = bwconncomp(~imgRot);
    numPixels = cellfun(@numel,CC.PixelIdxList);
    [biggest,idx] = min(numPixels);
    [y, x] = ind2sub([h,w],CC.PixelIdxList{idx}); % [row, col]
    x = median(x);
    y = median(y);
    
    [X_comp, Y_comp] = rot2or([x, y], H, W, deg2rad(alpha));
    
    %% results
    subplot(2,2, 1)
    imshow(img)
    viscircles([X, Y],r, 'Color', 'r');
    
    subplot(2,2, 2)
    imshow(imgRot)
    viscircles([x, y],r, 'Color', 'r');
    
    subplot(2, 2, 3)
    imshow(img)
    viscircles([X_comp, Y_comp],r, 'Color', 'r');
    drawnow
end