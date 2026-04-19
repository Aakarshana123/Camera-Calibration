clc; clear; close all;
%% ================================
%% 1. LOAD IMAGES
%% ================================
imageFolder = 'C:\Users\akars\OneDrive\Documents\MATLAB\image';
images = imageDatastore(imageFolder);

%% ================================
%% 2. DETECT CHECKERBOARD
%% ================================
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(images.Files);

images.Files = images.Files(imagesUsed);

%% 🔴 REMOVE NaN IMAGES (CRITICAL FIX)
validIdx = true(1,size(imagePoints,3));

for i = 1:size(imagePoints,3)
    pts = imagePoints(:,:,i);
    if any(isnan(pts(:))) || any(isinf(pts(:)))
        validIdx(i) = false;
    end
end

imagePoints = imagePoints(:,:,validIdx);
images.Files = images.Files(validIdx);

numImages = size(imagePoints,3);

fprintf('Final valid images used: %d\n', numImages);

if numImages < 3
    error('Not enough valid images. Use better images.');
end

%% ================================
%% 3. WORLD POINTS
%% ================================
squareSize = 5; % cm
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

%% ================================
%% 4. CAMERA CALIBRATION
%% ================================
I = readimage(images,1);
imageSize = [size(I,1), size(I,2)];

[cameraParams, ~, estimationErrors] = estimateCameraParameters( ...
    imagePoints, worldPoints, 'ImageSize', imageSize);

%% ================================
%% 5. INTRINSIC
%% ================================
A = cameraParams.IntrinsicMatrix';

disp('Intrinsic Matrix:');
disp(A);

%% ================================
%% 6. DISTORTION
%% ================================
disp('Radial Distortion:');
disp(cameraParams.RadialDistortion);

disp('Tangential Distortion:');
disp(cameraParams.TangentialDistortion);

%% ================================
%% 7. EXTRINSICS (SAFE LOOP)
%% ================================
R_all = cell(numImages,1);
t_all = cell(numImages,1);

for i = 1:numImages
    try
        [R, t] = extrinsics(imagePoints(:,:,i), worldPoints, cameraParams);
        
        R_all{i} = R;
        t_all{i} = t;
        
        fprintf('\nImage %d:\n', i);
        disp('Rotation:'); disp(R);
        disp('Translation:'); disp(t);
        
    catch
        fprintf('\nImage %d skipped (bad data)\n', i);
    end
end

%% ================================
%% 8. REPROJECTION ERROR
%% ================================
figure;
showReprojectionErrors(cameraParams);
title('Reprojection Error');

errors = cameraParams.ReprojectionErrors;

meanErrorPerImage = zeros(numImages,1);

for i = 1:numImages
    e = errors(:,:,i);
    meanErrorPerImage(i) = mean(sqrt(sum(e.^2,2)));
end

disp('Mean Error per Image:');
disp(meanErrorPerImage);

%% ================================
%% 9. BEST POSE ANALYSIS
%% ================================
[sortedError, idx] = sort(meanErrorPerImage);

fprintf('\nBest Pose Image: %d\n', idx(1));
fprintf('Worst Pose Image: %d\n', idx(end));

figure;
bar(meanErrorPerImage);
xlabel('Image Number');
ylabel('Error');
title('Pose Quality');

%% ================================
%% 10. VISUALIZATION
%% ================================
figure;
showExtrinsics(cameraParams, 'CameraCentric');

for i = 1:min(3,numImages)
    I = readimage(images,i);
    J = undistortImage(I, cameraParams);
    
    figure;
    imshowpair(I,J,'montage');
    title(['Image ', num2str(i)]);
end

%% ================================
%% FINAL SUMMARY
%% ================================
fprintf('\n===== FINAL RESULT =====\n');
fprintf('Images Used: %d\n', numImages);
fprintf('Best Pose: %d\n', idx(1));
fprintf('Avg Error: %.4f pixels\n', mean(meanErrorPerImage));