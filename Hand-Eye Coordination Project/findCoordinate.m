function [output, objCount] = findCoordinate(x, thresholdValue)
    % Convert RGB image to chosen color space
    I = rgb2lab(x);

    % Define thresholds for channel 1 based on histogram settings
    channel1Min = thresholdValue(1);
    channel1Max = thresholdValue(2);

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = thresholdValue(3);
    channel2Max = thresholdValue(4);

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = thresholdValue(5);
    channel3Max = thresholdValue(6);

    % Create mask based on chosen histogram thresholds
    sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    BW = sliderBW;
    
    cleanedImg = imfill(BW, 'holes');
    cleanedImg = medfilt2(cleanedImg);
    cleanedImg = bwareaopen(cleanedImg,50);
    
    output = regionprops(cleanedImg, 'Orientation', 'Centroid');
    
    label = bwlabel(cleanedImg);
    objCount = max(max(label));
end