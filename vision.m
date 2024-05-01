
classdef vision
    methods

        function mask = mask(obj,image, lowerBound, upperBound)
            mask = (image(:,:,1) >= lowerBound(1)) & (image(:,:,1) <= upperBound(1)) & ...
                   (image(:,:,2) >= lowerBound(2)) & (image(:,:,2) <= upperBound(2)) & ...
                   (image(:,:,3) >= lowerBound(3)) & (image(:,:,3) <= upperBound(3));
        end
        
       function [positions, sizes, highlightedImage] = detectSquaresAndRectangles(obj,binaryImage, color)
            % Convert binary image to RGB
            rgbImage = repmat(uint8(binaryImage) * 255, [1, 1, 3]); % Convert logical to uint8
        
            % Find connected components in the binary image
            cc = bwconncomp(binaryImage);
        
            % Get region properties
            stats = regionprops(cc, 'BoundingBox', 'Area', 'Centroid');
        
            % Initialize output variables
            positions = [];
            sizes = [];
            highlightedImage = rgbImage;
        
            % Loop through the regions and draw rectangles
            for i = 1:length(stats)
                boundingBox = stats(i).BoundingBox;
        
                % Highlight the region on the original image
                highlightedImage = insertShape(highlightedImage, 'Rectangle', boundingBox, 'LineWidth', 2, 'Color', color);
        
                % Save position and size information
                positions = [positions; stats(i).Centroid];
                sizes = [sizes; boundingBox(3:4)];
        
                % Display text with size and center information
                textPosition = [stats(i).Centroid(1), stats(i).Centroid(2)];
                textString = sprintf('Size: %.2f\nCenter: (%.2f, %.2f)', stats(i).Area, stats(i).Centroid(1), stats(i).Centroid(2));
                highlightedImage = insertText(highlightedImage, textPosition, textString, 'TextColor', color, 'FontSize', 8, 'BoxOpacity', 0);
            end
        
             % Display the original and highlighted images
             figure;
             subplot(1, 2, 1), imshow(binaryImage), title('Binary Image');
             subplot(1, 2, 2), imshow(highlightedImage), title('Highlighted Image');
        
        end

        function [position, all] = detect_vision(obj,image, roiXmin, roiXmax, roiYmin, roiYmax)
          
            % Define color ranges for red, green, and blue
            lowerRed = [70, 20, 23];
            upperRed = [75, 25, 30];
                
            lowerGreen = [120, 90, 10];
            upperGreen = [130, 100, 20];
                
            lowerBlue = [8, 60, 110];
            upperBlue = [38, 90, 130];
                
            % Create binary masks for each color using ColorThresholder
            redMask = obj.mask(image, lowerRed, upperRed);
            greenMask = obj.mask(image, lowerGreen, upperGreen);
            blueMask = obj.mask(image, lowerBlue, upperBlue);
                
            % Apply the masks to the original image
            redImage = bsxfun(@times, image, uint8(redMask));
            greenImage = bsxfun(@times, image, uint8(greenMask));
            blueImage = bsxfun(@times, image, uint8(blueMask));
        
                
            %add filters to better filter the image
            % Extract individual color channels from the masked images
            redChannelFiltered = redImage(:, :, 1);
            greenChannelFiltered = greenImage(:, :, 2);
            blueChannelFiltered = blueImage(:, :, 3);
                
            % Apply the Gaussian filter to each channel
            g_filter = fspecial('gaussian');
                
            Rfilter = filter2(g_filter, redChannelFiltered);
            Gfilter = filter2(g_filter, greenChannelFiltered);
            Bfilter = filter2(g_filter, blueChannelFiltered);
                
            subplot(2, 2, 1);
            imshow(Rfilter);
            title('Red Filtered');
            
            subplot(2, 2, 2);
            imshow(Gfilter);
            title('Yellow Filtered');
            
            subplot(2, 2, 3);
            imshow(Bfilter);
            title('Blue Filtered');
            

            % ----- Circles in each channel ----- %
        
            % Define parameters for circle detection
            % radiusRange = [10, 50]; % adjust the radius range as needed
            % sensitivity = 0.90; % adjust sensitivity as needed
            % 
            % % Detect circles in each filtered channel and its info
            % [centersR, radiiR, metricR] = imfindcircles(Rfilter, radiusRange, 'Sensitivity', sensitivity);
            % [centersG, radiiG, metricG] = imfindcircles(Gfilter, radiusRange, 'Sensitivity', sensitivity);
            % [centersB, radiiB, metricB] = imfindcircles(Bfilter, radiusRange, 'Sensitivity', sensitivity);
            % 
            % ----- Squares in each channel ----- %
            %pass filter image to binary
            bin_red = imbinarize(Rfilter);
            bin_green = imbinarize(Gfilter);
            bin_blue = imbinarize(Bfilter);
        
            [Rsq_xy, Rsq_size, Rsq_img] = obj.detectSquaresAndRectangles(bin_red, 'red');
            [Gsq_xy, Gsq_size, Gsq_img] = obj.detectSquaresAndRectangles(bin_green, 'green');
            [Bsq_xy, Bsq_size, Bsq_img] = obj.detectSquaresAndRectangles(bin_blue, 'blue');
                
        
        
                %squares
                 subplot(2,3,4);
                 imshow(Rsq_img);
                 title('Red Squares');
                 subplot(2,3,5);
                 imshow(Gsq_img);
                 title('Green Squares');
                 subplot(2,3,6);
                 imshow(Bsq_img);
                 title('Blue Squares');
        
            % ----- define Priorities -----

            % Verifica se foram detectados quadrados vermelhos
            if ~isempty(Rsq_xy)
                fprintf("\nRed:\nSquares xy: %.2f %.2f", Rsq_xy(1,1), Rsq_xy(1,2));
            end
            
            % Verifica se foram detectados quadrados verdes
            if ~isempty(Gsq_xy)
                fprintf("\nGreen:\nSquares xy: %.2f %.2f", Gsq_xy(1,1), Gsq_xy(1,2));
            end
            
            % Verifica se foram detectados quadrados azuis
            if ~isempty(Bsq_xy)
                fprintf("\nBlue:\nSquares xy: %.2f %.2f", Bsq_xy(1,1), Bsq_xy(1,2));
            end
        
            % Calculate distances from squares to ROI
            distR = sqrt((Rsq_xy(:, 1) - roiXmin).^2 + (Rsq_xy(:, 2) - roiYmin).^2);
            distG = sqrt((Gsq_xy(:, 1) - roiXmin).^2 + (Gsq_xy(:, 2) - roiYmin).^2);
            distB = sqrt((Bsq_xy(:, 1) - roiXmin).^2 + (Bsq_xy(:, 2) - roiYmin).^2);
        
            % Combine distances and positions
            allDistances = [distR; distG; distB];
            allPositions = [Rsq_xy; Gsq_xy; Bsq_xy];
            
            all = allPositions;
        
            % Find the index of the nearest square
            [~, minIdx] = min(allDistances);
            
            % Check if any part of the square is inside the ROI
            if (allPositions(minIdx, 1) >= roiXmin && allPositions(minIdx, 1) <= roiXmax) && ...
               (allPositions(minIdx, 2) >= roiYmin && allPositions(minIdx, 2) <= roiYmax)
                fprintf("\nNearest square position:\nPosition: %.2f %.2f", allPositions(minIdx, 1), allPositions(minIdx, 2));
            else
                fprintf("\nThe nearest square is not inside the specified ROI.");
            end
        
            % Return the position of the nearest square
            position = allPositions(minIdx, :);
    
        end

        function dominantColor = findDominantColor(obj, image)
            % Define color ranges for red, green, and blue
            lowerRed = [70, 20, 23];
            upperRed = [75, 25, 30];
                
            lowerYellow = [120, 90, 10];
            upperYellow = [130, 100, 20];
                
            lowerBlue = [8, 60, 110];
            upperBlue = [38, 90, 130];
                
            % Create binary masks for each color using ColorThresholder
            redMask = obj.mask(image, lowerRed, upperRed);
            greenMask = obj.mask(image, lowerYellow, upperYellow);
            blueMask = obj.mask(image, lowerBlue, upperBlue);
                
            % Apply the masks to the original image
            redImage = bsxfun(@times, image, uint8(redMask));
            greenImage = bsxfun(@times, image, uint8(greenMask));
            blueImage = bsxfun(@times, image, uint8(blueMask));
        
                
            %add filters to better filter the image
            % Extract individual color channels from the masked images
            redChannelFiltered = redImage(:, :, 1);
            greenChannelFiltered = greenImage(:, :, 2);
            blueChannelFiltered = blueImage(:, :, 3);
                
            % Apply the Gaussian filter to each channel
            g_filter = fspecial('gaussian');
                
            Rfilter = filter2(g_filter, redChannelFiltered);
            Gfilter = filter2(g_filter, greenChannelFiltered);
            Bfilter = filter2(g_filter, blueChannelFiltered);
            
            figure(1)  
            subplot(2, 2, 1);
            imshow(Rfilter);
            title('Red Filtered');
            
            subplot(2, 2, 2);
            imshow(Gfilter);
            title('Yellow Filtered');
            
            subplot(2, 2, 3);
            imshow(Bfilter);
            title('Blue Filtered');

            % Count the number of pixels in each filtered image
            redPixels = sum(redMask(:));
            yellowPixels = sum(greenMask(:));
            bluePixels = sum(blueMask(:));

            % Determine the dominant color
            [max_count , idx] = max([redPixels, yellowPixels, bluePixels]);
            if max_count > 0
                disp('Dominant color found: ')
                switch idx
                    case 1
                        disp('Dominant Color: Red ');
                        disp(redPixels);
                        dominantColor = 'Salsichas';
                    case 2
                        disp('Dominant Color: Yellow ');
                        disp(yellowPixels);
                        dominantColor = 'Salsichas';
                    case 3
                        disp('Dominant Color: Blue ');
                        disp(bluePixels);
                        dominantColor = 'Cogumelos';
                end
            else
                disp('NO color found');
                dominantColor = 'NOT RECOGNIZED';
            end

        end

    end
end


