 % Load an example image
%img = imread('lata_salsichas.png'); 
img = imread('salsichas2.png');
%img = imread('green.png'); 

obj = vision; 

dominantColor = obj.findDominantColor(img);
%image = obj.filterImageByColor(img);
%[~,~] = obj.detect_vision(img,20,20,20,20)
figure(2)
imshow(img);
% Display the dominant color
disp(['Object: ', dominantColor]);