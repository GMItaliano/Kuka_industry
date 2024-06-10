% Terminate any existing Python environment
% terminate(pyenv);

% Set the Python environment
pe = pyenv('Version', 'C:\Users\gugal\Desktop\Github\Kuka_industry\venv\Scripts\python.exe');
disp(pe);

% Add user site-packages directory to the Python path
user_site_packages = 'C:\Users\gugal\AppData\Roaming\Python\Python39\site-packages';

if count(py.sys.path, user_site_packages) == 0
    insert(py.sys.path, int32(0), user_site_packages);
end

disp(py.sys.path);

% Ensure the directory containing the Python script is in the Python path
py_Vision_path = 'C:\Users\gugal\Desktop\Github\Kuka_industry\Vision_ML'; % Replace with the correct path
disp(['Python Vision Path: ', py_Vision_path]);

if count(py.sys.path, py_Vision_path) == 0
    insert(py.sys.path, int32(0), py_Vision_path);
end

disp(py.sys.path);

% Import the required Python module
try
    vision = py.importlib.import_module('simple_sai');
    disp('Python module imported successfully'); 
catch e
    disp('Failed to import Python module');
    disp(e.message);
end

% Load an image in MATLAB
image = imread('C:/Users/gugal/Desktop/Github/Kuka_industry/images/cogumelos_1.png');

% Save the received image to a PNG file
output_image_path = 'C:/Users/gugal/Desktop/Github/Kuka_industry/Vision_ML/received_image.png';
imwrite(image, output_image_path);

% Define the Python function to call
python_function = vision.identify_class;

% Call the Python function and pass the path of the saved image
try
    predicted_class_name = python_function(output_image_path);
    disp(['Predicted class: ', char(predicted_class_name)]);
catch e
    disp('Failed to process image');
    disp(e.message);
end
