function predicted_class_name = identify_class(image_path)
    % Set the path to your Python environment with the required packages installed
    pyenv('Version', 'C:\Users\gugal\Desktop\Github\Kuka_industry\venv\Scripts\python.exe');
    
    py.importlib.import_module('numpy');
    py.importlib.import_module('cv2');
    % Import required Python modules
    py.importlib.import_module('tensorflow');
    

    % Load the model
    model = py.tensorflow.keras.models.load_model('can_classifier_model.h5', 'compile', false);

    % Constants for image resizing
    img_height = 224;
    img_width = 224;

    % Function to preprocess the image
    function image_array = preprocess_image(image_path)
        image = py.cv2.imread(image_path);
        % if isempty(image)
        %     error('Image at path ''' image_path ''' could not be loaded. Please check the file path.');
        % end
        image_rgb = py.cv2.cvtColor(image, py.cv2.COLOR_BGR2RGB);
        image_resized = py.cv2.resize(image_rgb, [img_height, img_width]);
        image_array = py.numpy.expand_dims(image_resized, 0);
        image_array = image_array / 255.0;
    end

    image_array = preprocess_image(image_path);

    % Predict the class of the image
    predictions = model.predict(image_array);
    [~, predicted_class] = max(predictions);
    class_names = ['CanTypeA', 'CanTypeB'];  % Replace with actual class names used during training
    predicted_class_name = class_names{predicted_class};
end
