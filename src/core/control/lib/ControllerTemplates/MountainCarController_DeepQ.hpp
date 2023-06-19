#ifndef MOUNTAINCARCONTROLLER_DEEPQ_HPP
#define MOUNTAINCARCONTROLLER_DEEPQ_HPP

#include "Controller.hpp"

template <const int DOF>
class MountainCarController_DeepQ : public Controller<DOF> {
    public:
    MountainCarController_DeepQ() {
        // Load the TensorFlow Lite model.
        model =
            tflite::FlatBufferModel::BuildFromFile("/home/mason/oasis_dev/src/control/models/mountain_car.tflite");
        if (!model) {
            std::cerr << "Failed to load model from file." << std::endl;
            return;
        }

        // Build the TensorFlow Lite interpreter.
        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder builder(*model, resolver);
        if (builder(&interpreter) != kTfLiteOk) {
            std::cerr << "Failed to build interpreter." << std::endl;
            return;
        }

        // Allocate memory for the input and output tensors.
        if (interpreter->AllocateTensors() != kTfLiteOk) {
            std::cerr << "Failed to allocate tensors." << std::endl;
            return;
        }

        // Get pointers to the input and output tensors.
        input_tensor = interpreter->input_tensor(0);
        output_tensor = interpreter->output_tensor(0);
    }
  
    
    ~MountainCarController_DeepQ() {}

    void compute_output() {
        cout << "computing output\n";
        if (below_tolerance()) {
            this->wp_reached = true;
            return;
        }
        // computer ouput
        float input_data[2] = {this->p_current(0,0),this->v_current(0,0)};
        
        // Copy the input data into the input tensor.
        memcpy(input_tensor->data.f, input_data, 2 * sizeof(float));

        // Run inference.
        if (interpreter->Invoke() != kTfLiteOk) {
            std::cerr << "Failed to invoke interpreter." << std::endl;
            return;
        }

        // Get the output data.
        float output_data[3];
        memcpy(output_data, output_tensor->data.f, 3 * sizeof(float));

        float* max_output_ptr = std::max_element(output_data, 
                                                 output_data+
                                                 sizeof(output_data)/sizeof(output_data[0]));
        int max_index = std::distance(output_data, max_output_ptr);
        this->output(0,0) = max_index-1;
    }

    bool below_tolerance() {
        Eigen::Matrix<float, DOF, 1> diff;

        diff = this->p_current - this->p_goal;
        diff = diff.cwiseAbs();
        // float diff_sum = diff.sum();

        auto linear = diff.block(0,0,3,1);
        auto angular = diff.block(0,0,3,1);

        std::cout << "error in pose: " << diff << std::endl;
        std::cout << "mason\n" << std::endl;

        if (linear.sum() < this->linear_tolerance && 
            angular.sum() < this->angular_tolerance) {
            std::cout << "linear: " << linear << std::endl;
            std::cout << "angular: " << angular << std::endl;
            
            std::cout << "below tolerance\n";
            return true;
        }

        return false;
    }

    // Load the TensorFlow Lite model.
    std::unique_ptr<tflite::FlatBufferModel> model;

    std::unique_ptr<tflite::Interpreter> interpreter;

    
    // Get pointers to the input and output tensors.
    TfLiteTensor* input_tensor;
    TfLiteTensor* output_tensor;


};


#endif // MOUNTAINCARCONTROLLER_DEEPQ