#ifndef MEMORYDYNAMICS_HPP
#define MEMORYDYNAMICS_HPP

#include "Controller.hpp"

template <const int DOF>
class MemoryDynamics : public Controller<DOF> {
    public:
    MemoryDynamics() {
        // Load the TensorFlow Lite model.
        model =
            tflite::FlatBufferModel::BuildFromFile("/home/mason/oasis_dev/src/control/models/1d_md_v1.tflite");
        if (!model) {
            std::cerr << "Failed to load model from file." << std::endl;
            return;
        }

        // tf components: model, resolver, builder, interprete

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
  
    
    ~MemoryDynamics() {}

    int mapping(float pose, float vel) {
        // computer ouput
        float input_data[2] = {pose,vel};
        // Copy the input data into the input tensor.
        memcpy(input_tensor->data.f, input_data, 2 * sizeof(float));
        // Run inference.
        if (interpreter->Invoke() != kTfLiteOk) {
            std::cerr << "Failed to invoke interpreter." << std::endl;
            return 1;
        }
        // Get the output data.
        float output_data[3];
        memcpy(output_data, output_tensor->data.f, 3 * sizeof(float));

        float* max_output_ptr = std::max_element(output_data, 
                                                 output_data+
                                                 sizeof(output_data)/sizeof(output_data[0]));
        int max_index = std::distance(output_data, max_output_ptr);

        return max_index-1;
    }

    void compute_output() {
        if (below_tolerance()) {
            this->wp_reached = true;
            return;
        }
        // // computer ouput
        // float input_data[2] = {this->p_current(0,0),this->v_current(0,0)};

        for (int i=0;i<DOF;i++){
            this->output(i,0) = mapping(this->p_current(i,0),this->v_current(i,0));
        }
        
    }

    bool below_tolerance() {
        Eigen::Matrix<float, DOF, 1> diff;

        // Eigen::Matrix<float,DOF,1> fuckyou;
        // fuckyou(0,0) = 5;
        // fuckyou(1,0) = 5;
        // fuckyou(2,0) = 5;
        // fuckyou(3,0) = 5;
        // fuckyou(4,0) = 5;
        // fuckyou(5,0) = 5;

        diff = (this->p_current) - this->p_goal;
        // cout << "true controller error: " << diff << endl;
        // cout << "pose: " << this->p_current << endl;

        diff = diff.cwiseAbs();
        // float diff_sum = diff.sum();

        auto linear = diff.block(0,0,3,1);
        auto angular = diff.block(0,0,3,1);



        if (linear(0,0) < this->linear_tolerance && 
            linear(1,0) < this->linear_tolerance &&
            linear(2,0) < this->linear_tolerance &&
            angular(0,0) < this->angular_tolerance &&
            angular(1,0) < this->angular_tolerance &&
            angular(2,0) < this->angular_tolerance) {
            
            std::cout << "below tolerance\n";

            for (int i=0;i<DOF;i++){
                this->output(i,0) = 0;
            }

            return true;
        
        }

        // if (linear.sum() < this->linear_tolerance && 
        //     angular.sum() < this->angular_tolerance) {
            

        // }

        return false;
    }

    // Load the TensorFlow Lite model.
    std::unique_ptr<tflite::FlatBufferModel> model;

    std::unique_ptr<tflite::Interpreter> interpreter;

    
    // Get pointers to the input and output tensors.
    TfLiteTensor* input_tensor;
    TfLiteTensor* output_tensor;


};


#endif // MEMORYDYNAMICS_HPP