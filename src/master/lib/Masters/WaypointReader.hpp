#ifndef WAYPOINTREADER_HPP
#define WAYPOINTREADER_HPP

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include "../MasterTemplates/MasterTemplate.hpp"

using namespace std;

class WaypointReader : public MasterTemplate {
    public:
    WaypointReader() {};
    ~WaypointReader() {};

    void plan_steps() {
        this->get_data("/home/mason/oasis_dev/src/master/Waypoints/master_waypoints.txt");
        this->steps = waypoints;
        this->current_step = waypoints[0];
        this->num_of_steps = waypoints.size();
    }

    void get_data(string filename, bool print_ouput=false) {
        std::ifstream inFile;
        string line;

        inFile.open(filename);

        if (!inFile) {
            cerr << "Unable to open file\n";
            return;
        }

        char space = ' ';

        while (getline(inFile, line)) {

            size_t pos = line.find(space);
            vector<float> line_vector;

            string dof;
            int i;
            char dof_let;            

            while (pos != string::npos) {

                dof = "";
                i = 1;
                dof_let = line[pos-i];                
                while (true) {
                    if (dof_let == '/' || dof_let == ' ') {
                        break;
                    }
                    i+=1;
                    dof.insert(0,1,dof_let);
                    dof_let = line[pos-i];
                }
                // store value in vector
                line_vector.push_back(this->string_to_float(dof));

                // search for next pose
                pos = line.find(space, pos+1);
            }

            // // store value in vector
            // line_vector.push_back(this->char_to_float(line.back()));

            // push goal pose into waypoints vector
            waypoints.push_back(line_vector);
        }

        if (print_ouput) {
            this->print_wps();
        }
    }

    float char_to_float(char c) {
        // convert character to string:
        string s(1, c);
        // covert string to float:
        return std::stof(s);
    }

    float string_to_float(string s) {
        return std::stof(s);
    }

    void print_wps() {
        for (size_t i =0; i<waypoints.size(); i++) {
            for (size_t j=0; j<waypoints[0].size(); j++) {
                cout << waypoints[i][j] << " ";
            }
            cout << "\n";
        }
    }

    vector<vector<float>> waypoints;
    string filename;
};

#endif // WAYPOINTREADER

