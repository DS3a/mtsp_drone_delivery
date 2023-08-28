#ifndef READ_CSV_H_
#define READ_CSV_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace read_csv{

std::vector<std::vector<double>> read_csv_drones(std::string file_name_drones);
std::vector<std::vector<double>> read_csv_payloads(std::string file_name_payloads);

std::vector<std::vector<double> > read_csv_drones(std::string file_name_drones) {
    // Define vectors to store the values
    std::vector<std::vector<double>> drone_params;
    std::vector<double> capacity;
    std::vector<double> avg_velocity;
    std::vector<double> initial_position_x;
    std::vector<double> initial_position_y;

    // Open the CSV file
   
    std::ifstream file(file_name_drones); // Adjust the file name as needed

    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;

    }
    std::string header;
    if (std::getline(file, header)) {
        
        std::cout << "Header: " << header << std::endl;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        double value;

        // Read and parse each value from the CSV line
        if (std::getline(iss, token, ',') && std::istringstream(token) >> value) {
            capacity.push_back(value);
        }

        if (std::getline(iss, token, ',') && std::istringstream(token) >> value) {
            avg_velocity.push_back(value);
        }

        if (std::getline(iss, token, ',') && std::istringstream(token) >> value) {
            initial_position_x.push_back(value);
            std::cout<<"initial_x:"<<value<<std::endl;
        }

        if (std::getline(iss, token) && std::istringstream(token) >> value) {
            initial_position_y.push_back(value);
            std::cout<<"initial_y:"<<value<<std::endl;
        }
    }

    file.close(); // Close the file when done

    drone_params.push_back(initial_position_x);
    drone_params.push_back(initial_position_y);
    drone_params.push_back(avg_velocity);
    drone_params.push_back(capacity);
    


    return drone_params;
}


std::vector<std::vector<double> > read_csv_payloads(std::string file_name_payloads) {
    // Define vectors to store the values
    std::vector<std::vector<double>> payload_params;
    std::vector<double> initial_position_x;
    std::vector<double> initial_position_y;
    std::vector<double> destination_x;
    std::vector<double> destination_y;
    std::vector<double> mass;
    
    

    // Open the CSV file
   
    std::ifstream file(file_name_payloads); // Adjust the file name as needed

    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;

    }
    std::string header;
    if (std::getline(file, header)) {
        // You can optionally print the header if needed
        std::cout << "Header: " << header << std::endl;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        double value;

        // Read and parse each value from the CSV line
        if (std::getline(iss, token, ',') && std::istringstream(token) >> value) {
            initial_position_x.push_back(value);
        }

        if (std::getline(iss, token, ',') && std::istringstream(token) >> value) {
            initial_position_y.push_back(value);
        }

        if (std::getline(iss, token, ',') && std::istringstream(token) >> value) {
            destination_x.push_back(value);
        }

         if (std::getline(iss, token, ',') && std::istringstream(token) >> value) {
            destination_y.push_back(value);
        }

        if (std::getline(iss, token) && std::istringstream(token) >> value) {
            mass.push_back(value);
        }
    }

    file.close(); // Close the file when done

    payload_params.push_back(initial_position_x);
    payload_params.push_back(initial_position_y);
    payload_params.push_back(mass);
    payload_params.push_back(destination_x);
    payload_params.push_back(destination_y);
    


    return payload_params;
}

}

#endif