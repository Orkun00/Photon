//
// Created by Orkun Acar on 22.09.2025.
//

#include "NI_Card_Control_Scan.h"
#include <NIDAQmx.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tuple>
#include <vector>
#include <string>
#include <thread>

// Helper: degrees → voltage
double calculate_voltage_for_degree(double voltage_range, double degree_range, double target_degree) {
    return (voltage_range * target_degree) / degree_range;
}

// Helper: indices → degrees
std::pair<double, double> pointsToDegree(int point_x, int point_y, double step_size) {
    double degree_x = point_x * step_size;
    double degree_y = point_y * step_size;
    return {degree_x, degree_y};
}

// Read CSV into vector of precomputed voltages (X, Y)
std::vector<std::pair<double, double>> readCSV_and_precompute(
        const std::string& filename,
        double step_size,
        double voltage_range,
        double degree_range)
{
    std::vector<std::pair<double, double>> voltages;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << "\n";
        return voltages;
    }

    std::string line;
    // Discard header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;

        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            try {
                int x_idx = std::stoi(x_str);
                int y_idx = std::stoi(y_str);

                // Convert indices → degrees using helper
                auto [degX, degY] = pointsToDegree(x_idx, y_idx, step_size);

                // Convert degrees → voltages using helper
                double voltX = calculate_voltage_for_degree(voltage_range, degree_range, degX);
                double voltY = calculate_voltage_for_degree(voltage_range, degree_range, degY);

                voltages.emplace_back(voltX, voltY);
            } catch (...) {
                std::cerr << "Skipping malformed row: " << line << "\n";
            }
        }
    }
    return voltages;
}

int main() {
    // Parameters
    const char* device = "Dev1";        // Change to match NI MAX device name
    const char* channelX = "Dev1/ao0";  // Galvo X output
    const char* channelY = "Dev1/ao1";  // Galvo Y output
    double voltage_range = 5.0;         // ±5 V
    double degree_range = 22.5;         // ±22.5° max scan
    double step_size = 0.10;            // degrees per index step
    int settle_ms = 1;                  // delay after each move

    // Load CSV and precompute voltages
    std::vector<std::pair<double, double>> voltages =
        readCSV_and_precompute("C:\\Users\\Administrator\\Downloads\\Photon\\Photon-main\\Example.csv", step_size, voltage_range, degree_range);

    if (voltages.empty()) {
        std::cerr << "No points loaded!\n";
        return -1;
    }

    // Setup DAQmx task
    TaskHandle taskHandle = 0;
    int32 error = 0;
    error = DAQmxCreateTask("", &taskHandle);
    error = DAQmxCreateAOVoltageChan(taskHandle, channelX, "", -voltage_range, voltage_range, DAQmx_Val_Volts, NULL);
    error = DAQmxCreateAOVoltageChan(taskHandle, channelY, "", -voltage_range, voltage_range, DAQmx_Val_Volts, NULL);
    error = DAQmxStartTask(taskHandle);

    // Iterate precomputed voltages
    for (auto& v : voltages) {
        float64 data[2] = { v.first, v.second };
        int32 written;
        error = DAQmxWriteAnalogF64(taskHandle, 1, 1, 10.0, DAQmx_Val_GroupByScanNumber, data, &written, NULL);

        if (error) {
            std::cerr << "Error writing voltages (" << v.first << "," << v.second << ")\n";
        } else {
            //std::cout << "Output voltages: X=" << v.first << " V, Y=" << v.second << " V\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
    }

    // Cleanup
    DAQmxStopTask(taskHandle);
    DAQmxClearTask(taskHandle);

    return 0;
}
