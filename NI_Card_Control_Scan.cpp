// Created by Orkun Acar on 22.09.2025.
//

#include "NI_Card_Control_Scan.h"
//#include <NIDAQmx.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tuple>
#include <vector>
#include <string>
#include <thread>

using namespace std;


// ---------- Error handling macro ----------
#define DAQmxErrChk(functionCall) do {                      \
    int32 error = (functionCall);                           \
    if (DAQmxFailed(error)) {                               \
        char errBuff[2048] = {'\0'};                        \
        DAQmxGetExtendedErrorInfo(errBuff, 2048);           \
        cerr << "DAQmx Error: " << errBuff << "\n";    \
        if (taskHandle != 0) {                              \
            DAQmxStopTask(taskHandle);                      \
            DAQmxClearTask(taskHandle);                     \
        }                                                   \
        return -1;                                          \
    }                                                       \
} while(0)

// Helper: degrees → voltage
double calculate_voltage_for_degree(double voltage_range, double degree_range, double target_degree) {
    return (voltage_range * target_degree) / degree_range;
}

// Helper: indices → degrees
pair<double, double> pointsToDegree(int point_x, int point_y, double step_size) {
    double degree_x = point_x * step_size;
    double degree_y = point_y * step_size;
    return {degree_x, degree_y};
}

// Read CSV into vector of precomputed voltages (X, Y)
vector<pair<double, double>> readCSV_and_precompute(
        const string& filename,
        double step_size,
        double voltage_range,
        double degree_range)
    {
        vector<pair<double, double>> voltages;
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error opening file: " << filename << "\n";
            return voltages;
        }

        string line;
        // Discard header
        getline(file, line);

        while (getline(file, line)) {
            stringstream ss(line);
            string x_str, y_str;

            if (getline(ss, x_str, ',') && getline(ss, y_str, ',')) {
                try {
                    int x_idx = stoi(x_str);
                    int y_idx = stoi(y_str);

                    // Convert indices → degrees
                    auto [degX, degY] = pointsToDegree(x_idx, y_idx, step_size);

                    // Convert degrees → voltages
                    double voltX = calculate_voltage_for_degree(voltage_range, degree_range, degX);
                    double voltY = calculate_voltage_for_degree(voltage_range, degree_range, degY);

                    voltages.emplace_back(voltX, voltY);
                } catch (...) {
                    cerr << "Skipping malformed row: " << line << "\n";
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
    const double voltage_range = 5.0;   // ±5 V
    const double degree_range = 22.5;   // ±22.5° max scan
    const double step_size = 0.15;      // degrees per index step
    const int settle_ms = 1;            // delay after each move ms
    const int settle_microseconds = 200;// delay after each move microseconds

    // Load CSV and precompute voltages
    vector<pair<double, double>> voltages =
        readCSV_and_precompute("ExampleRectangleCSV.csv", step_size, voltage_range, degree_range);

    if (voltages.empty()) {
        cerr << "No points loaded!\n";
        return -1;
    }

    // Setup DAQmx task
    TaskHandle taskHandle = 0;
    DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
    DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, channelX, "", -voltage_range, voltage_range, DAQmx_Val_Volts, NULL));
    DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, channelY, "", -voltage_range, voltage_range, DAQmx_Val_Volts, NULL));
    DAQmxErrChk(DAQmxStartTask(taskHandle));

    // Iterate precomputed voltages
    for (auto& v : voltages) {
        float64 data[2] = { v.first, v.second };  // X, Y
        int32 written = 0;
        int32 error = DAQmxWriteAnalogF64(taskHandle,
                                          1,                // samples per channel
                                          1,                // autostart
                                          10.0,             // timeout
                                          DAQmx_Val_GroupByScanNumber, // safer grouping
                                          data,
                                          &written,
                                          NULL);

        if (DAQmxFailed(error)) {
            char errBuff[2048] = {'\0'};
            DAQmxGetExtendedErrorInfo(errBuff, 2048);
            cerr << "DAQmx Error writing voltages (" << v.first << "," << v.second << "): "
                      << errBuff << "\n";
        } else {
            cout << "Output voltages: X=" << v.first << " V, Y=" << v.second << " V\n";
        }

        this_thread::sleep_for(chrono::microseconds(settle_microseconds));
    }

    // Cleanup
    DAQmxStopTask(taskHandle);
    DAQmxClearTask(taskHandle);

    return 0;
}
