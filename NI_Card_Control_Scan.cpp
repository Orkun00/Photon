//
// Combined Galvo Control + Simulated PMT + Heatmap Viewer
// Created by Orkun Acar on 22.09.2025
//

#include "NI_Card_Control_Scan.h"
// #include <NIDAQmx.h>
#include <iostream>
#include <fstream>
#include <random>
#include <sstream>
#include <tuple>
#include <vector>
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>

using namespace std;

// ---------- Error handling ----------
#define DAQmxErrChk(functionCall) do {                      \
    int32 error = (functionCall);                           \
    if (DAQmxFailed(error)) {                               \
        char errBuff[2048] = {'\0'};                        \
        DAQmxGetExtendedErrorInfo(errBuff, 2048);           \
        cerr << "DAQmx Error: " << errBuff << "\n";         \
        if (taskHandle != 0) {                              \
            DAQmxStopTask(taskHandle);                      \
            DAQmxClearTask(taskHandle);                     \
        }                                                   \
        return -1;                                          \
    }                                                       \
} while(0)

// degrees → voltage
double calculate_voltage_for_degree(double voltage_range, double degree_range, double target_degree) {
    return (voltage_range * target_degree) / degree_range;
}

// indices → degrees
pair<double, double> pointsToDegree(int point_x, int point_y, double step_size) {
    double degree_x = point_x * step_size;
    double degree_y = point_y * step_size;
    return {degree_x, degree_y};
}

// CSV reader: (x_idx, y_idx, voltX, voltY)
vector<tuple<int,int,double,double>> readCSV_and_precompute(
        const string& filename,
        double step_size,
        double voltage_range,
        double degree_range)
{
    vector<tuple<int, int, double, double>> voltages;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << "\n";
        return voltages;
    }

    string line;
    getline(file, line); // skip header

    while (getline(file, line)) {
        stringstream ss(line);
        string x_str, y_str;

        if (getline(ss, x_str, ',') && getline(ss, y_str, ',')) {
            try {
                int x_idx = stoi(x_str);
                int y_idx = stoi(y_str);

                auto [degX, degY] = pointsToDegree(x_idx, y_idx, step_size);

                double voltX = calculate_voltage_for_degree(voltage_range, degree_range, degX);
                double voltY = calculate_voltage_for_degree(voltage_range, degree_range, degY);

                voltages.emplace_back(x_idx, y_idx, voltX, voltY);
            } catch (...) {
                cerr << "Skipping malformed row: " << line << "\n";
            }
        }
    }
    return voltages;
}


int main() {
    // Parameters
    const char* device   = "Dev1";
    const char* channelX = "Dev1/ao0";
    const char* channelY = "Dev1/ao1";
    double voltage_range = 5.0;
    double degree_range  = 22.5;
    double step_size     = 0.01;
    int settle_microseconds = 200;

    // Load points
    vector<tuple<int,int,double,double>> voltages =
        readCSV_and_precompute("../newGUI.csv", step_size, voltage_range, degree_range);

    if (voltages.empty()) {
        cerr << "No points loaded!\n";
        return -1;
    }

    // ---- DAQmx setup ----
    TaskHandle taskHandle = 0;
    DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
    DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, channelX, "", -voltage_range, voltage_range, DAQmx_Val_Volts, NULL));
    DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, channelY, "", -voltage_range, voltage_range, DAQmx_Val_Volts, NULL));
    DAQmxErrChk(DAQmxStartTask(taskHandle));

    // ---- Heatmap setup ----
    int img_size = 200;
    cv::Mat heatmap = cv::Mat::zeros(img_size, img_size, CV_8UC1);

    cv::namedWindow("Scan Heatmap", cv::WINDOW_NORMAL);
    cv::resizeWindow("Scan Heatmap", 800, 800);

    // Random PMT simulator
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dist(10, 255);

    // Zoom/pan vars
    double scale_factor = 4.0;
    int offset_x = 0, offset_y = 0;

    // ---- Scan loop ----
    for (auto& [x_idx, y_idx, voltX, voltY] : voltages) {
        // Move galvos
        if (voltX < -voltage_range || voltX > voltage_range ||
            voltY < -voltage_range || voltY > voltage_range) {
            cerr << "Voltage out of range: (" << voltX << ", " << voltY << ")\n";
            // continue; // Skip invalid points
            // error out instead of continue for debugging
            return -1;
        }
        float64 data[2] = { voltX, voltY };
        int32 written;
        DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, 1, 1, 10.0,
                                        DAQmx_Val_GroupByChannel, data, &written, NULL));

        // Simulated PMT read
        int intensity = dist(gen);

        if (x_idx >= 0 && x_idx < img_size && y_idx >= 0 && y_idx < img_size) {
            heatmap.at<uchar>(y_idx, x_idx) = static_cast<uchar>(intensity);
        }

        // Update view
        cv::Mat colored;
        cv::applyColorMap(heatmap, colored, cv::COLORMAP_INFERNO);

        cv::Mat zoomed;
        cv::resize(colored, zoomed, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);

        int view_w = min(800, zoomed.cols);
        int view_h = min(800, zoomed.rows);
        int start_x = clamp(offset_x, 0, zoomed.cols - view_w);
        int start_y = clamp(offset_y, 0, zoomed.rows - view_h);

        cv::Rect roi(start_x, start_y, view_w, view_h);
        cv::Mat view = zoomed(roi);

        cv::imshow("Scan Heatmap", view);

        int key = cv::waitKey(1);
        if (key == 27) break;

        this_thread::sleep_for(chrono::microseconds(settle_microseconds));
    }

    cout << "Scan complete. Viewer mode active (ESC to exit).\n";

    // ---- Viewer loop ----
    while (true) {
        cv::Mat colored;
        cv::applyColorMap(heatmap, colored, cv::COLORMAP_INFERNO);

        cv::Mat zoomed;
        cv::resize(colored, zoomed, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);

        int view_w = min(800, zoomed.cols);
        int view_h = min(800, zoomed.rows);
        int start_x = clamp(offset_x, 0, zoomed.cols - view_w);
        int start_y = clamp(offset_y, 0, zoomed.rows - view_h);

        cv::Rect roi(start_x, start_y, view_w, view_h);
        cv::Mat view = zoomed(roi);

        cv::imshow("Scan Heatmap", view);

        int key = cv::waitKey(30);
        if (key == 27) break;
        if (key == '+') scale_factor *= 1.25;
        if (key == '-') scale_factor = max(1.0, scale_factor / 1.25);
        if (key == 'w' || key == 82) offset_y = max(0, offset_y - 20);
        if (key == 's' || key == 84) offset_y = min(zoomed.rows - view_h, offset_y + 20);
        if (key == 'a' || key == 81) offset_x = max(0, offset_x - 20);
        if (key == 'd' || key == 83) offset_x = min(zoomed.cols - view_w, offset_x + 20);
    }

    // ---- Cleanup ----
    DAQmxStopTask(taskHandle);
    DAQmxClearTask(taskHandle);

    return 0;
}
