#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

class Logger {
public:
    Logger(const std::string& filename) : filename(filename) {
        logFile.open(filename);
        logFile << "[\n";
    }

    ~Logger() {
        if (logFile.is_open()) {
            // Remove last comma if data was written
            long pos = logFile.tellp();
            logFile.seekp(pos - 2); 
            logFile << "\n]";
            logFile.close();
        }
    }

    void logStep(double time, double x, double y, double theta, 
                 const std::vector<double>& wheelAngles, 
                 const std::vector<double>& wheelSpeeds,
                 const std::vector<double>& forcesX,
                 const std::vector<double>& forcesY) {
        
        logFile << "  {\n";
        logFile << "    \"time\": " << time << ",\n";
        logFile << "    \"robot\": {\"x\": " << x << ", \"y\": " << y << ", \"theta\": " << theta << "},\n";
        logFile << "    \"modules\": [\n";
        for (size_t i = 0; i < wheelAngles.size(); ++i) {
            logFile << "      {\"angle\": " << wheelAngles[i] 
                    << ", \"speed\": " << wheelSpeeds[i] 
                    << ", \"fx\": " << forcesX[i] 
                    << ", \"fy\": " << forcesY[i] << "}";
            if (i < wheelAngles.size() - 1) logFile << ",";
            logFile << "\n";
        }
        logFile << "    ]\n";
        logFile << "  },\n";
    }

private:
    std::string filename;
    std::ofstream logFile;
};
