/*
BSD 3-Clause License

Copyright (c) 2023, Bibrak Qamar

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

int
main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file>" << std::endl;
        return 1;
    }

    const std::string inputFileName = argv[1];
    const std::string outputFileName = inputFileName + "_modified";

    std::ifstream inputFile(inputFileName);
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open input file: " << inputFileName << std::endl;
        return 1;
    }

    std::ofstream outputFile(outputFileName);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to create output file: " << outputFileName << std::endl;
        inputFile.close();
        return 1;
    }

    std::string line;
    bool dataStarted = false;

    // Seed the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the range of random numbers (1 to 5)
    std::uniform_int_distribution<int> distribution(1, 5);

    while (std::getline(inputFile, line)) {
        if (line.empty()) {
            continue; // Skip empty lines
        }

        if (!dataStarted) {
            if (line[0] != '%') {
                dataStarted = true;
                // Process the first data line
                std::istringstream iss(line);
                int num1, num2, num3;
                if (iss >> num1 >> num2 >> num3) {
                    outputFile << num1 << ' ' << num2 << "\n";
                    outputFile << num3 << "\n";
                }
            }
        } else {
            // Process the rest of the data lines
            std::istringstream iss(line);
            int num1, num2;
            if (iss >> num1 >> num2) {
                // Generate a random integer within the specified range
                int randomNum = distribution(gen);
                outputFile << num1 << ' ' << num2 << ' ' << randomNum << "\n";
            }
        }
    }

    inputFile.close();
    outputFile.close();

    std::cout << "Data processed and saved to " << outputFileName << std::endl;

    return 0;
}
