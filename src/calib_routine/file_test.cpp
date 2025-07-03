#include <iostream>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv2/core/core.hpp>

void updateCalibrationResult(
    const std::string &input_file,
    const std::string &output_file)
{
    // write only the intrinsics calib params to a temp file
    const std::string temp_file = "/home/tina/Documents/test_ws/src/camera_model/test_data/file_test/result_block.yaml";

    // Load original YAML file
    std::ifstream original_in(input_file);
    if (!original_in.is_open())
    {
        std::cerr << "Failed to open input YAML: " << input_file << std::endl;
        return;
    }
    std::stringstream original_buffer;
    original_buffer << original_in.rdbuf();
    std::string original_content = original_buffer.str();
    original_in.close();

    // Load new camera block
    std::ifstream block_in(temp_file);
    if (!block_in.is_open())
    {
        std::cerr << "Failed to read temp camera YAML block." << std::endl;
        return;
    }
    std::stringstream block_buffer;
    std::string line;
    while (std::getline(block_in, line))
    {
        if (line.find("%YAML") != std::string::npos ||
            line.find("---") != std::string::npos)
            continue; // skip YAML header and separator
        block_buffer << line << "\n";
    }
    // the content of the calibrated param block, only
    std::string new_block = block_buffer.str();
    block_in.close();

    // Rewrite original content with new camera block
    std::stringstream output_yaml;
    std::istringstream original_lines(original_content);

    bool inside_block = false;
    bool block_inserted = false;

    while (std::getline(original_lines, line))
    {
        if (!inside_block && (line.find("model_type:") != std::string::npos))
        {
            // Start replacing the block
            inside_block = true;
            output_yaml << new_block;
            block_inserted = true;
            continue; // Skip the original model_type line
        }

        // Detect end of block heuristically
        if (inside_block)
        {
            // If it's an empty line, we consider the block ended
            if (line.empty())
            {
                inside_block = false;        // End of camera block
                output_yaml << line << "\n"; // Keep this line
            }

            continue; // Skip old block lines
        }

        // Copy everything else
        output_yaml << line << "\n";
    }

    if (!block_inserted)
    {
        std::cerr << "Warning: Camera block not found or inserted. Check anchors." << std::endl;
    }

    std::string final_yaml = output_yaml.str();
    // Remove the last newline if present
    if (!final_yaml.empty() && final_yaml.back() == '\n')
    {
        final_yaml.pop_back();
    }

    // Write updated YAML
    std::ofstream out(output_file);
    if (!out.is_open())
    {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return;
    }
    out << output_yaml.str();
    out.close();

    std::cout << "Updated YAML written to " << output_file << std::endl;
}

int main(int argc, char **argv)
{
    std::string folder_path = "/home/tina/Documents/test_ws/src/camera_model/test_data/file_test";
    std::string input_file = folder_path + "/input_calib.yaml";
    std::string output_file = folder_path + "/output_calib.yaml";
    updateCalibrationResult(input_file, output_file);
    return 0;
}