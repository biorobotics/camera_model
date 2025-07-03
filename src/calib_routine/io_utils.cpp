/**
 * @file io_utils.cpp
 * @author Tina Tian
 * @brief Utility functions for file I/O operations in calibration routines.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "io_utils.h"
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

std::string io_utils::findLastSortedFileDescending(const std::string &folder)
{
    fs::path directory(folder);
    std::vector<fs::path> files;

    if (!fs::exists(directory) || !fs::is_directory(directory))
    {
        throw std::runtime_error("Invalid directory path: " + folder);
    }

    for (const auto &entry : fs::directory_iterator(directory))
    {
        if (fs::is_regular_file(entry.status()))
        {
            files.push_back(entry.path());
        }
    }

    if (files.empty())
    {
        std::string empty_str = "";
        return empty_str;
    }

    // Sort descending based on filename
    std::sort(files.begin(), files.end(), [](const fs::path &a, const fs::path &b)
              { return a.filename().string() > b.filename().string(); });

    return files.back().string();
}