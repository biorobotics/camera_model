/**
 * @file io_utils.h
 * @author Tina Tian
 * @brief Utility functions for file I/O operations in calibration routines.
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <iostream>
#include <vector>
#include <algorithm>

namespace io_utils
{
    /**
     * @brief Find the last sorted file in a directory, sorted in descending order by filename.
     *
     * @param folder Full path to the directory to search.
     * @return std::string Empty string for no files, otherwise the full path of the last file.
     */
    std::string findLastSortedFileDescending(const std::string &folder);

}