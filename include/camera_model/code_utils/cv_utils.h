#ifndef CV_UTLIS_H
#define CV_UTLIS_H

#include <opencv2/opencv.hpp>

namespace cv_utils
{
namespace fisheye
{
class PreProcess
{
    public:
    PreProcess( );

    /**
     * @brief Constructor for the PreProcess class.
     * @param _raw_image_size Size of the raw image.
     * @param _roi_size Size of the region of interest (ROI).
     * @param _center Center point of the ROI.
     * @param _resize_scale Scale factor for resizing the image.
     * @details This constructor initializes the PreProcess object with the given parameters and checks if the parameters are valid.
     * If the parameters are valid, it sets the `is_preprocess` flag to true. If the raw image size is equal to the ROI size, it sets the `is_resize_only` flag to true.
     * If the parameters are invalid, it sets the `is_preprocess` flag to false and prints an error message.
     */
    PreProcess( const cv::Size _raw_image_size,
                const cv::Size _roi_size,
                const cv::Point _center,
                const float _resize_scale );

    /**
     * @brief Reset the pre-process parameters such that the region of interest (ROI) can be adjusted.
     * @param _roi_size Size of the region of interest.
     * @param _center Center point of the region of interest.
     * @param _resize_scale Scale factor for resizing the image.
     */
    void resetPreProcess( cv::Size _roi_size, cv::Point _center, float _resize_scale );

    /**
     * @brief Preprocess the input image based on the pre-process parameters.
     * @param image_input Input image to be preprocessed.
     * @return Preprocessed image.
     * @details If `is_resize_only` is true, the input image is resized according to the `resize_scale`.
     * If `is_preprocess` is true, the input image is cropped to the region of interest (ROI) defined by `roi_row_start`, `roi_row_end`, `roi_col_start`, and `roi_col_end`, and then resized according to the `resize_scale`.
     * If neither `is_resize_only` nor `is_preprocess` is true, the input image is returned unchanged.
     * This function is useful for preparing images for further processing, such as calibration or feature extraction.
     */
    cv::Mat do_preprocess( cv::Mat image_input );

    float resize_scale;
    int roi_row_start;
    int roi_col_start;
    int roi_row_end;
    int roi_col_end;

    bool is_preprocess;
    bool is_resize_only;
};
}
}
#endif // CV_UTLIS_H
