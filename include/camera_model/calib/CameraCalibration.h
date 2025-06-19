#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include "camera_model/camera_models/Camera.h"

namespace camera_model
{

class CameraCalibration
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraCalibration( );

    CameraCalibration( const Camera::ModelType modelType,
                       const std::string& cameraName,
                       const cv::Size& imageSize,
                       const cv::Size& boardSize,
                       float squareSize );

    void clear( void );

    /**
     * @brief Add chessboard data to the calibration.
     * @param corners Detected corners in the image.
     * This function adds the detected corners of a chessboard pattern to the calibration data.
     * It also generates the corresponding 3D scene points based on the board size and square size.
     * The 3D points are assumed to be in the plane of the chessboard, with x-coordinate along height dimension, y-coordinate along width dimension, z-coordinate being zero.
     */
    void addChessboardData( const std::vector< cv::Point2f >& corners );

    /**
     * @brief Add chessboard data to the calibration with specified scene points.
     * @param corners Detected corners in the image.
     * @param scene_pts Corresponding 3D scene points.
     * This function adds the detected corners of a chessboard pattern along with their corresponding 3D scene points to the calibration data.
     */
    void addChessboardData( const std::vector< cv::Point2f >& corners,
                            const std::vector< cv::Point3f >& scene_pts );
    void addImage( const cv::Mat image, const std::string name );

    bool calibrate( void );

    int sampleCount( void ) const;
    std::vector< std::vector< cv::Point2f > >& imagePoints( void );
    const std::vector< std::vector< cv::Point2f > >& imagePoints( void ) const;
    std::vector< std::vector< cv::Point3f > >& scenePoints( void );
    const std::vector< std::vector< cv::Point3f > >& scenePoints( void ) const;
    CameraPtr& camera( void );
    const CameraConstPtr camera( void ) const;

    Eigen::Matrix2d& measurementCovariance( void );
    const Eigen::Matrix2d& measurementCovariance( void ) const;

    cv::Mat& cameraPoses( void );
    const cv::Mat& cameraPoses( void ) const;

    void drawResultsInitial( std::vector< cv::Mat >& images,
                             std::vector< std::string >& imageNames,
                             cv::Mat& DistributedImage ) const;
    void drawResultsFiltered( std::vector< cv::Mat >& images,
                              std::vector< std::string >& imageNames,
                              cv::Mat& DistributedImage ) const;
    void drawResults( std::vector< cv::Mat >& images,
                      std::vector< std::string >& imageNames,
                      const cv::Mat& DistributedImage,
                      const std::vector< cv::Mat > cameraPoseRs,
                      const std::vector< cv::Mat > cameraPoseTs,
                      const std::vector< std::vector< cv::Point2f > > imagePoints,
                      const std::vector< std::vector< cv::Point3f > > scenePoints ) const;
    void save2D( std::string point_file ) const;
    void writeParams( const std::string& filename ) const;

    bool writeChessboardData( const std::string& filename ) const;
    bool readChessboardData( const std::string& filename );

    void setVerbose( bool verbose );

    private:
    bool calibrateHelper( CameraPtr& camera,
                          std::vector< cv::Mat >& rvecs,
                          std::vector< cv::Mat >& tvecs,
                          std::vector< std::vector< cv::Point2f > >& imagePoints,
                          std::vector< std::vector< cv::Point3f > >& scenePoints ) const;
    /**
     * @brief Perform calibration optimization using Ceres Solver.
     * @param camera The camera to be calibrated.
     * @param rvecs Rotation vectors for each view.
     * @param tvecs Translation vectors for each view.
     * @param imagePoints 2D image points for each view.
     * @param scenePoints 3D scene points for each view.
     * @return True if optimization was successful, false otherwise.
     */
    bool CalibrationOptimization( CameraPtr& camera,
                                  std::vector< cv::Mat >& rvecs,
                                  std::vector< cv::Mat >& tvecs,
                                  std::vector< std::vector< cv::Point2f > >& imagePoints,
                                  std::vector< std::vector< cv::Point3f > >& scenePoints ) const;
    
    /**
     * @brief Optimize camera parameters using Ceres Solver.
     * @param camera Camera pointer to optimize.
     * @param rvecs Rotation vectors for each view.
     * @param tvecs Translation vectors for each view.
     * @param imagePoints 2D image points for each view.
     * @param scenePoints 3D scene points for each view.
     * 
     * This function sets up a Ceres problem to optimize the camera 
     * parameters based on the provided image and scene points.
     */
    void optimize( CameraPtr& camera,
                   std::vector< cv::Mat >& rvecs,
                   std::vector< cv::Mat >& tvecs,
                   const std::vector< std::vector< cv::Point2f > >& imagePoints,
                   const std::vector< std::vector< cv::Point3f > >& scenePoints ) const;

    template< typename T >
    void readData( std::ifstream& ifs, T& data ) const;

    template< typename T >
    void writeData( std::ofstream& ofs, T data ) const;

    cv::Size m_boardSize;
    float m_squareSize;

    CameraPtr m_camera;
    cv::Mat m_cameraPoses;

    std::vector< std::vector< cv::Point2f > > m_imagePoints;
    std::vector< std::vector< cv::Point3f > > m_scenePoints;

    std::vector< std::vector< cv::Point2f > > m_imageGoodPoints;
    std::vector< std::vector< cv::Point3f > > m_sceneGoodPoints;

    std::vector< cv::Mat > cbImages;
    std::vector< std::string > cbImageNames;

    Eigen::Matrix2d m_measurementCovariance;

    bool m_verbose;

    public:
    std::vector< cv::Mat > m_ImagesShow;
    std::vector< cv::Mat > m_ImagesGoodShow;
    std::vector< std::string > m_ImageNames;
    std::vector< std::vector< cv::Point2f > > m_imagePointsShow;
    std::vector< std::vector< cv::Point3f > > m_scenePointsShow;
    std::vector< std::vector< cv::Point2f > > m_imageGoodPointsShow;
    std::vector< std::vector< cv::Point3f > > m_sceneGoodPointsShow;
    std::vector< cv::Mat > m_cameraPoseRvecsShow;
    std::vector< cv::Mat > m_cameraPoseTvecsShow;
};
}

#endif
