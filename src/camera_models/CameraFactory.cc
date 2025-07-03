#include <boost/algorithm/string.hpp>

#include "camera_model/camera_models/CameraFactory.h"

#include "camera_model/camera_models/CataCamera.h"
#include "camera_model/camera_models/EquidistantCamera.h"
#include "camera_model/camera_models/FovCamera.h"
#include "camera_model/camera_models/PinholeCamera.h"
#include "camera_model/camera_models/PinholeFullCamera.h"
#include "camera_model/camera_models/PolyFisheyeCamera.h"
#include "camera_model/camera_models/ScaramuzzaCamera.h"
#include "camera_model/camera_models/SplineCamera.h"

#include "ceres/ceres.h"

namespace camera_model
{

boost::shared_ptr< CameraFactory > CameraFactory::m_instance;

CameraFactory::CameraFactory( ) {}

boost::shared_ptr< CameraFactory >
CameraFactory::instance( void )
{
    if ( m_instance.get( ) == 0 )
    {
        m_instance.reset( new CameraFactory );
    }

    return m_instance;
}

CameraPtr
CameraFactory::generateCamera( Camera::ModelType modelType, const std::string& cameraName, cv::Size imageSize ) const
{
    switch ( modelType )
    {
        case Camera::KANNALA_BRANDT:
        {
            EquidistantCameraPtr camera( new EquidistantCamera );

            EquidistantCamera::Parameters params = camera->getParameters( );
            params.cameraName( )                 = cameraName;
            params.imageWidth( )                 = imageSize.width;
            params.imageHeight( )                = imageSize.height;
            camera->setParameters( params );
            return camera;
        }
        case Camera::PINHOLE:
        {
            PinholeCameraPtr camera( new PinholeCamera );

            PinholeCamera::Parameters params = camera->getParameters( );
            params.cameraName( )             = cameraName;
            params.imageWidth( )             = imageSize.width;
            params.imageHeight( )            = imageSize.height;
            camera->setParameters( params );
            return camera;
        }
        case Camera::PINHOLE_FULL:
        {
            PinholeFullCameraPtr camera( new PinholeFullCamera );

            PinholeFullCamera::Parameters params = camera->getParameters( );
            params.cameraName( )                 = cameraName;
            params.imageWidth( )                 = imageSize.width;
            params.imageHeight( )                = imageSize.height;
            camera->setParameters( params );
            return camera;
        }
        case Camera::SCARAMUZZA:
        {
            OCAMCameraPtr camera( new OCAMCamera );

            OCAMCamera::Parameters params = camera->getParameters( );
            params.cameraName( )          = cameraName;
            params.imageWidth( )          = imageSize.width;
            params.imageHeight( )         = imageSize.height;
            camera->setParameters( params );
            return camera;
        }
        case Camera::POLYFISHEYE:
        {
            PolyFisheyeCameraPtr camera( new PolyFisheyeCamera );

            PolyFisheyeCamera::Parameters params = camera->getParameters( );
            params.cameraName( )                 = cameraName;
            params.imageWidth( )                 = imageSize.width;
            params.imageHeight( )                = imageSize.height;
            camera->setParameters( params );
            return camera;
        }
        case Camera::SPLINE:
        {
            SplineCameraPtr camera( new SplineCamera );

            SplineCamera::Parameters params = camera->getParameters( );
            params.cameraName( )            = cameraName;
            params.imageWidth( )            = imageSize.width;
            params.imageHeight( )           = imageSize.height;
            camera->setParameters( params );
            return camera;
        }
        case Camera::FOV:
        {
            FovCameraPtr camera( new FovCamera );

            FovCamera::Parameters params = camera->getParameters( );
            params.cameraName( )         = cameraName;
            params.imageWidth( )         = imageSize.width;
            params.imageHeight( )        = imageSize.height;
            camera->setParameters( params );
            return camera;
        }
        case Camera::MEI:
        default:
        {
            CataCameraPtr camera( new CataCamera );

            CataCamera::Parameters params = camera->getParameters( );
            params.cameraName( )          = cameraName;
            params.imageWidth( )          = imageSize.width;
            params.imageHeight( )         = imageSize.height;
            camera->setParameters( params );
            return camera;
        }
    }
}

CameraPtr
CameraFactory::generateCameraFromYamlFile( const std::string& filename )
{
    cv::FileStorage fs( filename, cv::FileStorage::READ );

    if ( !fs.isOpened( ) )
    {
        return CameraPtr( );
    }

    Camera::ModelType modelType = Camera::MEI;
    assert( !fs["model_type"].isNone( ) );
    std::string sModelType;
    fs["model_type"] >> sModelType;
    try
    {
        modelType = toCameraModelType(sModelType);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return CameraPtr( );
    }

    switch ( modelType )
    {
        case Camera::KANNALA_BRANDT:
        {
            EquidistantCameraPtr camera( new EquidistantCamera );

            EquidistantCamera::Parameters params = camera->getParameters( );
            params.readFromYamlFile( filename );
            camera->setParameters( params );
            return camera;
        }
        case Camera::PINHOLE:
        {
            PinholeCameraPtr camera( new PinholeCamera );

            PinholeCamera::Parameters params = camera->getParameters( );
            params.readFromYamlFile( filename );
            camera->setParameters( params );
            return camera;
        }
        case Camera::PINHOLE_FULL:
        {
            PinholeFullCameraPtr camera( new PinholeFullCamera );

            PinholeFullCamera::Parameters params = camera->getParameters( );
            params.readFromYamlFile( filename );
            camera->setParameters( params );
            return camera;
        }
        case Camera::SCARAMUZZA:
        {
            OCAMCameraPtr camera( new OCAMCamera );

            OCAMCamera::Parameters params = camera->getParameters( );
            params.readFromYamlFile( filename );
            camera->setParameters( params );
            return camera;
        }
        case Camera::POLYFISHEYE:
        {
            PolyFisheyeCameraPtr camera( new PolyFisheyeCamera );

            PolyFisheyeCamera::Parameters params = camera->getParameters( );
            params.readFromYamlFile( filename );
            camera->setParameters( params );
            return camera;
        }
        case Camera::SPLINE:
        {
            SplineCameraPtr camera( new SplineCamera );

            SplineCamera::Parameters params = camera->getParameters( );
            params.readFromYamlFile( filename );
            camera->setParameters( params );
            return camera;
        }
        case Camera::FOV:
        {
            FovCameraPtr camera( new FovCamera );

            FovCamera::Parameters params = camera->getParameters( );
            params.readFromYamlFile( filename );
            camera->setParameters( params );
            return camera;
        }
        case Camera::MEI:
        default:
        {
            CataCameraPtr camera( new CataCamera );

            CataCamera::Parameters params = camera->getParameters( );
            params.readFromYamlFile( filename );
            camera->setParameters( params );
            return camera;
        }
    }

    return CameraPtr( );
}
}
