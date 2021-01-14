#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeIntensityPointCloudFrame.h"

#include <sstream>
#include <thread>

#include <dynamic_reconfigure/server.h>
#include <cubeeye_scube/cubeeye_scubeConfig.h>

#define SCUBE_WIDTH 640
#define SCUBE_HEIGHT 480

#define PUB_DEPTH "/cubeeye/scube/depth_raw"
#define PUB_AMPLITUDE "/cubeeye/scube/amplitude_raw"
#define PUB_PCL "/cubeeye/scube/points"

enum Config{
    Config_AmplitudeThreshold   = 1,
    Config_ScatteringThreshold  = 2,
    Config_AutoExposuere        = 3,
    Config_StandbyMode          = 4,
    Config_Illumination         = 5,
    Config_MedianFilter         = 6,
    Config_EdgeFilter           = 7,
    Config_DepthOffset          = 8,
    Config_DepthRangeMin        = 9,
    Config_DepthRangeMax        = 10,
};

bool mLoopOk;

ros::Publisher pub_depth_raw;
ros::Publisher pub_amplitude_raw;
ros::Publisher pub_pcl_raw;

sensor_msgs::ImagePtr m_msgImgPtrDepth;
sensor_msgs::ImagePtr m_msgImgPtrAmplitude;

float* m_pDepthData;
float* m_pAmplitudeData;
sensor_msgs::PointCloud2 m_msgPCL2;
sensor_msgs::PointCloud2Ptr m_msgPCL2ptr;
meere::sensor::sptr_camera m_camera;

uint16_t m_nAmplitude_threshold;
uint16_t m_nScattering_threshold;
uint8_t m_nStandby_mode;

bool m_bAuto_exposure;
bool m_billumination;

bool m_bMedian_filter;
bool m_bEdge_filter;
short m_nDepth_offset;
short m_nDepth_range_min;
short m_nDepth_range_max;

void gracefulShutdown(int sigNum) {
    mLoopOk = false;
}

void callbackConfig(cubeeye_scube_node::cubeeye_scubeConfig &config, uint32_t level)
{

    meere::sensor::sptr_property _prop = nullptr;
    switch (level)
    {
        std::cout << "callbackConfig : " << level << std::endl;

    case Config_AmplitudeThreshold :
        m_nAmplitude_threshold = config.amplitude_threshold;
        _prop = meere::sensor::make_property_16u("amplitude_threshold_min", m_nAmplitude_threshold);
        break;

    case Config_ScatteringThreshold :
        m_nScattering_threshold = config.scattering_threshold;
        _prop = meere::sensor::make_property_16u("scattering_threshold", m_nScattering_threshold);
        break;
    case Config_AutoExposuere :
        m_bAuto_exposure = config.auto_exposure;
        _prop = meere::sensor::make_property_bool("auto_exposure", m_bAuto_exposure);

        break;
    case Config_StandbyMode :
        m_nStandby_mode = config.standby_mode;
        _prop = meere::sensor::make_property_bool("standby_mode", m_nStandby_mode);
        break;
    case Config_Illumination :
        m_billumination = config.illumination;
        _prop = meere::sensor::make_property_bool("illumination", m_billumination);
        break;
    case Config_MedianFilter :
        m_bMedian_filter = config.median_filter;
        _prop = meere::sensor::make_property_bool("median_filter", m_bMedian_filter);
        break;
    case Config_EdgeFilter :
        m_bEdge_filter = config.edge_filter;
        _prop = meere::sensor::make_property_bool("edge_filter", m_bEdge_filter);
        break;
    case Config_DepthOffset :
        m_nDepth_offset = config.depth_offset;
        _prop = meere::sensor::make_property_bool("depth_offset", m_nDepth_offset);
        break;
    case Config_DepthRangeMin :
        m_nDepth_range_min = config.depth_range_min;
        _prop = meere::sensor::make_property_bool("depth_range_min", m_nDepth_range_min);
        break;
    case Config_DepthRangeMax:
        m_nDepth_range_max = config.depth_range_max;
        _prop = meere::sensor::make_property_bool("depth_range_max", m_nDepth_range_max);
        break;

    }//switch

    if (nullptr != _prop)
    {
        // set property
        if (meere::sensor::result::success != m_camera->setProperty(_prop))
        {
            // error
            std::cout << "setProperty(" << _prop->key() << ") failed." << std::endl;
        }
    }
}


static class ReceivedIntensityPCLFrameSink : public meere::sensor::sink
 , public meere::sensor::prepared_listener
{
public:
    virtual std::string name() const {
        return std::string("ReceivedIntensityPCLFrameSink");
    }

    virtual void onCubeEyeCameraState(const meere::sensor::ptr_source source, meere::sensor::State state) {
        printf("%s:%d source(%s) state = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), state);
    }

    virtual void onCubeEyeCameraError(const meere::sensor::ptr_source source, meere::sensor::Error error) {
        printf("%s:%d source(%s) error = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), error);
    }

    virtual void onCubeEyeFrameList(const meere::sensor::ptr_source source , const meere::sensor::sptr_frame_list& frames) {
        for (auto it : (*frames)) {

#if 0
            printf("frame : %d, "
                    "frameWidth = %d "
                    "frameHeight = %d "
                    "frameDataType = %d "
                    "timestamp = %lu \n",
                    it->frameType(),
                    it->frameWidth(),
                    it->frameHeight(),
                    it->frameDataType(),
                    it->timestamp());

            auto _center_x = it->frameWidth() / 2;
            auto _center_y = it->frameHeight() / 2;
#endif
            int _frame_index = 0;

            // intensity-PointCloud frame
            if (it->frameType() == meere::sensor::CubeEyeFrame::FrameType_IntensityPointCloud) {
                // 32bits floating-point
                if (it->frameDataType() == meere::sensor::CubeEyeData::DataType_32F) {
                    // casting 32bits intensity point cloud frame
                    auto _sptr_intensity_pointcloud_frame = meere::sensor::frame_cast_ipcl32f(it);
                    auto _sptr_frame_dataX = _sptr_intensity_pointcloud_frame->frameDataX(); // x-point data array
                    auto _sptr_frame_dataY = _sptr_intensity_pointcloud_frame->frameDataY(); // y-point data array
                    auto _sptr_frame_dataZ = _sptr_intensity_pointcloud_frame->frameDataZ(); // z-point data array
                    auto _sptr_frame_dataI = _sptr_intensity_pointcloud_frame->frameDataI(); // intensity data array

                    for (int y = 0 ; y < _sptr_intensity_pointcloud_frame->frameHeight(); y++) {
                        for (int x = 0 ; x < _sptr_intensity_pointcloud_frame->frameWidth(); x++) {
                            _frame_index = y * _sptr_intensity_pointcloud_frame->frameWidth() + x;
#if 0
                            if (_center_x == x && _center_y == y) {
                                printf("intensity-PCL(%d,%d) data : %f, %f, %f, %f\n", _center_x, _center_y, \
                                (*_sptr_frame_dataX)[_frame_index] * 1000, (*_sptr_frame_dataY)[_frame_index] * 1000, \
                                (*_sptr_frame_dataZ)[_frame_index] * 1000, (*_sptr_frame_dataI)[_frame_index] * 1000);
                            }
#endif
                            float *pcl_a = (float *)&m_msgPCL2ptr->data[_frame_index * m_msgPCL2ptr->point_step];
                            float *pcl_b = pcl_a + 1;
                            float *pcl_c = pcl_b + 1;

                            //points
                            *pcl_a = (*_sptr_frame_dataY)[_frame_index]*-1;
                            *pcl_b = (*_sptr_frame_dataX)[_frame_index]*-1;
                            *pcl_c = (*_sptr_frame_dataZ)[_frame_index];

                            //depth
                            m_pDepthData[_frame_index] = (*_sptr_frame_dataZ)[_frame_index];

                            //intensity
                            m_pAmplitudeData[_frame_index] = (*_sptr_frame_dataI)[_frame_index];
                        }
                    }
                    pub_amplitude_raw.publish(m_msgImgPtrAmplitude);
                    pub_depth_raw.publish(m_msgImgPtrDepth);
                    pub_pcl_raw.publish(m_msgPCL2ptr);
                }
            }
        }
    }

public:
    virtual void onCubeEyeCameraPrepared(const meere::sensor::ptr_camera camera) {
        printf("%s:%d source(%s)\n", __FUNCTION__, __LINE__, camera->source()->uri().c_str());
    }

public:
    ReceivedIntensityPCLFrameSink() = default;
    virtual ~ReceivedIntensityPCLFrameSink() = default;
} _ReceivedIntensityPCLFrameSink;


void init_ros(ros::NodeHandle handle)
{
    mLoopOk = true;

    pub_depth_raw = handle.advertise<sensor_msgs::Image>(PUB_DEPTH, 1);
    pub_amplitude_raw = handle.advertise<sensor_msgs::Image>(PUB_AMPLITUDE, 1);
    pub_pcl_raw = handle.advertise<sensor_msgs::PointCloud2>(PUB_PCL, 1);


    //generate Depth
    m_msgImgPtrDepth = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    m_msgImgPtrDepth->header.frame_id = "distance";
    m_msgImgPtrDepth->width = SCUBE_WIDTH;
    m_msgImgPtrDepth->height = SCUBE_HEIGHT;
    m_msgImgPtrDepth->is_bigendian = false;
    m_msgImgPtrDepth->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    m_msgImgPtrDepth->step = (uint32_t)(sizeof(float) * SCUBE_WIDTH);
    m_msgImgPtrDepth->data.resize(sizeof(float) * SCUBE_WIDTH * SCUBE_HEIGHT);
    m_pDepthData = (float *)&m_msgImgPtrDepth->data[0];

    //generate Amplitude
    m_msgImgPtrAmplitude = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    m_msgImgPtrAmplitude->header.frame_id = "amplitude";
    m_msgImgPtrAmplitude->width = SCUBE_WIDTH;
    m_msgImgPtrAmplitude->height = SCUBE_HEIGHT;
    m_msgImgPtrAmplitude->is_bigendian = false;
    m_msgImgPtrAmplitude->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    m_msgImgPtrAmplitude->step = (uint32_t)(sizeof(float) * SCUBE_WIDTH);
    m_msgImgPtrAmplitude->data.resize(sizeof(float) * SCUBE_WIDTH * SCUBE_HEIGHT);
    m_pAmplitudeData = (float *)&m_msgImgPtrAmplitude->data[0];

    //generate Pointcloud
    m_msgPCL2ptr = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
    m_msgPCL2ptr->header.frame_id = "pcl";
    m_msgPCL2ptr->header.stamp = m_msgImgPtrAmplitude->header.stamp;
    m_msgPCL2ptr->width = SCUBE_WIDTH;
    m_msgPCL2ptr->height = SCUBE_HEIGHT;
    m_msgPCL2ptr->is_bigendian = false;
    m_msgPCL2ptr->is_dense = false;

    m_msgPCL2ptr->point_step = (uint32_t)(3 * sizeof(float));
    m_msgPCL2ptr->row_step = (uint32_t)(m_msgPCL2ptr->point_step * SCUBE_WIDTH);
    m_msgPCL2ptr->fields.resize(3);
    m_msgPCL2ptr->fields[0].name = "z";
    m_msgPCL2ptr->fields[0].offset = 0;
    m_msgPCL2ptr->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    m_msgPCL2ptr->fields[0].count = 1;

    m_msgPCL2ptr->fields[1].name = "y";
    m_msgPCL2ptr->fields[1].offset = m_msgPCL2ptr->fields[0].offset + (uint32_t)sizeof(float);
    m_msgPCL2ptr->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    m_msgPCL2ptr->fields[1].count = 1;

    m_msgPCL2ptr->fields[2].name = "x";
    m_msgPCL2ptr->fields[2].offset = m_msgPCL2ptr->fields[1].offset + (uint32_t)sizeof(float);
    m_msgPCL2ptr->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    m_msgPCL2ptr->fields[2].count = 1;
    m_msgPCL2ptr->data.resize(m_msgPCL2ptr->point_step * m_msgPCL2ptr->width * m_msgPCL2ptr->height);
}

bool connect()
{
    // find source
    int _selected_source = 0;
    meere::sensor::sptr_source_list _source_list = meere::sensor::search_camera_source();

    if (nullptr != _source_list) {
        int i = 0;
        for (auto it : (*_source_list)) {
            std::cout << i++ << ") source name : " << it->name() << \
                    ", serialNumber : " << it->serialNumber() << \
                    ", uri : " << it->uri() << std::endl;
        }
    }

    if (nullptr != _source_list) {
        if (1 < _source_list->size()) {
            std::cout << "Please enter the desired source number." << std::endl;
            scanf("%d", &_selected_source);
            getchar();
        }
        else {
            _selected_source = 0;
        }
    }
    else {
        std::cout << "no search device!" << std::endl;
        return -1;
    }

//    std::this_thread::sleep_for(std::chrono::milliseconds(1000));//sleep delay

    meere::sensor::add_prepared_listener(&_ReceivedIntensityPCLFrameSink);


    // create camera
//    meere::sensor::sptr_camera _camera = meere::sensor::create_camera(_source_list->at(_selected_source));
//    _camera = meere::sensor::create_camera(_source_list->at(_selected_source));
    m_camera = meere::sensor::create_camera(_source_list->at(_selected_source));
    if (nullptr != m_camera) {
        m_camera->addSink(&_ReceivedIntensityPCLFrameSink);

        meere::sensor::result _rt;
        _rt = m_camera->prepare();
        assert(meere::sensor::result::success == _rt);
        if (meere::sensor::result::success != _rt) {
            std::cout << "_camera->prepare() failed." << std::endl;
            meere::sensor::destroy_camera(m_camera);
            return -1;
        }

        // set wanted frame type
        int _wantedFrame = meere::sensor::CubeEyeFrame::FrameType_IntensityPointCloud;	// intensity(amplitude) & 3D(depth) & Point(x, y)


        _rt = m_camera->run(_wantedFrame);
        assert(meere::sensor::result::success == _rt);
        if (meere::sensor::result::success != _rt) {
            std::cout << "_camera->run() failed." << std::endl;
            meere::sensor::destroy_camera(m_camera);
            return -1;
        }
    }
    return true;
}

bool close()
{
    m_camera->stop();
    m_camera->release();
    meere::sensor::destroy_camera(m_camera);
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cubeeye_scube_node");
    ros::NodeHandle nh;

    init_ros(nh);


    dynamic_reconfigure::Server<cubeeye_scube_node::cubeeye_scubeConfig> server;
    dynamic_reconfigure::Server<cubeeye_scube_node::cubeeye_scubeConfig>::CallbackType f;

    f = boost::bind(&callbackConfig, _1, _2);
    server.setCallback(f);

    //loop speed 10Hz : It should be modified
    //to be in inverse proportion to the fps delay(nDelay) parameter.
    ros::Rate loop_rate(10);

    if(!connect()) {
        ROS_ERROR( "cubeeye connection failed!...");
        std::exit(1);
    }

    //Ctrl+C handler for standalone TEST.
    //It must be removed at Release version.
    signal(SIGINT, gracefulShutdown);

    ROS_INFO("cubeeye scube start\n");

    while(mLoopOk)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }    

    close();
    ROS_INFO("cubeeye scube stop\n");
    return 0;
}
