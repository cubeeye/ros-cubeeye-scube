#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeBasicFrame.h"
#include "CubeEyePointCloudFrame.h"
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


//int m_nWidth;
//int m_nHeight;
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
meere::sensor::sptr_camera _camera;

uint16_t m_nAmplitude;
uint16_t m_nScattering;



void gracefulShutdown(int sigNum) {
    mLoopOk = false;
}

void callbackConfig(cubeeye_scube_node::cubeeye_scubeConfig &config, uint32_t level)
{
    switch (level)
    {

    case 1:
        m_nAmplitude = config.Amplitude;
        {
            meere::sensor::sptr_property _prop = meere::sensor::make_property_16u("amplitude_threshold_min", m_nAmplitude);
            if (nullptr != _prop)
            {
                // set property
                if (meere::sensor::result::success != _camera->setProperty(_prop))
                {
                    // error
                    std::cout << "setProperty(" << _prop->key() << ", " << _prop->asInt16u() << ") failed." << std::endl;
                }
            }
        }
        break;

    case 2:
        m_nScattering = config.Scattering;
        {
            meere::sensor::sptr_property _prop = meere::sensor::make_property_16u("scattering_threshold ", m_nScattering);
            if (nullptr != _prop)
            {
                // set property
                if (meere::sensor::result::success != _camera->setProperty(_prop))
                {
                    // error
                    std::cout << "setProperty(" << _prop->key() << ", " << _prop->asInt16u() << ") failed." << std::endl;
                }
            }
        }
        break;
    }
}


auto printProperty = [](meere::sensor::sptr_property prop) {
    if (nullptr != prop && !prop->key().empty()) {
        std::cout << "-> result : " << prop->key();
        switch (prop->dataType()) {
        case meere::sensor::CubeEyeData::DataType_Boolean:
            std::cout << " , " << prop->asBoolean() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_8S:
            std::cout << " , " << prop->asInt8s() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_8U:
            std::cout << " , " << prop->asInt8u() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_16S:
            std::cout << " , " << prop->asInt16s() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_16U:
            std::cout << " , " << prop->asInt16u() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_32S:
            std::cout << " , " << prop->asInt32s() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_32U:
            std::cout << " , " << prop->asInt32u() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_32F:
            std::cout << " , " << prop->asFlt32() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_64F:
            std::cout << " , " << prop->asFlt64() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_64S:
            std::cout << " , " << prop->asInt64s() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_64U:
            std::cout << " , " << prop->asInt64u() << std::endl;
            break;
        case meere::sensor::CubeEyeData::DataType_String:
            std::cout << " , " << prop->asString() << std::endl;
            break;
        default:
            std::cout << std::endl;
            break;
        }
    }
};




static class TestSink : public meere::sensor::CubeEyeSink
 , public meere::sensor::CubeEyeCamera::PreparedListener
{
public:
    virtual std::string name() const {
        return std::string("TestSink");
    }

    virtual void onCubeEyeCameraState(const meere::sensor::CubeEyeSource* source, meere::sensor::CubeEyeCamera::State state) {
        printf("%s:%d source(%s) state = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), state);

    }

    virtual void onCubeEyeFrameList(const meere::sensor::CubeEyeSource* source , const meere::sensor::sptr_frame_list& frames) {
#if 0
        static int _frame_cnt = 0;
        if (30 > ++_frame_cnt) {
            return;
        }
        _frame_cnt = 0;
#endif

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
#endif
            int _frame_index = 0;
            auto _center_x = it->frameWidth() / 2;
            auto _center_y = it->frameHeight() / 2;

            if (it->frameType() == meere::sensor::CubeEyeFrame::FrameType_Depth) {
                if (it->frameDataType() == meere::sensor::CubeEyeData::DataType_16U) {
                    auto _sptr_basic_frame = meere::sensor::frame_cast_basic16u(it);
                    auto _sptr_frame_data = _sptr_basic_frame->frameData();

                    for (int y = 0 ; y < _sptr_basic_frame->frameHeight(); y++) {
                        for (int x = 0 ; x < _sptr_basic_frame->frameWidth(); x++) {
                            _frame_index = y * _sptr_basic_frame->frameWidth() + x;
                            if (_center_x == x && _center_y == y) {
                                printf("depth(%d,%d) data : %d\n", _center_x, _center_y, (*_sptr_frame_data)[_frame_index]);
                            }
                        }
                    }
                }
            }
            else if (it->frameType() == meere::sensor::CubeEyeFrame::FrameType_Amplitude) {
                if (it->frameDataType() == meere::sensor::CubeEyeData::DataType_16U) {
                    auto _sptr_basic_frame = meere::sensor::frame_cast_basic16u(it);
                    auto _sptr_frame_data = _sptr_basic_frame->frameData();

                    for (int y = 0 ; y < _sptr_basic_frame->frameHeight(); y++) {
                        for (int x = 0 ; x < _sptr_basic_frame->frameWidth(); x++) {
                            _frame_index = y * _sptr_basic_frame->frameWidth() + x;
                            if (_center_x == x && _center_y == y) {
                                printf("amplitude(%d,%d) data : %d\n", _center_x, _center_y, (*_sptr_frame_data)[_frame_index]);
                            }
                        }
                    }
                }
            }
            else if (it->frameType() == meere::sensor::CubeEyeFrame::FrameType_PointCloud) {
                if (it->frameDataType() == meere::sensor::CubeEyeData::DataType_16U) {

                }
                else if (it->frameDataType() == meere::sensor::CubeEyeData::DataType_32F) {
                    auto _sptr_pointcloud_frame = meere::sensor::frame_cast_pointcloud32f(it);
                    auto _sptr_frame_dataX = _sptr_pointcloud_frame->frameDataX();
                    auto _sptr_frame_dataY = _sptr_pointcloud_frame->frameDataY();
                    auto _sptr_frame_dataZ = _sptr_pointcloud_frame->frameDataZ();

                    for (int y = 0 ; y < _sptr_pointcloud_frame->frameHeight(); y++) {
                        for (int x = 0 ; x < _sptr_pointcloud_frame->frameWidth(); x++) {
                            _frame_index = y * _sptr_pointcloud_frame->frameWidth() + x;
                            if (_center_x == x && _center_y == y) {
                                printf("point-cloud(%d,%d) data : %f, %f, %f\n", _center_x, _center_y, \
                                (*_sptr_frame_dataX)[_frame_index] * 1000, (*_sptr_frame_dataY)[_frame_index] * 1000, (*_sptr_frame_dataZ)[_frame_index] * 1000);
                            }
                        }
                    }
                }
            }
            else if (it->frameType() == meere::sensor::CubeEyeFrame::FrameType_IntensityPointCloud) {
                if (it->frameDataType() == meere::sensor::CubeEyeData::DataType_16U) {

                }
                else if (it->frameDataType() == meere::sensor::CubeEyeData::DataType_32F) {
                    auto _sptr_intensity_pointcloud_frame = meere::sensor::frame_cast_intensity_pointcloud32f(it);
                    auto _sptr_frame_dataX = _sptr_intensity_pointcloud_frame->frameDataX();
                    auto _sptr_frame_dataY = _sptr_intensity_pointcloud_frame->frameDataY();
                    auto _sptr_frame_dataZ = _sptr_intensity_pointcloud_frame->frameDataZ();
                    auto _sptr_frame_dataI = _sptr_intensity_pointcloud_frame->frameDataI();

                    for (int y = 0 ; y < _sptr_intensity_pointcloud_frame->frameHeight(); y++) {
                        for (int x = 0 ; x < _sptr_intensity_pointcloud_frame->frameWidth(); x++) {
                            _frame_index = y * _sptr_intensity_pointcloud_frame->frameWidth() + x;

                            float *pcl_a = (float *)&m_msgPCL2ptr->data[_frame_index * m_msgPCL2ptr->point_step];
                            float *pcl_b = pcl_a + 1;
                            float *pcl_c = pcl_b + 1;

                            *pcl_a = (*_sptr_frame_dataY)[_frame_index]*-1;
                            *pcl_b = (*_sptr_frame_dataX)[_frame_index]*-1;
                            *pcl_c = (*_sptr_frame_dataZ)[_frame_index];

                            m_pDepthData[_frame_index] = (*_sptr_frame_dataZ)[_frame_index];
                            m_pAmplitudeData[_frame_index] = (*_sptr_frame_dataI)[_frame_index];

#if 0
                            if (_center_x == x && _center_y == y) {
                                printf("point-cloud(%d,%d) data : %f, %f, %f, %f\n", _center_x, _center_y, \
                                (*_sptr_frame_dataX)[_frame_index] * 1000, (*_sptr_frame_dataY)[_frame_index] * 1000, \
                                (*_sptr_frame_dataZ)[_frame_index] * 1000, (*_sptr_frame_dataI)[_frame_index] * 1000);
                            }
#endif
                        }
                    }
//                    m_PubAmplitudeRaw.publish(m_msgImgPtrAmplitude);
//                    m_PubDepthRaw.publish(m_msgImgPtrDepth);
//                    m_PubPCLRaw.publish(m_msgPCL2ptr);

                    pub_amplitude_raw.publish(m_msgImgPtrAmplitude);
                    pub_depth_raw.publish(m_msgImgPtrDepth);
                    pub_pcl_raw.publish(m_msgPCL2ptr);
                }
            }
        }
    }

public:
    virtual void onCubeEyeCameraPrepared(const meere::sensor::CubeEyeCamera* camera) {
        printf("%s:%d source(%s)\n", __FUNCTION__, __LINE__, camera->source()->uri().c_str());
    }

public:
    TestSink() = default;
    virtual ~TestSink() = default;
} _test_sink;

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

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));//sleep delay

    // create camera
//    meere::sensor::sptr_camera _camera = meere::sensor::create_camera(_source_list->at(_selected_source));
    _camera = meere::sensor::create_camera(_source_list->at(_selected_source));
    if (nullptr != _camera) {
#if 1
        _camera->addSink(&_test_sink);
        _camera->addPreparedListener(&_test_sink);
#endif

        meere::sensor::result _rt;
        _rt = _camera->prepare();
        assert(meere::sensor::result::success == _rt);
        if (meere::sensor::result::success != _rt) {
            std::cout << "_camera->prepare() failed." << std::endl;
            meere::sensor::destroy_camera(_camera);
            return -1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));//sleep delay

        // set wanted frame type
//		int _wantedFrame = meere::sensor::CubeEyeFrame::FrameType_Depth;				// raw depth only
//		int _wantedFrame = meere::sensor::CubeEyeFrame::FrameType_Depth | meere::sensor::CubeEyeFrame::FrameType_Amplitude;	// raw depth & amplitude
//		int _wantedFrame = meere::sensor::CubeEyeFrame::FrameType_PointCloud;			// 3D(depth) & Point(x, y)
        int _wantedFrame = meere::sensor::CubeEyeFrame::FrameType_IntensityPointCloud;	// intensity(amplitude) & 3D(depth) & Point(x, y)


        _rt = _camera->run(_wantedFrame);
        assert(meere::sensor::result::success == _rt);
        if (meere::sensor::result::success != _rt) {
            std::cout << "_camera->run() failed." << std::endl;
            meere::sensor::destroy_camera(_camera);
            return -1;
        }

#if 0
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));//sleep delay

        // set/get property : amplitude_threshold_min
        {
            meere::sensor::sptr_property _prop = meere::sensor::make_property_16u("amplitude_threshold_min", 100);
            if (nullptr != _prop) {
                // set property
                if (meere::sensor::result::success != _camera->setProperty(_prop)) {
                    // error
                    std::cout << "setProperty(" << _prop->key() << ", " << _prop->asInt16u() << ") failed." << std::endl;
                }

                // get property
                auto _rt_prop = _camera->getProperty(_prop->key());
                if (meere::sensor::result::success == std::get<0>(_rt_prop)) {
                    printProperty(std::get<1>(_rt_prop));
                }
                else {
                    std::cout << "getProperty(" << _prop->key() << ") failed." << std::endl;
                }
            }

            // set default 0
            {
                meere::sensor::sptr_property _prop = meere::sensor::make_property_16u("amplitude_threshold_min", 0);
                if (nullptr != _prop) {
                    _camera->setProperty(_prop);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));//sleep delay
        // set/get property : edge_filter
        {
            meere::sensor::sptr_property _prop = meere::sensor::make_property_string("edge_filter", "on");
            if (nullptr != _prop) {
                // set property
                if (meere::sensor::result::success != _camera->setProperty(_prop)) {
                    // error
                    std::cout << "setProperty(" << _prop->key() << ", " << _prop->asString() << ") failed." << std::endl;
                }

                // get property
                auto _rt_prop = _camera->getProperty(_prop->key());
                if (meere::sensor::result::success == std::get<0>(_rt_prop)) {
                    printProperty(std::get<1>(_rt_prop));
                }
                else {
                    std::cout << "getProperty(" << _prop->key() << ") failed." << std::endl;
                }
            }

            // set default : off
            {
                meere::sensor::sptr_property _prop = meere::sensor::make_property_string("edge_filter", "off");
                if (nullptr != _prop) {
                    _camera->setProperty(_prop);
                }
            }
        }
#endif
    }
    return true;
}

bool close()
{
    _camera->stop();    
    _camera->release();    
    meere::sensor::destroy_camera(_camera);    
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
        ROS_ERROR( "Depth Camera Connection Failed!...");
        std::exit(1);
    }

    //Ctrl+C handler for standalone TEST.
    //It must be removed at Release version.
    signal(SIGINT, gracefulShutdown);

    ROS_INFO("Depth Camera Start\n");

    while(mLoopOk)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }    

    close();
    ROS_INFO("Depth Camera Stop\n");
    return 0;
}
