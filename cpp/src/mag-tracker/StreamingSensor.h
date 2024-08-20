#ifndef STREAMING_SENSOR_H
#define STREAMING_SENSOR_H
#include "mag-tracker/ndi/NdiManager.h"
#include "vistendon/ManualRvizMarkerArrayPublisher.h"
#include <rclcpp/rclcpp.hpp>


class StreamingSensor:public QObject{
    Q_OBJECT
    public:
    StreamingSensor();
    private:
    ndi::NdiManager manager;
    vistendon::ManualRvizMarkerArrayPublisher tip_publisher;


};
#endif
