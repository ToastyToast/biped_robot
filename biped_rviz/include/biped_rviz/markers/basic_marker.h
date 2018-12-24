#ifndef BIPED_RVIZ_BASIC_MARKER_H
#define BIPED_RVIZ_BASIC_MARKER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace biped_rviz {

class BasicMarker {
public:
    BasicMarker(const std::string& marker_name, const std::string& frame_name);
    visualization_msgs::Marker getROSMsg();
    
    std::string getName() const;
    std::string getFrameName() const;
    
private:
    std::string m_name;
    std::string m_frame_name;
    uint32_t m_type = visualization_msgs::Marker::SPHERE;
};

}

#endif //BIPED_RVIZ_BASIC_MARKER_H
