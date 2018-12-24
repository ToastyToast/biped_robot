
#include <biped_rviz/markers/basic_marker.h>

#include "biped_rviz/markers/basic_marker.h"

using namespace biped_rviz;

BasicMarker::BasicMarker(const std::string& marker_name, const std::string& frame_name)
    : m_name(marker_name), m_frame_name(frame_name)
{

}

std::string BasicMarker::getName() const {
    return m_name;
}

std::string BasicMarker::getFrameName() const {
    return m_frame_name;
}

visualization_msgs::Marker BasicMarker::getROSMsg()
{
    visualization_msgs::Marker marker_msg;
    
    marker_msg.header.frame_id = m_frame_name;
    marker_msg.header.stamp = ros::Time::now();
    
    marker_msg.ns = m_name;
    marker_msg.id = 0;
    
    marker_msg.type = m_type;
    
    marker_msg.action = visualization_msgs::Marker::ADD;
    
    marker_msg.pose.position.x = 0.0f;
    marker_msg.pose.position.y = 0.0f;
    marker_msg.pose.position.z = 0.0f;
    
    marker_msg.pose.orientation.x = 0.0f;
    marker_msg.pose.orientation.y = 0.0f;
    marker_msg.pose.orientation.z = 0.0f;
    marker_msg.pose.orientation.w = 0.0f;
    
    marker_msg.scale.x = 1.0f;
    marker_msg.scale.y = 1.0f;
    marker_msg.scale.z = 1.0f;
    
    marker_msg.color.r = 0.0f;
    marker_msg.color.g = 1.0f;
    marker_msg.color.b = 0.0f;
    marker_msg.color.a = 1.0f;
    
    marker_msg.lifetime = ros::Duration();
    
    return marker_msg;
}
