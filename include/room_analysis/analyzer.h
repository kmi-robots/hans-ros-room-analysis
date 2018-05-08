#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/Twist.h"
#include "room_analysis/Analyzer_configuration.h"

#include "room_analysis/json.hpp"

#include "curlpp/cURLpp.hpp"
#include "curlpp/Easy.hpp"
#include "curlpp/Options.hpp"
#include "curlpp/Exception.hpp"

using json = nlohmann::json;

void detectTags(Variables_ptr v, Parameters_ptr p, const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg) {
    for (auto it = msg->markers.begin(); it != msg->markers.end(); it++) {
        ar_track_alvar_msgs::AlvarMarker marker = *it;
        if (marker.id == v->tag_number) {
            ROS_INFO_STREAM("tag found");
            v->listener.waitForTransform(p->global_frame, marker.header.frame_id, marker.header.stamp, ros::Duration(0.1));
            tf::StampedTransform transform;
            try {
                marker.pose.header.frame_id = marker.header.frame_id;
                marker.pose.header.stamp = marker.header.stamp;
                v->listener.transformPose(p->global_frame, marker.pose, v->marker);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                break;
            }
            v->timestamp = marker.header.stamp.toNSec();
            v->found = true;
        }
    }
}

geometry_msgs::Twist explore(Variables_ptr v, Parameters_ptr p) {
    geometry_msgs::Twist msg;
    msg.angular.z = p->velocity;
    return msg;
}

void transmitPosition(Variables_ptr v, Parameters_ptr p) {
    ROS_INFO_STREAM("Transmitting...");
    curlpp::Easy request;
    request.setOpt(new curlpp::options::Url(p->url));
//     std::list<std::string> header;
//     header.push_back("Content-Type: application/json");
//     request.setOpt(new curlpp::options::HttpHeader(header));
    json j;
    j["detections"] = {{
        {"behaviour", "checkHeaterFreeArea"},
        {"category", "object"},
        {"class", "Heater"},
        {"pose", {
            {"position", {
                {"x", v->marker.pose.position.x},
                {"y", v->marker.pose.position.y},
                {"z", v->marker.pose.position.z}}
            },
            {"orientation", {
                {"x", v->marker.pose.orientation.x},
                {"y", v->marker.pose.orientation.y},
                {"z", v->marker.pose.orientation.z},
                {"w", v->marker.pose.orientation.w}}
            }
        }},
        {"timestamp", v->timestamp}
    }};
    std::stringstream ss;
    ss << "json=" << j.dump();
    request.setOpt(new curlpp::options::PostFields(ss.str()));
    ROS_INFO_STREAM("Sending request: " << ss.str());
    request.perform();
}
