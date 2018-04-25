#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/Twist.h"
#include "room_analysis/Analyzer_configuration.h"

#include "curlpp/cURLpp.hpp"
#include "curlpp/Easy.hpp"
#include "curlpp/Options.hpp"
#include "curlpp/Exception.hpp"

void detectTags(Variables_ptr v, Parameters_ptr p, const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg) {
    v->found = false;
    for (auto it = msg->markers.begin(); it != msg->markers.end(); it++) {
        ar_track_alvar_msgs::AlvarMarker marker = *it;
        if (marker.id == v->tag_number) {
            v->marker = marker.pose;
            v->found = true;
        }
    }
}

geometry_msgs::Twist explore(Variables_ptr v, Parameters_ptr p) {
    geometry_msgs::Twist msg;
    msg.angular.z = p->velocity;
    return msg;
}

void transmitPosition(Variables_ptr v) {
    curlpp::Easy request;
    request.setOpt(new curlpp::options::Url("http://httpbin.org/post"));
    std::list<std::string> header;
    header.push_back("Content-Type: application/json");
    request.setOpt(new curlpp::options::HttpHeader(header));
    std::stringstream ss;
    ss<<"{\"pose\":{\"position\":["<<
        "{\"x\":"<<v->marker.pose.position.x<<"},"<<
        "{\"y\":"<<v->marker.pose.position.y<<"},"<<
        "{\"z\":"<<v->marker.pose.position.z<<"}]"<<
        ", \"orientation\":["<<
        "{\"x\":"<<v->marker.pose.orientation.x<<"},"<<
        "{\"y\":"<<v->marker.pose.orientation.z<<"},"<<
        "{\"z\":"<<v->marker.pose.orientation.y<<"},"<<
        "{\"w\":"<<v->marker.pose.orientation.w<<"}]"<<
        "}}";
    request.setOpt(new curlpp::options::PostFields(ss.str()));
    request.perform();
}
