/**
 * Node Analyzer
 * File auto-generated on 22/03/2018 10:57:44
 */
#include "ros_base/ROSNode.h"
#include "room_analysis/Analyzer_configuration.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "room_analysis/analyzer.h"
#include "geometry_msgs/Twist.h"
#include "actionlib/server/simple_action_server.h"
#include "room_analysis/ExploreAction.h"


class Analyzer : public ros_base::ROSNode {
    private:
        bool prepare();
        void tearDown();
        void errorHandling();
        void tags_callback_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
        void velocity_publisher_callback(const ros::TimerEvent &);
        void goalCB();
        InternalState is;
        ros::Subscriber sub_tags_callback;
        ros::Publisher pub_velocity_publisher;
        ros::Timer timer_velocity_publisher;
        actionlib::SimpleActionServer<room_analysis::ExploreAction> action_server;
    public:
        Analyzer();
};

/**
 * Method nodeSigintHandler auto-generated
 */
void nodeSigintHandler(int sig) {
    g_request_shutdown = 1;
}

/**
 * Method main auto-generated
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "Analyzer", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeSigintHandler);
    Analyzer node;
    node.start();
    return 0;
}

/**
 * Method prepare auto-generated
 */
bool Analyzer::prepare() {
    Parameters p;
    handle.param<double>("velocity", p.velocity, 0.5);
    handle.param<std::string>("global_frame", p.global_frame, "map");
    is.initialize(&p);
    pub_velocity_publisher = handle.advertise < geometry_msgs::Twist > ("/cmd_vel", 10);
    timer_velocity_publisher = handle.createTimer(ros::Duration(0.1), &Analyzer::velocity_publisher_callback, this, false, false);
    action_server.registerGoalCallback(boost::bind(&Analyzer::goalCB, this));
    action_server.start();
    return true;
}

void Analyzer::goalCB() {
    is.vars()->tag_number = action_server.acceptNewGoal()->tag_number;
    sub_tags_callback = handle.subscribe("/ar_pose_marker", 1, &Analyzer::tags_callback_callback, this);
    timer_velocity_publisher.start();
    ros::Duration(12.0).sleep();
    timer_velocity_publisher.stop();
    sub_tags_callback.shutdown();
    room_analysis::ExploreResult result_;
    result_.found = is.vars()->found;
    if(is.vars()->found) {
        result_.pose = is.vars()->marker;
        transmitPosition(is.vars());
    }
    action_server.setSucceeded(result_);
}

/**
 * Method tearDown auto-generated
 */
void Analyzer::tearDown() {
    ROS_INFO("Node is shutting down");
    return;
}

/**
 * Method errorHandling auto-generated
 */
void Analyzer::errorHandling() {
    ROSNode::errorHandling();
}

/**
 * Method tags_callback_callback auto-generated
 */
void Analyzer::tags_callback_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg) {
    detectTags(is.vars(), is.params(), msg);
}

/**
 * Method velocity_publisher_callback auto-generated
 */
void Analyzer::velocity_publisher_callback(const ros::TimerEvent &) {
    pub_velocity_publisher.publish(explore(is.vars(), is.params()));
}

/**
 * Method Analyzer auto-generated
 */
Analyzer::Analyzer() : action_server(handle, "explore", false) {
    setName(ros::this_node::getName());
}
