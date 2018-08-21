#ifndef _ANALYZER_CONFIGURATION_H
#define _ANALYZER_CONFIGURATION_H
/**
 * Auto-generated Internal State
 */
#include "node_base/Configuration.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

struct Variables: node_base::VariablesBase {
    tf::TransformListener listener;
    std::vector<geometry_msgs::PoseStamped> markers;
    std::vector<uint64_t> timestamps;
    bool found;
    std::vector<std::string> classes;
    
    Variables() {
        found = false;
    };
};

struct Parameters: node_base::ParametersBase {
    double velocity;
    std::string global_frame;
    std::string url;
};

typedef std::shared_ptr < const Parameters > Parameters_ptr;
typedef std::shared_ptr < Variables > Variables_ptr;

class InternalState: node_base::InternalStateBase {
    public:
        Variables_ptr vars() {
            return std::static_pointer_cast < Variables > (_vars);
        };

        Parameters_ptr params() const {
            return std::static_pointer_cast < const Parameters > (_params);
        };

        void initialize(node_base::ParametersBase *p = NULL) {
            _params = std::make_shared < const Parameters > (*static_cast < Parameters * >(p));
            _vars = std::make_shared < Variables > ();
        }
};
#endif
