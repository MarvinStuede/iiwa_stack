#include <stop_movement_service.h>
namespace iiwa_ros {
StopMovementService::StopMovementService(): iiwaServices< iiwa_msgs::StopMovement>() {}
StopMovementService::StopMovementService(const std::string& service_name, const bool verbose) : iiwaServices< iiwa_msgs::StopMovement >(service_name, verbose) {}


bool StopMovementService::stopMovement(float timeout)
{
    config_.request.stop_timeout = timeout;
    return callService();
}

bool StopMovementService::callService()
{
    if (service_ready_) {
      if (client_.call(config_)) {
        if(!config_.response.success && verbose_) {
          ROS_ERROR_STREAM(service_name_ << " failed");
        }
        else if (verbose_) {
          ROS_INFO_STREAM(ros::this_node::getName() << ":" << service_name_ << " successfully called.");
        }
      }
      else if (verbose_) {
        ROS_ERROR_STREAM(service_name_ << " could not be called");
      }
      return config_.response.success;
    }
    ROS_ERROR_STREAM("The service client was not intialized yet.");
}
}
