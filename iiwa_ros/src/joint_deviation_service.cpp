#include <joint_deviation_service.h>
namespace iiwa_ros {
JointDeviationService::JointDeviationService(): iiwaServices<iiwa_msgs::SetMaxJointDeviation>() {}
JointDeviationService::JointDeviationService(const std::string& service_name, const bool verbose) : iiwaServices<iiwa_msgs::SetMaxJointDeviation>(service_name, verbose) {}


bool JointDeviationService::setMaxJointDeviation(double maxDeviation)
{
    config_.request.max_deviation= maxDeviation;
    return callService();
}

bool JointDeviationService::callService()
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
