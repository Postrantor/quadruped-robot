/**
 * @brief
 * @date 2024-03-11
 * @copyright Copyright (c) 2024
 */

#include <string>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo {
class UnitreeFootContactPlugin : public SensorPlugin {
public:
  UnitreeFootContactPlugin() : SensorPlugin() {}
  ~UnitreeFootContactPlugin() {}

  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
    this->parentSensor =
        std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);  // Make sure the parent sensor is valid.
    if (!this->parentSensor) {
      gzerr << "UnitreeFootContactPlugin requires a ContactSensor.\n";
      return;
    }
    this->contact_namespace = "contact/";
    this->rosnode = new ros::NodeHandle(this->contact_namespace);
    // add "visual" is for the same name of draw node
    this->force_pub =
        this->rosnode->advertise<geometry_msgs::WrenchStamped>("/visual/" + _sensor->Name() + "/the_force", 100);
    // Connect to the sensor update event.
    this->update_connection = this->parentSensor->ConnectUpdated(std::bind(&UnitreeFootContactPlugin::OnUpdate, this));
    this->parentSensor->SetActive(true);  // Make sure the parent sensor is active.
    count = 0;
    Fx = 0;
    Fy = 0;
    Fz = 0;
    ROS_INFO("Load %s plugin.", _sensor->Name().c_str());
  }

private:
  void OnUpdate() {
    msgs::Contacts contacts;

    contacts = this->parentSensor->Contacts();
    count = contacts.contact_size();
    // std::cout << "count = " << count <<std::endl;
    for (unsigned int i = 0; i < count; ++i) {
      if (contacts.contact(i).position_size() != 1) {
        ROS_ERROR("Contact count isn't correct!!!!");
      }
      for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j) {
        // std::cout << i <<" "<< contacts.contact(i).position_size() <<" Force:"
        //           << contacts.contact(i).wrench(j).body_1_wrench().force().x() << " "
        //           << contacts.contact(i).wrench(j).body_1_wrench().force().y() << " "
        //           << contacts.contact(i).wrench(j).body_1_wrench().force().z() << "\n";
        // const auto& pos = contacts.contact(i).position(j);
        // std::cout << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
        // x +=pos.x();
        // y +=pos.y();
        // z +=pos.z();

        Fx += contacts.contact(i).wrench(0).body_1_wrench().force().x();  // Notice: the force is in local coordinate,
                                                                          // not in world or base coordnate.
        Fy += contacts.contact(i).wrench(0).body_1_wrench().force().y();
        Fz += contacts.contact(i).wrench(0).body_1_wrench().force().z();
      }
    }
    if (count != 0) {
      force.wrench.force.x = Fx / double(count);
      force.wrench.force.y = Fy / double(count);
      force.wrench.force.z = Fz / double(count);
      count = 0;
      Fx = 0;
      Fy = 0;
      Fz = 0;
    } else {
      force.wrench.force.x = 0;
      force.wrench.force.y = 0;
      force.wrench.force.z = 0;
    }
    force.header.stamp = ros::Time::now();
    this->force_pub.publish(force);
  }

private:
  ros::NodeHandle* rosnode;
  ros::Publisher force_pub;
  event::ConnectionPtr update_connection;
  std::string contact_namespace;
  sensors::ContactSensorPtr parentSensor;
  geometry_msgs::WrenchStamped force;
  int count = 0;
  double Fx = 0, Fy = 0, Fz = 0;
};
GZ_REGISTER_SENSOR_PLUGIN(UnitreeFootContactPlugin)
}  // namespace gazebo
