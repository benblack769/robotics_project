#ifndef _ROBOT_PLUGIN_HH_
#define _ROBOT_PLUGIN_HH_

//#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>


namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class MyCustomRobotPlugin : public ModelPlugin
  {

     //private: ros::NodeHandle* rosnode_;
     //private: ros::Publisher pub_;
     private: std::string name;

    /// \brief Constructor
    public: MyCustomRobotPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
         //ModelPlugin::Load(_model, _sdf);

      // Just output a message for now
      std::cerr << "\nThe model plugin is attach to model[" <<
        _model->GetName() << "]" << std::endl;
        this->name = _model->GetName();

        //ignition::math::Pose3d  pose3d = _model->WorldPose();
        //std::cerr << pose3d << std::endl;

        //this->rosnode_ = new ros::NodeHandle(this->name);
        //ros::Timer  worldmodel_update_timer_ = this->rosnode_->createTimer(ros::Duration(0.03),&MyCustomRobotPlugin::update,this);
    }
    //void update(const ros::TimerEvent& event){
    //    std::cerr << "updated!!!" << std::endl;
    //}
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MyCustomRobotPlugin)
}
#endif
