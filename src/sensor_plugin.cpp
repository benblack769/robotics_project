#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

//#include <ros/ros.h>
//#include <ros/callback_queue.h>
//#include <ros/advertise_options.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/util/system.hh>
#include <gazebo/plugins/RayPlugin.hh>


#include <memory>
#include <vector>
#include <cstdlib>

#include <ros_pyenv/scan_msg.h>


#define EPSILON_DIFF 0.000001

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class RobotPlugin : public RayPlugin
  {
     private: sensors::RaySensorPtr parent_ray_sensor_;
     private: event::ConnectionPtr newLaserScansConnection;

     private: std::string topic_name_;
     private: double gaussian_noise_;

     private: std::string frame_name_;
    /// \brief Constructor
    public: RobotPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        RayPlugin::Load(_parent, _sdf);
      this->parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

     if (!this->parent_ray_sensor_)
       gzthrow("GazeboRosBlockLaser controller requires a Ray Sensor as its parent");

        if (!_sdf->HasElement("gaussianNoise")){
            //ROS_INFO_NAMED("block_laser", "Block laser plugin missing <gaussianNoise>, defaults to 0.0");
            this->gaussian_noise_ = 0.001;
        }
        else
            this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();

      // Just output a message for now
      std::cerr << "\nThe robot sensor plugin is attach to model[" <<
        _parent->Name() << "]\n";
        if (_sdf->HasElement("path")){
            std::string s = _sdf->Get<std::string>("identifier");
            std::cerr << "found element" <<"\n";
            std::cerr << s <<"\n";
        }
        auto minAngle = this->parent_ray_sensor_->AngleMin();
        std::cerr << "min angle" <<"\n";
        std::cerr << minAngle <<"\n";

        this->frame_name_ = "/sensor1";

           this->parent_ray_sensor_->SetActive(true);
  this->newLaserScansConnection =
    this->parent_ray_sensor_->LaserShape()->ConnectNewLaserScans(
      boost::bind(&RobotPlugin::OnNewLaserScans, this));
    }
    // Put laser data to the interface

    common::Time last_time;
     // Update the controller
     void OnNewLaserScans()
     {
         common::Time sensor_update_time = this->parent_ray_sensor_->LastUpdateTime();
         if(last_time < sensor_update_time){
             PutLaserData(sensor_update_time);
             last_time = sensor_update_time;
         }
         //std::cerr << "got new lazer scan" << std::endl;
     }

 void PutLaserData(common::Time &_updateTime)
 {
   int i, hja, hjb;
   int j, vja, vjb;
   double vb, hb;
   int    j1, j2, j3, j4; // four corners indices
   double r1, r2, r3, r4, r; // four corner values + interpolated range
   double intensity;

   this->parent_ray_sensor_->SetActive(false);

   auto maxAngle = this->parent_ray_sensor_->AngleMax();
   auto minAngle = this->parent_ray_sensor_->AngleMin();

   double maxRange = this->parent_ray_sensor_->RangeMax();
   double minRange = this->parent_ray_sensor_->RangeMin();
   int rayCount = this->parent_ray_sensor_->RayCount();
   int rangeCount = this->parent_ray_sensor_->RangeCount();

   int verticalRayCount = this->parent_ray_sensor_->VerticalRayCount();
   int verticalRangeCount = this->parent_ray_sensor_->VerticalRangeCount();
   auto verticalMaxAngle = this->parent_ray_sensor_->VerticalAngleMax();
   auto verticalMinAngle = this->parent_ray_sensor_->VerticalAngleMin();

   double yDiff = maxAngle.Radian() - minAngle.Radian();
   double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

   std::vector<float> pointsx;
   std::vector<float> pointsy;
   std::vector<uint8_t> point_type;

   double dist = -1;
   std::string entity = "bob";

      for(int i = 0; i < rayCount; i++){
           // Should check these pointers for NULL
           this->parent_ray_sensor_->LaserShape()->Ray(i)->GetIntersection(dist, entity);
           if(dist < 100)
           std::cerr << dist << "   " << entity << "\n";

      }
   for (j = 0; j<verticalRangeCount; j++)
   {
     // interpolating in vertical direction
     vb = (verticalRangeCount == 1) ? 0 : (double) j * (verticalRayCount - 1) / (verticalRangeCount - 1);
     vja = (int) floor(vb);
     vjb = std::min(vja + 1, verticalRayCount - 1);
     vb = vb - floor(vb); // fraction from min

     assert(vja >= 0 && vja < verticalRayCount);
     assert(vjb >= 0 && vjb < verticalRayCount);

     for (i = 0; i<rangeCount; i++)
     {
       // Interpolate the range readings from the rays in horizontal direction
       hb = (rangeCount == 1)? 0 : (double) i * (rayCount - 1) / (rangeCount - 1);
       hja = (int) floor(hb);
       hjb = std::min(hja + 1, rayCount - 1);
       hb = hb - floor(hb); // fraction from min

       assert(hja >= 0 && hja < rayCount);
       assert(hjb >= 0 && hjb < rayCount);

       // indices of 4 corners
       j1 = hja + vja * rayCount;
       j2 = hjb + vja * rayCount;
       j3 = hja + vjb * rayCount;
       j4 = hjb + vjb * rayCount;
       // range readings of 4 corners
       r1 = std::min(this->parent_ray_sensor_->LaserShape()->GetRange(j1) , maxRange-minRange);
       r2 = std::min(this->parent_ray_sensor_->LaserShape()->GetRange(j2) , maxRange-minRange);
       r3 = std::min(this->parent_ray_sensor_->LaserShape()->GetRange(j3) , maxRange-minRange);
       r4 = std::min(this->parent_ray_sensor_->LaserShape()->GetRange(j4) , maxRange-minRange);

       // Range is linear interpolation if values are close,
       // and min if they are very different
       r = (1-vb)*((1 - hb) * r1 + hb * r2)
          +   vb *((1 - hb) * r3 + hb * r4);

       // Intensity is averaged
       intensity = 0.25*(this->parent_ray_sensor_->LaserShape()->GetRetro(j1) +
                         this->parent_ray_sensor_->LaserShape()->GetRetro(j2) +
                         this->parent_ray_sensor_->LaserShape()->GetRetro(j3) +
                         this->parent_ray_sensor_->LaserShape()->GetRetro(j4));

       // std::cout << " block debug "
       //           << "  ij("<<i<<","<<j<<")"
       //           << "  j1234("<<j1<<","<<j2<<","<<j3<<","<<j4<<")"
       //           << "  r1234("<<r1<<","<<r2<<","<<r3<<","<<r4<<")"
       //           << std::endl;

       // get angles of ray to get xyz for point
       double yAngle = 0.5*(hja+hjb) * yDiff / (rayCount -1) + minAngle.Radian();
       double pAngle = 0.5*(vja+vjb) * pDiff / (verticalRayCount -1) + verticalMinAngle.Radian();

       /***************************************************************/
       /*                                                             */
       /*  point scan from laser                                      */
       /*                                                             */
       /***************************************************************/
       //compare 2 doubles
       double diffRange = maxRange - minRange;
       double diff  = diffRange - r;
       double xval,yval,zval;
       if (fabs(diff) < EPSILON_DIFF)
       {
         // no noise if at max range
         //pAngle is rotated by yAngle:
         xval = r * cos(pAngle) * cos(yAngle);
         yval = r * cos(pAngle) * sin(yAngle);
         zval = r * sin(pAngle);

       }
       else
       {
         //pAngle is rotated by yAngle:
         xval = r * cos(pAngle) * cos(yAngle) + this->GaussianKernel(0,this->gaussian_noise_);
         yval = r * cos(pAngle) * sin(yAngle) + this->GaussianKernel(0,this->gaussian_noise_);
         zval = r * sin(pAngle) + this->GaussianKernel(0,this->gaussian_noise_);
       } // only 1 channel

        pointsx.push_back(xval);
        pointsy.push_back(yval);
     }
   }
   this->parent_ray_sensor_->SetActive(true);

   // send data out via ros message



 }

// Utility for adding noise
double GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_SENSOR_PLUGIN(RobotPlugin)
}
#endif
