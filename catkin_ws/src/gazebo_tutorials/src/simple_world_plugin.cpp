#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/physics/Contact.hh"

// namespace gazebo
// {
//   using namespace sensors;

//   GZ_REGISTER_STATIC_SENSOR("contact", ContactSensor)
//   class ContactPlugin : public SensorPlugin
//   {
//   public:
//     void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
//     {
//       // Get the parent sensor.
//       this->parentSensor =
//           std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

//       // Make sure the parent sensor is valid.
//       if (!this->parentSensor)
//       {
//         gzerr << "ContactPlugin requires a ContactSensor.\n";
//         return;
//       }
//       std::cout << "\nCONTACT PLUGIN OK......" << std::endl;
//       // Connect to the sensor update event.
//       this->updateConnection = this->parentSensor->ConnectUpdated(
//           std::bind(&ContactPlugin::OnUpdate, this));

//       // Make sure the parent sensor is active.
//       this->parentSensor->SetActive(true);
//     }

//   private:
//     void OnUpdate()
//     { // Get all the contacts.
//       // msgs::Contacts contacts;
//       // contacts = this->parentSensor->Contacts();
//       // for (unsigned int i = 0; i < contacts.contact_size(); ++i)
//       // {
//       //   std::cout << "Collision between[" << contacts.contact(i).collision1()
//       //             << "] and [" << contacts.contact(i).collision2() << "]\n";

//       //   for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
//       //   {
//       //     std::cout << j << "  Position:"
//       //               << contacts.contact(i).position(j).x() << " "
//       //               << contacts.contact(i).position(j).y() << " "
//       //               << contacts.contact(i).position(j).z() << "\n";
//       //     std::cout << "   Normal:"
//       //               << contacts.contact(i).normal(j).x() << " "
//       //               << contacts.contact(i).normal(j).y() << " "
//       //               << contacts.contact(i).normal(j).z() << "\n";
//       //     std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
//       //   }
//       // }
//     }
//     sensors::ContactSensorPtr parentSensor;
//     event::ConnectionPtr updateConnection;
//   };
//   // GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)
// }

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->sdf = _sdf;
      this->model = _parent;
      this->world = this->model->GetWorld();

      
      // std::cout << model->WorldPose().Pos() << std::endl;
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this, std::placeholders::_1)));

      Reset();

      if (_sdf->HasElement("target_weight"))
        this->targetWeight = _sdf->Get<double>("target_weight");
      else
        this->targetWeight = 1.15;

      // Read in the obstacle weight
      if (_sdf->HasElement("obstacle_weight"))
        this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
      else
        this->obstacleWeight = 1.5;

      // Add our own name to models we should ignore when avoiding obstacles.
      this->ignoreModels.push_back(this->model->GetName());

      // Read in the other obstacles to ignore
      if (_sdf->HasElement("ignore_obstacles"))
      {
        sdf::ElementPtr modelElem =
            _sdf->GetElement("ignore_obstacles")->GetElement("model");
        while (modelElem)
        {
          this->ignoreModels.push_back(modelElem->Get<std::string>());
          modelElem = modelElem->GetNextElement("model");
        }
      }

      std::cout << this->targetWeight << " " << this->obstacleWeight << " " << std::endl;

    }

    /////////////////////////////////////////////////
    void Reset()
    {
      this->velocity = 0.8;
      this->lastUpdate = 0;

      if (this->sdf && this->sdf->HasElement("target"))
        this->target = this->sdf->Get<ignition::math::Vector3d>("target");
      else
        this->target = ignition::math::Vector3d(3, 2, 0);
    }

    void ChooseNewTarget()
    {
      ignition::math::Vector3d newTarget(this->target);
      while ((newTarget - this->target).Length() < 2.0)
      {
        newTarget.X(ignition::math::Rand::DblUniform(-10, 10));
        newTarget.Y(ignition::math::Rand::DblUniform(-10, 10));

        for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
        {
          double dist = (this->world->ModelByIndex(i)->WorldPose().Pos() - newTarget).Length();
          if (dist < 2.0)
          {
            newTarget = this->target;
            break;
          }
        }
      }
      this->target = newTarget;
    }

    void HandleObstacles(ignition::math::Vector3d &_pos)
    {
      for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
      {
        physics::ModelPtr model = this->world->ModelByIndex(i);
        if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                      model->GetName()) == this->ignoreModels.end())
        {
          ignition::math::Vector3d offset = model->WorldPose().Pos() -
                                            this->model->WorldPose().Pos();
          double modelDist = offset.Length();
          if (modelDist < 4.0)
          {
            double invModelDist = this->obstacleWeight / modelDist;
            offset.Normalize();
            offset *= invModelDist;
            _pos -= offset;
          }
        }
      }
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo &_info)
    {

      // Time delta
      double dt = (_info.simTime - this->lastUpdate).Double();
  
      ignition::math::Pose3d pose = this->model->WorldPose();
      ignition::math::Vector3d pos = this->target - pose.Pos();
      ignition::math::Vector3d rpy = pose.Rot().Euler();

      double distance = pos.Length();

      // Choose a new target position if the model has reached its current
      // target.
      if (distance < 0.3)
      {
        this->ChooseNewTarget();
        pos = this->target - pose.Pos();
      }

      // Normalize the direction vector, and apply the target weight
      pos = pos.Normalize() * this->targetWeight;

      // Adjust the direction vector by avoiding obstacles
      this->HandleObstacles(pos);

      // Compute the yaw orientation
      ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
      yaw.Normalize();

      // Rotate in place, instead of jumping.
      if (std::abs(yaw.Radian()) > IGN_DTOR(10))
      {
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian() * 0.001);
      }
      else
      {
        pose.Pos() += pos * this->velocity * dt;
        // this->model->SetLinearVel(pose.Pos());
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian());
        // this->model->SetAngularVel(ignition::math::Vector3d(rpy.Z() + yaw.Radian(), 0, rpy.Z() + yaw.Radian()));
      }

      // Make sure the model stays within bounds
      pose.Pos().X(std::max(-10.0, std::min(10.0, pose.Pos().X())));
      pose.Pos().Y(std::max(-10.0, std::min(10.0, pose.Pos().Y())));
      pose.Pos().Z(0);

      // Distance traveled is used to coordinate motion with the walking
      // animation
      // double distanceTraveled = (pose.Pos() - this->model->WorldPose().Pos()).Length();

      this->model->SetWorldPose(pose);

      this->lastUpdate = _info.simTime;
    }

    // Pointer to the model
  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    sdf::ElementPtr sdf;
    float velocity;
    common::Time lastUpdate;
    double targetWeight;
    double obstacleWeight;
    // Pointer to the update event connection
    std::vector<event::ConnectionPtr> updateConnection;
    std::vector<std::string> ignoreModels;
    ignition::math::Vector3d target;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}