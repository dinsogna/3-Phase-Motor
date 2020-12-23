#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <math.h>

namespace gazebo
{
  class TestPendulum : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      // Sets the joint pointer
      this->R1 = this->model->GetJoint("J01");
      this->R1->SetProvideFeedback(true);
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TestPendulum::OnUpdate, this));
      //this->model->SetGravityMode(false);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, .3, 0));
        count=count +.001;
        this->R1->SetPosition(0,M_PI/4);
        std::cout<< this->R1->GetForceTorque(0).body1Torque << std::endl;
        this->R1->GetVelocity(0);
    }

    // Pointer to the model
    private:
        physics::ModelPtr model;
        // Pointer to the joint
        physics::JointPtr R1;
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        double count=0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TestPendulum)
}