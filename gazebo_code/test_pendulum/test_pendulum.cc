#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Vector3.hh>
#include <eigen3/Eigen/Dense>
#include <boost/bind.hpp>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <gazebo/util/system.hh>





namespace gazebo
{
  class TestPendulum : public ModelPlugin
  {
    const size_t N = 2;
    typedef Eigen::VectorXd state_type;

    //maybe set initial conditions here instead of in Load()??? call in load()
    private: void ModelInit()
    {

    }

    private:  state_type UpdateMotor(const state_type X, const double tor) {
      if(Vm>Vmax)
        Vm=Vmax;
      else if(Vm<0-Vmax)
        Vm=0-Vmax;

      state_type state(N);

      Eigen::MatrixXd A(2,2);
      A << (-B/J), (K/J), (-K/L), (-R/L);
      
      Eigen::MatrixXd B(2,2);
      B << (-1/J), 0, 0, (1/L);
      
      Eigen::MatrixXd C(2,1);
      C << tor, Vm;

      state = A*X + B*C;
      
      return state;
    }

    private: state_type RK4_Step(state_type state, double dt, double &tor) 
    {

      tor/=gear_ratio;
      state(0)*=gear_ratio;

      double h = dt;
      double h2 = 0.5*h;
      double h6 = h/6.0;

      state_type k1 = UpdateMotor(state, tor);
      state_type k2 = UpdateMotor(state + h2*k1, tor);
      state_type k3 = UpdateMotor(state + h2*k1, tor);
      state_type k4 = UpdateMotor(state + h*k3, tor);
    
      double a=(k1(0) + (2.0*(k2(0) + k3(0))) + k4(0))/(6*gear_ratio);
    
      state_type newState = state+(h6*(k1 + (2.0*(k2 + k3)) + k4));
      tor= ((-J*a) - (B*newState(0)) + (K*newState(1)))*gear_ratio;
      newState(0)/= gear_ratio;
      relative_theta += (newState(0)*dt);
      return newState;
  }



    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      //========================
      //INITIAL CONDITIONS
      //========================
      this->theta=.3;      //initial theta (radians)
      this->theta_dot=0;  //initial theta dot (radians/s)
      this->iq=0;         //initial iq (amps)
      this->voltage=.07801;  //initial voltage to motor Vm (volts)
      this->torque=0;     //initial external torque (N*m)
      // double time=30;      //total time interval (seconds)
      // double dt=0.0001;       //size of one time step (No longer need!)
      ///////////////////////////

      //========================
      //INITIALIZE MOTOR
      //========================
      this->Vmax= 30;
      this->R= 0.16;
      this->L= 0.00018;
      this->K= 0.088;
      this->B= .001;
      this->J= 0.0001;
      this->Vm= voltage;
      this->relative_theta=0;
      this->gear_ratio = 10;

      //========================
      //INITIALIZE PENDULUM (***use sdf file***)
      //========================
      this->g=9.81;
      this->l= 0.37;
      this->b= 0.07;
      this->m= 0.4;


      // Store the pointer to the model
      this->model = _parent;
      // Sets the joint pointer
      this->R1 = this->model->GetJoint("J01");
      // get the link pointer for L1 (could have a problem if multiple children for now gets the fist child?)
      this->L1=this->R1->GetChild();
      this->L0=this->R1->GetParent();
      //this->R1->SetProvideFeedback(true);
      
      //simTime starts at 0
      this->lastUpdate = 0;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TestPendulum::OnUpdate, this, std::placeholders::_1));
      //this->model->SetGravityMode(false);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & _info)
    {
      // Calculate the dt at each step
      double dt = (_info.simTime - this->lastUpdate).Double();
      this->lastUpdate = _info.simTime;
      // std::cout << "dt: " << dt <<"; lastUpdate: " << this->lastUpdate<< "; simtime: " << _info.simTime << std::endl; //test 

      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, .3, 0));
        count=count +.001;

        //this->R1->SetPosition(0,M_PI/4);
        //std::cout<< this->R1->GetForceTorque(0).body1Torque << std::endl; // WRONG
        ignition::math::Vector3d t1(100,0,0);
        // this->L1->AddTorque(t1);
        // std::cout<< this->L1->RelativeAngularAccel()<< std::endl;
        // std::cout<< this->L1->RelativeAngularVel()<< std::endl;
        // std::cout<< this->L1->RelativeTorque()<< std::endl;
        // std::cout<< "" << std::endl;
        //this->R1->GetVelocity(0);

        //get initial pendulum state
        // state_type pend1(N) << this

    }

    // Pointer to the model
    private:
        //pointer to model
        physics::ModelPtr model;
        // Pointer to the joint
        physics::JointPtr R1;
        //pointer to link 0 (base)
        physics::LinkPtr L0;
        //pointer to link 1
        physics::LinkPtr L1;
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        //SYSTEM CONSTANTS
        double count=0;
        common::Time lastUpdate;
        std::vector<state_type> values;
        std::vector<double> T; // for motor only testing

        double theta;      //initial theta (radians)
        double theta_dot;  //initial theta dot (radians/s)
        double iq;         //initial iq (amps)
        double voltage;  //initial voltage to motor Vm (volts)
        double torque; 

        //MOTOR CONSTANTS
        double Vmax;
        double R;
        double L;
        double K;
        double B;
        double J;
        double Vm;
        double Iq;
        double gear_ratio; 
        double relative_theta;

        //PENDULUM CONSTANTS (set in .world file; can access through sdf parameter in Load())
        double g;
        double l;
        double b;
        double m;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TestPendulum)
}
