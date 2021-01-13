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
#include <unsupported/Eigen/MatrixFunctions>



// export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/Desktop/Gazebo-Projects/gazebo_tutorials/gazebo_plugin_tutorial/build


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

    private: double UpdateMotor() {
      if(v_m>v_max)
        v_m=v_max;
      else if(v_m<0-v_max)
        v_m=0-v_max;
      
      state_type X(N);
      X = state;
      state(0) = (this->L1->RelativeAngularVel().X()*gear_ratio);
      
      state_type U(N);
      U <<this->L1->RelativeTorque().X()/gear_ratio, v_m;

      state = this->G*X + this->H*U;

      double torque = (C.transpose() * state)(0); //Test correct size matrix
      return torque *= gear_ratio;
        
    }

    private: void convertToDiscrete() {
      // G = e^AT (Matrix exponential)
      std::cout<<"CTD"<<std::endl;
      this->G = (A*T).exp();
      std::cout<<"G:"<<G<<std::endl;
      // std::cout<<(G - I)*(B*(A.inverse()))<<std::endl;
      // H = (G-I_2x2)*B(A^-1)
     
      this->H = (G - I)*(B*(A.inverse()));
      std::cout<<"H: "<<H<<std::endl;
      //G and H are now global vars

      // std::cout<"G: " <<G<<std::endl;
      // std::cout<"H: " <<H<<std::endl;

    }



    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      //TEST
      std::cout << "Load() 1" << std::endl;
      

      //========================
      //INITIAL CONDITIONS
      //========================
      this->theta=M_PI;      //initial theta (radians)
      this->theta_dot=0.0;  //initial theta dot (radians/s)
      this->iq=0;         //initial iq (amps)
      this->voltage=0.009;  //initial voltage to motor Vm (volts)
      this->torque=0;     //initial external torque (N*m)
      // double time=30;      //total time interval (seconds)
      // double dt=0.0001;       //size of one time step (No longer need!)

      std::cout <<"Initial Conditions: "<<theta <<";"<<iq<<";"<<voltage<<";"<<torque<<std::endl;
      ///////////////////////////

      //========================
      //INITIALIZE MOTOR
      //========================
      this->v_max= 30;
      this->r= 0.16; //resistance
      this->l= 0.00018; //inductance
      this->k= 0.088; //motor constant
      this->b= .001;  //damping
      this->j= 0.0001;  //inertia of motor
      this->v_m= voltage;
      this->gear_ratio = 10;
      std::cout<<"Motor Init: "<<v_max<<";"<<r<<";"<<l<<";"<<k<<";"<<b<<";"<<j<<";"<<v_m<<";"<<gear_ratio<<std::endl;

      //========================
      //INITIALIZE PENDULUM (***use sdf file***)
      //========================
      // this->g=9.81;
      // this->l= 0.2;
      // this->b= 0.00;
      // this->m= 1.0;

      //TEST
      std::cout << "N: " <<N<<std::endl;
      std::cout << "Load() 2" << std::endl;
      // state(N);
      // state_type state2(2);
      std::cout<<"theta_dot: " <<theta_dot<<std::endl;
      std::cout<<"iq: "<<iq<<std::endl;
      // state2 << theta_dot,iq;
      
      state << theta_dot, iq;
      std::cout <<"state: " << state << std::endl;

      // std::cout <<"state: " << state2 << std::endl;

      // this->A(2,2);
      A << (-b/j), (k/j), (-k/l), (-r/l);

      std::cout <<"A: " << A << std::endl;
      
      
      // this->B(2,2);
      B << (-1/j), 0, 0, (1/l);
      
      std::cout <<"B: " << B << std::endl;

      // this->C(2,1);
      C << torque, v_m;

      std::cout <<"C: " << C << std::endl;

      // this->D(2,1);
      D << 0, 0;

      std::cout <<"D: " << D << std::endl;

      // this->I(2,2);
      I << 1,0,0,1;
      std::cout <<"I: " << I << std::endl;

      // this->G(2,2);
      // this->H(2,2);
      T = .001;
      convertToDiscrete();
      
      std::cout << "Load() 2a" << std::endl;
      // Store the pointer to the model
      this->model = _parent;
      // Sets the joint pointer
      this->R1 = this->model->GetJoint("J01");

      // get the link pointer for L1 (could have a problem if multiple children for now gets the fist child?)
      this->L1=this->R1->GetChild();
      this->L0=this->R1->GetParent();
      //set pendulum position via joint
      this->R1->SetPosition(0,theta);
      
      //simTime starts at 0
      this->lastUpdate = 0;

      //TEST
      std::cout << "Load() 3" << std::endl;
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TestPendulum::OnUpdate, this, std::placeholders::_1));

      //TEST
      std::cout << "Load() 4" << std::endl;
      //this->model->SetGravityMode(false);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & _info)
    {

      // std::cout << "OnUpdate()" << std::endl;
      
      // Calculate the dt at each step
      double dt = (_info.simTime - this->lastUpdate).Double();
      this->lastUpdate = _info.simTime;
      
      //first step or when T!=dt
      if (T != dt) {
        
        std::cout<<"if statement"<<std::endl;
        this->T = dt;
        convertToDiscrete();
        // std::cout<"G: " <<G<<std::endl;
        // std::cout<"H: " <<H<<std::endl;
      }

      
      // std::cout << "dt: " << dt <<"; lastUpdate: " << this->lastUpdate<< "; simtime: " << _info.simTime << std::endl; //test 
      
      // X = state;
      // state(0) = this->L1->RelativeAngularVel();
      // // U (input)
      // //Will need to consider gear ratio; torque / 10, vel * 10???
      // state_type U <<this->L1->RelativeTorque(), Vm; //2x1 vector

      // state_type state = A*X + B*U;

      // torque = C * state
      // torque *= 10;
      // this->L1->AddTorque(ignition::math::Vector3d (torque,0,0));

      this->L1->AddTorque(ignition::math::Vector3d (UpdateMotor(),0,0));

      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, .3, 0));
      // count=count +.001;

      //this->R1->SetPosition(0,M_PI/4);
      //std::cout<< this->R1->GetForceTorque(0).body1Torque << std::endl; // WRONG
      // ignition::math::Vector3d t1(100,0,0); // (torque,0,0)
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
        common::Time lastUpdate;
        double T=-1; //discrete time step

        //INITIAL CONDITIONS (originally main())
        double theta;      //initial theta (radians)
        double theta_dot;
        // state_type state;
        // Eigen::VectorXd state;  //initial motor state (theta dot, iq)
        Eigen::Vector2d state;
        double voltage;  //initial voltage to motor Vm (volts)
        double torque; 

        //MOTOR CONSTANTS
        double v_max;
        double r;
        double l;
        double k;
        double b; //damping
        double j;
        double v_m;
        double iq;
        double gear_ratio; 

        //Continuous time state space/ initialize in Load()
        // Eigen::Matrix2d A;
        // Eigen::MatrixXd B;
        // Eigen::MatrixXd C;
        // Eigen::MatrixXd D;
        // Eigen::MatrixXd G;
        // Eigen::MatrixXd H;
        // Eigen::MatrixXd I; //2x2 identity matrix

        Eigen::Matrix2d A;
        Eigen::Matrix2d B;
        Eigen::Vector2d C;
        Eigen::Vector2d D;
        Eigen::Matrix2d G;
        Eigen::Matrix2d H;
        Eigen::Matrix2d I;

        
        //PENDULUM CONSTANTS (set in .world file; can access through sdf parameter in Load())
        // double g;
        // double l;
        // double b;
        // double m;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TestPendulum)
}
