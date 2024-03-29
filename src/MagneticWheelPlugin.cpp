#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class MagneticWheelPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      this->model = _model;

      // 원기둥 모델 찾기
      this->cylinder = this->model->GetWorld()->ModelByName("tank_structure");
      // box model 찾기
      this->box = this->model->GetWorld()->ModelByName("steel_wall");
      

      if (!this->cylinder)
      {
        gzerr << "Could not find a model named 'tank structure' in the simulation." << std::endl;
        return;
      }
      else
      {
        gzmsg << "SUCCESS Magnet Plugin! Found a model named 'tank structure' in the simulation." << std::endl;
      }

        if (!this->box)
        {
            gzerr << "Could not find a model named 'steel wall' in the simulation." << std::endl;
            return;
        }
        else
        {
            gzmsg << "SUCCESS Magnet Plugin! Found a model named 'steel wall' in the simulation." << std::endl;
        }

      // Update 이벤트에 OnUpdate 함수 연결
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MagneticWheelPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // 원기둥의 현재 위치 가져오기
        auto cylinderPos = this->cylinder->WorldPose().Pos();
        auto boxPos = this->box->WorldPose().Pos();

      // 각 바퀴에 대해 반복
      for (auto wheelName : {"front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"})
      {
        auto wheel = this->model->GetLink(wheelName);
        if (!wheel)
          continue;

        auto wheelPos = wheel->WorldPose().Pos();
        auto distance_cylinder = wheelPos.Distance(cylinderPos);
        auto distance_box = wheelPos.Distance(boxPos);

        // print distance_cylinder
        //std::cout << "Distance_cylinder: " << distance_cylinder<< std::endl;
        //std::cout << "Distance_box     : " << distance_box << std::endl;

        // 바퀴가 원기둥에 가까이 있을 경우 부착 효과 적용
        if (distance_cylinder < 83.9 )  // 부착 거리 임계값
        {
          // 원기둥 중심으로부터 바퀴까지의 방향
            auto direction = (cylinderPos - wheelPos).Normalize();

            //std::cout << "Direction: " << direction << std::endl;

          // 바퀴에 적용할 부착 효과
            double forceMagnitude = 550.0;  // 부착 힘 크기, 필요에 따라 조정 unit : N
            wheel->AddForce(direction * forceMagnitude); // 부착 힘 적용 (방향 * 크기) 

        }
        if (distance_box < 0.50 )  // 부착 거리 임계값
        {
            // 원기둥 중심으로부터 바퀴까지의 방향
            auto direction = (boxPos - wheelPos).Normalize();

            //std::cout << "Direction: " << direction << std::endl;

            // 바퀴에 적용할 부착 효과
            double forceMagnitude = 300.0;  // 부착 힘 크기, 필요에 따라 조정 unit : N
            wheel->AddForce(direction * forceMagnitude); // 부착 힘 적용 (방향 * 크기) 
            
        }

      }
    }

    private: physics::ModelPtr model;
    private: physics::ModelPtr cylinder;
    private: physics::ModelPtr box;
    private: event::ConnectionPtr updateConnection;
  };

  // 플러그인 등록
  GZ_REGISTER_MODEL_PLUGIN(MagneticWheelPlugin)
}
