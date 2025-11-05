#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <memory>
#include "motion_editor/motion_editor.hpp"

using namespace ROBIT_HUMANOID_MOTION_EDITOR;

int main()
{
  try {
    // 모션 YAML 경로 불러오기
    const std::string share = ament_index_cpp::get_package_share_directory("motion_editor");
    const std::string yaml_path = share + "/motion/test_motion.yaml";     // ROS2의 패키지 공유 디렉토리에서 파일 R/W을 관리하는 것이 권한 문제에 안전함 해당 방법을 권장함

    // MotionEditor 생성
    auto me = std::make_shared<MotionEditor>();

    // YAML 로드
    me->loadFromFile(yaml_path);

    // 프레임 이름 출력
    std::cout << "[test] Step Names:\n";
    auto stepNames = me->listStepNames();
    for (auto& n : stepNames) {
      std::cout << n << "\n";
    }

    // 첫번째 프레임 정보 보기
    if (!stepNames.empty()) {
      auto f = me->getFrame(stepNames.front());
      if (f) {
        me->printFrame(*f);
      }
    }

    // 특정 프레임 임의 관절 수정
    {
      JointPosMap q2;
      q2["rotate_1"] = 0.33;
      q2["rotate_7"] = -0.11;
      me->editJoints("2", q2);
    }

    // YAML 저장 >> 공유 디렉토리에서 덮어쓰기 지원함
    me->saveToFile(yaml_path);
    std::cout << "done\n";
  }
  catch (const std::exception& e) {
    std::cerr << "ERR: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
