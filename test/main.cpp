#include <gtest/gtest.h>
//#include <ros/ros.h>
#include <Eigen/Dense>
#include "frankpiv_controller/pivot_controller.h"


//TODO: Test static functions
// calcPivotOrientation, getRoll, getPivotError
TEST(TestStaticFunctionsSuite, testCalcPivotOrientation) {
  Eigen::Vector3d pivot_point {0,0,0};
  Eigen::Vector4d tipPose {0,0,1, 0};
  Eigen::Quaterniond result =
      frankpiv_controller::PivotController::calcPivotOrientation(
          pivot_point, tipPose);
  Eigen::Quaterniond expected {0.707,0,0,0.707};
  auto diff = result.angularDistance(expected);
  EXPECT_LT(diff, 0.01);
}

//TODO: Test member function compute_error
class TestMemberFunctionsSuite : public ::testing::Test {
public:
  TestMemberFunctionsSuite() {

  }
  ~TestMemberFunctionsSuite() {

  }
};

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "frankpiv_controller_test_node");
  auto res = RUN_ALL_TESTS();
//  ros::shutdown();
  return res;
}
