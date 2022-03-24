#include <gtest/gtest.h>
//#include <ros/ros.h>
#include <tuple>
#include <Eigen/Dense>
#include "frankpiv_controller/pivot_controller.h"

#define ROTATION_EPSILON 0.01
typedef std::array<double, 9> Array9d;
typedef std::tuple<Array9d, double> GetRollTestType;

//TODO: Test static functions
// getPivotError
TEST(TestStaticFunctionsSuite, calcPivotOrientationTest) {
  Eigen::Vector3d pivot_point {0,0,0};
  Eigen::Vector4d tipPose {0,0,1, 0};
  Eigen::Quaterniond result =
      frankpiv_controller::PivotController::calcPivotOrientation(
          pivot_point, tipPose);
  Eigen::Quaterniond expected {0.707,0,0,0.707};
  auto diff = result.angularDistance(expected);
  EXPECT_LT(diff, ROTATION_EPSILON);
}

class TestStaticFunctionsSuiteOrientation : public ::testing::TestWithParam<GetRollTestType> {};

TEST_P(TestStaticFunctionsSuiteOrientation, getRollTest) {
  Array9d arr = std::get<0>(GetParam());
  Eigen::Map<Eigen::Matrix3d> mat(arr.data());
  double result =
      frankpiv_controller::PivotController::getRoll(mat);
  EXPECT_LT(std::abs(result - std::get<1>(GetParam())) , ROTATION_EPSILON);
}

INSTANTIATE_TEST_SUITE_P(
      getRollTestInstantiation, TestStaticFunctionsSuiteOrientation,
      ::testing::Values(
          std::make_tuple(Array9d({
            1,0,0,
            0,1,0,
            0,0,1}), 0),
          std::make_tuple(Array9d({
            1,0,0,
            0,0,1,
            0,-1,0}), 0)
          )
    );



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
