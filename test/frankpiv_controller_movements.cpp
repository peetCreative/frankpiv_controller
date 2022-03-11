#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(TestSuite , frankpiv_controller_movements_follows_and_pivot)
{
  ros::NodeHandle n;

  // TODO: write code to read whether the simulated

}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "frankpiv_controller_movements");
  return RUN_ALL_TESTS();
}