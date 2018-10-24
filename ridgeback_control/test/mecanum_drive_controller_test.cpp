#include <mecanum_drive_controller/mecanum_drive_controller.h>
#include <gtest/gtest.h>

namespace mecanum_drive_controller {
namespace {

// Normal configuration: wheel main rotation axis is aligned with the robot's y axis
// Flipped configuration: wheel main rotation axis is aligned with the robot's x axis

TEST(MecanumDriveControllerTest, calculateInverseKinematicNormalConfiguration)
{
  // IK equation used:
  // v_0 = 1/R * V_x - V_y - w*(wheel_k)
  // v_1 = 1/R * V_x + V_y - w*(wheel_k)
  // v_2 = 1/R * V_x - V_y + w*(wheel_k)
  // v_3 = 1/R * V_x + V_y + w*(wheel_k)

  // The robot design is:

  //             ^ (y)
  //      //     +     \\
  //      -------|-------
  //      |      b      |
  //      |      |      |
  //      |      o---a--+---> (x)
  //      |             |
  //      |             |
  //      ---------------
  //      \\           //
  //
  // where:
  //   the // and \\ outside of the robot body mark the rollers orientation.
  //   a = distance from robot center (o) to wheel center along Y axis.
  //   b = distance from robot center (o) to wheel center along X axis.
  //   r = main wheel radius.

  // In our example:
  double a = 1;
  double b = 0.5;
  double r = 0.1;

  // Positive in x
  MecanumDriveController::WheelVelocities velocities;

  velocities = MecanumDriveController::calculateIkNormal(1, 0, 0, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, 10);
  // Negative in x
  velocities = MecanumDriveController::calculateIkNormal(-1, 0, 0, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, -10);

  // Positive in y
  velocities = MecanumDriveController::calculateIkNormal(0, 1, 0, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, 10);
  // Negative in y
  velocities = MecanumDriveController::calculateIkNormal(0, -1, 0, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, -10);

  // Positive rotation
  velocities = MecanumDriveController::calculateIkNormal(0, 0, 1, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, -15);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, -15);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, 15);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, 15);
  // Negative rotation
  velocities = MecanumDriveController::calculateIkNormal(0, 0, -1, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, 15);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, 15);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, -15);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, -15);
}

TEST(MecanumDriveControllerTest, calculateInverseKinematicFlippedConfigurationX)
{
  // The robot design is:

  //                   ^ (x)
  //          \\       |        //
  //          ---------+---------
  //          |        |        |
  //          |        a        |
  //          |        |        |
  //   (y) <--+--b-----o        |
  //          |                 |
  //          |                 |
  //          |                 |
  //          -------------------
  //          //               \\

  // where:
  //   the // and \\ outside of the robot body mark the rollers orientation.
  //   a = distance from robot center (o) to wheel center along Y axis.
  //   b = distance from robot center (o) to wheel center along X axis.
  //   r = main wheel radius.

  // In our example:
  double a = 0.5;
  double b = 1;
  double r = 0.1;

  // Positive in x (Forward)
  MecanumDriveController::WheelVelocities velocities;
  velocities = MecanumDriveController::calculateIkFlipped(1, 0, 0, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, -10);

  // Negative in x (back)
  velocities = MecanumDriveController::calculateIkFlipped(-1, 0, 0, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, 10);
}

TEST(MecanumDriveControllerTest, calculateInverseKinematicFlippedConfigurationY)
{
  double a = 0.5;
  double b = 1;
  double r = 0.1;
  MecanumDriveController::WheelVelocities velocities;

  // Positive in y (right)
  velocities = MecanumDriveController::calculateIkFlipped(0, 1, 0, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, -10);

  // Negative in y (left)
  velocities = MecanumDriveController::calculateIkFlipped(0, -1, 0, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, 10);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, -10);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, 10);
}

TEST(MecanumDriveControllerTest, calculateInverseKinematicFlippedConfigurationRotations)
{
  double a = 0.5;
  double b = 1;
  double r = 0.1;
  MecanumDriveController::WheelVelocities velocities;

  // Positive rotation (clockwise)
  velocities = MecanumDriveController::calculateIkFlipped(0, 0, 1, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, -5);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, -5);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, -5);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, -5);

  // Negative rotation (counter closwise)
  velocities = MecanumDriveController::calculateIkFlipped(0, 0, -1, r, a, b);
  EXPECT_DOUBLE_EQ(velocities.w0_vel, 5);
  EXPECT_DOUBLE_EQ(velocities.w1_vel, 5);
  EXPECT_DOUBLE_EQ(velocities.w2_vel, 5);
  EXPECT_DOUBLE_EQ(velocities.w3_vel, 5);
}

}
}
