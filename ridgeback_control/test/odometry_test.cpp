#include <mecanum_drive_controller/odometry.h>
#include <mecanum_drive_controller/mecanum_drive_controller.h>
#include <gtest/gtest.h>

namespace mecanum_drive_controller {
namespace {


TEST(MecanumOdometryTest, normalKinematicsX)
{
  // Uses the same geometry as the IK tests in the controller.
  double r = 0.1;
  double a = 1;
  double b = 0.5;
  Odometry::BodyVelocities velocities;

  velocities = Odometry::calculateKinematicsNormal(10, 10, 10, 10, r, a, b);
  EXPECT_FLOAT_EQ(1.0, velocities.linearX);
  EXPECT_FLOAT_EQ(0.0, velocities.linearY);
  EXPECT_FLOAT_EQ(0.0, velocities.angular);

  velocities = Odometry::calculateKinematicsNormal(-10, -10, -10, -10, r, a, b);
  EXPECT_FLOAT_EQ(-1.0, velocities.linearX);
  EXPECT_FLOAT_EQ(0.0, velocities.linearY);
  EXPECT_FLOAT_EQ(0.0, velocities.angular);
}

TEST(MecanumOdometryTest, normalKinematicsY)
{
    // Uses the same geometry as the IK tests in the controller.
    double r = 0.1;
    double a = 1;
    double b = 0.5;
    Odometry::BodyVelocities velocities;

    velocities = Odometry::calculateKinematicsNormal(-10, 10, -10, 10, r, a, b);
    EXPECT_FLOAT_EQ(0.0, velocities.linearX);
    EXPECT_FLOAT_EQ(1.0, velocities.linearY);
    EXPECT_FLOAT_EQ(0.0, velocities.angular);

    velocities = Odometry::calculateKinematicsNormal(10, -10, 10, -10, r, a, b);
    EXPECT_FLOAT_EQ(0.0, velocities.linearX);
    EXPECT_FLOAT_EQ(-1.0, velocities.linearY);
    EXPECT_FLOAT_EQ(0.0, velocities.angular);
}

TEST(MecanumOdometryTest, normalKinematicsAngular)
{
  // Uses the same geometry as the IK tests in the controller.
  double r = 0.1;
  double a = 1;
  double b = 0.5;
  Odometry::BodyVelocities velocities;

  velocities = Odometry::calculateKinematicsNormal(-15, -15, 15, 15, r, a, b);
  EXPECT_FLOAT_EQ(0.0, velocities.linearX);
  EXPECT_FLOAT_EQ(0.0, velocities.linearY);
  EXPECT_FLOAT_EQ(1.0, velocities.angular);

  velocities = Odometry::calculateKinematicsNormal(15, 15, -15, -15, r, a, b);
  EXPECT_FLOAT_EQ(0.0, velocities.linearX);
  EXPECT_FLOAT_EQ(0.0, velocities.linearY);
  EXPECT_FLOAT_EQ(-1.0, velocities.angular);
}

TEST(MecanumOdometryTest, flippedKinematicsRoundTrip)
{
  // Uses the same geometry as the IK tests in the controller.
  double r = 0.1;
  double a = 0.5;
  double b = 1;

  Odometry::BodyVelocities body_velocities;
  MecanumDriveController::WheelVelocities wheel_velocities;
  wheel_velocities = MecanumDriveController::calculateIkFlipped(1, 0, 0, r, a, b);
  body_velocities = Odometry::calculateKinematicsFlipped(wheel_velocities.w0_vel,
                                                         wheel_velocities.w1_vel,
                                                         wheel_velocities.w2_vel,
                                                         wheel_velocities.w3_vel,
                                                         r, a, b);
  EXPECT_FLOAT_EQ(1.0, body_velocities.linearX);
  EXPECT_FLOAT_EQ(0.0, body_velocities.linearY);
  EXPECT_FLOAT_EQ(0.0, body_velocities.angular);

  wheel_velocities = MecanumDriveController::calculateIkFlipped(-1, 1, 0.1, r, a, b);
  body_velocities = Odometry::calculateKinematicsFlipped(wheel_velocities.w0_vel,
                                                         wheel_velocities.w1_vel,
                                                         wheel_velocities.w2_vel,
                                                         wheel_velocities.w3_vel,
                                                         r, a, b);
  EXPECT_FLOAT_EQ(-1, body_velocities.linearX);
  EXPECT_FLOAT_EQ(1, body_velocities.linearY);
  EXPECT_FLOAT_EQ(0.1, body_velocities.angular);
}

TEST(MecanumOdometryTest, flippedKinematicsX)
{
  // Uses the same geometry as the IK tests in the controller.
  double r = 0.1;
  double a = 0.5;
  double b = 1;
  Odometry::BodyVelocities velocities;

  velocities = Odometry::calculateKinematicsFlipped(10, 10, -10, -10, r, a, b);
  EXPECT_FLOAT_EQ(1.0, velocities.linearX);
  EXPECT_FLOAT_EQ(0.0, velocities.linearY);
  EXPECT_FLOAT_EQ(0.0, velocities.angular);

  velocities = Odometry::calculateKinematicsFlipped(-10, -10, 10, 10, r, a, b);
  EXPECT_FLOAT_EQ(-1.0, velocities.linearX);
  EXPECT_FLOAT_EQ(0.0, velocities.linearY);
  EXPECT_FLOAT_EQ(0.0, velocities.angular);
}

TEST(MecanumOdometryTest, flippedKinematicsY)
{
  // Uses the same geometry as the IK tests in the controller.
  double r = 0.1;
  double a = 0.5;
  double b = 1;
  Odometry::BodyVelocities velocities;

  velocities = Odometry::calculateKinematicsFlipped(-10, 10, 10, -10, r, a, b);
  EXPECT_FLOAT_EQ(0.0, velocities.linearX);
  EXPECT_FLOAT_EQ(1.0, velocities.linearY);
  EXPECT_FLOAT_EQ(0.0, velocities.angular);

  velocities = Odometry::calculateKinematicsFlipped(10, -10, -10, 10, r, a, b);
  EXPECT_FLOAT_EQ(0.0, velocities.linearX);
  EXPECT_FLOAT_EQ(-1.0, velocities.linearY);
  EXPECT_FLOAT_EQ(0.0, velocities.angular);
}

TEST(MecanumOdometryTest, flippedKinematicsAngular)
{
  // Uses the same geometry as the IK tests in the controller.
  double r = 0.1;
  double a = 0.5;
  double b = 1;
  Odometry::BodyVelocities velocities;

  velocities = Odometry::calculateKinematicsFlipped(-5, -5, -5, -5, r, a, b);
  EXPECT_FLOAT_EQ(0.0, velocities.linearX);
  EXPECT_FLOAT_EQ(0.0, velocities.linearY);
  EXPECT_FLOAT_EQ(1.0, velocities.angular);

  velocities = Odometry::calculateKinematicsFlipped(5, 5, 5, 5, r, a, b);
  EXPECT_FLOAT_EQ(0.0, velocities.linearX);
  EXPECT_FLOAT_EQ(0.0, velocities.linearY);
  EXPECT_FLOAT_EQ(-1.0, velocities.angular);
}

} // anom namespace
} // mecanum_drive_controller namespace

