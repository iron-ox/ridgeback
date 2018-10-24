#include <mecanum_drive_controller/speed_limiter.h>
#include <gtest/gtest.h>

namespace mecanum_drive_controller {
namespace {

TEST(SpeedLimiterTest, noLimitsEnabledDoesNotClamp)
{
  // By default, no limits are set.
  SpeedLimiter speed_limiter;
  double fast_velocity = 100.0;
  double timestep = 20;
  double initial_velocity = fast_velocity;
  double current_velocity = fast_velocity + 10;
  speed_limiter.limit_velocity(initial_velocity);
  EXPECT_DOUBLE_EQ(fast_velocity, initial_velocity);
  speed_limiter.limit_acceleration(current_velocity, initial_velocity, timestep);
  EXPECT_DOUBLE_EQ(fast_velocity+10, current_velocity);
}

TEST(SpeedLimiterTest, velocityShouldBeLimited)
{
  // Min vel 1.0, max vel 10.0 no acceleration limit.
  double max_velocity = 10.0;
  double min_velocity = 1.0;
  SpeedLimiter speed_limiter(true, false, min_velocity, max_velocity, 0.0, 0.0);
  double velocity = 100.0;
  speed_limiter.limit_velocity(velocity);
  EXPECT_DOUBLE_EQ(max_velocity, velocity);

  velocity = 0.1;
  speed_limiter.limit_velocity(velocity);
  EXPECT_DOUBLE_EQ(min_velocity, velocity);

  velocity = 5.0;
  speed_limiter.limit_velocity(velocity);
  EXPECT_DOUBLE_EQ(5.0, velocity);
}

TEST(SpeedLimiterTest, accelerationShouldBeLimited)
{
  // Min vel 1.0, max vel 10.0 no acceleration limit.
  double max_acceleration = 10.0;
  double min_acceleration = 1.0;
  SpeedLimiter speed_limiter(false, true, 0.0, 0.0, min_acceleration, max_acceleration);
  double initial_velocity = 1.0;
  double current_velocity = 20.0; // more than the max acceleration.
  double time_step = 1.0;
  speed_limiter.limit_acceleration(current_velocity, initial_velocity, time_step);
  EXPECT_DOUBLE_EQ(initial_velocity+max_acceleration*time_step, current_velocity);

  current_velocity = 1.0; // less than min acceleration.
  speed_limiter.limit_acceleration(current_velocity, initial_velocity, time_step);
  EXPECT_DOUBLE_EQ(initial_velocity + min_acceleration * time_step, current_velocity);

  current_velocity = 5.0; // should be within acceptable limits
  speed_limiter.limit_acceleration(current_velocity, initial_velocity, time_step);
  EXPECT_DOUBLE_EQ(5, current_velocity);
}

TEST(SpeedLimiterTest, bothShouldBeLimited)
{
  double max_velocity = 20.0;
  double min_velocity = 1.0;
  double max_acceleration = 10.0;
  double min_acceleration = 1.0;

  SpeedLimiter speed_limiter(true, true, min_velocity, max_velocity,
                             min_acceleration, max_acceleration);

  double initial_velocity = 1.0;
  double timestep = 1.0;
  double current_velocity = 30;
  speed_limiter.limit(current_velocity, initial_velocity, timestep);
  // 30 will be limited to 20, then max acceleration will limit dv to 10
  EXPECT_DOUBLE_EQ(max_acceleration*timestep + initial_velocity, current_velocity);
}

}
}
