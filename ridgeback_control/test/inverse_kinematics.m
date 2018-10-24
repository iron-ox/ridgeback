# Quick and dirty test of the inverse kinematics presented on the paper:
# Modeling and Adaptive Control of an Omni-Mecanum-Wheeled Robot
# Lih-Chang Lin, Hao-Yin Shih
# http://dx.doi.org/10.4236/ica.2013.42021

R = 0.1;
b = 1;  # 1m from robot center to wheel center along Y axis
a = 0.5; # 0.5cm from robot center to wheel center along X axis
theta = pi/2;

l = sqrt(a*a + b*b);
wk = l*sin(pi/4-atan2(b, a));
k = sqrt(2)/2;

M = [k k wk; k -k wk; -k -k wk; -k k wk];
B = [cos(theta) sin(theta) 0; - sin(theta) cos(theta) 0; 0 0 1];

t = [1;0;0];
v = -(sqrt(2)/R) * M * B * t;

# To verify that B con be replaced by a velocity swap.
t = [0;-1;0];
v = -(sqrt(2)/R) * M * t;

t = [0;1;0];
v = -(sqrt(2)/R) * M * B * t;

t = [0;0;1];
v = -(sqrt(2)/R) * M * B * t;

t = [0;0;-1];
v = -(sqrt(2)/R) * M * B * t;

# When theta = pi/2, B, rotates the velocities so that Vx -> Vy and Vy -> -Vx
#      [0 1 0; * [Vx;  = [ Vy;
#      -1 0 0;    Vy;     -Vx;
#       0 0 1]    R]       R ]

