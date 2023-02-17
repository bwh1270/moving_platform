#ifndef MOVING_PLATFORM
#define MOVING_PLATFORM

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"


// Declaration 
namespace AIMS {
	class MovingPlatform {
		/*움직이는 플랫폼을 표현한다.

		  물체의 좌표는 드론의 이륙지점을 원점으로 하는 좌표계를 기준으로 한다.
		*/
		public:
			MovingPlatform();
			void start_position(double x, double y, double z);
			void uniform_motion(double x, double y, double z, int total_t);


		private:
			ros::NodeHandle _nh;
			double _x, _y, _z;
			ros::Publisher _vector_pub;

	};
}


// Definition 
AIMS::MovingPlatform::MovingPlatform():_x(0.0), _y(0.0), _z(0.0)
{
	_vector_pub = _nh.advertise<geometry_msgs::Vector3>("aims/trajectory", 1000);
}


void AIMS::MovingPlatform::start_position(double x, double y, double z)
{
	_x = x;
	_y = y;
	_z = z;
}


void AIMS::MovingPlatform::uniform_motion(double x, double y, double z, int total_t)
{
	/* input -> final position of trajectory
	 */

	double vel_x, vel_y, vel_z;
	double inverse_total_t;

	if (total_t != 0) {
		inverse_total_t = (double) 1 / total_t;
	}

	vel_x = (x - _x) * inverse_total_t;
	vel_y = (y - _y) * inverse_total_t;
	vel_z = (z - _z) * inverse_total_t;


	int hz = 10;    // 몇 hz로 moving platform의 위치를 알려줄 것인가
	double delta_t = (double) 1 / hz;	
	ros::Rate rate(hz);

	while (ros::ok())
	{

		geometry_msgs::Vector3 vector;
		vector.x = _x + (vel_x * delta_t);
		vector.y = _y + (vel_y * delta_t);
		vector.z = _z + (vel_z * delta_t);

		_vector_pub.publish(vector);
		delta_t += delta_t;

		if (delta_t == total_t) {
			ROS_INFO("Moving Platform is arrived at the final position of trajectory!!");
			ros::shutdown();
		}

		ros::spinOnce(); // -> callback function이 없으니 없어도 되는게 아닌가?
		rate.sleep();
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "moving_platform");
	AIMS::MovingPlatform car;

	car.start_position(0, 0, 0);
	car.uniform_motion(10, 0, 0, 10);
	
	return 0;
}



#endif
