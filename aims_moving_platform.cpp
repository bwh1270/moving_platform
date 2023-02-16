#ifndef MOVING_PLATFORM
#define MOVING_PLATFORM

#include <cmath>


// Declaration 
namespace AIMS {
	Class MovingPlatfrom {
		/*움직이는 플랫폼을 표현한다.

		  물체의 좌표는 드론의 이륙지점을 원점으로 하는 좌표계를 기준으로 한다.
		*/
		public:
			void uniform_motion(double vel_x, double vel_y, double vel_z);
			// void calculate_velocity(double x, double y, double z) // add arg of time);
			void position_to_angle(double x, double y, double z);




		private:
			double _x, _y, _z;
			double _vel_x, _vel_y, _vel_z;
			double _phi, _theta, _psi;

	};

	const double PI = 3.14159265358979323846;
	void radian_to_degree(double radian);
	void radar_coordinate();
	void coordinate_transform();
}


// Definition 
double AIMS::radian_to_degree(double radian)
{
	double degree;
	degree = AIMS::PI * radian / 180;
	return degree;
}


#endif MOVING_PLATFORM
