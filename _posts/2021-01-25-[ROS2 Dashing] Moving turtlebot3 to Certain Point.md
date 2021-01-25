# [ROS2 Dashing] Moving turtlebot3 to Certain Point

Turtlebot3 emanual을 따라하다 보면 Navigation 패키지에서 지도 상의 특정 위치를 지정하여 로봇을 해당 position까지 이동하는 과정이 있다.

이 항목에서 불편했던 점이 Navigation 패키지를 실행하기 위해서는 반드시 SLAM이 선행되어야 한다는 점이다.

자동으로 SLAM을 수행하기 위해 로봇이 장애물을 피해 구역을 스캔한다던가 등의 Mapping 파일 없이도 특정 지점까지 이동하는 패키지가 필요해서 간단하게 MovingRobot(v1.0.0)을 만들어보았다.



일단은 가장 먼저 공부한 Topic을 통한 송수신을 사용하였고, 아마 추후에 업데이트 하게된다면 Action을 사용하게 될 것 같다.

따라서 노드를 작성하기에 앞서 반드시 알아야 하는 부분은 어떤 형식의 메시지를 송수신 해야하며, 어떤 토픽을 사용하는가에 대한 내용이다.

가장 기본적으로 사용하는 메시지는 모터 구동에 관련된 것이다.

ros에서는 주로 /cmd_vel이라는 topic으로 메시지가 오가며, gemetry_msgs 의존성 패키지에 포함된 twist 자료형을 사용하며, 그 구조는 대략 다음과 같다.

```python
linear:
	x
    y
    z
angular:
    x
    y
    z   
```

turtlebot3에서는 linear.x가 직진 속도, angular.z가 반시계방향 회전 속도 담당하는 변수다.



로봇을 움직이는 방법을 안다면, turtlebot3가 얼마나 이동했는지 확인할 수 있어야 목적지까지의 도달 여부를 알 수 있다. 여기에서는 /odom topic을 이용한다.

/odom은 Odometry를 전달받는 토픽으로 주행기록계를 의미한다.

모터의 엔코더와 로봇에 장비된 센서를 통해 position을 측정하고 움직이는 상태에서 위치를 추정해낼 수 있다.

이 Odometry 또한 메시지를 다루기 위한 자료형이 제공되고 있다. 그 세부 갈래 중에서도 움직인 거리를 찾아내기 위해서 Position과 Quaternion을 받아온다.

```python
position:
    x
    y
    z
orientation:
    x
    y
    z
    w
```





Position은 우리가 흔히 생각하는 3차원 직각좌표계와 동일하게 볼 수 있다.

일단 로봇이 z=0 평면에서만 움직인다고 가정하고, position에서 (x,y) 좌표를 통해 목적지까지 도달했는지 판단한다.

목적지까지 이동할 때 축에 평행한 방향으로 움직이는 것이 아니라 한 번의 직진으로 도착할 수 있도록 로봇을 회전시키게 되는데, 이 과정에서 Quaternion을 사용한다.



Quaternion은 우리가 알고있는 Euler angle과는 다르게 4개의 성분(x,y,z,w)으로 구성되고, 이는 벡터(x,y,z)와 스칼라(w)를 의미한다.

처음에는 Quaternion과 Euler angle 사이의 관계를 추정해보려고 로봇을 회전시키며 토픽의 메시지를 관찰해봤는데, **직관적으로 파악하기가 매우 어려웠다..**

그래서 Wikipedia의 도움을 받았다.

 [Conversion between quaternions and Euler angles - Wikipedia](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles) 

Quaternion은 회전을 나타내는 벡터와 스칼라의 조합이며, 여기에서 yaw방향(theta 방향)을 아래와 같이 얻어낼 수 있다.

```c++
double siny_cosp = 2 * (w * z + x * y);
double cosy_cosp = 1 - 2 * (y * y + z * z);
double yaw = std::atan2(siny_cosp, cosy_cosp);
```



필요한 topic은 /cmd_vel과 /odom이 전부다.

코드에선 이 자료형들을 사용하기 위해 

```
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
```

두 문장을 추가하고,  의존성 설정도 수정해준다.



동작 방식은 우선 로봇을 목적지 방향까지 회전시킨 후 목표 지점과 angle이 동일하다면 직진한다.

이후 목적지에 도착하면 정지한다.

angle과 position의 오차는 if문을 수정해서 고칠 수 있고, odometry msg가 초당 33회정도 수신되므로 로봇의 속도에 맞춰 조정해줄 수 있다.



목적지는 우선 코드 내에서 전역변수로 추가해뒀고, 수정 업데이트 예정에 있다.



c++ 코드와 동작영상은 아래와 같다.

```c++
#define _USE_MATH_DEFINES
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

double goal_x = 2;
double goal_y = 2;
double goal_angle = atan2(goal_y, goal_x);

class MovingRobot : public rclcpp::Node
{
public:
	MovingRobot()
		: Node("moving_robot"), count_(0)
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1000);
		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1000, std::bind(&MovingRobot::topic_callback, this, _1));
	}

private:
	void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
	{
        // Euler angle의 yaw 계산
		double x = msg->pose.pose.orientation.x;
		double y = msg->pose.pose.orientation.y;
		double z = msg->pose.pose.orientation.z;
		double w = msg->pose.pose.orientation.w;

		double siny_cosp = 2 * (w * z + x * y);
		double cosy_cosp = 1 - 2 * (y * y + z * z);
		double yaw = std::atan2(siny_cosp, cosy_cosp);

		auto message = geometry_msgs::msg::Twist();
        // 오차 설정
		if (abs(goal_x - msg->pose.pose.position.x>0.1)) {
			if (abs(goal_angle - yaw)>0.03) {
				message.linear.x = 0;
				message.angular.z = 0.2;
			}
			else {
				message.linear.x = 0.2;
				message.angular.z = 0;
			}
		}
		else {
			message.linear.x = 0;
		}
		RCLCPP_INFO(this->get_logger(), "I heard: '%f'", yaw);
		RCLCPP_INFO(this->get_logger(), "Publishing: '%f.2'", message.linear.x);
		publisher_->publish(message);
	}
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
	size_t count_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MovingRobot>());
	rclcpp::shutdown();
	return 0;
}
```

![moving_certain_point_gif](https://imgur.com/4z9RrH6)