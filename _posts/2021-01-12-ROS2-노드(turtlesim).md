## ROS2 개요

해당 포스트는 [ROS 2 Overview](https://index.ros.org/doc/ros2/) 의 내용으로 작성되었습니다.

ROS는 2007년 최초의 프로젝트로부터 추상화과정 및 기능 개선을 통해 지속적으로 업데이트 되어왔고, 현재에도 많은 로봇에서 ROS를 사용하고 있다. 

ROS2에서는 ROS 프로젝트 시작 시 고려하지 않았던 사용 사례를 보완한다.

* Multi Robot System은 ROS를 사용하여 구축할 수 있지만 표준적인 접근 방식은 없으며, 단일 마스터 구조로 인해 해킹의 위험이 있다.
* MCU 등의 소형 컴퓨터가 ROS 환경의 메이저 참가자가 되도록 한다.
* 프로세스, 기계 간 통신을 포함하여 ROS에서 실시간 제어를 지원하고자 한다.
* 불완전한 네트워크 환경에서도 최대한의 작업을 수행하고자 한다.
* ROS가 실제 애플리케이션에 더욱 적합한 제품으로 진화할 수 있도록 하고자 한다.
* 시스템 구축 및 구조화를 위한 규정된 패턴을 제공하고자 한다.



ROS2는 shell 환경을 사용하여 workspace를 결합하는 개념에 의존한다. Workspace는 ROS2로 개발 중인 시스템의 위치를 나타내는 표현이며, 코어가 되는 작업 영역을 Underlay, 이 외의 지역성 작업 영역을 overlay라고 한다. ROS2로 개발할 때 여러 workspace가 동시에 활성화 되고, 다양한 버전의 다양한 패키지 집합에 대해 쉽게 개발할 수 있다.



## ROS 노드

노드는 작업을 위한 최소단위의 프로세스이다. 노드들은 그래프로 결합되어 있고, 각각 Publisher, Subscriber 등의 역할을 수행한다. ROS 1과2의 차이점은 Master 노드가 존재하지 않는다는 점이다.

ROS1에서는 로봇 시스템을 시작하는 단계에서 반드시 ros core 명령어를 통해 master node를 실행시켜야 했지만, ROS2에서는 ros2 run을 통해 하나 이상의 노드를 실행시키며 시스템이 시작된다.

전체 로봇 시스템은 동작하고 있는 다수의 노드들로 구성된다. 

간단한 turtlesim 예제를 통해 구조를 확인해보자

````
$ ros2 run turtlesim turtlesim_node
````

![turtlesim_node](C:\SPB_Data\JaeYoon\dustnia11.github.io\img\turtlesim_node.png)

```
$ ros2 run turtlesim turtle_teleop_key
```

![turtlesim_teleop](C:\SPB_Data\JaeYoon\dustnia11.github.io\img\turtlesim_teleop.png)

서로 다른 터미널에서 두 노드를 실행시키면 키보드의 방향키와 f를 중심으로 한 키셋으로 거북이를 움직일 수 있다.

이에 해당하는 node list와 node 정보를 확인하기 위해서

```
$ ros2 node list
$ ros2 node info /turtlesim
```

을 입력해본다.

```
/turtlesim
/teleop_turtle
```

```
/turtlesim
 Subscribers:
  /parameter_events: rcl_interfaces/msg/ParameterEvent
  /turtle1/cmd_vel: geometry_msgs/msg/Twist
 Publishers:
  /parameter_events: rcl_interfaces/msg/ParameterEvent
  /rosout: rcl_interfaces/msg/Log
  /turtle1/color_sensor: turtlesim/msg/Color
  /turtle1/pose: turtlesim/msg/Pose
  /turtle1/rotate_absolute/_action/feedback: turtlesim/action/RotateAbsolute_FeedbackMessage
  /turtle1/rotate_absolute/_action/status: action_msgs/msg/GoalStatusArray
 Services:
  /clear: std_srvs/srv/Empty
  /kill: turtlesim/srv/Kill
  /reset: std_srvs/srv/Empty
  /spawn: turtlesim/srv/Spawn
  /turtle1/rotate_absolute/_action/cancel_goal: action_msgs/srv/CancelGoal
  /turtle1/rotate_absolute/_action/get_result: turtlesim/action/RotateAbsolute_GetResult
  /turtle1/rotate_absolute/_action/send_goal: turtlesim/action/RotateAbsolute_SendGoal
  /turtle1/set_pen: turtlesim/srv/SetPen
  /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
  /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
  /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
  /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
  /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
  /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
  /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
```



위와 같이 publisher, subscriber, services 등에 관한 정보를 확인할 수 있다.

<!--각각의 용어는 패키지 작성 포스트에서 설명한다.-->



rqt_graph 명령어를 통해 연결 상태를 간단한 그래프로 확인할 수 있다.

```
$ rqt_graph
```

![turtlesim_rqtgraph](C:\SPB_Data\JaeYoon\dustnia11.github.io\img\turtlesim_rqtgraph.png)