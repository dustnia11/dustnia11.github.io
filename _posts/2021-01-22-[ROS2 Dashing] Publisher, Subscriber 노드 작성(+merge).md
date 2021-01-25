## [ROS2 Dashing] Publisher, Subscriber 노드 작성(+merge)

ROS2는 ROS와 마찬가지로 message를 통한 통신방법을 사용한다.

이에 따른 용어를 한번 정리하자면 다음과 같다.

* 노드(Node) - 실행되는 최소 단위의 프로세스, 프로그램.
* 패키지(Package) - 하나 이상의 노드를 포함하거나 다른 node를 실행하기 위한 설정 파일들의 집합
* 메시지(Message) - 노드간 주고받는 데이터의 형식 / 토픽, 서비스, 액션의 3가지 방식으로 송수신
* 토픽(Topic) - 메시지가 오고가기 위한 채널 / Publisher에서 Subscriber로 단방향 통신
* 퍼블리셔(Publisher) - Topic을 통해 Message를 송신하는 개체
* 서브스크라이버(Subscriber) - Topic을 통해 Message를 수신하는 개체
* 서비스(Service) - 실시간 양방향 통신 방식
* 액션(Action) - 비동기 양방향 통신 방식

해당 포스트에서는 퍼블리셔, 서브스크라이버, 토픽을 이용하여 간단한 패키지 작성을 수행한다.

일련의 과정은 [Writing a simple publisher and subscriber (C++) (ros.org)](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/) 에서 찾아볼 수 있다.



1. cpp_pubsub라는 이름의 패키지를 create 한다.

```
$ ros2 pkg create --build-type ament_cmake cpp_pubsub
```

2. github에서 퍼블리셔 cpp 파일을 받아온다.

   ```
   $ cd dev_ws/src/cpp_pubsub/src
   $ wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp
   ```

3. 의존성 항목을 작성한다.

   - package.xml에서 

   ```xml
   <description>Examples of minimal publisher/subscriber using rclcpp</description>
   <maintainer email="you@email.com">Your Name</maintainer>
   <license>Apache License 2.0</license>
   ```

   ```xml
   <depend>rclcpp</depend>
   <depend>std_msgs</depend>
   ```

   항목을 작성한다. (중복작성 하지 않도록 주의)

   - CMakeList.txt에서

   ```c++
   find_package(rclcpp REQUIRED)
   find_package(std_msgs REQUIRED)
   ```

   ```c++
   add_executable(talker src/publisher_member_function.cpp)
   ament_target_dependencies(talker rclcpp std_msgs)
   // 추가 dependency 작성 시 std_msgs 옆에 적어나가면 된다.
   ```

   ```c++
   install(TARGETS
     talker
     DESTINATION lib/${PROJECT_NAME})
   ```

   위의 항목들을 추가 / 수정 한다.

4. Subscriber에 대해서도 위의 과정을 반복한다. (의존성은 중복작성 필요 x)

   ```
   $ wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp
   ```

   ```c++
   add_executable(listener src/subscriber_member_function.cpp)
   ament_target_dependencies(listener rclcpp std_msgs)
   
   install(TARGETS
     talker
     listener
     DESTINATION lib/${PROJECT_NAME})
   ```



### Publisher_member_function.cpp

```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0) // 노드 이름 작성, count_ 변수 초기화
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      // publish할 topic 명시 (std_msgs.msg.String 사용)
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
      // 0.5초마다 timer_callback 함수 발생
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      // 송신할 message setting
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // 터미널에 [INFO] 문구 출력
      publisher_->publish(message);
      // publish
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    // 멤버변수
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    // MinimalPublisher thread 반복
    rclcpp::shutdown();
    return 0;
  }
```



### Subscriber_member_function.cpp

```c++
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      // 수신할 topic 명시
    }

  private:
    // 토픽을 통해 메시지를 수신할 때마다 topic_callback 함수 동작, 수신 메시지를 변수 msg로 사용
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      // 수신 메시지 출력
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```



모두 작성이 완료되었다면

```
$ cd ~/devel_ws && colcon build --symlink-install
$ . install/setup.bash
```

입력 후 빌드 완료를 기다린다.

```
$ ros2 run cpp_pubsub talker
```

```
$ ros2 run cpp_pubsub listener
```

를 입력하면 작성한 패키지의 talker, listener 노드를 실행할 수 있다.



```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

각 터미널에서 이런 문장이 출력된다면 정상 작동 중이고, ctrl + c 키로 종료할 수 있다.





### +++ Merge

Publisher와 Subscriber를 한 노드에서 동작시키고 싶다면 간단하게 두 cpp 파일을 합치면 된다.

```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalMerge : public rclcpp::Node
{
  public:
    MinimalMerge()
    : Node("minimal_merge"), count_(0) // 노드 이름 작성, count_ 변수 초기화
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      // publish할 topic 명시 (std_msgs.msg.String 사용)
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      // subscribe할 topic 명시
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
      // 0.5초마다 timer_callback 함수 발생
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      // 송신할 message setting
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // 터미널에 [INFO] 문구 출력
      publisher_->publish(message);
      // publish
    }
    // 토픽을 통해 메시지를 수신할 때마다 topic_callback 함수 동작, 수신 메시지를 변수 msg로 사용
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      // 수신 메시지 출력
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
    // 멤버변수
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalMerge>());
    // MinimalPublisher thread 반복
    rclcpp::shutdown();
    return 0;
  }
```

의존성 파일은 이에 맞춰 수정해주면 된다.