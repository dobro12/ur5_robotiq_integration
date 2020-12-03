# UR5 robot_driver

## Calibration

calibration은 필수가 아니라고 한다.
그러나 end effector의 위치가 cm 수준으로 오차가 발생할 수 있기 때문에 하는 것을 권장한다고 한다.

실행방법은 아래와 같이 진행할 수 있다.

``` bash
$ roslaunch ur_calibration calibration_correction.launch \
robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
```

위 명령어를 통해 home directory에 my_robot_calibration.yaml 파일이 생성된다.
생성된 파일을 "ur_description" 프로젝트 (fmauch_universal_robot 폴더 아래에 있을 것) 의 config 폴더로 옮긴 후, "Universal_Robots_ROS_Driver/ur_robot_driver/launch/ur5_bringup.launch" 파일을 열어 아래와 같이 수정한다.

- before :

  ``` bash
  <arg name="kinematics_config" default="$(find ur_description)/config/ur5_default.yaml" doc="Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description."/>
  ```

- after :

  ``` bash
    <arg name="kinematics_config" default="$(find ur_description)/config/my_robot_calibration.yaml" doc="Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description."/>
    ```

## Controller_stopper

이 노드는 UR5의 polyscope (태블릿 위에서 돌아가는 소프트웨어) 에서 "External Control"이라는 URCap 프로그램이 실행되지 않을때, controller들을 멈추게 한다.
다만, 해당 노드의 파라미터인 consistent_controllers에 저장된 controller들은 멈추지 않는다.
그냥 웬만하면 안건드리는게 좋아보인다.

## URScript

위에서 언급한 "External Control"이라는 프로그램이 실행되면, 해당 프로그램은 연결된 pc로부터 "Universal_Robots_ROS_Driver/ur_robot_driver/launch/ur_control.launch" 에 args로 지정된 "urscript_file" 을 전송해달라고 요청한다.

## etc.

README.md 에서 TP (teach pendant) 는 그 투박한 UR5 전용 태블릿을 말하는 듯 하다.

client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
=> client는 주어진 topic의 /goal 및 /cancel 이라는 하위 토픽에 publish를 하며, /status, /result, /feedback 이라는 하위 토픽을 subscribe 한다.

client.send_goal(goal)
=> goal 이란 message 변수를 위에서 말한 토픽으로 publish 한다.

일단 되도록이면 moveit이 rviz를 통해 visualization도 해주고, 파이썬으로 확장도 간편하므로 moveit을 살려보도록 합시다.

- 가능성 1 : "fmauch_universal_robot/ur5_moveit_config/launch/planning_context.launch" 파일에서 load_robot_description이라는 arg 값이 false 여서, kinematic에 대한 정보를 모른다. 이 arg 값은 "fmauch_universal_robot/ur5_moveit_config/launch/move_group.launch" 파일에서 planning_context.launch 파일을 include할때 설정해줄 수 있다.

그게 안되면 어쩔 수 없다. kinematic을 이용해 직접 모션 플래닝한 후에, 호건이 인턴분께서 제공한 문서파일 대로 "scaled_pos_joint_traj_controller/command" 이라는 토픽을 이용하거나, 위에서 내가 발견한, actionlib.SimpleActionClient를 이용해야할 듯 하다.

rqt_graph 뿐만 아니라, rostopic list 를 통해 좀 더 디테일한 토픽들을 보면서 디버깅해줘야할 것 같다.
과연 rosrun을 통해 실행되는 rqt_joint_traj뭐시기는 어떤 노드를 생성하고, 어떤 토픽에 subscribe, publish하는지 정확히 알아가야, moveit을 못 쓰는 무서운 경우에도 어느 정도 대처할 수 있을 듯 하다.

위와 비교해서 moveit을 launch 하였을때, moveit이 어떤 노드를 생성, 어떤 토픽을 듣고 publish하는지 좀 더 구체적으로 확인하여, 과연 controller와의 연결이 안된건지 (현재 유력한 이유라 봄), 아니면 다른 부분에서 문제가 발생하는 건지 확실히 디버깅해봅시다.

## 2020.12.02

- robotiq finger joint cannot be found by moveit. maybe this can be a reason. and by this problem, the planner said, start state is invalid.

### robotiq 설정

- gazebo에서 로봇을 띄우기 위해, gazebo의 parameter server로 urdf와 같은 로봇관련 설정파일을 보내야한다. 이는 robot_description이라는 param값에 urdf파일을 load해주게 되면, 자동으로 parameter server가 설정파일을 확인한다. 이때 단순하게 urdf파일 자체를 보내지않고, xacro라는 언어를 통해 urdf 파일을 경우에 따라 동적으로 바꿀 수 있게한 후, param태그의 command값을 통해 xacro 파일을 컴파일하여 load한다.

- 실제 로봇에서도 kinematics를 위해 robot_description이라는 param을 정의한다. 다만 simulation과의 차이점이라면, `xacro:arg name="transmission_hw_interface" default="hardware_interface/PositionJointInterface"/>` 이라는 xacro arg값을 지정하지 않는다는 것이다. 이 transmission_hw_interface 값은 gazebo에서 시뮬레이션 할때 사용하는 물리엔진을 지정해주는 역할이라 보면 된다.

- xacro 문법 :

  - xacro:arg 태그는 `<param unless="$(arg limited)" name="robot_description" command="$(find xacro)/xacro --inorder '$(find ur_description)/urdf/ur5_robot.urdf.xacro' transmission_hw_interface:=$(arg transmission_hw_interface)" />` 이런식으로 xacro를 실행할때 argument처럼 값을 넣어줄 수 있다. 따라서 이처럼 `transmission_hw_interface`라는 argument를 넣어주고, xacro 파일내에서는 xacro:arg 태그를 이용해 해당 변수를 정의해줘야한다.
  
  - xacro:macro 태그는 하나의 함수를 만드는것과 같다. 태그내에서 name="ur5_robot"이라고 정의를 했으면, 새로운 줄에서 `<xacro:ur5_robot>`을 추가하여 macro 태그를 통해 정의한 함수를 실행시킬 수 있다. 함수의 입력은 macro 태그내에서 params라는 값을 통해 정의해줄 수 있다.

- robotiq_2f_action_server라는 노드에서 생성하는 action_driver 서버가 joint_states 토픽으로 직접 state를 publish함.
  
  - 시뮬레이션 입장 : 이 토픽을 subscribe하는 robot_state_publisher (launch 파일에서 robot_descripition이란 param을 가지고 생성하였음.) 가 tf토픽으로 publish한 데이터를 바탕으로 rviz가 화면에 뿌림. 즉, action_driver 서버가 다이나믹을 고려하지 않고 state를 joint_states로 publish하면 그 값으 그대로 rviz에 반영됨.

  - 실제로봇 입장 : moveit이 joint_states 토픽을 보고 현재의 state를 추정, planning을 하는 듯함. moveit이 관건인듯함. moveit이 joint_states토픽으로 위에서 언급한 action_driver 서버의 메세지가 들어와도 무시한다면 사실상, 그냥 둬도 됨.
  
  - 해보니깐 moveit은 rviz를 통해서도 동작이 가능. 근데 rviz는 /joint_states를 subscribe하는 robot_state_publisher node와 통신하며 화면에 뛰운다. 따라서 같은 토픽으로 (/joint_states) 메세지를 보내면, rviz가 헷갈려할 수도 있다. 흠, 근데 실제 로봇에선 /joint_states 토픽으로 publish하는 노드가 /ur_hardware_interface 인데 이 노드가 robot_description에 있는 모든 joint에 대한 값을 publish 안할 수도 있다 (ur5 joint 정보만, robotiq은 안하고). 이렇게 될경우 rviz는 다른 노드에서 다른 메세지가 같은 topic으로 오는 거니깐 헷갈려하지 않을 것 같기도 함. 시뮬레이션에서는 gazebo가 robot_description을 통해 default로 모든 joint 정보를 publish하기 때문에, robotiq_2f_action_server가 publish하는 값과 충돌이 남. (/finger_joint라는 이름을 가진 joint의 정보가 gazebo에서도 나오고, robotiq_2f_action_server에서도 나오는데, robotiq_2f_action_server는 시뮬레이션의 경우 gazebo로 action을 취한후 joint의 state를 publish하는게 아니라 막무가내로 publish하기때문에, 값이 서로 달라져버림.)

  - 결론은, 영향을 주는 건 rviz 화면 뿐이다~. 물론 moveit의 충돌검사시, gripper가 열려있냐 닫혀있냐가 아주 작은 영향을 줄 수 있기 때문에, 실제 로봇에서도 sync를 맞추면 좋을 것 같긴 하다. 아직은 실제 로봇에선 값이 충돌났는지 확인이 안된 상태니, 충돌안나면 깔끔하고, 충돌나면 흐음 그때 생각해봅시다.
  
  ## GAZEBO Default file Download

  - gazebo는 default로 필요한 파일을 따로 받아야하는 듯 하다. 링크는 `https://github.com/osrf/gazebo_models`, 이고 해당 파일을 `~/.gazebo/models`에 저장하거나 또는 `GAZEBO_MODEL_PATH` 환경변수로 모델 위치를 지정해주면 된다고 한다.
  