# Diamond_Age_Robotics_Challenge
Hamza Muzammal

This is my attempt at Diamond Age Robotics Challenge. Here you can find the breakdown of the project workspace and how to run the code.

## File structure
![File_structure](https://user-images.githubusercontent.com/101940817/189349754-3f466820-bf32-426c-a379-a0f8809ca541.PNG)


## How to launch 

All dependencies for this package are included in repositories of panda_moveit_config and franka_ros. Firstly launch the MoveIt Commander, RViz, and Gazebo from the panda_moveit_config package using following command  
`roslaunch panda_moveit_config demo_gazebo.launch`

Depending upon you want to call the service or action choose one of the followings

### Using service call
- launch the file so both spawning and stacking services are up  
`roslaunch diamond_age_challenge service_servers.launch`  
- Launch file which starts client node to spawning service  
`roslaunch diamond_age_challenge spawn_service_call.launch`
- Lastly launch file to start stacking service clietn node  
`roslaunch diamond_age_challenge stack_service_call.launch`

### Using action call  
- launch the file so both spawning and stacking actions are up  
`roslaunch diamond_age_challenge action_servers.launch`
- Launch file which starts client node to spawning action  
`roslaunch diamond_age_challenge spawn_action_call.launch`  
- Launch file which starts client node to stacking action  
`roslaunch diamond_age_challenge stack_action_call.launch`

## Result
### Box spawning with random pose



https://user-images.githubusercontent.com/101226336/189368978-095ffce5-99ba-4a0e-8257-e33060febe49.mp4

### Box stacking



https://user-images.githubusercontent.com/101226336/189369033-be946b82-6843-4f26-aa28-8d6df6c7152c.mp4





