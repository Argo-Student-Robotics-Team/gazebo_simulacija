# gazebo_simulacija
Link ka 3D modelu terena: https://drive.google.com/drive/u/1/folders/1Z1coc3Wxft7mOWQPk4EuTyV-D1dODW8- , terrain.obj
Link ka workspace-u gazebo simulacije: https://drive.google.com/drive/u/1/folders/1OAcS3Eza_zGC1mCQS3JAXDSqhVG2cnU1
Koraci:
1. klonirati projekat
2. cd ~/ws_mobile
3. colcon build
4. source ~/ws_mobile/install/setup.bash
5. ros2 launch mobile_robot gazebo_model.launch.py
6. otvoriti novi terminal
7. ros2 run teleop_twist_keyboard teleop_twist_keyboard
