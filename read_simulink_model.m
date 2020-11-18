x_traj = out.ref_trajectory.x_ref.data; 
y_traj = out.ref_trajectory.y_ref.data;

x_robot = out.robot_pose.x.data;
y_robot = out.robot_pose.y.data;

plot(x_traj, y_traj)
hold on
plot(x_robot, y_robot)
hold off
legend("Trajetoria referencia","Trajetoria robo")
