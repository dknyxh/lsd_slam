max=15
for i in {1..$max}
do
    echo "$i"
    rosrun lsd_slam_core dataset _files:=/media/rpl/Data/rgbd_dataset_freiburg3_sitting_xyz/rgb _masks:=/media/rpl/Data/rgbd_dataset_freiburg3_sitting_xyz/mask_all_good _hz:=30 _calib:=/home/rpl/ros_workspace/src/lsd_slam/TUM_calib/fr3_calib.txt
    sleep 1
    python3 /home/rpl/ros_workspace/src/lsd_slam/scripts/calculate_rmse.py
    python2 /home/rpl/ros_workspace/src/lsd_slam/scripts/evaluate_ate_scale.py /home/rpl/ros_workspace/src/lsd_slam/scripts/associate_precessed.txt /home/rpl/ros_workspace/src/lsd_slam/scripts/algorithm_trajectory.txt --plot PLOT_good
done

for i in {1..$max}
do
    echo "$i"
    rosrun lsd_slam_core dataset _files:=/media/rpl/Data/rgbd_dataset_freiburg3_sitting_xyz/rgb _masks:=/media/rpl/Data/rgbd_dataset_freiburg3_sitting_xyz/mask _hz:=30 _calib:=/home/rpl/ros_workspace/src/lsd_slam/TUM_calib/fr3_calib.txt
    sleep 1
    python3 /home/rpl/ros_workspace/src/lsd_slam/scripts/calculate_rmse.py
    python2 /home/rpl/ros_workspace/src/lsd_slam/scripts/evaluate_ate_scale.py /home/rpl/ros_workspace/src/lsd_slam/scripts/associate_precessed.txt /home/rpl/ros_workspace/src/lsd_slam/scripts/algorithm_trajectory.txt --plot PLOT_mask
done
