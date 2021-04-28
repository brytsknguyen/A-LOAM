catkin_make -C /home/$USER/dev_ws ;
source /home/$USER/dev_ws/devel/setup.bash


# Get the current directory
CURR_DIR=$(pwd)
# Get the location of the viral package
roscd aloam
PACKAGE_DIR=$(pwd)
# Return to the current dir, print the directions
cd $CURR_DIR
echo CURRENT DIR: $CURR_DIR
echo VIRAL DIR:   $PACKAGE_DIR

export EPOC_DIR=/home/$USER/nucssd2/MATLAB_WS/RAL_edits/viralslam/RAL_VIRAL_SLAM/NTUVIRAL_20210428
export DATASET_LOCATION=/home/$USER/DATASETS/NTU_VIRAL/
# export DATASET_LOCATION=/media/$USER/myHPSSD/NTU_VIRAL

export CAPTURE_SCREEN=false;
export LOG_DATA=true;

#region USE VIS, NO UWB -----------------------------------------------------------------------------------------------

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_01 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_02 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_03 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
 
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_01 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_02 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_03 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_01 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_02 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_03 $CAPTURE_SCREEN $LOG_DATA 450 0 1 0.75 -1;

#endregion USE VIS, NO UWB --------------------------------------------------------------------------------------------


#region USE VIS, USE UWB, 2 ANCHORS------------------------------------------------------------------------------------

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_01 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_02 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_03 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 1;
 
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_01 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_02 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_03 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 1;

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_01 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_02 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_03 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 1;

#endregion USE VIS, USE UWB, 2 ANCHORS --------------------------------------------------------------------------------


#region USE VIS, USE UWB, 3 ANCHORS------------------------------------------------------------------------------------

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_01 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_02 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_03 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 2;
 
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_01 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_02 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_03 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 2;

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_01 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_02 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_03 $CAPTURE_SCREEN $LOG_DATA 450 1 1 0.75 2;

#endregion USE VIS, USE UWB, 3 ANCHORS --------------------------------------------------------------------------------


#region NO VIS, NO UWB ------------------------------------------------------------------------------------------------

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_01 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_02 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_03 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
 
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_01 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_02 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_03 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_01 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_02 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_03 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;

#endregion NO VIS, NO UWB ---------------------------------------------------------------------------------------------


#region NO VIS, USE UWB, 2 ANCHORS---------------------------------------------------------------------------------------

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_01 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_02 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_03 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 1;
 
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_01 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_02 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_03 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 1;

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_01 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_02 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 1;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_03 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 1;

#endregion NO VIS, USE UWB, 2 ANCHORS ---------------------------------------------------------------------------------


#region NO VIS, USE UWB, 3 ANCHORS---------------------------------------------------------------------------------------

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_01 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_02 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_03 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 2;
 
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_01 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_02 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_03 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 2;

wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_01 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_02 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 2;
wait;
./run_one_bag_ntuviral.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_03 $CAPTURE_SCREEN $LOG_DATA 450 1 0 0.75 2;

#endregion NO VIS, USE UWB, 3 ANCHORS ---------------------------------------------------------------------------------


#region ## Poweroff ---------------------------------------------------------------------------------------------------

wait;
# poweroff;

#endregion ## Poweroff ------------------------------------------------------------------------------------------------