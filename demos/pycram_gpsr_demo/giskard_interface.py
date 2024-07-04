import pycram.external_interfaces.giskard_new as giskard

giskard.init_giskard_interface()


giskard.move_head_to_pose()
#setup_demo.tf_listener.lookupTransform(source_frame='iai_kitchen/popcorn_table:p_table:table_center', target_frame='map', time=rospy.get_rostime())
#Pose(frame='map', position=[2.1302192752068003, 5.955716606845915, 0.71], orientation=[0.0, 0.0, 0.706825181105366, 0.7073882691671998])