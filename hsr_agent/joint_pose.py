import rospy
import math
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)

from sensor_msgs.msg import JointState



class JointPose:
    def __init__(self, table_dimension, gripper):
        self.table_dimension = table_dimension

        self.gripper = gripper
        self.joint_value = {}
        self._joint_control_client = rospy.ServiceProxy('/safe_pose_changer/change_joint', SafeJointChange)
        self.joint_states_sub = rospy.Subscriber('/hsrb/robot_state/joint_states', JointState, self._joint_state_callback)

    def _joint_state_callback(self, data):
        gripper_radian = 0

        for i, name in enumerate(data.name):
            if name in ["hand_l_proximal_joint", "hand_l_spring_proximal_joint"]:
                # print('name', name, data.position[i])
                if name == 'hand_l_proximal_joint':
                    gripper_radian += data.position[i]
                elif name == 'hand_l_spring_proximal_joint':
                    gripper_radian -= data.position[i]

            self.gripper_radian = gripper_radian

            if name == 'arm_lift_joint':
                self.joint_value['arm_lift_joint'] = data.position[i]
            if name == 'arm_flex_joint':
                self.joint_value['arm_flex_joint'] = data.position[i]
            if name == 'arm_roll_joint':
                self.joint_value['arm_roll_joint'] = data.position[i]
            if name == 'wrist_flex_joint':
                self.joint_value['wrist_flex_joint'] = data.position[i]
            if name == 'wrist_roll_joint':
                self.joint_value['wrist_roll_joint'] = data.position[i]

    def check_grasp(self, threshold=-1.74):
        print('gripper_radian', self.gripper_radian)
        if self.gripper_radian <= threshold: # not grasping
            print('not grasp')
            return False
        else:                                # grasping
            print('grasp')
            return True

    def check_grasp_radian(self):
        return self.gripper_radian

    # arm pose
    def set_pose(self, joints, positions):
        goal_joint_states = JointState()
        goal_joint_states.name.extend(joints)
        goal_joint_states.position.extend(positions)
        try:
            req = SafeJointChangeRequest(goal_joint_states)
            res = self._joint_control_client(req)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        return res.success

    def move_pose(self, vertical=False):
        wrist_roll_joint = 0
        if vertical:
            wrist_roll_joint = -1.57
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0, 0, -1.57, -1.57, wrist_roll_joint])

    def table_search_pose_breakfast(self):
        self.gripper.grasp(0.1)
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint',
                       'head_tilt_joint'],
                      [0.3, 0, -1.57, -1.57, -1.04, -0.52])

    def restaurant_move_pose(self):
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0, 0, -1.57, -1.57, 3.14])

    def restaurant_give_pose(self):
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0, 0, 0, -1.57, 3.14])

    def dish_washer_ready_pose(self, vertical=False, table='dishwasher'):
        target_table_height = self.table_dimension[table][2]
        table_to_gripper = 0.12
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + table_to_gripper - robot_default_height
        if arm_lift_joint > 0.69:
            arm_lift_joint = 0.69
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, 0, -1.57, 0])

    def neutral_pose(self, vertical=False):
        wrist_roll_joint = 0
        if vertical:
            wrist_roll_joint = -1.57

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0, 0, 0, -1.57, wrist_roll_joint])

    def bye1_pose(self):
        wrist_roll_joint = 1.57
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0, 0, -1.57, -0.6, wrist_roll_joint])

    def bye2_pose(self, vertical=False):
        wrist_roll_joint = 1.57
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0, 0, -1.57, 0.6, wrist_roll_joint])
    def dish_washer_neutral_pose(self, vertical=False, table='dishwasher'):
        target_table_height = self.table_dimension[table][2]
        table_to_gripper = 0.12
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + table_to_gripper - robot_default_height
        if arm_lift_joint > 0.69:
            arm_lift_joint = 0.69

        wrist_roll_joint = 0
        if vertical:
            wrist_roll_joint = -1.57

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, 0, -1.57, wrist_roll_joint])

    def pick_top_pose(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.11
        arm_lift_joint = target_table_height - robot_default_height
        if arm_lift_joint > 0.69:
            arm_lift_joint = 0.69
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, -1.57, 0])

    def pick_top_pose_last(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.11
        arm_lift_joint = target_table_height - robot_default_height
        if arm_lift_joint >= 0.69:
            arm_lift_joint = 0.69

        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.2, -1.9, 0])

    def pick_plate_pose(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.11
        arm_lift_joint = target_table_height - robot_default_height
        print("here!")
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, -1.57, 1.57])



    def pick_bowl_pose(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        table_to_gripper = 0.12
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + table_to_gripper - robot_default_height
        print('arm_lift_joint', arm_lift_joint)
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, -1.57, 1.57])

    def pick_bowl_pose_last(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        table_to_gripper = 0.12
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + table_to_gripper - robot_default_height
        if arm_lift_joint > 0.69:
            arm_lift_joint = 0.69

        print('arm_lift_joint', arm_lift_joint)
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.2, -1.9, 1.57])

    def pick_plate_pose_fold(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        table_to_gripper = 0.12
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + table_to_gripper - robot_default_height
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0.4, 0, -1.57, 0, 1.57])


    def pick_bowl_max_pose(self, table='kitchen_table', height=0):  # added height parameter by Minjun at June 11th
        target_table_height = self.table_dimension[table][2]
        table_to_gripper = 0.12
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + table_to_gripper + height - robot_default_height
        if arm_lift_joint > 0.69:
            arm_lift_joint = 0.69
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, -1.57, 1.57])
    def pick_up_bowl_pose(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        table_to_gripper = 0.12
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + table_to_gripper - robot_default_height
        if arm_lift_joint > 0.69:
            arm_lift_joint = 0.69
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1., -1.57, 1.57])
    def bring_bowl_pose(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        table_to_gripper = 0.12
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + table_to_gripper - robot_default_height
        if arm_lift_joint > 0.69:
            arm_lift_joint = 0.69
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.4, -1.57, 1.57])

    def pick_side_pose(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.3
        offset = 0.03
        arm_lift_joint = target_table_height - robot_default_height + offset

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 0, 0, 0])

    def pick_side_pose_by_height(self, height=0.69):
        arm_lift_joint = height

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 0, 0, 0])

    def pick_shelf_pose(self, table, offset=0.12):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.7
        arm_lift_joint = target_table_height - robot_default_height + offset
        print(arm_lift_joint)
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -0.5, 0, -1.07, 0])

    def pick_bread_final_pose(self, table, offset=0.12):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.7
        arm_lift_joint = target_table_height - robot_default_height + offset
        print('arm_lift_joint', arm_lift_joint)
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -0.5, 0, -1.07, -1.57])
    def open_shelf_pose1_by_height(self, height=0.69):
        arm_lift_joint = height

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 1.57, 0, 0])
    def open_shelf_pose2_by_height(self, height=0.69):
        arm_lift_joint = height

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 1.57, -1.57, 0])



    def put_plate_dish_washer(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        table_to_gripper = 0.12
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + table_to_gripper - robot_default_height
        if arm_lift_joint > 0.69:
            arm_lift_joint = 0.69
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.3, -1.57, 1.57])

    def arm_lift_up(self, length=0.65):
        self.set_pose(['arm_lift_joint'], [length])

    def arm_lift_object_table_down(self, object_height, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.3
        offset = -0.02  # table to object before open_gripper
        # 0.74(kitchen_table) + 0.16(cereal) + 0.01 - 0.3 = 0.61
        arm_lift_joint = target_table_height + object_height + offset - robot_default_height
        print('arm_lift_joint', arm_lift_joint)
        self.set_pose(['arm_lift_joint'], [arm_lift_joint])
    
    def arm_lift_top_table_down(self, height, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.11
        # 0.625(breakfast_table) + 0.03(height) - 0.11 = 0.545
        arm_lift_joint = target_table_height + height - robot_default_height
        print('arm_lift_joint', arm_lift_joint)
        self.set_pose(['arm_lift_joint'], [arm_lift_joint])

    def arm_lift_top_down(self, height):
        robot_default_height = 0.17
        # 0.625(breakfast_table) + 0.03(height) - 0.11 = 0.545
        arm_lift_joint = height - robot_default_height
        print('arm_lift_joint', arm_lift_joint)
        self.set_pose(['arm_lift_joint'], [arm_lift_joint])
    
    def place_top_pose(self, offset, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.11
        arm_lift_joint = target_table_height + offset - robot_default_height
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, -1.57, 0])


    def place_side_pose(self, table):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.3
        offset = 0.05
        arm_lift_joint = target_table_height - robot_default_height + offset

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 0, 0, 0])

    def pick_shelf_low_pose(self, table):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.3
        offset = 0.035
        arm_lift_joint = target_table_height - robot_default_height + offset

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 0, 0, 0])


    def place_bowl_pose(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        hand_down_length = 0.15 # amount of hand-end going down, due to wrist_flex_joint rotation by 45 degree 
        robot_default_height = 0.3
        bowl_offset = 0.03
        arm_lift_joint = target_table_height + bowl_offset + hand_down_length - robot_default_height

        if arm_lift_joint > 0.69:
            arm_lift_joint = 0.69

        self.set_pose(['head_tilt_joint'], [0])
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, -0.785, 1.57])


    def place_cutlery_pose(self, table='dishwasher'):
        target_table_height = self.table_dimension[table][2]
        hand_down_length = 0.15  # amount of hand-end going down, due to wrist_flex_joint rotation by 45 degree
        robot_default_height = 0.3
        object_offset = 0.03
        arm_lift_joint = target_table_height + object_offset + hand_down_length - robot_default_height

        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, -1.57, 0])


    def place_object_pose(self, table='dishwasher', item='bowl'):
        tab_name = 'peach'

        item_offset = {'bowl': -0.03, 'plate': 0.03, 'mug': -0.05, 'fork': -0.03, 'spoon': -0.03, 'knife': -0.03,
                       tab_name: -0.03}  # modify
        arm_flex_angle = {'bowl': -1.57, 'plate': -1.0, 'mug': -1.57, 'fork': -1.57, 'spoon': -1.57, 'knife': -1.57,
                          tab_name: -1.57}  # modify
        wrist_flex_angle = {'bowl': -1.57, 'plate': -1.57, 'mug': 0, 'fork': -1.57, 'spoon': -1.57, 'knife': -1.57,
                            tab_name: -0.785}  # modify
        arm_roll_angle = {'bowl': 1.47, 'plate': 0, 'mug': 0, 'fork': 0, 'spoon': 0, 'knife': 0,
                          tab_name: 0}  # modify
        # if pose_1f:
        wrist_roll_angle = {'bowl': 3.14, 'plate': 0, 'mug': 3.14, 'fork': 0, 'spoon': 0,'knife': 0, tab_name: 1.57}
        robot_default_height = 0.25
        # else:
        #     wrist_roll_angle = {'bowl': 1.57, 'plate': -1.57, 'mug': 1.57, 'fork': 0, 'spoon': 0, 'knife': 0, tab_name: 1.57}
        #     robot_default_height = -0.15

        target_table_height = self.table_dimension[table][2]
        hand_down_length = 0.15               # amount of hand-end going down, due to wrist_flex_joint rotation by 45 degree
        object_offset = item_offset[item]
        arm_lift_joint = target_table_height + object_offset + hand_down_length - robot_default_height
        print('arm_lift_joint', arm_lift_joint)
        # if item == 'spoon' or item == 'fork' or item == 'knife':
        #     rospy.sleep(1)
        #     if pose_1f:
        #         self.set_pose(['arm_lift_joint'], [target_table_height - 0.27])  # arm_height: 0.53
        #     else:
        #         print('line 467', target_table_height +0.13)
        #         self.set_pose(['arm_lift_joint'], [target_table_height +0.13])  # arm_height: 0.53
        # else:
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, arm_roll_angle[item], arm_flex_angle[item], wrist_flex_angle[item], wrist_roll_angle[item]])

        if item =='mug':
            self.set_pose(['arm_lift_joint',
                           'arm_roll_joint',
                           'arm_flex_joint',
                           'wrist_flex_joint',
                           'wrist_roll_joint'],
                          [arm_lift_joint - 0.1, arm_roll_angle[item], arm_flex_angle[item], wrist_flex_angle[item],
                           wrist_roll_angle[item]])
        if item == 'plate':
            rospy.sleep(1)
            self.set_pose(['wrist_flex_joint'], [1.57])
            self.set_pose(['arm_lift_joint'], [0.69-(0.97 - 0.13 - 0.34)])   # arm_lift_joint - (robot_pointer - object_radius - dishwasher_height)

    def place_shelf_pose(self, table):
        target_table_height = self.table_dimension[table][2]
        offset = 0.15
        robot_default_height = 0.7
        arm_lift_joint = target_table_height - robot_default_height + offset

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -0.5, 0, -1.07, 0])

    def pick_side_inclined_pose(self, table):
        target_table_height = self.table_dimension[table][2]
        offset = 0.085
        robot_default_height = 0.7
        arm_lift_joint = target_table_height - robot_default_height + offset

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -0.5, 0, -1.07, 0])
    
    def pick_object_side_pose(self, object_height, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.33
        arm_lift_joint = target_table_height + object_height - robot_default_height
        print('arm_lift_joint, ', arm_lift_joint)
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 0, 0, 0])

    def spill_object_pose(self, object_height, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.3
        offset = 0.04 # object to table
        # 0.74(kitchen_table) + 0.2 - 0.3 = 0.64
        arm_lift_joint = target_table_height + object_height + offset - robot_default_height
        print('arm_lift_joint', arm_lift_joint)

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 0, 0, 0])

    def pick_side_cup_pose(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.3
        object_height = 0.03
        offset = 0.04 # object to table
        # 0.74(kitchen_table) + 0.2 - 0.3 = 0.64
        arm_lift_joint = target_table_height + object_height + offset - robot_default_height
        print('arm_lift_joint', arm_lift_joint)

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 0, 0, 0])

    def pick_dish_pose_2(self, table='kitchen_table'):
        target_table_height = self.table_dimension[table][2]
        robot_default_height = 0.3
        object_height = 0.03
        offset = 0.04 # object to table
        # 0.74(kitchen_table) + 0.2 - 0.3 = 0.64
        arm_lift_joint = target_table_height + object_height + offset - robot_default_height
        print('arm_lift_joint', arm_lift_joint)

        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, -1.57, 0, 1.57, 1.57])

    def door_open_pose(self, table='door_handle'):
        target_door_handle_height = self.table_dimension[table][2]
        robot_default_height = 0.34
        arm_lift_joint = target_door_handle_height - robot_default_height
        print("door open!")
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, 0, 1.57])


    def door_handle_down_pose(self, table='door_handle'):
        target_door_handle_height = self.table_dimension[table][2]
        robot_default_height = 0.34
        arm_lift_joint = target_door_handle_height - robot_default_height
        print("door open!")
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.6, -0.1, 0.6])

    def point_seat_pose(self):
        print("point seat pose!")
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0.4, 0, -1.57, 0, 1.57])


    def table_search_pose(self):
        self.gripper.grasp(0.1)
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint',
                       'head_tilt_joint'],
                      [0.2, 0, -1.57, -1.57, -1.04, -0.52])

    def table_search_pose_high(self):
        self.gripper.grasp(0.1)
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint',
                       'head_tilt_joint'],
                      [0.35, 0, -1.57, -1.57, -1.04, -0.52])

    def table_search_go_pose(self):
        self.gripper.grasp(0.1)
        self.set_pose(['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint',
                       'head_tilt_joint'],
                      [0.2, 0, -1.57, -1.77, -0.04, -0.52])

    # for carry my luggage
    def pick_up_bag(self, bag_height=.25, bag_orientation_rad=-1.57):
        if bag_orientation_rad is None:
            bag_orientation_rad = -1.57
        rospy.loginfo("bag orientation : {0}".format(bag_orientation_rad * 180 / math.pi))
        robot_default_height = 0.07
        arm_lift_joint = bag_height - robot_default_height

        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, -1.57, bag_orientation_rad])


    def bag_inspection_pose(self):

        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0.3, -1.57, -0.7, -1.57, 0])

    def hand_me_bag(self):

        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0, 0, 0, -1.57, 0])


    def move_to_go(self):
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [0, -1.57, 0, -1.57, 0])

    def plate_dishwasher_pose(self):
        self.set_pose(['arm_roll_joint',
                       'wrist_roll_joint'],
                      [0, 0])
    def dishwasher_door_open_pose(self, arm_lift_joint=0.69):
        self.set_pose(['arm_lift_joint',
                       'arm_roll_joint',
                       'arm_flex_joint',
                       'wrist_flex_joint',
                       'wrist_roll_joint'],
                      [arm_lift_joint, 0, -1.57, -1.57 * 2 / 3, 1.57])

    def head_tilt(self, angle):
        angle = math.radians(int(angle))
        self.set_pose(['head_tilt_joint'], [angle])

    def head_pan(self, angle):
        angle = math.radians(int(angle))
        self.set_pose(['head_pan_joint'], [angle])

    def head_pan_tilt(self, pan_angle, tilt_angle):
        pan_angle = math.radians(int(pan_angle))
        tilt_angle = math.radians(int(tilt_angle))
        self.set_pose(["head_pan_joint", "head_tilt_joint"], [pan_angle, tilt_angle])
    
    def wrist_roll(self, angle):
        angle = math.radians(int(angle))
        self.set_pose(['wrist_roll_joint'], [angle])

    def wrist_flex(self, angle):
        angle = math.radians(int(angle))
        self.set_pose(['wrist_flex_joint'], [angle])

    def arm_flex(self, angle):
        angle = math.radians(int(angle))
        self.set_pose(['arm_flex_joint'], [angle])
    def arm_roll(self, angle):
        angle = math.radians(int(angle))
        self.set_pose(['arm_roll_joint'], [angle])