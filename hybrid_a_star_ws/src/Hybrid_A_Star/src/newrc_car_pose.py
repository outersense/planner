
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from collections import deque
class ObstacleLatcher:
    def __init__(self):
        rospy.init_node('rccar_pose_node', anonymous=True)
        self.obstacle_pose_sub = rospy.Subscriber('rccar_pose', Float32MultiArray, self.callback)
        self.obstacle_pose_pub = rospy.Publisher('rccar_pose_new', Float32MultiArray, queue_size=10)
        self.maxlen = 40
        # self.poses_dq = deque(maxlen=self.maxlen)
        # self.ids_dq = deque(maxlen=self.maxlen)
        # self.ids = None
        # self.unique_ids = None
        self.poses_dq_find_idx = deque(maxlen=self.maxlen)
        # self.updated_pose = deque(maxlen=self.maxlen)

        

    def callback(self, msg):

        # pose = msg.data

        if len(self.poses_dq_find_idx) == self.poses_dq_find_idx.maxlen:

            # self.ids = self.extract_ids_from_msg(msg)
            # print("new id: ",self.ids)
            # print("ids dq: ",self.ids_dq)
            val = self.compare_message_lengths(self.poses_dq_find_idx,msg)
            print(val)

            if val == 999:
                self.obstacle_pose_pub.publish(msg)
            else:
                # self.updated_pose = self.update_pose(self.poses_dq_find_idx[val],msg)
                self.obstacle_pose_pub.publish(self.poses_dq_find_idx[val])

        self.poses_dq_find_idx.append(msg)       
        # self.poses_dq.append(pose)
        # self.ids_dq = self.extract_ids(self.poses_dq)





    # def update_pose(self,old_pose_msg, current_pose_msg):
    # # Function to parse the pose data into a dictionary
    #     def parse_pose_data(msg):
    #         pose_data = {}
    #         for i in range(0, len(msg.data), 5):
    #             id = msg.data[i]
    #             pose_data[id] = msg.data[i+1:i+5]  # x, y, velocity, yaw
    #         return pose_data

    #     # Parse the old and current poses
    #     old_pose = parse_pose_data(old_pose_msg)
    #     current_pose = parse_pose_data(current_pose_msg)

    #     # Update the old pose with the current pose data for IDs > 1000
    #     for id, data in old_pose.items():
    #         if id > 1000 and id in current_pose:
    #             old_pose[id] = current_pose[id]

    #     # Construct the updated pose message
    #     updated_pose_data = []
    #     for id, data in old_pose.items():
    #         updated_pose_data.extend([id] + list(data)) 

    #     return Float32MultiArray(data=updated_pose_data)

    # def extract_ids_from_msg(self,msg):
    # # Ensure that the input is a Float32MultiArray
    #     if not isinstance(msg, Float32MultiArray):
    #         raise ValueError("Input must be a Float32MultiArray")

    #     # Extract every fifth element starting from the first element (index 0)
    #     return [msg.data[i] for i in range(0, len(msg.data), 5)]
    
    # def extract_ids(self,data_deque):
    #     return deque([[data[i] for i in range(0, len(data), 5)] for data in data_deque])
 
    # def get_unique_ids(self,id_deque):
    #     unique_ids = set()
    #     for id_list in id_deque:
    #         unique_ids.update(id_list)
    #     return list(unique_ids)

    # def get_appropriate_id_list_index(self,id_list, id_deque):
    #     # Iterate through the deque in reverse order with index
    #     for index, ids in enumerate(reversed(id_deque)):
    #         if len(ids) > len(id_list):
    #             # Return the index from the end of the deque
    #             return len(id_deque) - 1 - index

    #     # If no list in the deque is longer, return 999
    #     return 999

    def compare_message_lengths(self,msg_deque, new_msg):
    # Calculate the length of the new message
        new_msg_length = len(new_msg.data)

        # Calculate the lengths of all messages in the deque
        deque_lengths = [len(msg.data) for msg in msg_deque]

        idx = self.find_index_of_highest(deque_lengths)

        return idx

        # Compare the new message length with lengths in the deque
        # for i, length in enumerate(deque_lengths):
        #     if new_msg_length < length:
        #         return i

        # print(new_msg_length,deque_lengths)

        # return 999

        # If the new message length is more than or equal to all lengths in the deque
        # return 999

    def find_index(self, value, int_list):
        # Initialize the index to a default value
        last_index = -1

        # Iterate over the list and update the last_index if the condition is met
        for i, num in enumerate(int_list):
            if value >= num:
                last_index = i

        # Check if the value is strictly less than any element in the list
        if any(value < num for num in int_list):
            return last_index if last_index != -1 else 999
        else:
            return 999
        
    def find_index_of_highest(self,int_list):
    # Check if the list is empty
        if not int_list:
            return -1  # or any other value that indicates an empty list

        # Find the index of the highest value
        max_index = int_list.index(max(int_list))
        return max_index

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    node = ObstacleLatcher()
    node.run()
