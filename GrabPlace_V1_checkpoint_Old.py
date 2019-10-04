    def Grab_Place(self, theta, z_offset):
        self.status_message = "State: Grab_Place - Grabbing a Cube at one global coordinate and placing the cube in another"
        self.current_state = "Grab_Place"
        self.rexarm.set_speeds_normalized_global(0.1,update_now=True)
        self.rexarm.open_gripper()
        
        world_frame = self.clickCoordnates(2)
        world_frame[0:2,2] += z_offset
        homogeneous = np.transpose(np.ones((1,2)))
        coordinates_global = np.concatenate((world_frame,homogeneous),axis = 1)
        orientation_gripper = np.zeros((4,4))
        pose = np.zeros((4,4))
        size = np.size(coordinates_global,0)

        for i in range(size):
            orientation_gripper = np.dot(np.dot(-1*rotation(theta[i][0],'z'),rotation(theta[i][1],'y')),rotation(theta[i][2],'z'))
            print("actual",orientation_gripper)
            pose[0:4,0:4]=orientation_gripper
            pose[:,3] = np.transpose(coordinates_global[i])
            print(pose)
            self.rexarm.set_pose(pose)
            self.rexarm.pause(4)
            pose[2][3] = 13
            #print(pose)
            self.rexarm.set_pose(pose)
            self.rexarm.pause(3)
            if i==0:
                self.rexarm.close_gripper()

            else: 
                print("Made it")
                self.rexarm.open_gripper()

            self.rexarm.pause(1)

        pose[2][3] = z_offset
        print(pose)
        self.rexarm.set_speeds_normalized_global(0.05,update_now=True)
        self.rexarm.set_pose(pose)
        self.rexarm.pause(2)
        self.rexarm.set_speeds_normalized_global(0.1,update_now=True)

        for joint in self.rexarm.joints:
            joint.set_position(0.0)
        self.set_next_state("idle")
        self.rexarm.get_feedback()