
from sympy import *
from time import time
from mpmath import radians
import tf
import numpy as np

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm
forward_kinematics.launch` From here you can adjust the joint angles to find thetas, use the gripper to
extract positions and orientation (in quaternion xyzw) and lastly use link 5to find the position of the
wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):

    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]

    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])  # in quaternions

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    # print("\ncheck1p:\n%r\ncheck1o:\n%r\n" % (req.poses[0].position.x, req.poses[0].orientation.y))

    ########################################################################################

    # print("test_case: %r" % test_case)
    # print("test_case[0][0]: %r" % test_case[0][0])
    # print("test_case[0][1]: %r\n" % test_case[0][1])


    ######################## Insert IK code here
    
    ####################### Create Modified DH parameters 

    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

    alpha, a, d, q = symbols('alpha a d q')

    ######################## Define Modified DH Transformation matrix

    s = {	alpha0: 0,    	a0: 0,		d1: 0.75,	q1: q1,
    		alpha1: -pi/2,	a1: 0.35,	d2: 0,		q2: q2-pi/2,
    		alpha2:   0,	a2: 1.25,	d3: 0,		q3: q3,
    		alpha3: -pi/2,	a3: -0.054,	d4: 1.5,	q4: q4,
    		alpha4: pi/2,	a4: 0,		d5: 0,		q5: q5,
    		alpha5: -pi/2,	a5: 0,		d6: 0,		q6: q6,
    		alpha6:   0,	a6: 0,		d7: 0.303,	q7: q7} # gripper
    # TODO: maybe we can/should convert from UDRF to DH in T6_G


    

    ####################### Create individual transformation matrices

    def TF_matrix(alpha, a, d, q):

        TF = Matrix([	[cos(q),		-sin(q),		0,		a],
        		[sin(q)*cos(alpha),	cos(q)*cos(alpha),	-sin(alpha),	-sin(alpha)*d],
    			[sin(q)*sin(alpha),	cos(q)*sin(alpha),	cos(alpha),	cos(alpha)*d],
        		[0,			0,			0,		1]])

        return TF

    T0_1 = TF_matrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = TF_matrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = TF_matrix(alpha2, a2, d3, q3).subs(s)
    T3_4 = TF_matrix(alpha3, a3, d4, q4).subs(s)
    T4_5 = TF_matrix(alpha4, a4, d5, q5).subs(s)
    T5_6 = TF_matrix(alpha5, a5, d6, q6).subs(s)
    T6_G = TF_matrix(alpha6, a6, d7, q7).subs(s)

    T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

    # R3_6_sym = T3_4[0:3,0:3]*T4_5[0:3,0:3]*T5_6[0:3,0:3]

    # print("R3_6_sym:\n%r\n" % R3_6_sym)

    ###################### Do math

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
		req.poses[x].orientation.x, req.poses[x].orientation.y,
		req.poses[x].orientation.z, req.poses[x].orientation.w])

    EE_xyz = [req.poses[0].position.x, req.poses[0].position.y, req.poses[0].position.z]

    
    print("\nRoll:\t%04.3f\nPitch:\t%04.3f\nYaw:\t%04.3f\n" % (roll, pitch, yaw))

    # print("\nWrist center:\n%r\n" % WC)

    # R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3] # math check seems good
    # R3_6 = (R0_3**-1) * T0_G[:3,:3]

    # print("\nR3_6:\n%r\n" % R3_6)


    r, p, y = symbols('r p y')      # roll, pitch, yaw

    R_x = Matrix([	[1,	0,		0],
    			[0,	cos(r),		-sin(r)],
    			[0,	sin(r),		cos(r)]])

    R_y = Matrix([	[cos(p),	0,	sin(p)],
          		[0,		1,	0],
        		[-sin(p),	0,	cos(p)]])

    R_z = Matrix([	[cos(y),	-sin(y),	0],
    			[sin(y),	cos(y),		0],
    			[0,		0,		1]])


    #TODO: Verify that the EE_TF matrix is correct, apply the 0.303m translation to get WC
    #    Verify WC is correct. Use WC value to find q1, q2, q3

    R_corr = R_z.subs(y, pi) * R_y.subs(p, -pi/2)
    URDF_to_DH = simplify(R_z * R_y * R_x * R_corr)

    EE_TF = URDF_to_DH.row_join(Matrix([[EE_xyz[0]],[EE_xyz[1]],[EE_xyz[2]]])).col_join(Matrix([[0,0,0,1]]))
    EE_pose = EE_TF.evalf(subs={r:roll, p:pitch, y:yaw})



    wrist_offset = Matrix([	[1,	0,	0,	0],
				[0,	1,	0,	0],
				[0,	0,	1,	-0.303],
				[0,	0,	0,	1]])

    WC_pose = EE_pose * wrist_offset
    # WC_pose2 = wrist_offset * EE_pose		# doesn't work

    # WC_xyz = tf.transformations.translation_from_matrix(np.array(WC_pose1))
    # WCr, WCp, WCy = tf.transformations.euler_from_matrix(np.array(WC_pose))

    ################################ / // / / / / / / / / / / / / /

    print("\nWC_pose =\n%r\n" % WC_pose)

    # print("\nEE_TF_check:\n%r\n" % EE_TF)
    #print("\nWC_pose2 =\n%r\n" % WC_pose2)


    ### triangle side length
    Lxy = sqrt((WC_pose[0,3] * WC_pose[0,3]) + (WC_pose[1,3] * WC_pose[1,3]))	# hypotenuse of X/Y component of WC
    L23 = a2.subs(s)
    L34 = sqrt((a3**2) + (d4**2)).subs(s)
    L24 = sqrt((Lxy - a1)**2 + (WC_pose[2,3]-d1)**2).subs(s)

    # print("\nLxy_sqrt_test:\n\t%r\n" % Lxy_a)

    ### law of cosines

    phi2 = acos( (L23**2 + L24**2 - L34**2) / (2*L23*L24) )
    phi3 = acos( (L23**2 + L34**2 - L24**2) / (2*L23*L34) )
    phi4 = acos( (L24**2 + L34**2 - L23**2) / (2*L24*L34) )

    print("\npi =\t%r\n" % (phi2+phi3+phi4))

    ##################### Write values to joint angles

    theta1 = atan2(WC_pose[1,3], WC_pose[0,3])
    theta2 = pi/2 - phi2 - atan2(WC_pose[2,3]-d1.subs(s), Lxy-a1.subs(s) - 0.35)	# where does 0.35 come from?
    theta3 = pi/2 - phi3 - 0.036							# where does 0.036 come from?

    ###################### spherical wrist
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
    R3_6 = R0_3.inv() * EE_pose[0:3,0:3]

    W_roll, W_pitch, W_yaw = tf.transformations.euler_from_matrix(np.array(R3_6))

    # R3_6_sym = R3_6_sym * R_corr
    # print("\nR3_6_sym =\n%r\n" % R3_6_sym)

    theta4 = atan2(R3_6[1,0], -R3_6[0,0])
    theta5 = atan2(-R3_6[2,0],sqrt(R3_6[0,0]**2 + R3_6[1,0]**2))
    theta6 = atan2(R3_6[2,1], R3_6[2,2])

    joint_angles = {q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6}

    print("\ntheta2:\t%r\nphi2:\t%r\n" % (theta2,phi2))

    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    EE_f = T0_G.evalf(subs=(joint_angles))

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_ee = [EE_f[0,3],EE_f[1,3],EE_f[2,3]] # <--- Load your calculated WC values in this array
    your_wc = [WC_pose[0,3],WC_pose[1,3],WC_pose[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################


    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = your_wc[0]-test_case[1][0]
        wc_y_e = your_wc[1]-test_case[1][1]
        wc_z_e = your_wc[2]-test_case[1][2]
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = your_ee[0]-test_case[0][0][0]
        ee_y_e = your_ee[1]-test_case[0][0][1]
        ee_z_e = your_ee[2]-test_case[0][0][2]
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %r" % ee_x_e)
        print ("End effector error for y position is: %r" % ee_y_e)
        print ("End effector error for z position is: %r" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1


    test_code(test_cases[test_case_number])
