from sympy import Matrix, symbols, cos, sin, simplify, pi, pprint, diff, transpose
from numpy import linspace, meshgrid, ones_like, shape
import matplotlib.pyplot as plt
import time

# Simulation parameters 
tool_length = 10     # (cm)
N = 200             # no. of data points
delta_time = 5.0/N     # time between successive data points

# Panda Link offsets (cm)
a3 = 8.8
d1 = 33.3
d3 = 31.6
d5 = 38.4
d7 = 10.7 + tool_length  # Including pen length in d7 parameter

# Defining the symbolic joint angle symbolic variables 
q1, q2, q4, q5, q6, q7 = symbols('q1, q2, q4, q5, q6, q7')

# Function to obtain Transformation matrix between consecutive links 
def get_tf(q,d,a,alpha):
    T = Matrix([[cos(q),-sin(q)*cos(alpha),sin(q)*sin(alpha),a*cos(q)], 
                [sin(q),cos(q)*cos(alpha),-cos(q)*sin(alpha),a*sin(q)], 
                [0,sin(alpha),cos(alpha),d], 
                [0,0,0,1]])    
    return T

# D-H table for the Panda robot as given in the question.
T_01 = get_tf(q1,  d1,   0,  pi/2) # A_1
T_12 = get_tf(q2,   0,   0, -pi/2)
T_23 = get_tf(0,   d3,  a3, -pi/2)
T_34 = get_tf(q4,   0, -a3,  pi/2)
T_45 = get_tf(q5,  d5,   0,  pi/2)
T_56 = get_tf(q6,   0,  a3, -pi/2)
T_6n = get_tf(q7, -d7,   0,     0)

T_02 = T_01 * T_12 # A_2
T_03 = T_02 * T_23
T_04 = T_02 * T_23 * T_34 # A_4
T_05 = T_04 * T_45 # A_5
T_06 = T_05 * T_56 # A_6
T_0n = T_06 * T_6n # A_7


print("\n****************************************************************************")
print("       Transformation Matrix (T_0n) between end-effector & base frames:       ")
print("****************************************************************************\n")

print("T_0n: ")
pprint(simplify(T_0n))

# Setting up the Jacobian Matrix 

Z_0 = Matrix([0, 0, 1])
print("Z_0 shape: ", len(Z_0), Z_0)
Z_1 = simplify(T_01[0:3,2])
Z_2 = simplify(T_02[0:3,2])
Z_4 = simplify(T_04[0:3,2])
Z_5 = simplify(T_05[0:3,2])
Z_6 = simplify(T_06[0:3,2])

#######################################
# Using Method-2 to compute Jacobian
#######################################
x_p = T_0n[0:3,3]

h_1 = diff(x_p,q1)
h_2 = diff(x_p,q2)
h_3 = diff(x_p,q4)
h_4 = diff(x_p,q5)
h_5 = diff(x_p,q6)
h_6 = diff(x_p,q7)
J_v1 = simplify(h_1)
print("J_v1: ", J_v1)
J_v2 = simplify(h_2)
J_v3 = simplify(h_3)
J_v4 = simplify(h_4)
J_v5 = simplify(h_5)
# print("J_v4: ")
# pprint(J_v4)
# print("J_v5: ")
# pprint(J_v5)
J_v6 = simplify(h_6)
# print("J_v6: ")
# pprint(J_v6)
# pprint(shape(J_v1))

#######################################
# Using Method-1 to compute Jacobian
#######################################
# O_0 = Matrix([0, 0, 0])
# O_1 = T_01[0:3,3]
# O_2 = T_02[0:3,3]
# O_4 = T_04[0:3,3]
# O_5 = T_05[0:3,3]
# O_6 = T_06[0:3,3]
# O_n = T_0n[0:3,3]
# J_v1 = simplify(Z_0.cross((O_n-O_0)))
# J_v2 = simplify(Z_1.cross((O_n-O_1)))
# J_v3 = simplify(Z_2.cross((O_n-O_2)))
# J_v4 = simplify(Z_4.cross((O_n-O_4)))
# J_v5 = simplify(Z_5.cross((O_n-O_5)))
# J_v6 = simplify(Z_6.cross((O_n-O_6)))


J_v = Matrix().col_insert(0,J_v1).col_insert(1,J_v2).col_insert(2,J_v3).col_insert(3,J_v4).col_insert(4,J_v5).col_insert(5,J_v6)
print("J_v is of shape", J_v.shape)
pprint(J_v)
J_w = Matrix().col_insert(0,Z_0).col_insert(1,Z_1).col_insert(2,Z_2).col_insert(3,Z_4).col_insert(4,Z_5).col_insert(5,Z_6) 
# print("J_w: ")
print("J_w is of shape", J_w.shape)
pprint(J_w)
# pprint(J_w)
J = Matrix().row_insert(0,J_v).row_insert(3,J_w)
J2 = Matrix([[h_1, h_2, h_3, h_4, h_5, h_6],
    [Z_0, Z_1, Z_2, Z_4, Z_5, Z_6]])
# print("first column of jacobian: ")
# pprint(simplify(J[:,0]))
# print("2nd column of jacobian: ")
# pprint(simplify(J[:,1]))
# print("3rd column of jacobian: ")
# pprint(simplify(J[:,2]))
# print("4th column of jacobian: ")
# pprint(simplify(J[:,3]))
# print("5th column of jacobian: ")
# pprint(simplify(J[:,4]))
# print("6th column of jacobian: ")
# pprint(simplify(J[:,5]))

print("\n***************************************************************************")
print("                           Jacobian Matrix (J):                              ")
print("****************************************************************************\n")
# q_joint = Matrix([0.0, 0.0, float(pi/2), 0.0, float(pi), 0.0])
pprint(simplify(J))

print("\n***************************************************************************")
print("                           Jacobian Matrix 2 (J):                              ")
print("****************************************************************************\n")
# q_joint = Matrix([0.0, 0.0, float(pi/2), 0.0, float(pi), 0.0])
pprint(simplify(J2))



# Generating circle velocity trajectory using cylindrical coordinate equations of circle points
def generate_circle_velocity(th):
    y_dot = -4.0*pi*sin(th)
    z_dot = 4.0*pi*cos(th)
    X_dot = Matrix([0.0, y_dot, z_dot, 0.0, 0.0, 0.0]) # Generalized end_effector cartesian velocity components
    return X_dot


# Function to compute the inverse kinematics q_dot values from end-effector velocities.
def inverse_velocity_kinematics(X_dot, q_joint):
    J_inv = J.evalf(3,subs={q1: q_joint[0],q2: q_joint[1], q4: q_joint[2], q5: q_joint[3], q6: q_joint[4], q7: q_joint[5]}).inv()
    q_dot = J_inv * X_dot # Generalized joint velocity components from Jacobian inverse
    print("x_dot: ", X_dot, "is of shape: ", shape(X_dot))
    print("velocity IK: ", q_dot, "is of shape: ", shape(q_dot))
    return q_dot


# Function to compute new value of joint angle based on q_dot and previous angle
def update_joint_angle(q, q_dot):
    q = q + q_dot*delta_time
    return q


# Function to compute forward kinematics (position) to obtain the end-effector (Pen) (x,y,z) co-ords wrt base frame
def forward_position_kinematics(q):
    T = T_0n.evalf(subs={q1: q[0], q2: q[1], q4: q[2], q5: q[3], q6: q[4], q7: q[5]})
    return (T[0,3].round(4),T[1,3].round(4),T[2,3].round(4))

##############################################################
                        # HW 5 #
##############################################################

# the following values are from https://github.com/justagist/franka_panda_description/blob/master/robots/panda_arm.xacro and the solidworks assembly files provided. More info is in the report
m0 = 3.06
m1 = 4.97
m2 = .646
m3 = 3.22
m4 = 3.58
m5 = 1.23
m6 = 1.66
m7 = 7.355e-01

F = Matrix([5, 0, 0, 0, 0, 0])
print("F: ")
pprint(F)

R01 = T_01[0:2, 0:2]
R02 = T_02[0:2, 0:2]
R03 = T_03[0:2, 0:2]
R04 = T_04[0:2, 0:2]
R05 = T_05[0:2, 0:2]
R06 = T_06[0:2, 0:2]
R07 = T_0n[0:2, 0:2]

# transformation matrices for rough estimate of C.O.G of each link
T_01_rc1 = get_tf(q1, pi/2, 0, d1/2) #looks good 11/21
T_02_rc2 = T_01*get_tf(q2, -pi/2, 0, 0)
T_03_rc3 = T_02*get_tf(0, -pi/2, a3/2, d3/2) # q3 is fixed at zero degrees #looks good 11/21
T_04_rc4 = T_03*get_tf(q4, pi/2, -a3/2, 0) #looks good 11/21
T_05_rc5 = T_04*get_tf(q5, pi/2, 0, d5/2) #looks good 11/21
T_06_rc6 = T_05*get_tf(q6, -pi/2, a3/2, 0) #looks good 11/21
T_07_rc7 = T_06*get_tf(q7, 0, 0, -d7/2) #looks good 11/21

M0 = Matrix([
    [.3, 0.0, 0.0],
    [0.0, 0.3, 0.0],
    [0.0, 0.0, 0.3]
    ])
rc0 = Matrix([-23.99, -1.32, 69.61])

M1 = Matrix([
    [.070337, -.000139, .006772],
    [-.000139, .70661, .019169],
    [.006772, .019169, .009117]
    ])
rc1 = Matrix([.01, 15.92, 237.29])

M2 = Matrix([
    [.007962, -.003925, .010254],
    [-3.925e-3, 2.8110e-2, 7.0400e-04],
    [1.0254e-02, 7.0400e-04, 2.5995e-02]
    ])
rc2 = Matrix([-.10, -18.41, 412.75])

M3 = Matrix([
    [3.7242e-02, -4.7610e-03, -1.1396e-02],
    [-4.7610e-03, 3.6155e-02, -1.2805e-02],
    [-1.1396e-02, -1.2805e-02, 1.0830e-02]
])
rc3 = Matrix([35.95, -12.82, 596.17])

M4 = Matrix([
    [2.5853e-02, 7.7960e-03, -1.3320e-03],
    [7.7960e-03, 1.9552e-02, 8.6410e-03],
    [-1.3320e-03, 8.6410e-03, 2.8323e-02]
])
rc4 = Matrix([130.33, 14.68, 690.40])

M5 = Matrix([
    [3.5549e-02, -2.1170e-03, -4.0370e-03],
    [-2.1170e-03, 2.9474e-02, 2.2900e-04],
    [-4.0370e-03, 2.2900e-04, 8.6270e-03]
])
rc5 = Matrix([357.47, -57.13, 731.21])

M6 = Matrix([
    [1.9640e-03, 1.0900e-04, -1.1580e-03],
    [1.0900e-04, 4.3540e-03, 3.4100e-04],
    [-1.1580e-03, 3.4100e-04, 5.4330e-03]
])
rc6 = Matrix([457.06, 8.41, 795.49])

M7 = Matrix([
    [1.2516e-02, -4.2800e-04, -1.1960e-03],
    [-4.2800e-04, 1.0027e-02, -7.4100e-04],
    [-1.1960e-03, -7.4100e-04, 4.8150e-03]
])
rc7 = Matrix([541.84, 7.02, 826.46])

M8 = Matrix([
    [0.001, 0.0, 0.0],
    [0.0, 0.001, 0.0],
    [0.0, 0.0, 0.001]
])
rc8 = Matrix([596.94, .26, 818.52])

g = Matrix([0, 0, -9.8])
gt = transpose(g)


def KE(qdot, J):
#   T = .5 *    transpose(qdot) *   mi* transpose ( Jvi )      * Jvi      + transpose(Jwi)       * Ri*Ii*transpose(Ri)    * Jwi
    T = .5 * transpose(qdot) * ( m1*transpose(J[0:2, 0])*J[0:2, 0]+transpose(J[3:5,0])*R01*M1*transpose(R01)*J[3:5,0] \
                                    + m2*transpose(J[0:2, 1])*J[0:2, 1]+transpose(J[3:5,1])*R02*M2*transpose(R01)*J[3:5,1]\
                                    + m3*transpose(J[0:2, 2])*J[0:2, 2]+transpose(J[3:5,2])*R03*M3*transpose(R03)*J[3:5,2]\
                                    + m4*transpose(J[0:2, 3])*J[0:2, 3]+transpose(J[3:5,3])*R04*M4*transpose(R04)*J[3:5,3]\
                                    + m5*transpose(J[0:2, 4])*J[0:2, 4]+transpose(J[3:5,4])*R05*M5*transpose(R05)*J[3:5,4]\
                                    + m6*transpose(J[0:2, 5])*J[0:2, 5]+transpose(J[3:5,5])*R06*M6*transpose(R06)*J[3:5,5]\
                                    + m7*transpose(J[0:2, 6])*J[0:2, 6]+transpose(J[3:5,6])*R07*M7*transpose(R07)*J[3:5,7] ) * qdot
    return T

# print("shape of T01_rc1: ", shape(T_01_rc1[0:3,3]), "shape of gt: ", shape(gt))
V = m1*(gt*T_01_rc1[0:3,3]) + m2*(gt*T_02_rc2[0:3,3]) + m3*(gt*T_03_rc3[0:3,3]) + m4*(gt*T_04_rc4[0:3,3]) + m5*(gt*T_05_rc5[0:3,3]) + m6*(gt*T_06_rc6[0:3,3]) + m7*(gt*T_07_rc7[0:3,3])

def generateGravity(V):
    g1 = diff(V, q1)
    g2 = diff(V, q2)
    g4 = diff(V, q4)
    g5 = diff(V, q5)
    g6 = diff(V, q6)
    g7 = diff(V, q7)

    return (Matrix([g1, g2, g4, g5, g6, g7]))

dpdq = generateGravity(V)
print("gravity matrix: ")
pprint(dpdq)

joint1Torque = []
joint2Torque = []
joint3Torque = []
joint4Torque = []
joint5Torque = []
joint6Torque = []

##############################################################
                        # HW 5 #
##############################################################

if __name__ == '__main__':

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.axes.set_xlim(left=-10, right=90) 
    # ax.axes.set_ylim(bottom=-30, top=55) 
    # ax.axes.set_zlim(bottom=0, top=85) 
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')

    # figure, axis = plt.subplots(3,2)

    q_joint = Matrix([0.0, 0.0, float(pi/2), 0.0, float(pi), 0.0]) # initial joint angles of the robot
    
    i = 0

    # Looping cylindrical coordinate theta from pi/2 to 5pi/2 for full circle
    for theta in linspace(float(pi/2), float((5*pi)/2), num=N):

        i = i+1
        print("iteration: ", i)

        circle_vel_traj = generate_circle_velocity(theta)
        # print("circle_vel_traj: ")
        # pprint(circle_vel_traj)

        q_dot_joint = inverse_velocity_kinematics(circle_vel_traj, q_joint)
        # print("qdot joint: ")
        # pprint(q_dot_joint)

        q_joint = update_joint_angle(q_joint, q_dot_joint)
        # print("q_joint: ")
        # pprint(q_joint)

        (x_0p, y_0p, z_0p) = forward_position_kinematics(q_joint)

        # plot on circle as live points from forward kinematics for every delta_time seconds
        ax.scatter(x_0p,y_0p,z_0p)

        # calculate joint torques using updated angle and store in a list to plot later
        jointTorques = dpdq - transpose(J.evalf(3,subs={q1: q_joint[0],q2: q_joint[1], q4: q_joint[2], q5: q_joint[3], q6: q_joint[4], q7: q_joint[5]}))*F

        joint1Torque.append(jointTorques[0, 0])
        joint2Torque.append(jointTorques[0, 0])
        joint3Torque.append(jointTorques[0, 0])
        joint4Torque.append(jointTorques[0, 0])
        joint5Torque.append(jointTorques[0, 0])
        joint6Torque.append(jointTorques[0, 0])

        # plt.pause(delta_time)

    axis[0,0].plot(linspace(0, N, N), joint1Torque)
    axis[0,1].plot(linspace(0, N, N), joint2Torque)
    axis[1,0].plot(linspace(0, N, N), joint3Torque)
    axis[1,1].plot(linspace(0, N, N), joint4Torque)
    axis[2,0].plot(linspace(0, N, N), joint5Torque)
    axis[2,1].plot(linspace(0, N, N), joint6Torque)

    plt.show() 