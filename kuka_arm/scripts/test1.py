import rospy
from sympy import *
import tf
import numpy as np

a, b, c = symbols('a b c')

q1, q2, q3, q4, q5, q6, q7, q8 = symbols('uh_1:9')

x = {a:1, b:2, c:3, q1:4}

q = (a + b).subs(x)	# this works
w = (b*b).subs(x)

q1 = q1.subs(x)

#print("\n%d\t%d\n" % (q, w))

#print("q1: %r\t\tq8: %r" % (q1, q8)

EE_pos = [2.16135, -1.42635, 1.55109]
WC_pos = [1.89451, -1.44302, 1.69366]
delta = [EE_pos[0]-WC_pos[0], EE_pos[1]-WC_pos[1], EE_pos[2]-WC_pos[2]]

displacement = sqrt((delta[0]**2)+(delta[1]**2)+(delta[2]**2))

print("displacement = ", displacement)



A = Matrix([	[0,	1,	2,	3],
		[4,	5,	6,	7],
		[8,	9,	10,	11],
		[12,	13,	14,	15]])

C = np.array(A)

B = A[0:3,3]

print("C = ", C)



R = tf.transformations.euler_matrix(1, 0, 0, 'szyx')
print("R =\n%r\n\nA =\n%r\n" % (R, A))
