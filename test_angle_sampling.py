# rrt* steps
# sample new node within a distance from one of the nodes
# node_i = randint
# dx,dy = rand(),rand()
# new_node = node_i + [dx,dy]
# this would require to calculate the inverse kinematics.
# And not sure if we can nicely put all the constraints like angle constraints in here.


# or with angles
# dtheta = rand.choice([-1,0,1])
# new_node = node_i + dtheta
# and then you want this to be in velocities
# for one next step, you have 3 angle samples, 27 possibilities
# can prefer the same movement, or increase prob, for similar velocities.
#  which is actually the same as sampling acceleration per angle