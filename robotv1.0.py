# --------------
# USER INSTRUCTIONS
#
# Now you will put everything together.
#
# First make sure that your sense and move functions
# work as expected for the test cases provided at the
# bottom of the previous two programming assignments.
# Once you are satisfied, copy your sense and move
# definitions into the robot class on this page, BUT
# now include noise.
#
# A good way to include noise in the sense step is to
# add Gaussian noise, centered at zero with variance
# of self.bearing_noise to each bearing. You can do this
# with the command random.gauss(0, self.bearing_noise)
#
# In the move step, you should make sure that your
# actual steering angle is chosen from a Gaussian
# distribution of steering angles. This distribution
# should be centered at the intended steering angle
# with variance of self.steering_noise.
#
# Feel free to use the included set_noise function.
#
# Please do not modify anything except where indicated
# below.

from math import *
import random

# --------
#
# some top level parameters
#

max_steering_angle = pi / 4.0 # You do not need to use this value, but keep in mind the limitations of a real car.
bearing_noise = .0 # Noise parameter: should be included in sense function.
steering_noise = 0.0 # Noise parameter: should be included in move function.
distance_noise = .05 * 1.5 # Noise parameter: should be included in move function.

tolerance_xy = 0.01 # Tolerance for localization in the x and y directions.
tolerance_orientation = 0.25 # Tolerance for orientation.


# --------
#
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!

landmarks  = [0.0, 0.0] # position of 4 landmarks in (y, x) format.
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"

# ------------------------------------------------
#
# this is the robot class
#
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class robot:
    
    # --------
    # init:
    #    creates robot and initializes location/orientation
    #
    
    def __init__(self, x, y, heading, turning = 2*pi/10, distance = 1.0):
        self.x = x # initial x position
        self.y = y # initial y position
        self.heading = heading # initial orientation
        self.turning_noise  = 0.0 # initialize bearing noise to zero
        self.distance_noise = 0.0 # initialize steering noise to zero
        self.measurement_noise = 0.0 # initialize distance noise to zero
        self.distance = distance
        self.turning = turning
    
    # --------
    # set:
    #    sets a robot coordinate
    #
    
    #def set(self, new_x, new_y, new_orientation):
        
        #   if new_orientation < 0 or new_orientation >= 2 * pi:
        #    raise ValueError, 'Orientation must be in [0..2pi]'
        #self.x = float(new_x)
        #self.y = float(new_y)
        #self.orientation = float(new_orientation)
    
    # --------
    # set_noise:
    #    sets the noise parameters
    #
    def set_noise(self, new_t_noise, new_d_noise, new_m_noise):
        """This lets us change the noise parameters, which can be very
            helpful when using particle filters."""
        self.turning_noise    = float(new_t_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)
    
    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #
    
    def measurement_prob(self, measurements):
        
        # calculate the correct measurement
        predicted_measurements = self.sense() # Our sense function took 0 as an argument to switch off noise.
        
        
        # compute errors
        #print 'noise ', self.measurement_noise
        error = 1.0
        for i in range(len(measurements)):
            error_bearing = abs(measurements[i] - predicted_measurements[i])
            #error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
            
            
            # update Gaussian
            error *= (exp(- (error_bearing ** 2) / ((1.0 - self.measurement_noise) ** 2) / 2.0) /
                      sqrt(2.0 * pi * ((self.measurement_noise) ** 2)))
        return error
    
    def __repr__(self): #allows us to print robot attributes.
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y),
                                                str(self.orientation))
    def move_in_circle(self):
        """This function is used to advance the runaway target bot."""
        self.move(self.turning, self.distance)
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################
    
    # --------
    # move:
    #
    def move(self, turning, distance, tolerance = 0.001, max_turning_angle = pi): # Do not change the name of this function
        
        # ADD CODE HERE
        
        
        turning = random.gauss(turning, self.turning_noise)
        distance = random.gauss(distance, self.distance_noise)
        
        # truncate to fit physical limitations
        turning = max(-max_turning_angle, turning)
        turning = min( max_turning_angle, turning)
        distance = max(0.0, distance)
        
        # Execute motion
        heading = self.heading + turning
        heading = angle_trunc(heading)
        x = self.x + distance * cos(self.heading)
        y = self.y + distance * sin(self.heading)
        result = robot(x, y, heading, turning, distance)
        result.set_noise(self.turning_noise, self.distance_noise, self.measurement_noise)
        return result
    
    
    # copy your code from the previous exercise
    # and modify it so that it simulates motion noise
    # according to the noise parameters
    #           self.steering_noise
    #           self.distance_noise
    
    # --------
    # sense:
    #
    def sense(self): #do not change the name of this function
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))
    def __repr__(self):
        return '[%.5f, %.5f]'  % (self.x, self.y)
# copy your code from the previous exercise
# and modify it so that it simulates bearing noise
# according to
#           self.bearing_noise

############## ONLY ADD/MODIFY CODE ABOVE HERE ####################

# --------
#
# extract position from a particle set
#

def get_position(p):
    x = 0.0
    y = 0.0
    #orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        #orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi))
        #                + p[0].orientation - pi)
    return [x / len(p), y / len(p)]

# --------
#
# The following code generates the measurements vector
# You can use it to develop your solution.
#


def generate_ground_truth(motions, myrobot):
    
    #myrobot = robot()
    #myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
    
    Z = []
    T = len(motions)
    
    for t in range(T):
        myrobot = myrobot.move(motions[t][0], motions[t][1])
        Z.append(myrobot.sense())
    print Z
    return [myrobot, Z]

# --------
#
# The following code prints the measurements associated
# with generate_ground_truth
#

def print_measurements(Z):
    
    T = len(Z)
    
    print 'measurements = [[%.8s, %.8s],' % \
        (str(Z[0][0]), str(Z[0][1]))
    for t in range(1,T-1):
        print '                [%.8s, %.8s],' % \
            (str(Z[t][0]), str(Z[t][1]))
    print '                [%.8s, %.8s]]' % \
        (str(Z[T-1][0]), str(Z[T-1][1]))

# --------
#
# The following code checks to see if your particle filter
# localizes the robot to within the desired tolerances
# of the true position. The tolerances are defined at the top.
#

def check_output(final_robot, estimated_position):
    
    error_x = abs(final_robot.x - estimated_position[0])
    error_y = abs(final_robot.y - estimated_position[1])
    correct = error_x < tolerance_xy
    return correct



def particle_filter(motions, measurements, N=1000): # I know it's tempting, but don't change N!
    # --------
    #
    # Make particles
    #
    
    p = []
    for i in range(N):
        x = random.gauss(measurements[0][0], 0.05 * 1.5)#random.random() * 10.0
        y = random.gauss(measurements[0][1], 0.05 * 1.5)#random.random() * 10.0
        heading = random.random() * 2 * pi
        r = robot(x, y, heading)
        r.set_noise(0.0, 0.0, distance_noise)
        p.append(r)
    
    # --------
    #
    # Update particles
    #
    
    for t in range(len(motions)):
        
        # motion update (prediction)
        p2 = []
        for i in range(N):
            p2.append(p[i].move(motions[t][0], motions[t][1]))
        p = p2
        
        # measurement update
        w = []
        for i in range(N):
            w.append(p[i].measurement_prob(measurements[t]))
        
        # resampling
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(p[index])
        p = p3
    
    return get_position(p)

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def estimate_next_pos(measurement, OTHER = None):
    xy_estimated = measurement
    if not OTHER:
        OTHER = {'measurements': []}
        OTHER['measurements'].append(measurement)
    else:
        N = 1000
        measurements = OTHER['measurements']
        measurements.append(measurement)
            #if 'particles' not in OTHER:
        particles = []
        for i in range(N):
            x = myrobot.x
            y = myrobot.y
            heading = 2*pi / 34.0#random.random() * 2 * pi
            r = robot(x, y, 0.5)
            r.set_noise(0.0, 0.0, distance_noise)
            particles.append(r)
        OTHER['particles'] = particles
        #particles = OTHER['particles']

        p = []
        w = []
        distance = distance_between(measurements[-1], measurements[-2])
        for i in range(N):
            
            p.append(particles[i].move(2. * pi / 34.0, 1.5))
            p[i].set_noise(0.0, 0.0, distance_noise)
            w.append(p[i].measurement_prob(measurement))

        p2 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p2.append(p[index])
        p = p2
        OTHER['particles'] = p2
        xy_estimated = get_position(p)
    return xy_estimated, OTHER


def demo_grading(target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.03 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 10:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos(measurement, OTHER)
        print 'Real: ', (target_bot.x, target_bot.y)
        print 'MEm: ', measurement
        print 'Guess: ',position_guess

        true_position = (target_bot.x, target_bot.y)
        
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 10:
            print "Sorry, it took you too many steps to localize the target."
        target_bot = target_bot.move(2*pi / 34.0, 1.5)
    return localized


test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

#demo_grading(test_target)

## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out.
##
## You can test whether your particle filter works using the
## function check_output (see test case 2). We will be using a similar
## function. Note: Even for a well-implemented particle filter this
## function occasionally returns False. This is because a particle
## filter is a randomized algorithm. We will be testing your code
## multiple times. Make sure check_output returns True at least 80%
## of the time.



## --------
## TEST CASES:
##
##1) Calling the particle_filter function with the following
##    motions and measurements should return a [x,y,orientation]
##    vector near [x=93.476 y=75.186 orient=5.2664], that is, the
##    robot's true location.
##
#motions = [[2. * pi / 10, 20.] for row in range(8)]
measurements = [[4.746936, 3.859782, 3.045217, 2.045506],
                [3.510067, 2.916300, 2.146394, 1.598332],
                [2.972469, 2.407489, 1.588474, 1.611094],
                [1.906178, 1.193329, 0.619356, 0.807930],
                [1.352825, 0.662233, 0.144927, 0.799090],
                [0.856150, 0.214590, 5.651497, 1.062401],
                [0.194460, 5.660382, 4.761072, 2.471682],
                [5.717342, 4.736780, 3.909599, 2.342536]]

#print particle_filter(motions, measurements)

## 2) You can generate your own test cases by generating
##    measurements using the generate_ground_truth function.
##    It will print the robot's last location when calling it.
##
##
number_of_iterations = 1
motions = [[2. * pi / 34.0, 1.5] for row in range(number_of_iterations)]
##
myrobot = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)

myrobot.set_noise(0.0, 0.0, 0.05 * 1.5)
x = generate_ground_truth(motions, myrobot)
final_robot = x[0]
measurements = x[1]
estimated_position = particle_filter(motions, measurements)
#print_measurements(measurements)
print 'Error:           ', distance_between((final_robot.x,final_robot.y), (estimated_position[0], estimated_position[1]))
print 'Ground truth:    ', final_robot
print 'Particle filter: ', estimated_position
print 'Code check:      ', check_output(final_robot, estimated_position)



