
from math import *
import random


from math import *
import random

# helper function to map all angles onto [-pi, pi]
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class robot:
    
    def __init__(self, x = 0.0, y = 0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
        """This function is called when you create a new robot. It sets some of
            the attributes of the robot, either to their default values or to the values
            specified when it is created."""
        self.x = x
        self.y = y
        self.heading = heading
        self.turning = turning # only applies to target robots who constantly move in a circle
        self.distance = distance # only applies to target bot, who always moves at same speed.
        self.turning_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0
    
    
    def set_noise(self, new_t_noise, new_d_noise, new_m_noise):
        """This lets us change the noise parameters, which can be very
            helpful when using particle filters."""
        self.turning_noise    = float(new_t_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)
    
    
    def move(self, turning, distance, tolerance = 0.001, max_turning_angle = pi):
        """This function turns the robot and then moves it forward."""
        # apply noise, this doesn't change anything if turning_noise
        # and distance_noise are zero.
        turning = random.gauss(turning, self.turning_noise)
        distance = random.gauss(distance, self.distance_noise)
        
        # truncate to fit physical limitations
        turning = max(-max_turning_angle, turning)
        turning = min( max_turning_angle, turning)
        distance = max(0.0, distance)
        
        # Execute motion
        self.heading += turning
        self.heading = angle_trunc(self.heading)
        self.x += distance * cos(self.heading)
        self.y += distance * sin(self.heading)
    
    def move_in_circle(self):
        """This function is used to advance the runaway target bot."""
        self.move(self.turning, self.distance)
    
    def sense(self):
        """This function represents the robot sensing its location. When
            measurements are noisy, this will return a value that is close to,
            but not necessarily equal to, the robot's (x, y) position."""
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))
    
    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)

def calculate_w(r, measurement):
    predicted_measurement = r.sense()
    error = 1.0
    for i in range(len(measurement)):
        error_bearing = abs(measurement[i] - predicted_measurement[i])
        error *= (exp(- (error_bearing ** 2) / ((0.01) ** 2) / 2.0) /
        sqrt(2.0 * pi * ((0.01) ** 2)))
    return error

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

def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
        based on noisy (x, y) measurements."""
    xy_estimate = (.0, .0)
    if not OTHER:
        OTHER = {"measurements": []}
        OTHER["measurements"].append(measurement)
    else:
        N = 1000
        measurements = OTHER["measurements"]
        measurements.append(measurement)
        p = []
        for i in range(N):
            x = random.gauss(measurements[0][0], -0.075)
            y = random.gauss(measurements[0][1], -0.075)
            r = robot(x, y, 0.5)
            p.append(r)
        m = []
        for i in range(len(measurements)-1):
            distance = distance_between(measurements[i], measurements[i+1])
            turning = 2*pi / 34.0
            m.append((turning, distance))
        for t in range(len(m)):
            p2 = []
            for r in p:
                r.move(turning, distance)
                p2.append(r)
            p = p2
            
            w = []
            for r in p:
                w.append(calculate_w(r,measurements[t+1]))
                    
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
        xy_estimate = get_position(p)
#print xy_estimate

    #xy_estimate = (1.1, 4.3)
    return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any
# information that you want.
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 5:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        
        #target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        print "Guess: ", position_guess
        print "Real: ", true_position
        print "MMM: ", measurement
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 5:
            print "Sorry, it took you too many steps to localize the target."
        target_bot.move_in_circle()
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
        assumes that eventually the target bot will eventually return to that
        position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)




