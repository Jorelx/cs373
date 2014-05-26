# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position.
#
# ----------
# GRADING
#
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
        based on noisy (x, y) measurements."""
    xy_estimate = [0.0, 0.0]
    N = 500
    if not OTHER:
        OTHER = {'previous_measurement': None, 'heading': None,  'particles': None}
    elif not OTHER['particles'] and OTHER['heading']:
        p = []
        delta_x = abs(measurement[0] - OTHER['previous_measurement'][0]);
        delta_y = abs(measurement[1] - OTHER['previous_measurement'][1]);
        heading = atan2(delta_y, delta_x)
        turning = heading - OTHER['heading']
        distance = distance_between(measurement, OTHER['previous_measurement'])
        
        for i in range(N):
            x = random.uniform(0, measurement[0] *2)
            y = random.uniform(0, measurement[1] *2)
            r = robot(x, y, 0.5, 2*pi / 34.0, distance)
            measurement_noise = 0.05 * r.distance
            r.set_noise(0.0, 0.0, measurement_noise)
            p.append(r)
        OTHER['particles'] = p
    elif OTHER['particles']:
        p = OTHER['particles']
        w = []
        for r in p:
            #r.heading = head
            #r.distance = distance
            #r.turning = turning
            sigma = r.measurement_noise **2
            r.move_in_circle()
            estimated = r.sense()
            error = 1.0
            for i in range(len(estimated)):
                error = abs(measurement[i] - estimated[i])
                error *= (exp(-(error ** 2 / (2.0 * sigma))) / sqrt(2.0 * pi * sigma))
            w.append(error)
        
        #total_w = sum(w)
        #print 'Total_:', total_w
        #print total_w
        #for i in range(N):
        #    w[i] /= total_w
        # Resampling
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        #print mw
        for i in range(N):
            beta += random.random() * 2.0 * mw
            count = 0
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
                count += 1
            r = robot(p[index].x, p[index].y, p[index].heading, p[index].turning, p[index].distance)
            r.set_noise(0.0, 0.0, p[index].measurement_noise)
            p3.append(r)
        p = p3
        
        m_heading, m_distance, m_turning, m_x, m_y, m_count = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        
        for i in range(N):
            m_count += 1
            m_x += p[i].x
            m_y += p[i].y
            m_heading += p[i].heading
            m_distance += p[i].distance
            m_turning += p[i].turning
        
        m_x /= m_count
        m_y /= m_count
        #m_heading /= m_count
        #m_distance /= m_count
        #m_turning /= m_count
        #estimated_r = robot(m_x, m_y, m_heading, m_turning, m_distance)
        #estimated_r.move_in_circle()
        xy_estimate = (m_x, m_y)
    #print xy_estimate
    
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    #OTHER = None
    if OTHER['previous_measurement']:
        delta_x = abs(measurement[0] - OTHER['previous_measurement'][0]);
        delta_y = abs(measurement[1] - OTHER['previous_measurement'][1]);
        OTHER['heading'] = atan2(delta_x, delta_y)
    OTHER['previous_measurement'] = measurement
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    #xy_estimate = (3.2, 9.1)
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
    while not localized and ctr <= 100:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos(measurement, OTHER)
        print (target_bot.x, target_bot.y)
        print position_guess
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 100:
            print "Sorry, it took you too many steps to localize the target."
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




