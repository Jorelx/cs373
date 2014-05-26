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



def estimate(measurements, current):
    measurements.append(current)
    data_len = len(measurements)
    sum_x, sum_y = 0.0, 0.0
    for x, y in measurements:
        sum_x += x
        sum_y += y
    m_x = sum_x / data_len
    m_y = sum_y / data_len
    ui = [x - m_x for x, _ in measurements]
    vi = [y - m_y for _, y in measurements]
    
    Suu = sum(u**2 for u in ui)
    Suuu = sum(u**3 for u in ui)
    Suv = sum(u*v for u, v in zip(ui, vi))
    Suvv = sum(u*v**2 for u, v in zip(ui, vi))
    Svuu = sum(v*u**2 for u, v in zip(ui, vi))
    Svv = sum(v**2 for v in vi)
    Svvv = sum(v**3 for v in vi)
    
    try:
        #A*X = B
        A = matrix([[Suu, Suv],[Suv, Svv]])
        B = matrix([[0.5*(Suuu + Suvv)],[0.5*(Svvv + Svuu)]])
        X =  A.inverse() * B
    except ZeroDivisionError:
        return current
    
    uc, vc = (X.value[0][0], X.value[1][0])
    R = sqrt(uc**2 + vc**2 + (Suu + Svv)/float(len(measurements)))
    xc, yc = (uc + m_x, vc + m_y)
    
    theta = atan2(y - yc, x - xc)
    sum_delta_theta = sum([atan2(y2-yc, x2-xc) - atan2(y1-yc, x1-xc)
                           for ((x1, y1), (x2, y2)) in zip(measurements[:-1], measurements[1:])])
                           alpha = sum_delta_theta / (len(measurements) - 1)
                           theta += alpha
                           xy_estimate = (R*cos(theta) + xc, R*sin(theta) + yc)
    return xy_estimate

def measurement_prob(r, measurement):
    predicted_measurements = r.sense()
    sigma = r.measurement_noise **2
    # compute errors
    #error = abs(distance_between(measurement, (0.0, 0.0)) - distance_between(predicted_measurements, (0.0, 0.0)))
    #error = (exp(-(error ** 2 / (2.0 * sigma))) / sqrt(2.0 * pi * sigma))
    error = 1.0
    for i in range(len(measurement)):
        error = abs(measurement[i] - predicted_measurements[i])
        error *= (exp(-(error ** 2 / (2.0 * sigma))) / sqrt(2.0 * pi * sigma))
    return error

# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
        based on noisy (x, y) measurements."""
    if not OTHER:
        xy_estimate = measurement
        OTHER = {'measurements' : [measurement]}
    #OTHER = {'headings' : 0.0 }
    else:
        N = 100
        x,y = measurement[0], measurement[1]
        heading = atan2(y - OTHER['measurements'][-1][1], x - OTHER['measurements'][-1][0])
        #print angle_trunc(heading)
        if 'headings' not in OTHER:
            OTHER['headings'] = [heading]
            OTHER['measurements'].append(measurement)
            return measurement, OTHER
        OTHER['measurements'].append(measurement)
        sum_deta_dist = sum([distance_between(p1, p2)
                             for (p1, p2) in zip(OTHER['measurements'][:-1], OTHER['measurements'][1:])])
                             distance = sum_deta_dist / (len(OTHER['measurements']) - 1)
                             OTHER['headings'].append(heading)
                             sum_delta_heading = sum([abs(h2-h1)
                                                      for (h1, h2) in zip(OTHER['headings'][:-1], OTHER['headings'][1:])])
                                                      turning = sum_delta_heading / (len(OTHER['headings']) - 1)
                                                      print 'Test   ',len(OTHER['headings'])
                                                      print turning
                                                      if 'particles' not in OTHER:
                                                          OTHER['particles'] = []
                                                          for i in range(N):
                                                              r_x = random.random() * 100
                                                              r_y = random.random() * 100
                                                              r = robot(r_x, r_y, random.random() * 2.0 *pi)
                                                              OTHER['particles'].append(r)
                                                          OTHER['measurements'].append(measurement)
                                                          return measurement, OTHER
                                                      w = []
                                                      particles = OTHER['particles']
                                                      for p in particles:
                                                          p.move(2*pi / 34.0 , distance)
                                                          p.set_noise(0.0, 0.0, 0.05 * distance)
                                                          w.append(measurement_prob(p, measurement))
                                                      
                                                      p3 = []
                                                      index = int(random.random() * N)
                                                      beta = 0.0
                                                      mw = max(w)
                                                      for i in range(N):
                                                          beta += random.random() * 2.0 * mw
                                                          count = 0
                                                          while beta > w[index]:
                                                              beta -= w[index]
                                                              index = (index + 1) % N
                                                              count += 1
                                                          r = robot(particles[index].x, particles[index].y, particles[index].heading)
                                                          #r.set_noise(0.0, 0.0, particles[index].measurement_noise)
                                                          p3.append(r)
                                                      OTHER['particles'] = p3
                                                      
                                                      m_x, m_y, m_count = 0.0, 0.0, 0.0
                                                      
                                                      for p in OTHER['particles']:
                                                          m_count += 1
                                                          m_x += p.x
                                                          m_y += p.y
                                                      m_x /= m_count
                                                      m_y /= m_count
                                                      measurement = (m_x, m_y)
        xy_estimate = measurement#estimate(OTHER['measurements'], measurement)
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    #OTHER['measurements'].append(measurement)
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
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        #print 'turn ', target_bot.turning
        #print 'heading ', target_bot.heading
        #print 'test: ',position_guess
        #print 'real: ',(target_bot.x, target_bot.y)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
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




