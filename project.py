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


from math import *
import random
import bisect

def create_particles(N, world = 10.0):
    particles = []
    for _ in range(N):
        x = random.random() * world
        y = random.random() * world
        heading = random.random() * 2 * pi
        myrobot = robot(x, y, heading)
        particles.append(myrobot)
    #print len(particles)
    return particles

def mean_estimation(particles, w):
    m_x, m_y = 0.0, 0.0
    #print len(particles)
    for i in range(len(particles)):
        m_x += (particles[i].x)
        m_y += (particles[i].y)
    m_x /= len(particles)
    m_y /= len(particles)
    return (m_x, m_y)

def compute_weight(measurement, p_measurement):
    sigma2 = 0.05 * 1.5
    w = 1.0
    for i in range(2):
        error = measurement[i] - p_measurement[1]
        w *= exp(- ((error) ** 2) / (sigma2) / 2.0) / sqrt(2.0 * pi * (sigma2))
    ##w *= exp(-(error ** 2 / (2 * sigma2)))
    #print w
    return w

#def compute_weight(measurement, p_measurement):
#    sigma2 = (0.9) ** 2
#    w = 1.0
#    a = distance_between(measurement , (0.0, 0.0))
#    b = distance_between(p_measurement , (0.0, 0.0))
        #for i in range(len(measurement)):
        #    error = a - b
    
    #    w = exp(-(error ** 2 / (2 * sigma2)))
    #print measurement, '====', p_measurement, ' weight: ', w
#return w

#def calculate_distribution(weights):
#    accum = 0.0
#    distribution = []
#    for w in weights:
#        accum += w
#        distribution.append(w)
#    return distribution

#def pick_particle(particles, distribution):
#    try:
#        return particles[bisect.bisect_left(distribution, random.uniform(0.0, 1.0))]
#    except IndexError:
#        return None

def estimate_next_pos(measurement, heading, turning, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
        based on noisy (x, y) measurements."""
    N = 500
    if not OTHER:
        OTHER = {'particles' : create_particles(N), 'last_m': measurement}
    else:
        distance = distance_between(OTHER['last_m'], measurement)
        for p in OTHER['particles']:
            #p.measurement_noise = 0.05 * distance
            p.move(turning, distance)
    #print len(OTHER['particles'])
    w = []
    for p in OTHER['particles']:
        w.append(compute_weight(measurement, (p.x, p.y)))
    xy_estimate = mean_estimation(OTHER['particles'], w)

#print xy_estimate
    new_particles = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p = OTHER['particles'][index]
        r = robot(p.x, p.y, heading) #prob we need to calculate de heading
        new_particles.append(r)
#p = p3

#nu = sum(w)
    #print nu
#   for i in range(len(OTHER['particles'])):
#       if w[i] != 0:
#            w[i] /= nu
#distribution = calculate_distribution(w)
#print len(w)
#new_particles = []
#   for _ in range(N):
        #p = pick_particle(OTHER['particles'], distribution)
        #if p:
        #    r = robot(p.x, p.y, p.heading) #prob we need to calculate de heading
        #    new_particles.append(r)
        #else:
#    new_particles.append(create_particles(1)[0])
    
    OTHER['particles'] = new_particles
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    #OTHER['measurements'].append(measurement)
#xy_estimate = (0,0)

    return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    while not localized and ctr <= 100:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement,target_bot.heading, target_bot.turning, OTHER)
        print 'Real: ', (target_bot.x, target_bot.y)
        print 'guess: ',position_guess
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        #print 'Error: ', error
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 100:
            print "Sorry, it took you too many steps to localize the target."
        target_bot.move_in_circle()
    return localized

test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)