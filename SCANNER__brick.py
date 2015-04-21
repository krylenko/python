# (c) daniel ford, daniel.jb.ford@gmail.com

# controller for one-dimensional ultrasonic scanner

#!/usr/bin/env python

import nxt.locator
from nxt.sensor import *
from nxt.motor import *
from numpy import *

#print 'Touch:', Touch(b, PORT_1).get_sample()
#print 'Sound:', Sound(b, PORT_2).get_sample()
#print 'Light:', Light(b, PORT_3).get_sample()

# print instructions
def instruct():

  print " "
  print "************** TEST **********************"
  print "open - run a motor in open-loop mode" 
  print "id - step input w/ 5 seconds of data logging"
  print "ctrl - uncompensated control to a position"
  print "pid - PID control test w/ logging" 
  print " "

  print " "
  print "********** MANUAL CONTROL ****************"
  print "l - left | r - right | u - up | d - down"
  print "s - stop | read      | reset  | q - quit program"

  print " "
  print "********** AUTO CONTROL ******************"
  print "fscan - flat scan | scan - full scan" 
  print "lswp - left sweep | rswp - right sweep" 
  print " "

  print " "
  print "i - print instructions"  
  print " "

# handle key input
def keyInput(key, brick):

  if key == 'l':
    brick.left()
  if key == 'r':
    brick.right()
  if key == 'u':
    brick.up()
  if key == 'd':
    brick.down()

  if key == 'i':
    instruct()  
  
  if key == 's':
    brick.stop()
  if key == 'reset':
    brick.reset()
  if key == 'q':
    brick._quitBrick()
  
  if key == 'fscan':
    brick.flat_scan()

  if key == 'open':
    brick.open_loop()

  if key == 'rtz':
    brick.rtz(brick.heading)
  
  if key == 'lswp':
    left = True
    brick.sweep(left)

  if key == 'rswp':
    brick.sweep()

  if key == 'id':
    brick.sysid()

  if key == 'ctrl':
    brick.ctrl()
    
  if key == 'pid':
    brick.pid(brick.heading)
    
# brick w/ motors, sensor
class Brick():

  # set up NXT brick
  def __init__(self):

    self.brick = nxt.locator.find_one_brick()

    self.heading = Motor(self.brick, PORT_A)
    self.elevation = Motor(self.brick, PORT_B)
    self.test = Motor(self.brick, PORT_C)
    
    self.sensor = Ultrasonic(self.brick, PORT_1)

    self.heading.reset_position(relative=False)
    self.elevation.reset_position(relative=False)
    self.test.reset_position(relative=False)
    
    self.distRes = 100    # count spacing for distance measurement
    #self.interval = 0.01
    self.interval = 0.001
    #self.prec = 0.05
    self.prec = 0.05
    self.vcirc = array([0,0,0,0,0])

  # return heading
  def theta(self, motor):
    return (motor.get_tacho()).rotation_count     
    
  # return elevation
  def elevate(self):
    return (self.elevation.get_tacho()).rotation_count

  def read(self):
    print 'heading:', self.theta(self.heading), ' elevation:', self.elevate(), ' distance:', self.sensor.get_sample() 

  def reset(self):  
    self.heading.reset_position(relative=False)
    self.elevation.reset_position(relative=False)
    self.test.reset_position(relative=False)
    
  def left(self,power=80):
    self.heading.run(-1*power)

  def right(self,power=80):
    self.heading.run(power)    

  def up(self):
    self.elevation.run(-80)  

  def down(self):
    self.elevation.run(80)    

  def stop(self):
    self.heading.brake()
    self.elevation.brake()
    self.test.brake()
    
  def rtz(self, motor):
    if motor == self.heading:
      if self.theta(self.heading) < 0:
        while self.theta(self.heading) < 0:
          #self.heading.run(60)
          self.right(60)
        self.heading.brake()  
      if self.theta(self.heading) > 0:
        while self.theta(self.heading) > 0:
          #self.heading.run(-60)
          self.left(60)
        self.heading.brake()
        
  # run open-loop: need motor, direction, power, duration  
  def open_loop(self):
    duration = 0
    #interval = 0.01
    #prec = 0.01
    print "Choose motor (h, e, t): "
    motor = raw_input()
    if motor == 't':
      direction = 1
      motor = self.test
    if motor == 'h':
      print "Enter direction (l or r): "
      direction = raw_input()
      if direction == 'l':
        direction = -1
      if direction == 'r':
        direction = 1
      motor = self.heading
    if motor == 'e':
      print "Enter direction (u or d): "
      direction = raw_input()
      if direction == 'u':
        direction = -1
      if direction == 'd':
        direction = 1
      motor = self.elevation
    print "Enter power (1-100): "
    power = raw_input()
    power = int(power)
    if power > 100:
      power = 100
    print "Enter duration in ms: "
    duration = raw_input()
    duration = float(duration)
    # run motor
    self.reset()
    elapsed = 0
    start = time.clock()
    pos_curr = self.theta(motor)
    pos_last = pos_curr
    t_last = 0
    vel = 0
    t_del = 0
    data = array([[elapsed, vel, pos_curr]])
    motor.run(power*direction)
    while elapsed < (duration/1000.):
      elapsed = time.clock() - start
      t_del = elapsed - t_last
      if t_del > (self.interval-self.prec) and t_del < (self.interval+self.prec):
        pos_curr = self.theta(motor)
        vel = (pos_curr - pos_last)/t_del
        data = concatenate((data,[[elapsed, vel, pos_curr]]),axis=0)
        t_last = elapsed
        pos_last = pos_curr

    motor.idle()
    print "elapsed time: ",elapsed
    savetxt('open.txt',data,fmt="%g")
  
  # make a single scan from 45 to -45 degrees and record distances
  def flat_scan(self):
    print "Enter # of counts: "
    counts = raw_input()
    counts = int(counts)
    direction = 1
    
    # variables for scanning
    total = counts*2
    jump = total/self.distRes
    scan = zeros((jump+1),dtype=int8)
    #print total, jump, len(scan)
    
    # turn right from center
    if self.theta() >= 0 and self.theta() < counts and direction == 1:
      self.right(100)
      while self.theta() <= counts:
        pass
      self.heading.brake()
      direction = 0

    # sweep across field of view  
    if direction == 0 and self.theta() > (-1*counts):
      self.left(100)
      idx = 0
      scan[idx] = self.sensor.get_sample()
      idx = 1
      first = False
      while self.theta() >= (-1*counts):
        if (self.theta()+counts) <= (total-self.distRes) and first == False:
          first = True
          total = total-self.distRes
        if first == True:
          #print "scanned ",idx+1
          scan[idx] = self.sensor.get_sample()
          idx = idx+1
          first = False
      self.heading.brake()
    
    # reverse array to match my POV
    print scan[::-1]

  def sweep(self, left=False):
    if left == True:
      self.heading.run(-100);
      while self.theta() > -2900:
        pass
      self.heading.idle()
    if left == False:
      self.heading.run(100);
      while self.theta() < 2900:
        pass
      self.heading.idle()    

  # run test motor to measure sys response    
  def sysid(self):
    self.reset()
    '''
    print "Enter speed (0-100): "
    speed = raw_input()
    speed = int(speed)
    '''
    print "Enter step length (ms): "
    duration = raw_input()
    duration = float(duration)

    #motor = self.test
    motor = self.heading
    vlo = 0
    vhi = 100
    vrng = vhi-vlo
    #refspeed = 885 # 100% power
    refspeed = 575 # 80% power
    vnorm = 1./refspeed
    K = 7
    
    # set up vars
    t_total = 0
    t_total_last = 0
    t_delta = 0
    vel = 0
    error = 0
    pos_curr = self.theta(motor) 
    pos_last = pos_curr
    data = array([[t_total, vel, refspeed, error, pos_curr,0]])
    t_start = time.clock()
    vidx = 0
    
    # run motor for specified duration  
    while t_total < (duration/1000.):
      t_current = time.clock()
      t_total = t_current - t_start
      t_delta = t_current - t_total_last
        
      # update controller at specified "sampling" interval
      if t_delta > (self.interval-self.prec) and t_delta < (self.interval+self.prec):
        pos_curr = self.theta(motor)
        self.vcirc[vidx] = (pos_curr - pos_last)/t_delta
        vidx = vidx + 1
        if vidx == len(self.vcirc):
          vidx = 0
        vel = mean(self.vcirc)
        error = refspeed - vel
        outspeed = int(K*error*vnorm*vrng)
        if outspeed > 0:
          outspeed = vlo + outspeed
        if outspeed < 0:
          outspeed = -vlo + outspeed
        if outspeed > 100:
          outspeed = 100
        if outspeed < -100:
          outspeed = -100
        data = concatenate((data,[[t_total, vel, refspeed, error, pos_curr, outspeed]]),axis=0)
        motor.run(outspeed)
        pos_last = pos_curr
        t_delta = 0
        t_total_last = t_total
    
    motor.idle()

    savetxt('sysID.txt', data, fmt="%g")

  # control function test fixture
  def ctrl(self):

    speed = 575 # 80% power
    
    print "Enter position: "
    target = raw_input()
    target = int(target)    
    
    self.control(speed, target, self.heading, PID=True)
    
  # control function
  # if PID=False, runs with simple closed-loop control
  def control(self, speed, target, motor, PID):

    #self.reset()
    safe = target
    vlo = 0
    vhi = 100
    vrng = vhi-vlo
    refspeed = speed
    vnorm = 1./refspeed
    radnorm = (pi/4)/3000   # rads/count (est.)
    
    K = 7
    precision = 7
    Kp = 0.3480
    Ki = 5.22
    Kd = 0.0058
    
    #Kp = 0.5
    #Ki = 0.05
    #Kd = 4
    
    # set up vars
    t_total = 0
    t_total_last = 0
    t_delta = 0
    vel = 0
    vidx = 0
    error = 0
    
    pos_curr = self.theta(motor) 
    pos_last = pos_curr    
    p_error = target - pos_curr
    if p_error*-1 > 0:
      direction = -1
    else:
      direction = 1

    scanmax = 35
    distance = self.sensor.get_sample()
    angle = pos_curr*radnorm 
    X = distance * sin(angle)
    Y = distance * cos(angle)
    scan_space = 50
    scan_ct = 0
    scans = array([[distance, angle, X, Y]])      
      
    err_last = 0
    err_sum = 0
    sum_limit_scale = 9.4
    sum_max = refspeed/sum_limit_scale
    sum_min = -sum_max      
      
    t_start = time.clock()
    data = array([[t_total, target, pos_curr, vel, error, refspeed]])
    sweeps = 1
    
    for i in range(1,sweeps+1):
      # run motor to target    
      while abs(p_error) > precision:
        t_current = time.clock()
        t_total = t_current - t_start
        t_delta = t_current - t_total_last
        scan_ct = scan_ct+1
        
        # update controller at specified "sampling" interval
        if t_delta > (self.interval-self.prec) and t_delta < (self.interval+self.prec):        
          pos_curr = self.theta(motor)
          p_error = target - pos_curr
          if p_error*-1 > 0:
            direction = -1
          else:
            direction = 1
          self.vcirc[vidx] = (pos_curr - pos_last)/t_delta
          vidx = vidx + 1
          if vidx == len(self.vcirc):
            vidx = 0
          vel = mean(self.vcirc)
          error = refspeed*direction - vel
          if PID == True:
            err_sum = err_sum + error
            int_term = Ki*err_sum
            if int_term > sum_max:
              int_term = sum_max
            if int_term < sum_min:
              int_term = sum_min
            outspeed = K*(Kp*error + int_term + Kd*(error-err_last))*vnorm*vrng
          else:
            outspeed = int(K*error*vnorm*vrng)
          if outspeed > 0:
            outspeed = vlo + outspeed
          if outspeed < 0:
            outspeed = -vlo + outspeed
          if outspeed > 100:
            outspeed = 100
          if outspeed < -100:
            outspeed = -100
          if scan_ct >= scan_space:
            distance = self.sensor.get_sample()
            if distance > scanmax:
              distance = scanmax
            angle = pos_curr*radnorm 
            X = distance * sin(angle)
            Y = distance * cos(angle)
            scans = concatenate((scans,[[distance, angle, X, Y]]),axis=0)  
            scan_ct = 0
          data = concatenate((data,[[t_total, target, pos_curr, vel, error, refspeed]]),axis=0)
          motor.run(outspeed)

          pos_last = pos_curr
          t_delta = 0
          t_total_last = t_total
          err_last = error
          
      # update target
      target = -1*target  
      p_error = target - pos_curr
      if p_error*-1 > 0:
        direction = -1
      else:
        direction = 1
        
    motor.brake()
    savetxt('ctrl.txt', data, fmt="%g", delimiter = '\t')
    savetxt('scans.txt', scans, fmt="%g", delimiter='\t')
    
  # run PID controller to measure sys response    
  def pid(self,motor):
    self.reset()
 
    print "Enter step length (ms): "
    duration = raw_input()
    duration = float(duration)

    #motor = self.test
    motor = self.heading
    vlo = 0
    vhi = 100
    vrng = vhi-vlo
    #refspeed = 885 # 100% power
    refspeed = 575 # 80% power
    vnorm = 1./refspeed
    K = 7
    Kp = .5
    Ki = 0.01
    Kd = 0
    
    # set up vars
    t_total = 0
    t_total_last = 0
    t_delta = 0
    vel = 0
    error = 0
    pos_curr = self.theta(motor) 
    pos_last = pos_curr
    data = array([[t_total, vel, refspeed, error, pos_curr,0]])
    t_start = time.clock()
    vidx = 0
    err_last = 0
    err_sum = 0
    sum_limit_scale = 9.4
    sum_max = refspeed/sum_limit_scale
    sum_min = -sum_max
    
    # run motor for specified duration  
    while t_total < (duration/1000.):
      t_current = time.clock()
      t_total = t_current - t_start
      t_delta = t_current - t_total_last
        
      # update controller at specified "sampling" interval
      if t_delta > (self.interval-self.prec) and t_delta < (self.interval+self.prec):
        pos_curr = self.theta(motor)
        self.vcirc[vidx] = (pos_curr - pos_last)/t_delta
        vidx = vidx + 1
        if vidx == len(self.vcirc):
          vidx = 0
        vel = mean(self.vcirc)
        error = refspeed - vel
        err_sum = err_sum + error
        int_term = Ki*err_sum
        if int_term > sum_max:
          int_term = sum_max
        if int_term < sum_min:
          int_term = sum_min
        outspeed = K*(Kp*error + int_term + Kd*(error-err_last))*vnorm*vrng
        #outspeed = int(K*error*vnorm*vrng)
        if outspeed > 0:
          outspeed = vlo + outspeed
        if outspeed < 0:
          outspeed = -vlo + outspeed
        if outspeed > 100:
          outspeed = 100
        if outspeed < -100:
          outspeed = -100
        data = concatenate((data,[[t_total, vel, refspeed, error, pos_curr, outspeed]]),axis=0)
        motor.run(outspeed)
        pos_last = pos_curr
        t_delta = 0
        t_total_last = t_total
        err_last = error
    
    motor.idle()
    savetxt('PID.txt', data, fmt="%g")
    
  # stop motors and exit program  
  def _quitBrick(self):  
    self.heading.run(0)
    self.elevation.run(0)
    exit()