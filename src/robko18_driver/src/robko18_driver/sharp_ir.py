# inspired by http://playground.arduino.cc/Main/SharpIR

class SharpIR(object):
  def __init__(self):
    self.last_cm = 0
    self.ticks = 0
    
  def get_cm(self, adc):
    if adc == 0:
      return 99
      
    adc_to_volts = adc * 0.001221001221 
    
    # for sensor with range 10-80
    cm = 27.728 * (adc_to_volts ** -1.2045)  
    
    # for sensor with range 20-150
    #volts_to_cm = 61.573 * (adc_to_volts ** -1.1068)  
    
    if cm > 10 and cm < 80:
      self.ticks = 0
      self.last_cm = cm
    else:
        self.ticks = self.ticks + 1
    
    if self.ticks > 10:
        self.last_cm = 99
    
    return self.last_cm
    
