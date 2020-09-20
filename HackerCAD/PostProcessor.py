#import ModelIKa
from ModelIKa import IKClient
from math import pi
from OCC.Core.gp import *

class MachineState:

    v_tool_tip = gp_Vec(0,0,80)
    v_tool_orientation = gp_Vec(0,0,1)

class PostProcessor:

    def __init__(self):
        self.outfilename = "./out.ngc"
        self.convergence_distance = 0.1
        self.convergence_angle = 2.0 * pi / 180
        self.gcode = ""
        self.ik = IKClient("127.0.0.1",8826) 

        self.current_position = MachineState()


    def accept(self,motion,u):

        v_tool_tip, v_tool_orientation = motion.evaluate(u)

        I = v_tool_tip.X() / 1000
        J = v_tool_tip.Y() / 1000
        K = v_tool_tip.Z() / 1000

        v_tool_orientation = v_tool_orientation.Normalized() * (0.500+1e-8)
        U = v_tool_orientation.X()
        V = v_tool_orientation.Y()
        W = v_tool_orientation.Z()

        x,y,z,a,b = self.ik.solve((I,J,K,U,V,W),1e-6,False)
        cut_distance = (v_tool_tip - self.current_position.v_tool_tip).Magnitude()
        f = motion.feedrate / cut_distance
        gcode = "G01 X{:.6f} Y{:.6f} Z{:.6f} A{:.6f} B{:.6f} F{:.6f}\n".format(x*1e3,y*1e3,z*1e3,a,b,f)
        self.gcode += gcode
        #print(gcode)

        self.current_position.v_tool_tip = v_tool_tip
        self.current_position.v_tool_orientation = v_tool_orientation

    def test_convergence(self,motion,u_now,u_next):
        v_tip_0, v_dir_0 = motion.evaluate(u_now)
        v_tip_1, v_dir_1 = motion.evaluate(u_next)

        # test distance between tool tip positions
        distance = (v_tip_1 - v_tip_0).Magnitude()
        distance_okay = (distance <= self.convergence_distance)

        # test angle between tool top orientations
        angle = v_tip_1.Angle(v_tip_0)
        angle_okay = (angle <= self.convergence_angle)

        # TODO find maximum deviation between the tool tip's desired path and the path resulting from on-machine interpolation


        return distance_okay and angle_okay

    def process(self,motions):
        self.gcode = ""
        for motion in motions:
            u_min = motion.curve.FirstParameter()
            u_max = motion.curve.LastParameter()
            u_now = u_min
            self.accept(motion,u_now)
            done = False
            while u_now < u_max: # done with motion (done with edge)
                # make a guess for how far to advance u
                # TODO use the second derivative to guess based on curvature
                p = gp_Pnt()
                v1 = gp_Vec()
                v2 = gp_Vec()
                motion.curve.D2(u_now,p,v1,v2)
                du = 0.9 * self.convergence_distance / v1.Magnitude() 
                converged = False
                while not converged: # is u step converged?
                    u_next = u_now + du
                    converged = self.test_convergence(motion,u_now,u_next)
                    if converged:
                        self.accept(motion,u_next)
                        u_now = u_next
                    else:
                        du = du / 2.0


        return self.gcode

class Motion:

  def __init__(self):
      pass

  def evaluate(self,u):
    return gp_Vec(0,0,0)

