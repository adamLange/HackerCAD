from OCC.Core.gp import *
from math import pi

class MachineState:

    v_tool_tip = gp_Vec(0,0,100)

class ThreeAxisPostProcessor:

    def __init__(self):
        self.convergence_distance = 0.1
        self.gcode = ""

        self.current_position = MachineState()

    def accept(self,motion,u):

        v_tool_tip, v_tool_orientation = motion.evaluate(u)

        f = motion.feedrate
        gcode = "G01 X{:.6f} Y{:.6f} Z{:.6f} F{:.6f}\n".format(v_tool_tip.X(),
                                                               v_tool_tip.Y(),
                                                               v_tool_tip.Z(),f)
        self.gcode += gcode

    def test_convergence(self,motion,u_now,u_next):
        v_tip_0, v_dir_0 = motion.evaluate(u_now)
        v_tip_1, v_dir_1 = motion.evaluate(u_next)

        # test distance between tool tip positions
        distance = (v_tip_1 - v_tip_0).Magnitude()
        distance_okay = (distance <= self.convergence_distance)


        # TODO find maximum deviation between the tool tip's desired path and the path resulting from on-machine interpolation

        return distance_okay

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
