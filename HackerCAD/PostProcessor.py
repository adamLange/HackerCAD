#import ModelIKa
from ModelIKa import IKClient

class PostProcessor:

    def __init__(self):
      self.outfilename = "./out.ngc"

    def process(self,motions):
        iksolver = IKClient("127.0.0.1",8826) 
        self.gcode = ""
        for motion in motions:
            u_min = motion.curve.FirstParameter()
            u_max = motion.curve.LastParameter()
            n_pts = 10
            du = (u_max - u_min) / (n_pts - 1)
            for i in range(n_pts):
                v_tool_tip, v_tool_orientation = motion.evaluate(i*du+u_min)

                I = v_tool_tip.X() / 1000
                J = v_tool_tip.Y() / 1000
                K = v_tool_tip.Z() / 1000

                v_tool_orientation = v_tool_orientation.Normalized() * (0.500+1e-8)
                U = v_tool_orientation.X()
                V = v_tool_orientation.Y()
                W = v_tool_orientation.Z()

                x,y,z,a,b = iksolver.solve((I,J,K,U,V,W),1e-6,False)
                cut_distance = 1
                f = 100 / cut_distance
                gcode = "G01 X{:.6f} Y{:.6f} Z{:.6f} A{:.6f} B{:.6f} F{:.6f}\n".format(x*1e3,y*1e3,z*1e3,a,b,f)
                self.gcode += gcode
                #print(gcode)
        return self.gcode
