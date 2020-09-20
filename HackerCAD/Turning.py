from OCC.Core.gp import *
from OCC.Core.BRepBuilderAPI import *

class CylindricalTurning:

    def __init__(self,cylindrical_face):
        self.face = cylindrical_face
        self.motions = []

    def compute(self):
        self.motions = []
        self.motions.append(CylindricalTurningMotion())
        return BRepBuilderAPI_MakeEdge(gp_Pnt(0,0,0),gp_Pnt(0,0,100)).Edge()

class CylindricalTurningMotion:

    def __init__(self):
        self.bla = True

    def compute(self,u):
        return gp_Vec(0,0,0), gp_Vec(0,1,0)
      
