import unittest

import HackerCAD

from OCC.Core.BRepTools import breptools_Read
from OCC.Core.TopoDS import *
from OCC.Core.BRep import BRep_Builder
from OCC.Core.gp import *
from OCC.Core.BRepBuilderAPI import *
from math import pi,degrees,radians
from OCC.Core.BRepTools import breptools_Write
import OCCUtils
import OCCUtils.edge
import numpy as np
import OCCUtils.face
from OCC.Core.BRepAlgoAPI import *
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh

class TestAdaptive(unittest.TestCase):

    def test_adaptive(self):
        gcode = "G21 G94\n"
        gcode += "G54\n"
        gcode += "M6 T1 G43 H1\n"
        hex_flats = TopoDS_Shape()
        builder = BRep_Builder()
        assert breptools_Read(hex_flats, './inputs/hex_flats.brep', builder)
        t = OCCUtils.Topo(hex_flats)

        num_faces = 0
        for face_i in t.faces():
            print("face")
            a2d = HackerCAD.Adaptive2d(face_i)
            a2d.tool_diameter = 25.4/32.0
            a2d.helix_diameter = a2d.tool_diameter * 0.35
            a2d.depth = 5.0
            a2d.helix_angle = 0.5
            a2d.flip_ax_dir()
            wire = a2d.compute()
            breptools_Write(wire,"output/hex_flat_{}.brep".format(num_faces))
            num_faces += 1
            pp = HackerCAD.PostProcessor()
            gcode += pp.process(a2d.motions)

        gcode += "M2\n"
        f = open("./output/bla.ngc","w")
        f.write(gcode)
        f.close()

if __name__ == '__main__':
    unittest.main()
