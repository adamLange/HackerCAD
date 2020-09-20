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

class TestRuled(unittest.TestCase):

    def test_rule_motion_00(self):
        gcode = "G21 G94\n"
        gcode += "G54\n"
        gcode += "M6 T1 G43 H1\n"

        motions = []
        for i in range(3):

            builder = BRep_Builder()
            shape = TopoDS_Shape()
            assert breptools_Read(shape, './input/tip_{}.brep'.format(i+1), builder)
            tip_edge = OCCUtils.Topo(shape).edges().__next__()
            print(OCCUtils.Topo(shape).number_of_edges())

            builder = BRep_Builder()
            shape = TopoDS_Shape()
            assert breptools_Read(shape, './input/shank_{}.brep'.format(i+1), builder)
            shank_edge = OCCUtils.Topo(shape).edges().__next__()
            print(OCCUtils.Topo(shape).number_of_edges(),'\n')

            rm = HackerCAD.RuledMotion(tip_edge,shank_edge)
            motions.append(rm)

        pp = HackerCAD.PostProcessor()
        gcode += pp.process(motions)

        gcode += "M2\n"
        f = open("./output/bla.ngc","w")
        f.write(gcode)
        f.close()

if __name__ == '__main__':
    unittest.main()
