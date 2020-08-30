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
        square_pacman = TopoDS_Shape()
        builder = BRep_Builder()
        assert breptools_Read(square_pacman, './inputs/square_pacman.brep', builder)
        t = OCCUtils.Topo(square_pacman)
        square_pacman = t.faces().__next__()

        hacker_a2d = HackerCAD.Adaptive2d(square_pacman)
        wire = hacker_a2d.compute()

        breptools_Write(wire,"output/adaptive.brep")


if __name__ == '__main__':
    unittest.main()
