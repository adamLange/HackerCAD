import unittest

import HackerCAD

from OCC.Core.BRepTools import breptools_Read, breptools_Write
from OCC.Core.TopoDS import *
from OCC.Core.BRep import BRep_Builder
from OCC.Core.gp import *
from OCC.Core.BRepBuilderAPI import *
import OCCUtils

class TestTurning(unittest.TestCase):

    def test_001(self):
        shape = TopoDS_Shape()
        builder = BRep_Builder()
        assert breptools_Read(shape,"./input/cylinder.brep",builder)

        face = OCCUtils.Topo(shape).faces().__next__()

        ct = HackerCAD.CylindricalTurning(face)
        edge = ct.compute()

        breptools_Write(edge,"output/path.brep")

if __name__ == "__main__":
    unittest.main()
