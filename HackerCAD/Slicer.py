from OCC.Core.BRepBuilderAPI import *
from OCC.Core.BRepAlgoAPI import *
from OCC.Core.BRepBndLib import *
from OCC.Core.Bnd import *
from OCC.Core.gp import *
import numpy as np
from math import pi

class Slicer:

    def __init__(self,shape):
        self.shape = shape
        self.compute_bounding_box()
        
        self.v_dir = gp_Vec(0,0,1)
        self.x = np.linspace(0,10,10)
        self.faces = []

    def compute_bounding_box(self):
        bbox = Bnd_Box()
        bbox.SetGap(1e-2)
        brepbndlib_Add(self.shape, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        self.v_center = gp_Vec((xmax + xmin)/2,
                  (ymax + ymin)/2,
                  (zmax + zmin)/2)
        self.v_diagonal = gp_Vec(xmax - xmin,
                    ymax - ymin,
                    zmax - zmin)
        self.diagonal_mag = self.v_diagonal.Magnitude()

        self.bb_xmin = xmin
        self.bb_ymin = ymin
        self.bb_zmin = zmin
        self.bb_xmax = xmax
        self.bb_ymax = ymax
        self.bb_zmax = zmax

    def compute(self):
        faces = []
        for x_i in self.x:
            v_i = self.v_center + self.v_dir.Normalized() * x_i
            pln = gp_Pln(gp_Pnt(v_i.XYZ()),gp_Dir(self.v_dir.XYZ()))
            ax = gp_Ax2(gp_Pnt(v_i.XYZ()),gp_Dir(self.v_dir.XYZ()))
            circ = gp_Circ(ax,0.6*self.diagonal_mag)
            me = BRepBuilderAPI_MakeEdge(circ,0,2*pi)
            edge = me.Edge()
            mw = BRepBuilderAPI_MakeWire()
            mw.Add(edge)
            wire = mw.Wire()
            mf = BRepBuilderAPI_MakeFace(pln,wire)
            face = mf.Face()
            mc = BRepAlgoAPI_Common(face,self.shape)
            cut_face = mc.Shape()
            self.faces.append(cut_face)
