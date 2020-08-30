import sys
sys.path.append("/home/adam/projects/FreeCAD/build/Mod/Path")
import area

import OCCUtils
from OCC.Core.gp import *
from OCC.Core.BRepBuilderAPI import *
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
import OCC.Core.Tesselator
from OCC.Core.TopoDS import *
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.GCE2d import *
from OCC.Core.Geom import *
from math import cos, sin, tan, radians, pi
from OCC.Core.BRepAdaptor import *

def trsf_from_Ax2(ax2):
    
    # finds the transformation matrix from main c.s. to c.s. in ax2
    v_origin = gp_Vec(ax2.Location().XYZ())

    mat = gp_Mat() # rotation from B to main c.s.
    mat.SetRows(
           ax2.XDirection().XYZ(),
           ax2.YDirection().XYZ(),
           ax2.Direction().XYZ()
           )

    R = mat.Inverted()

    v_origin_mat = gp_Mat()
    v_origin_mat.SetRow(1,v_origin.XYZ())

    v_T_inv = gp_Vec(gp_XYZ((v_origin_mat * R).Value(1,1),
                            (v_origin_mat * R).Value(1,2),
                            (v_origin_mat * R).Value(1,3)))

    r_to_b = mat.Inverted() # rotation matrix from main c.s. to b c.s.

    trsf = gp_Trsf()
    trsf.SetValues(mat.Value(1,1),mat.Value(1,2),mat.Value(1,3),-v_T_inv.X(),
                   mat.Value(2,1),mat.Value(2,2),mat.Value(2,3),-v_T_inv.Y(),
                   mat.Value(3,1),mat.Value(3,2),mat.Value(3,3),-v_T_inv.Z())
    return trsf

class Adaptive2dMotion:

  def __init__(self,a2d,edge):
      """
      a2d is an instance onf Adaptive2d
      edge is the TopoDS_Edge defining the tool tip position
      """
      self.a2d = a2d
      self.edge = edge
      self.curve = BRepAdaptor_Curve(edge)

  def evaluate(self,u):
      v_tip = gp_Vec(self.curve.Value(u).XYZ())
      v_shank = gp_Vec(
                        self.a2d.ax.Direction().XYZ()
                      )
      return v_tip,v_shank

class Adaptive2d:
    
    def __init__(self,face,ax2=None):
        self.face = face
        if (ax2 is None):
            self.guess_ax()
        else:
            self.ax = ax2

        self.tesselation_mesh_quality = 0.1
        self.z_lift_distance = 1.0
        self.helix_angle = 15.0 # degrees
        self.helix_diameter = 4.0 # must be <= tool diameter
        self.tool_diameter = 5.0
        self.depth = 10.0
        self.step_over_factor = 0.5

        self.motions = []
        
    def guess_ax(self):
        f = OCCUtils.face.Face(self.face)
        pln = f.adaptor.Plane()
        self.ax = gp_Ax2(pln.Location(),pln.Axis().Direction(),pln.XAxis().Direction())
        
    def flip_ax_dir(self):
        self.ax.Rotate(gp_Ax1(self.ax.Location(),self.ax.YDirection()),pi)
        
    def compute(self):
        self.motions = []
        trsf = trsf_from_Ax2(self.ax)
        trsf_inv = trsf.Inverted()
        mt = BRepBuilderAPI_Transform(self.face,trsf)
        face_transformed = mt.Shape()
        tess = OCC.Core.Tesselator.ShapeTesselator(face_transformed)
        tess.Compute(compute_edges=True,mesh_quality=self.tesselation_mesh_quality)
        vertices = []
        initialized = False
        for i_edge in range(tess.ObjGetEdgeCount()):
            for i_vertex in range(tess.ObjEdgeGetVertexCount(i_edge)):
                x,y,z = tess.GetEdgeVertex(i_edge,i_vertex)
                if not initialized:
                    initialized = True
                    v_new = gp_Vec(x,y,z)
                    v_old = v_new
                    need_vertex = True
                else:
                    v_old = v_new
                    v_new = gp_Vec(x,y,z)
                    if (v_new - v_old).Magnitude() >= 1e-5:
                        need_vertex = True
                    else:
                        need_vertex = False
                if need_vertex:
                    vertices.append(tess.GetEdgeVertex(i_edge,i_vertex))

        a2d = area.Adaptive2d()
        a2d.tolerance
        a2d.opType
        a2d.stockToLeave
        a2d.toolDiameter = self.tool_diameter
        a2d.helixRampDiameter = self.helix_diameter
        a2d.stepOverFactor = self.step_over_factor

        def callback(something):
            return False # True stops processing

        stock_path = [[[0,0],[100,0],[100,100],[0,100]]]

        path = [vertices]
        a2d_output = a2d.Execute(stock_path,path,callback)

        comp = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(comp)
        edges = []
        mw = BRepBuilderAPI_MakeWire()

        x = 0
        y = 0
        for o_i in a2d_output:
            print("helix center point")
            x, y = o_i.HelixCenterPoint
            # peek at first point
            motion_type,path_i = o_i.AdaptivePaths[0]
            x_peek, y_peek = path_i[0]

            ax = gp_Ax2(gp_Pnt(x,y,0),gp_Dir(0,0,1),gp_Dir(x_peek-x,y_peek-y,0))
            cyl = gp_Cylinder(gp_Ax3(ax),self.helix_diameter/2.0)

            dv_du = tan(radians(self.helix_angle))
            du = self.depth / dv_du
            dv = self.depth
            dt = du/cos(radians(self.helix_angle))
            aLine2d = gp_Lin2d(gp_Pnt2d(-du,dv),gp_Dir2d(du,-dv))
            segment = GCE2d_MakeSegment(aLine2d,0,dt)
            helixEdge = BRepBuilderAPI_MakeEdge(segment.Value(),Geom_CylindricalSurface(cyl)).Edge()

            curve = BRepAdaptor_Curve(helixEdge)
            v0 = gp_Vec(curve.Value(curve.FirstParameter()).XYZ())
            v1 = gp_Vec(curve.Value(curve.LastParameter()).XYZ())
            mw.Add(helixEdge)
            mt = BRepBuilderAPI_Transform(helixEdge,trsf_inv)
            self.motions.append(Adaptive2dMotion(self,mt.Shape()))
            v_now = v1

            # add circle at bottom
            aLin2d = gp_Lin2d(gp_Pnt2d(0,0),gp_Dir2d(du,0))
            dt = 2 * pi 
            segment = GCE2d_MakeSegment(aLin2d,0,dt)
            circleEdge = BRepBuilderAPI_MakeEdge(segment.Value(),Geom_CylindricalSurface(cyl)).Edge()
            mw.Add(circleEdge)
            mt = BRepBuilderAPI_Transform(circleEdge,trsf_inv)
            self.motions.append(Adaptive2dMotion(self,mt.Shape()))

            curve = BRepAdaptor_Curve(circleEdge)
            v1 = gp_Vec(curve.Value(curve.LastParameter()).XYZ())

            v_now = v1

            for motion_type,path_i in o_i.AdaptivePaths:
                if (motion_type == 1):
                    z = self.z_lift_distance 
                else:
                    z = 0.0
                for x,y in path_i:
                    v_next = gp_Vec(x,y,z)
                    if (v_next - v_now).Magnitude() <= 1e-6:
                        #raise Warning("v:  !")
                        print("skipped a point because |v_next - v_now| <= 1e-6")
                        continue
                    me = BRepBuilderAPI_MakeEdge(gp_Pnt(v_now.XYZ()),gp_Pnt(v_next.XYZ()))
                    mw.Add(me.Edge())
                    mt = BRepBuilderAPI_Transform(me.Edge(),trsf_inv)
                    self.motions.append(Adaptive2dMotion(self,mt.Shape()))
                    v_now = v_next



        mt = BRepBuilderAPI_Transform(mw.Wire(),trsf_inv)
        return mt.Shape()
