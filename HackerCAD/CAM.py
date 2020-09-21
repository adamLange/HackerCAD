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

  def __init__(self,a2d,edge,feedrate):
      """
      a2d is an instance onf Adaptive2d
      edge is the TopoDS_Edge defining the tool tip position
      """
      self.a2d = a2d
      self.edge = edge
      self.curve = BRepAdaptor_Curve(edge)

      self.feedrate = feedrate # mm / min

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
        self.tolerance = 0.0001
        #self.z_lift_distance = 1.0
        self.z_link_clear = 0.0
        self.z_clearance = 20.0
        self.helix_angle = 15.0 # degrees
        self.helix_diameter = 4.0 # must be <= tool diameter
        self.tool_diameter = 5.0
        self.cut_depths = [10,5,0]
        self.step_over_factor = 0.5
        self.stockToLeave = 0.0

        self.plunge_feedrate = 100 # mm/min
        self.milling_feedrate = 100 # mm/min
        self.rapid_feedrate = 1000 # mm/min

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
                    need_vertex = True
                if need_vertex:
                    vertices.append(tess.GetEdgeVertex(i_edge,i_vertex))

        a2d = area.Adaptive2d()
        a2d.tolerance = self.tolerance
        a2d.opType = area.AdaptiveOperationType.ClearingInside
        a2d.stockToLeave = self.stockToLeave
        a2d.toolDiameter = self.tool_diameter
        a2d.helixRampDiameter = self.helix_diameter
        a2d.stepOverFactor = self.step_over_factor

        def callback(something):
            return False # True stops processing

        stock_path = [[[-1000,-1000],[1000,-1000],[1000,1000],[-1000,1000]]]

        path = [vertices]
        a2d_output = a2d.Execute(stock_path,path,callback)

        self.comp = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(self.comp)
        edges = []
        mw = BRepBuilderAPI_MakeWire()

        x = 0
        y = 0
        print("len(a2d_output):",len(a2d_output))
        regions = []
        need_depth_to_depth_link = False
        for i_depth,current_z_cut in enumerate(self.cut_depths):
            for region in a2d_output:
                motions_region_i = []
                print("helix center point")
                x, y = region.HelixCenterPoint
                # peek at first point
                motion_type,path_i = region.AdaptivePaths[0]
                x_peek, y_peek = path_i[0]

                ax = gp_Ax2(gp_Pnt(x,y,current_z_cut),gp_Dir(0,0,1),gp_Dir(x_peek-x,y_peek-y,0))
                cyl = gp_Cylinder(gp_Ax3(ax),self.helix_diameter/2.0)

                if i_depth == 0:
                    dv = self.z_clearance - self.cut_depths[i_depth]
                else:
                    dv = self.cut_depths[i_depth - 1] - self.cut_depths[i_depth]
                dv_du = tan(radians(self.helix_angle))
                du = dv / dv_du
                dt = du/cos(radians(self.helix_angle))
                aLine2d = gp_Lin2d(gp_Pnt2d(-du,dv),gp_Dir2d(du,-dv))
                segment = GCE2d_MakeSegment(aLine2d,0,dt)
                helixEdge = BRepBuilderAPI_MakeEdge(segment.Value(),Geom_CylindricalSurface(cyl)).Edge()

                curve = BRepAdaptor_Curve(helixEdge)
                v0 = gp_Vec(curve.Value(curve.FirstParameter()).XYZ())
                v1 = gp_Vec(curve.Value(curve.LastParameter()).XYZ())

                if need_depth_to_depth_link:
                    # go up to clearance plane
                    me = BRepBuilderAPI_MakeEdge(gp_Pnt(v_now.XYZ()),
                                                 gp_Pnt(v_now.X(),v_now.Y(),self.z_clearance))
                    mt = BRepBuilderAPI_Transform(me.Edge(),trsf_inv)
                    motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),self.rapid_feedrate))
                    mw.Add(me.Edge())
                    builder.Add(self.comp,me.Edge())
                    v_now = v_next
                    # go to helix start
                    me = BRepBuilderAPI_MakeEdge(gp_Pnt(v_now.XYZ()),
                                                 gp_Pnt(v0.XYZ()))
                    mt = BRepBuilderAPI_Transform(me.Edge(),trsf_inv)
                    motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),self.rapid_feedrate))
                    mw.Add(me.Edge())
                    builder.Add(self.comp,me.Edge())
                    v_now = v_next
                need_depth_to_depth_link = True

                mw.Add(helixEdge)
                builder.Add(self.comp,helixEdge)
                mt = BRepBuilderAPI_Transform(helixEdge,trsf_inv)
                motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),self.plunge_feedrate))
                v_now = v1


                # add circle at bottom
                aLin2d = gp_Lin2d(gp_Pnt2d(0,0),gp_Dir2d(du,0))
                dt = 2 * pi 
                segment = GCE2d_MakeSegment(aLin2d,0,dt)
                circleEdge = BRepBuilderAPI_MakeEdge(segment.Value(),Geom_CylindricalSurface(cyl)).Edge()
                mw.Add(circleEdge)
                builder.Add(self.comp,circleEdge)
                mt = BRepBuilderAPI_Transform(circleEdge,trsf_inv)
                motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),self.plunge_feedrate))

                curve = BRepAdaptor_Curve(circleEdge)
                v1 = gp_Vec(curve.Value(curve.LastParameter()).XYZ())

                v_now = v1

                PATH_TYPE = {0:"Cutting",
                            1:"LinkClear",
                            2:"LinkNotClear",
                            3:"LinkClearAtPrevPass"}

                for path_type,points in region.AdaptivePaths:
                    if PATH_TYPE[path_type] == "Cutting":
                        # handle the first point
                        x,y = points[0]
                        # choose feedrate
                        if (v_now.Z() - current_z_cut) > 1e-3:
                            feedrate = self.rapid_feedrate
                        else:
                            feedrate = self.milling_feedrate

                        v_next = gp_Vec(x,y,v_now.Z())
                        if (v_now - v_next).Magnitude() > 1e-6:
                            me = BRepBuilderAPI_MakeEdge(gp_Pnt(v_now.XYZ()),
                                                         gp_Pnt(v_next.XYZ()))
                            mt = BRepBuilderAPI_Transform(me.Edge(),trsf_inv)
                            motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),feedrate))
                            mw.Add(me.Edge())
                            builder.Add(self.comp,me.Edge())
                            v_now = v_next

                        if v_now.Z() != current_z_cut:
                            v_next = gp_Vec(x,y,current_z_cut)
                            me = BRepBuilderAPI_MakeEdge(gp_Pnt(v_now.XYZ()),
                                                         gp_Pnt(v_next.XYZ()))
                            mt = BRepBuilderAPI_Transform(me.Edge(),trsf_inv)
                            motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),self.plunge_feedrate))
                            v_now = v_next
                            mw.Add(me.Edge())
                            builder.Add(self.comp,me.Edge())
                            
                        # handle the rest of the points
                        for x,y in points[1:]:
                            v_next = gp_Vec(x,y,current_z_cut)
                            if (v_now - v_next).Magnitude() > 1e-6:
                                me = BRepBuilderAPI_MakeEdge(gp_Pnt(v_now.XYZ()),
                                                             gp_Pnt(v_next.XYZ()))
                                mt = BRepBuilderAPI_Transform(me.Edge(),trsf_inv)
                                motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),self.milling_feedrate))
                                v_now = v_next
                                mw.Add(me.Edge())
                                builder.Add(self.comp,me.Edge())

                    elif (PATH_TYPE[path_type] == "LinkClear") or (PATH_TYPE[path_type] == "LinkNotClear"):
                        if (PATH_TYPE[path_type] == "LinkClear"):
                            z = current_z_cut + self.z_link_clear
                        else:
                            z = self.z_clearance

                        if v_now.Z() != z:
                            if v_now.Z() >= z:
                                feedrate = self.plunge_feedrate
                            else:
                                feedrate = self.rapid_feedrate
                            v_next = gp_Vec(v_now.X(),v_now.Y(),z)
                            me = BRepBuilderAPI_MakeEdge(gp_Pnt(v_now.XYZ()),
                                                         gp_Pnt(v_next.XYZ()))
                            mt = BRepBuilderAPI_Transform(me.Edge(),trsf_inv)
                            motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),self.plunge_feedrate))
                            v_now = v_next
                            mw.Add(me.Edge())
                            builder.Add(self.comp,me.Edge())

                        if len(points) > 0:
                            # go to each point
                            for x,y in points[1:]:
                                v_next = gp_Vec(x,y,current_z_cut)
                                me = BRepBuilderAPI_MakeEdge(gp_Pnt(v_now.XYZ()),
                                                             gp_Pnt(v_next.XYZ()))
                                mt = BRepBuilderAPI_Transform(me.Edge(),trsf_inv)
                                motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),self.rapid_feedrate))
                                v_now = v_next
                                mw.Add(me.Edge())
                                builder.Add(self.comp,me.Edge())
                    else:
                        raise Warning("path type not implemented")


                """
                n_not_skipped = 0
                n_skipped = 0
                for motion_type,path_i in region.AdaptivePaths:
                    if (motion_type == 1):
                        z = self.z_lift_distance 
                    else:
                        z = 0.0
                    for x,y in path_i:
                        v_next = gp_Vec(x,y,z)
                        if not (v_next - v_now).Magnitude() <= 1e-9:
                            n_not_skipped += 1
                        else:
                            #raise Warning("v:  !")
                            #print("skipped a point because |v_next - v_now| <= 1e-9")
                            #print("x:{}, y:{}".format(x,y))
                            n_skipped += 1
                            continue
                        me = BRepBuilderAPI_MakeEdge(gp_Pnt(v_now.XYZ()),gp_Pnt(v_next.XYZ()))
                        mw.Add(me.Edge())
                        mt = BRepBuilderAPI_Transform(me.Edge(),trsf_inv)
                        motions_region_i.append(Adaptive2dMotion(self,mt.Shape(),self.milling_feedrate))
                        v_now = v_next

                print("n_not_skipped:{}".format(n_not_skipped))
                print("n_skipped:{}".format(n_skipped))
            """

                self.motions.append(motions_region_i)
        mt = BRepBuilderAPI_Transform(mw.Wire(),trsf_inv)
        self.wire = mt.Shape()
        return self.wire



class RuledMotion:

    "Motion from a tip edge and a shank edge"

    def __init__(self,tip_edge,shank_edge):

        self.tip_curve =   BRepAdaptor_Curve(tip_edge)
        self.shank_curve = BRepAdaptor_Curve(shank_edge)
        self.curve = self.tip_curve

        self.u_tip_min = self.tip_curve.FirstParameter()
        self.u_tip_max = self.tip_curve.LastParameter()
        self.du_tip = self.u_tip_max - self.u_tip_min

        self.u_shank_min = self.shank_curve.FirstParameter()
        self.u_shank_max = self.shank_curve.LastParameter()
        self.du_shank = self.u_shank_max - self.u_shank_min

        self.dshank_dtip = self.du_shank / self.du_tip

        self.feedrate = 100

        self.surface_normal_offset = 0.0
        self.tool_axis_offset = 0.0 # + makes the tool go deeper

    def evaluate(self,u):

        u_star = (u - self.u_tip_min) * self.dshank_dtip + self.u_shank_min

        pnt_tip = gp_Pnt()
        dP_dutip = gp_Vec()  # P is for position
        self.tip_curve.D1(u,pnt_tip,dP_dutip)

        #v_tip   = gp_Vec(self.tip_curve.Value(u).XYZ())
        v_tip = gp_Vec(pnt_tip.XYZ())
        v_shank = gp_Vec(self.shank_curve.Value(u_star).XYZ())

        v_tip_to_shank = (v_shank - v_tip).Normalized()

        v_normal_offset = dP_dutip.Crossed(v_tip_to_shank).Normalized()*self.surface_normal_offset
        v_tool_axis_offset = v_tip_to_shank.Normalized()*-self.tool_axis_offset

        v_tip = v_tip + v_normal_offset + v_tool_axis_offset

        return v_tip,v_tip_to_shank
