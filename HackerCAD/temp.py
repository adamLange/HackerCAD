class Adaptive2d:
    
    def __init__(self,face,ax2=None):
        self.face = face
        if (ax2 is None):
            self.guess_ax()
        else:
            self.ax = ax2
        self.tesselation_mesh_quality = 0.1
        self.z_lift_distance = 1.0
        
    def guess_ax(self):
        f = OCCUtils.face.Face(self.face)
        pln = f.adaptor.Plane()
        self.ax = gp_Ax2(pln.Location(),pln.Axis().Direction(),pln.XAxis().Direction())
        
    def flip_ax_dir(self):
        self.ax.Rotate(gp_Ax1(ax_on_sqp.Location(),ax_on_sqp.XDirection()),pi)
        
    def compute(self):
        trsf = trsf_from_Ax2(self.ax)
        trsf_inv = trsf.Inverted()
        mt = BRepBuilderAPI_Transform(self.face,trsf)
        face_transformed = mt.Shape()
        tess = OCC.Core.Tesselator.ShapeTesselator(face_transformed)
        tess.Compute(compute_edges=True,mesh_quality=self.tesselation_mesh_quality)
        vertices = []
        for i_edge in range(tess.ObjGetEdgeCount()):
            for i_vertex in range(tess.ObjEdgeGetVertexCount(i_edge)):
                if (i_vertex % 2) == 0:
                    vertices.append(tess.GetEdgeVertex(i_edge,i_vertex+1))

        a2d = area.Adaptive2d()
        a2d.tolerance
        a2d.opType
        a2d.stockToLeave
        a2d.toolDiameter = 10
        a2d.helixRampDiameter = 3
        a2d.stepOverFactor = 0.5

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
            v_now = gp_Vec(x,y,0)
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
                    v_now = v_next
   



     mt = BRepBuilderAPI_Transform(mw.Wire(),trsf_inv)
        return mt.Shape()
        
