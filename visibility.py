import visilibity as vis

EPSILON = 0.00000001

def point_to_vis(point):
    x,y = point
    return vis.Point(x,y)

def points_to_vis_points(tupled_points):
    return [vis.Point(x,y) for x,y in tupled_points]

def vis_poly_to_pointlist(polygon):
    end_poss = []
    for i in range(polygon.n()):
        x = polygon[i].x()
        y = polygon[i].y()

        end_poss.append((x,y))

    return end_poss

class LibVisibility:
    def __init__(self,blocker_polygons,width,height):
        self.environment = vis.Environment()
        for poly in blocker_polygons:
            points = [vis.Point(x,y) for x,y in poly]
            #self.polys.append(vis.Polygon(points))
            poly = vis.Polygon(points)
            if poly.area() > 0:
                poly.reverse()
            self.environment.add_hole(poly)
        #exit(0)
        #self.polys.append()
        self.environment.set_outer_boundary(vis.Polygon([
            vis.Point(0,0),
            vis.Point(width,0),
            vis.Point(width,height),
            vis.Point(0,height),
        ]))
        #self.environment.reverse_holes()

    def filter_points(self,points):
        res_points = []
        for px,py in points:
            vpoint = vis.Point(px,py)
            if vpoint._in(self.environment,EPSILON):
                res_points.append((px,py))
        return res_points

    def get_point_visibility_graph(self,points):
        vis_points = points_to_vis_points(points)
        graph = vis.Visibility_Graph(vis_points,self.environment,1e-12)

        res = []
        for pidx1 in range(len(points)):
            adjlist = []
            for pidx2 in range(len(points)):
                if graph(pidx1,pidx2):
                    adjlist.append(pidx2)
            res.append(adjlist)
        return res

    def get_visibilily_polygon(self,origin):
        origin_vis = point_to_vis(origin)
        isovist = vis.Visibility_Polygon(origin_vis, self.environment, EPSILON)
        pointlist = vis_poly_to_pointlist(isovist)
        return pointlist

    def get_libvis_polygon(self,origin):
        origin_vis = point_to_vis(origin)
        return vis.Visibility_Polygon(origin_vis, self.environment, EPSILON)

    def in_vispoly(self, vispoly, point):
        return point_to_vis(point)._in(vispoly)

    def in_env(self,point):
        p1 = point_to_vis(point)
        return p1._in(self.environment)

    def can_see(self,p1,p2):
        p1 = point_to_vis(p1)
        p2 = point_to_vis(p2)
        poly = vis.Visibility_Polygon(p1, self.environment, EPSILON)
        return p2._in(poly)
