import math
import unittest

from research.forge_geometry import (BORE_RADIUS_M, PEG_RADIUS_M,
    audit_socket_points, mesh_sha256, resize_socket_points)


def socket_points():
    points=[]
    for z,r in ((0.,.0045),(.024,.0045),(.025,.0055)):
        points.extend([[r*math.cos(i*math.tau/144),r*math.sin(i*math.tau/144),z]
                       for i in range(144)])
    return points+[[.02,.02,0.],[.02,.02,.025],[0.,0.,0.]]


class ForgeGeometryTests(unittest.TestCase):
    def test_shrinking_preserves_outer_geometry_chamfer_and_height(self):
        original=socket_points()
        for gap in (.4,.3,.2,.1):
            edited=resize_socket_points(original,gap)
            self.assertEqual(edited[-3:],original[-3:])
            self.assertEqual([p[2] for p in edited],[p[2] for p in original])
            target=PEG_RADIUS_M+gap/1000
            audit=audit_socket_points(edited,expected_bore_radius_m=target)
            self.assertAlmostEqual(audit['measured_bore_diameter_mm'],2*target*1000)
            self.assertAlmostEqual(audit['measured_radial_vertex_clearance_mm'],gap)
            self.assertAlmostEqual(audit['socket_chamfer_radial_mm'],1.)
            self.assertAlmostEqual(audit['conservative_radial_clearance_mm'],
                                   (target*math.cos(math.pi/144)-PEG_RADIUS_M)*1000)
            self.assertNotEqual(mesh_sha256(original),mesh_sha256(edited))

    def test_explicit_baseline_preserves_exact_geometry(self):
        original=socket_points()
        self.assertEqual(resize_socket_points(original,.507),original)
        audit=audit_socket_points(original)
        self.assertAlmostEqual(audit['conservative_radial_clearance_mm'],.5059291218595914)

    def test_conservative_gap_uses_measured_peg_radius(self):
        points=socket_points()
        nominal=audit_socket_points(points)
        larger=audit_socket_points(points,peg_radius_m=PEG_RADIUS_M+.00001)
        self.assertAlmostEqual(nominal['conservative_radial_clearance_mm']-
                               larger['conservative_radial_clearance_mm'],.01)

    def test_reject_unexpected_mesh_and_unsafe_inputs(self):
        for gap in (0.,-.1,.6,float('nan'),float('inf')):
            with self.assertRaises(ValueError):resize_socket_points(socket_points(),gap)
        missing=socket_points();missing.pop(0)
        with self.assertRaisesRegex(ValueError,'ring vertices'):resize_socket_points(missing,.1)
        duplicate=socket_points();duplicate[0]=duplicate[1]
        with self.assertRaisesRegex(ValueError,'angular sectors'):resize_socket_points(duplicate,.1)
        with self.assertRaises(ValueError):audit_socket_points(socket_points(),peg_radius_m=.005)

    def test_mesh_hash_includes_topology(self):
        points=socket_points()
        a=mesh_sha256(points,[3],[0,1,2])
        self.assertEqual(a,mesh_sha256(points,[3],[0,1,2]))
        self.assertNotEqual(a,mesh_sha256(points,[3],[2,1,0]))


if __name__=='__main__':unittest.main()
