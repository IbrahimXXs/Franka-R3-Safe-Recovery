"""Project-owned FR3 workbench and insertion geometry (SI units)."""

import math
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema

ROBOT_SOURCE = f"{ISAAC_NUCLEUS_DIR}/Robots/FrankaRobotics/FrankaFR3/fr3.usd"
_CACHED_ROBOT = Path(__file__).resolve().parents[1] / ".deps/fr3/fr3.usd"
ROBOT_USD = str(_CACHED_ROBOT) if _CACHED_ROBOT.is_file() else ROBOT_SOURCE
TABLE_Z = 0.75
HOLE_CENTER = (0.5, 0.0, 0.865)
PEG_RADIUS = 0.004
PEG_LENGTH = 0.060
PEG_MASS = 0.030
PEG_CENTER_IN_HAND = (0.0, 0.0, 0.105)
PEG_TIP_IN_HAND = (0.0, 0.0, PEG_CENTER_IN_HAND[2] + PEG_LENGTH / 2)
HOLE_RADIUS = 0.0045
HOLE_DEPTH = 0.025
START_GAP = 0.025
TARGET_DEPTH = 0.020
DT = 1 / 120
DECIMATION = 4


def material(color, metallic=0.0, roughness=0.45):
    return sim_utils.PreviewSurfaceCfg(diffuse_color=color, metallic=metallic, roughness=roughness)


def box(path, size, pos, color, collision=True):
    cfg = sim_utils.CuboidCfg(
        size=size, visual_material=material(color),
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0002, rest_offset=0.0)
        if collision else None,
    )
    cfg.func(path, cfg, translation=pos)


def make_socket(stage, hole_radius=HOLE_RADIUS, contact_offset=0.00005, segments=192):
    """Static triangle mesh with a genuine open bore and a 1 mm entry chamfer."""
    path = "/World/Fixture/Socket"
    mesh = UsdGeom.Mesh.Define(stage, path)
    n = segments
    # Cross-section around the solid annulus: bore bottom, bore shoulder,
    # chamfer mouth, outer top, outer bottom. No triangles cover the bore.
    profile = [(hole_radius, -HOLE_DEPTH), (hole_radius, -0.001),
               (hole_radius + 0.001, 0.0), (0.035, 0.0), (0.035, -HOLE_DEPTH)]
    points = [(HOLE_CENTER[0] + radius * math.cos(2 * math.pi * i / n),
               HOLE_CENTER[1] + radius * math.sin(2 * math.pi * i / n),
               HOLE_CENTER[2] + z) for radius, z in profile for i in range(n)]
    indices = []
    for ring in range(len(profile)):
        next_ring = (ring + 1) % len(profile)
        for i in range(n):
            j = (i + 1) % n
            a, b, c, d = ring*n+i, ring*n+j, next_ring*n+j, next_ring*n+i
            indices.extend([a, c, b, a, d, c])
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
    sim_utils.modify_collision_properties(path, sim_utils.CollisionPropertiesCfg(
        contact_offset=contact_offset, rest_offset=0.0))
    mat = material((0.55, 0.62, 0.66), metallic=0.85, roughness=0.28)
    mat.func(path + "/Metal", mat)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(path + "/Metal")))


def build_scene(sim, friction=0.3, clearance=0.0005, contact_offset=0.00005, segments=192,
                contact_stiffness=0., contact_damping=0., torque_control=False):
    UsdGeom.Xform.Define(sim.stage, "/World/Fixture")
    UsdGeom.Xform.Define(sim.stage, "/World/Bench")
    ground = sim_utils.GroundPlaneCfg(color=(0.12, 0.14, 0.17), size=(20.0, 20.0))
    ground.func("/World/Ground", ground)
    light = sim_utils.DomeLightCfg(intensity=1800.0, color=(0.85, 0.91, 1.0))
    light.func("/World/Lighting", light)
    box("/World/Bench/Top", (1.05, 0.75, 0.05), (0.30, 0, TABLE_Z - 0.025), (0.12, 0.17, 0.21))
    for x in (-0.14, 0.74):
        for y in (-0.30, 0.30):
            name = f"Leg_{int((x+1)*100)}_{int((y+1)*100)}"
            box(f"/World/Bench/{name}", (0.055, 0.055, 0.70), (x, y, 0.35), (0.08, 0.09, 0.11))
    box("/World/Bench/FrontAccent", (1.05, 0.006, 0.012), (0.30, -0.377, 0.726), (0.05, 0.65, 0.70), False)
    box("/World/RobotMount", (0.20, 0.20, 0.012), (0, 0, TABLE_Z + 0.006), (0.28, 0.32, 0.35))
    box("/World/Fixture/Base", (0.14, 0.14, 0.012), (0.5, 0, TABLE_Z + 0.006), (0.04, 0.34, 0.39))
    # Pedestal ends at the bottom of the socket; this forms a 25 mm blind hole.
    bottom = HOLE_CENTER[2] - HOLE_DEPTH
    box("/World/Fixture/Pedestal", (0.080, 0.080, bottom - TABLE_Z - 0.012),
        (0.5, 0, (bottom + TABLE_Z + 0.012) / 2), (0.10, 0.16, 0.19))
    make_socket(sim.stage, PEG_RADIUS + clearance, contact_offset, segments)
    physical = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction,
        restitution=0.0, friction_combine_mode="average", restitution_combine_mode="average",
        compliant_contact_stiffness=contact_stiffness, compliant_contact_damping=contact_damping)
    physical.func("/World/ContactMaterial", physical)
    for path in ("/World/Fixture/Socket", "/World/Fixture/Pedestal/geometry/mesh"):
        sim_utils.bind_physics_material(path, "/World/ContactMaterial")
    for i, (dx, dy) in enumerate(((-.052, -.052), (-.052, .052), (.052, -.052), (.052, .052))):
        bolt = sim_utils.CylinderCfg(radius=0.004, height=0.003, visual_material=material((.35, .38, .40), .8))
        bolt.func(f"/World/Fixture/Bolt{i}", bolt, translation=(.5 + dx, dy, TABLE_Z + .0135))
    cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=ROBOT_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, max_depenetration_velocity=0.1),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True, solver_position_iteration_count=16, solver_velocity_iteration_count=4),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0002, rest_offset=0.0),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, TABLE_Z + 0.012),
            joint_pos={"fr3_joint1": 0.0, "fr3_joint2": -0.5, "fr3_joint3": 0.0,
                       "fr3_joint4": -2.3, "fr3_joint5": 0.0, "fr3_joint6": 1.8,
                       "fr3_joint7": math.pi / 4, "fr3_finger_joint.*": PEG_RADIUS},
        ),
        actuators={
            "arm": ImplicitActuatorCfg(joint_names_expr=["fr3_joint[1-7]"],
                stiffness=0.0 if torque_control else 400.0, damping=0.0 if torque_control else 80.0, effort_limit_sim={"fr3_joint[1-4]": 87., "fr3_joint[5-7]": 12.}),
            "hand": ImplicitActuatorCfg(joint_names_expr=["fr3_finger_joint.*"],
                stiffness=2000.0, damping=100.0, effort_limit_sim=40.0),
        },
    )
    robot = Articulation(cfg)
    # The supplied USD has two massless fixed frames with unspecified inertia.
    # Give these simulation-only frames negligible explicit mass/inertia.
    for name in ("fr3_link8", "fr3_hand_tcp"):
        mass = UsdPhysics.MassAPI.Apply(sim.stage.GetPrimAtPath("/World/Robot/" + name))
        mass.CreateMassAttr(0.0001)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-7))
    # Merge a 30 g peg into the hand's rigid-body mass properties. This is
    # dynamically equivalent to a rigid grasp and avoids a tiny fixed leaf link.
    # First query the stock hand inertia so the original robot properties are
    # preserved rather than replaced by an assumed box inertia.
    import numpy as np
    from scipy.spatial.transform import Rotation
    sim.reset()
    sim._disable_app_control_on_stop_handle = True
    hand_id = robot.find_bodies("fr3_hand")[0][0]
    hand_path = "/World/Robot/fr3_hand"
    original_mass = robot.root_physx_view.get_masses()[0, hand_id].item()
    original_com = robot.root_physx_view.get_coms()[0, hand_id].cpu().numpy().copy()
    original_inertia = robot.root_physx_view.get_inertias()[0, hand_id].cpu().numpy().copy().reshape(3, 3, order="F")
    sim.stop()
    peg_mass = PEG_MASS
    center = np.array(PEG_CENTER_IN_HAND)
    total_mass = original_mass + peg_mass
    combined_com = (original_mass*original_com[:3] + peg_mass*center)/total_mass
    def parallel_axis(mass, offset):
        return mass*(np.dot(offset, offset)*np.eye(3)-np.outer(offset, offset))
    inertia_xy = peg_mass*(3*PEG_RADIUS**2+PEG_LENGTH**2)/12
    peg_inertia = np.diag([inertia_xy, inertia_xy, peg_mass*PEG_RADIUS**2/2])
    combined_inertia = (original_inertia
        + parallel_axis(original_mass, original_com[:3]-combined_com)
        + peg_inertia + parallel_axis(peg_mass, center-combined_com))
    eigenvalues, eigenvectors = np.linalg.eigh(combined_inertia)
    if np.linalg.det(eigenvectors) < 0:
        eigenvectors[:, 0] *= -1
    quaternion = Rotation.from_matrix(eigenvectors).as_quat()
    hand = sim.stage.GetPrimAtPath(hand_path)
    mass = UsdPhysics.MassAPI.Apply(hand)
    mass.CreateMassAttr(float(total_mass))
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*combined_com))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*eigenvalues))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(float(quaternion[3]), Gf.Vec3f(*quaternion[:3])))
    peg = sim_utils.CylinderCfg(radius=PEG_RADIUS, height=PEG_LENGTH,
        visual_material=material((0.72, 0.43, 0.14), metallic=0.8, roughness=0.22),
        physics_material=physical,
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=contact_offset, rest_offset=0.0))
    peg.func(hand_path+"/HeldPeg", peg, translation=PEG_CENTER_IN_HAND)
    PhysxSchema.PhysxContactReportAPI.Apply(hand).CreateThresholdAttr(0.)
    robot.payload_model = {"hand_mass_kg": float(total_mass), "hand_com_m": combined_com.tolist(),
                           "hand_inertia_kg_m2": combined_inertia.tolist(), "peg_mass_kg": peg_mass}
    sim.set_camera_view(eye=(1.60, -1.60, 1.65), target=(0.25, 0.0, 1.10))
    return robot
