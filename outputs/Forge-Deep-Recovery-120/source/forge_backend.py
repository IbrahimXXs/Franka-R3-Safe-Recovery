"""Controlled pose interface to the installed FORGE scene and Factory torque law."""
import math
import torch
from pxr import UsdPhysics,UsdGeom
import isaaclab.sim as sim_utils
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_mul, quat_conjugate, quat_from_euler_xyz
from isaaclab_tasks.direct.forge.forge_env import ForgeEnv
from isaaclab_tasks.direct.forge.forge_env_cfg import ForgeTaskPegInsertCfg
from contact import ContactWrench


def configuration(args):
    cfg=ForgeTaskPegInsertCfg()
    cfg.seed=args.seed
    cfg.task.fixed_asset_cfg.diameter=.009  # Correct stale metadata to the actual USD bore.
    cfg.scene.num_envs=1
    cfg.scene.clone_in_fabric=False
    cfg.sim.device=args.device
    cfg.sim.dt=1/args.physics_hz
    if not args.stock_buffers:
        # Capacity only: one environment has tens of active contacts. Keep large
        # headroom while avoiding the upstream multi-environment allocations.
        cfg.sim.physx.gpu_max_rigid_contact_count=2**18
        cfg.sim.physx.gpu_max_rigid_patch_count=2**16
    cfg.decimation=args.physics_hz//15
    cfg.sim.render_interval=args.physics_hz//30
    cfg.episode_length_s=120.
    cfg.sim.render=sim_utils.RenderCfg(antialiasing_mode='DLSS',dlss_mode=0)
    cfg.viewer.eye=(1.1,.6,.6);cfg.viewer.lookat=(.6,0.,.1)
    cfg.events=None
    cfg.task.hand_init_pos_noise=[0.,0.,0.]
    cfg.task.hand_init_orn_noise=[0.,0.,0.]
    cfg.task.fixed_asset_init_pos_noise=[0.,0.,0.]
    cfg.task.fixed_asset_init_orn_deg=0.
    cfg.task.fixed_asset_init_orn_range_deg=0.
    cfg.task.held_asset_pos_noise=[0.,0.,0.]
    cfg.obs_rand.fixed_asset_pos=[0.,0.,0.]
    cfg.obs_rand.fingertip_pos=0.
    cfg.obs_rand.fingertip_rot_deg=0.
    cfg.obs_rand.ft_force=0.
    cfg.ctrl.default_dead_zone=[0.]*6
    cfg.ctrl.task_prop_gains_noise_level=[0.]*6
    cfg.ctrl.pos_threshold_noise_level=[0.]*3
    cfg.ctrl.rot_threshold_noise_level=[0.]*3
    cfg.ctrl.ema_factor_range=[1.,1.]
    # Preserve the native 120 Hz EMA time constant across physics resolutions.
    cfg.ft_smoothing_factor=1.-.75**(120./args.physics_hz)
    return cfg


class Bench:
    def __init__(self,args,app):
        self.cfg=configuration(args)
        self.env=ForgeEnv(self.cfg)
        self.sim=self.env.sim
        self.sim._disable_app_control_on_stop_handle=True
        self.app=app;self.dt=self.cfg.sim.dt;self.steps=0
        self.device=self.env.device
        self.down=torch.tensor([[0.,1.,0.,0.]],device=self.device)
        self.contacts=None
        self.sensor_joint=None
        self.scene_info={'colliders':[],'joints':[],
                         'stage_meters_per_unit':UsdGeom.GetStageMetersPerUnit(self.sim.stage),
                         'stage_up_axis':str(UsdGeom.GetStageUpAxis(self.sim.stage))}
        if self.scene_info['stage_meters_per_unit']!=1. or self.scene_info['stage_up_axis']!='Z':
            raise RuntimeError('Geometry audit requires the composed metre-scale Z-up simulation stage')
        held=[];fixed=[]
        for prim in self.sim.stage.Traverse():
            path=str(prim.GetPath())
            if not path.startswith('/World/envs/env_0/'):continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                if '/HeldAsset/' in path:held.append(path)
                if '/FixedAsset/' in path:fixed.append(path)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                self.scene_info['colliders'].append({'path':path,'settings':{
                    a.GetName():str(a.Get()) for a in prim.GetAttributes()
                    if any(k in a.GetName().lower() for k in ('sdf','approximation','contactoffset','restoffset'))}})
                if '/FixedAsset/' in path and prim.IsA(UsdGeom.Mesh):
                    transform=UsdGeom.XformCache().GetLocalToWorldTransform(prim)
                    scales=[sum(transform[i][j]**2 for j in range(3))**.5 for i in range(3)]
                    if any(abs(s-1.)>1e-6 for s in scales):
                        raise RuntimeError('Socket geometry is scaled relative to the audited metre dimensions')
                    self.scene_info['socket_mesh_world_transform']=str(transform)
                    self.scene_info['socket_mesh_world_scales']=scales
                    points=UsdGeom.Mesh(prim).GetPointsAttr().Get()
                    ring=[math.hypot(p[0],p[1]) for p in points if abs(p[2]-.024)<1e-6 and math.hypot(p[0],p[1])<.006]
                    if len(ring)!=144 or max(abs(r-.0045) for r in ring)>1e-6:
                        raise RuntimeError('Socket USD no longer matches the audited 9 mm bore')
                    self.scene_info['measured_bore_diameter_mm']=2*sum(ring)/len(ring)*1000
                    self.scene_info['conservative_radial_clearance_mm']=.5059
            if prim.IsA(UsdPhysics.Joint):
                joint=UsdPhysics.Joint(prim)
                bodies=[str(x) for x in joint.GetBody1Rel().GetTargets()]
                if any(x.endswith('/force_sensor') for x in bodies):
                    q=joint.GetLocalRot1Attr().Get();p=joint.GetLocalPos1Attr().Get()
                    self.sensor_joint=(torch.tensor([[q.GetReal(),*q.GetImaginary()]],device=self.device),
                                       torch.tensor([list(p)],device=self.device))
                    self.scene_info['joints'].append({'path':path,'child':bodies,'rotation_wxyz':[q.GetReal(),*q.GetImaginary()], 'position_m':list(p)})
        if len(held)!=1 or len(fixed)!=1 or self.sensor_joint is None:
            raise RuntimeError(f'Unexpected asset structure: {held}, {fixed}, {self.sensor_joint}')
        self.contacts=ContactWrench(self.sim,held[0],fixed)
        self.scene_info['effective_assets']={name:{'bodies':asset.body_names,
            'masses':asset.root_physx_view.get_masses().tolist(),
            'materials':asset.root_physx_view.get_material_properties().tolist()}
            for name,asset in self.env.scene.articulations.items()}

    def tick(self,p,q):
        if not self.app.is_running():raise KeyboardInterrupt('Simulation window closed')
        e=self.env
        e.generate_ctrl_signals(p,q,0.)
        e.scene.write_data_to_sim()
        before=self.sim.current_time
        self.sim.step(render=False)
        if abs(self.sim.current_time-before-self.dt)>1e-7:
            raise RuntimeError('Actual simulation clock does not match requested physics timestep')
        self.steps+=1
        if self.steps%self.cfg.sim.render_interval==0 and self.sim.has_gui():self.sim.render()
        e.scene.update(self.dt)
        e._compute_intermediate_values(self.dt)
        self.sim.carb_settings.set_bool('/rtx/ecoMode/enabled',True)

    def prepare(self,seed):
        e=self.env
        e.reset(seed=seed)
        self.steps=0
        p=e.fingertip_midpoint_pos.clone();q=e.fingertip_midpoint_quat.clone()
        for _ in range(round(3/self.dt)):self.tick(p,q)
        self.grasp_p=quat_apply_inverse(e.fingertip_midpoint_quat,e.held_pos-e.fingertip_midpoint_pos).clone()
        self.grasp_q=quat_mul(quat_conjugate(e.fingertip_midpoint_quat),e.held_quat).clone()
        # Nominal peg axis is exactly the socket axis. Compensate the measured
        # grasp tilt with the hand target instead of inheriting reset error.
        self.peg_q=torch.tensor([[0.,0.,0.,1.]],device=self.device)
        # Calibrate grasp once in free space; retain it throughout the trial.
        self.start_timestamp=e.last_update_timestamp
        self.start_physics_time=self.sim.current_time
        self.initial_grasp_p=self.grasp_p.clone();self.initial_grasp_q=self.grasp_q.clone()
        return self.observe(phase='initial',command_depth=-10.)

    def target(self,depth_mm,x_mm=0.,y_mm=0.,roll_deg=0.,pitch_deg=0.):
        e=self.env
        tilt=quat_from_euler_xyz(torch.tensor([math.radians(roll_deg)],device=self.device),
                                torch.tensor([math.radians(pitch_deg)],device=self.device),
                                torch.zeros(1,device=self.device))
        peg_q=quat_mul(tilt,self.peg_q)
        peg_p=e.fixed_pos_obs_frame.clone()
        peg_p+=torch.tensor([[x_mm/1000,y_mm/1000,-depth_mm/1000]],device=self.device)
        hand_q=quat_mul(peg_q,quat_conjugate(self.grasp_q))
        hand_p=peg_p-quat_apply(hand_q,self.grasp_p)
        return hand_p,hand_q

    def observe(self,phase,command_depth):
        e=self.env
        origin=e._held_asset.data.root_pos_w[0]
        c=self.contacts.read(origin)
        rel=quat_apply_inverse(e.fixed_quat,e.held_pos-e.fixed_pos)[0]
        axis=quat_apply(e.held_quat,torch.tensor([[0.,0.,1.]],device=self.device))[0]
        gp=quat_apply_inverse(e.fingertip_midpoint_quat,e.held_pos-e.fingertip_midpoint_pos)
        gq=quat_mul(quat_conjugate(e.fingertip_midpoint_quat),e.held_quat)
        slip_angle=2*torch.acos((gq*self.initial_grasp_q).sum(-1).abs().clamp(max=1.)).item()
        si=e.force_sensor_body_idx
        sensor_q=e._robot.data.body_quat_w[:,si]
        sensor_p=e._robot.data.body_pos_w[:,si]
        jq,jp=self.sensor_joint
        joint_q=quat_mul(sensor_q,jq)
        joint_p=sensor_p+quat_apply(sensor_q,jp)
        raw=e.force_sensor_world.clone()
        wrist_f=quat_apply(joint_q,raw[:,:3])[0]
        wrist_t=quat_apply(joint_q,raw[:,3:])[0]+torch.cross(joint_p[0]-origin,wrist_f,dim=-1)
        # COM quantities make contact power meaningful even if the peg slips.
        peg=e._held_asset.data
        v=peg.root_com_lin_vel_w[0];w=peg.root_com_ang_vel_w[0]
        com=peg.root_com_pos_w[0]
        torque_com=c['torque']-torch.cross(com-origin,c['force'],dim=-1)
        row={'time_s':e.last_update_timestamp-self.start_timestamp,'phase':phase,
             'physics_time_s':self.sim.current_time-self.start_physics_time,
             'command_depth_mm':command_depth,'depth_mm':(self.cfg.task.fixed_asset_cfg.height-rel[2].item())*1000,
             'tip_x_mm':rel[0].item()*1000,'tip_y_mm':rel[1].item()*1000,
             'tilt_deg':math.degrees(math.acos(max(-1.,min(1.,axis[2].item())))),
             'grasp_slip_mm':(gp-self.initial_grasp_p).norm().item()*1000,
             'grasp_slip_deg':math.degrees(slip_angle),
             'force_norm_n':c['force'].norm().item(),'torque_norm_nm':c['torque'].norm().item(),
             'normal_load_n':c['normal_load_n'],'min_separation_mm':c['min_separation_m']*1000,
             'contact_count':c['contact_count'],
             'wrist_force_n':raw[0,:3].norm().item(),'wrist_torque_nm':raw[0,3:].norm().item(),
             'wrist_contact_force_balance_n':(wrist_f+c['force']).norm().item(),
             'wrist_contact_torque_balance_nm':(wrist_t+c['torque']).norm().item(),
             'contact_power_w':(c['force']*v).sum().item()+(torque_com*w).sum().item(),
             'lowest_peg_z_above_mouth_mm':(rel[2].item()+min(0.,.05*axis[2].item())-.003993*math.sqrt(max(0.,1.-axis[2].item()**2))-.025)*1000}
        for names,values in (
            (('fx','fy','fz'),c['force']), (('taux','tauy','tauz'),c['torque']),
            (('qw','qx','qy','qz'),e.held_quat[0]),
            (('vx','vy','vz'),v),(('omegax','omegay','omegaz'),w),
            (tuple(f'joint{i}_rad' for i in range(1,8)),e.joint_pos[0,:7]),
            (tuple(f'joint_velocity{i}_rad_s' for i in range(1,8)),e.joint_vel[0,:7]),
        ):row.update(zip(names,values.tolist()))
        for name,values in (('hand_pose',torch.cat((e.fingertip_midpoint_pos[0],e.fingertip_midpoint_quat[0]))),
                            ('grasp_position',gp[0]),('grasp_quat',gq[0]),
                            ('wrist_raw_child',raw[0]),('wrist_force_world',wrist_f),
                            ('wrist_torque_about_peg_base_world',wrist_t),
                            ('com_offset_world',com-origin),
                            ('finger_position',e.joint_pos[0,7:]),('finger_velocity',e.joint_vel[0,7:])):
            row.update({f'{name}_{i}':v for i,v in enumerate(values.tolist())})
        if not all(math.isfinite(v) for v in row.values() if isinstance(v,(int,float))):
            raise RuntimeError('Non-finite measurement')
        if abs(row['physics_time_s']-row['time_s'])>1e-6:
            raise RuntimeError('Measured physics clock and logged state clock diverged')
        return row

    def close(self):self.env.close()
