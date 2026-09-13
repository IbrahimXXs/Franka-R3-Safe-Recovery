"""Known external loads on the held peg to verify wrist wrench axes and moment."""
import csv
import json
import torch


def run_calibration(bench,directory,seed):
    e=bench.env
    bench.prepare(seed)
    hp=e.fingertip_midpoint_pos.clone();hq=e.fingertip_midpoint_quat.clone()
    cases=[('zero',[0.,0.,0.],[0.,0.,0.])]
    for axis in range(3):
        f=[0.,0.,0.];f[axis]=1.;cases.append((f'force_{axis}',f,[0.,0.,0.]))
        t=[0.,0.,0.];t[axis]=.01;cases.append((f'torque_{axis}',[0.,0.,0.],t))
    cases.append(('mixed',[-.7,-.5,-1.],[-.005,.007,-.003]))
    summary=[]
    try:
        for name,force,torque in cases:
            f=torch.tensor([[force]],device=bench.device);t=torch.tensor([[torque]],device=bench.device)
            rows=[]
            with (directory/f'{name}.csv').open('w') as stream:
                writer=None
                for step in range(round(3./bench.dt)):
                    applied_position=e._held_asset.data.root_com_pos_w[:,None,:].clone()
                    e._held_asset.set_external_force_and_torque(f,t,positions=applied_position,is_global=True)
                    bench.tick(hp,hq);r=bench.observe('calibration',-15.)
                    arm_f=torch.tensor([r[f'wrist_force_world_{i}'] for i in range(3)],device=bench.device)
                    arm_t=torch.tensor([r[f'wrist_torque_about_peg_base_world_{i}'] for i in range(3)],device=bench.device)
                    moment=torch.cross(applied_position[0,0]-e._held_asset.data.root_pos_w[0],f[0,0],dim=-1)+t[0,0]
                    force_error=arm_f+f[0,0];torque_error=arm_t+moment
                    r.update(force_residual_n=force_error.norm().item(),
                             torque_residual_nm=torque_error.norm().item(),calibration_time_s=(step+1)*bench.dt)
                    for i in range(3):
                        r[f'force_residual_{i}']=force_error[i].item()
                        r[f'torque_residual_{i}']=torque_error[i].item()
                    if writer is None:writer=csv.DictWriter(stream,fieldnames=list(r));writer.writeheader()
                    writer.writerow(r)
                    if step>=round(2./bench.dt):rows.append(r)
            mean=lambda key:sum(r[key] for r in rows)/len(rows)
            s={'case':name,'applied_force_world_n':force,'applied_torque_world_nm':torque,
               'mean_force_residual_n':mean('force_residual_n'),'mean_torque_residual_nm':mean('torque_residual_nm'),
               'max_socket_force_n':max(r['force_norm_n'] for r in rows),
               'max_grasp_slip_mm':max(r['grasp_slip_mm'] for r in rows),
               'mean_measured_force_world_n':[mean(f'wrist_force_world_{i}') for i in range(3)],
               'mean_raw_child_force_n':[mean(f'wrist_raw_child_{i}') for i in range(3)]}
            s['force_bias_n']=sum(mean(f'force_residual_{i}')**2 for i in range(3))**.5
            s['torque_bias_nm']=sum(mean(f'torque_residual_{i}')**2 for i in range(3))**.5
            # A joint reaction contains distal-link dynamics. Check mean static
            # balance for axes/scale, and bound the remaining instantaneous
            # residual separately (1% of the operational limits).
            s['passed']=(s['force_bias_n']<.05 and s['torque_bias_nm']<.002
                         and s['mean_force_residual_n']<.2 and s['mean_torque_residual_nm']<.01
                         and s['max_socket_force_n']<.001)
            summary.append(s);print('Wrist calibration:',s,flush=True)
    finally:
        e._held_asset.set_external_force_and_torque(torch.zeros((1,1,3),device=bench.device),torch.zeros((1,1,3),device=bench.device),is_global=True)
        (directory/'calibration.json').write_text(json.dumps({'passed':bool(summary) and len(summary)==len(cases) and all(s['passed'] for s in summary),'cases':summary},indent=2)+'\n')
