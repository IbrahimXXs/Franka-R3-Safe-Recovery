"""Static force profiles for controlled FORGE references and final retreats."""
import csv
from pathlib import Path


def write_profiles(directory,manifest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory)
    fig,axes=plt.subplots(4,1,figsize=(11,11),sharex=True,constrained_layout=True)
    df,da=plt.subplots(figsize=(10,6),constrained_layout=True)
    for a in manifest['attempts']:
        if a['status']!='complete':continue
        color=None
        for filename,style in (('insertion.csv','-'),('final_retreat.csv','--')):
            with (directory/a['folder']/filename).open() as f:rows=list(csv.DictReader(f))
            time=[float(r['time_s']) for r in rows]
            for ax,key in zip(axes,('depth_mm','force_norm_n','wrist_force_n','wrist_torque_nm')):
                line=ax.plot(time,[float(r[key]) for r in rows],style,color=color,lw=.8,
                             label=a['folder'] if filename=='insertion.csv' else None)[0]
                color=line.get_color()
            da.plot([float(r['depth_mm']) for r in rows],[float(r['force_norm_n']) for r in rows],
                    style,color=color,lw=.8,label=a['folder'] if filename=='insertion.csv' else None)
    for ax,label in zip(axes,('Actual depth [mm]','Peg–socket net force [N]','Raw wrist force [N]','Raw wrist torque [Nm]')):
        ax.set_ylabel(label);ax.grid(alpha=.2)
    axes[0].legend(fontsize=7,ncol=3);axes[-1].set_xlabel('Simulation time from reference start [s]')
    fig.savefig(directory/'force_profiles.png',dpi=150);plt.close(fig)
    da.set(xlabel='Actual depth [mm]',ylabel='Peg–socket net force [N]',title='Solid: reference; dashed: final withdrawal')
    da.legend(fontsize=7,ncol=2);da.grid(alpha=.2)
    df.savefig(directory/'force_vs_depth.png',dpi=150);plt.close(df)
