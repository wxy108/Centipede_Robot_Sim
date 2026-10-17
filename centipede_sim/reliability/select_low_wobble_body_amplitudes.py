"""Step 36: select lower-wobble amplitudes for every original-contact wave.

Wobble diagnostics remain separate. Selection maximizes forward displacement
subject to explicit per-diagnostic limits between the 15 and 40 degree cases.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import optimize_original_contact_body_amplitude as amp

iid = amp.iid
ROOT = amp.ROOT
OUTPUT = ROOT / 'exp_data/reliability/step36_original_contact_low_wobble_amplitudes'
WAVES = list(amp.WAVES)
AMPLITUDES = [15., 20., 25., 30., 35., 40.]
METRICS = [
    'lateral_sway_pp_m',
    'vertical_heave_pp_m',
    'chassis_tilt_rms_deg',
    'heading_rate_rms_deg_s',
    'actual_body_yaw_rms_deg',
]
EXCESS_RETAINED = 0.60  # remove at least 40% of the 15-to-40 excess


def write_json(name, value):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT/name).write_text(json.dumps(value, indent=2), encoding='utf-8')


def read_rows(path):
    if not path.exists(): return []
    with path.open(newline='', encoding='utf-8') as f: return list(csv.DictReader(f))


def numeric(row):
    converted={}
    for k,v in row.items():
        if k in {'stage','gates'}: converted[k]=v
        elif k in {
            'feasible',
            'step35_reference_available',
            'step35_displacement_replay_passed',
            'wobble_limits_passed',
        }:
            converted[k]=str(v).lower()=='true'
        else:
            try: converted[k]=float(v)
            except (ValueError,TypeError): converted[k]=v
    return converted


def detrended_peak_to_peak(t, values):
    degree=min(3,len(t)-1)
    residual=values-np.polyval(np.polyfit(t,values,degree),t)
    return float(np.percentile(residual,95)-np.percentile(residual,5)),residual


def wobble_metrics(assets, states, start):
    import mujoco
    model=assets.model; d=mujoco.MjData(model)
    spec=mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    ids=np.unique(np.r_[start-1,np.arange(start,len(states),20),len(states)-1])
    chassis=np.array([mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'Chasis{i}') for i in range(1,11)])
    mujoco.mj_kinematics(model,d)
    reference_up=d.xmat[chassis].reshape(-1,3,3)[:,2,:].copy()
    com=[]; tilt=[]; heading=[]
    for idx in ids:
        mujoco.mj_setState(model,d,states[idx],spec)
        mujoco.mj_kinematics(model,d); mujoco.mj_comPos(model,d)
        com.append(d.subtree_com[assets.root_body_id].copy())
        rotations=d.xmat[chassis].reshape(-1,3,3)
        up=np.einsum('bij,bj->bi',rotations,reference_up)[:,2]
        tilt.append(np.degrees(np.arccos(np.clip(up,-1,1))))
        vector=d.xpos[chassis[0],:2]-d.xpos[chassis[-1],:2]
        heading.append(math.atan2(vector[0],vector[1]))
    com=np.asarray(com); tilt=np.asarray(tilt); heading=np.unwrap(heading)
    t=(ids-(start-1))*assets.dt
    lateral_pp,_=detrended_peak_to_peak(t,com[:,0])
    vertical_pp,_=detrended_peak_to_peak(t,com[:,2])
    _,heading_residual=detrended_peak_to_peak(t,heading)
    heading_rate=np.gradient(heading_residual,t)
    by=assets.indices[4]
    qbody=model.jnt_qposadr[model.actuator_trnid[by,0]]
    body_values=states[start:,1+qbody]
    return {
        'lateral_sway_pp_m':lateral_pp,
        'vertical_heave_pp_m':vertical_pp,
        'chassis_tilt_rms_deg':float(np.sqrt(np.mean(tilt**2))),
        'chassis_tilt_p95_deg':float(np.percentile(tilt,95)),
        'heading_rate_rms_deg_s':float(np.degrees(np.sqrt(np.mean(heading_rate**2)))),
        'actual_body_yaw_rms_deg':float(np.degrees(np.sqrt(np.mean(body_values**2)))),
        'sample_rate_hz':50.,
        'sample_count':len(ids),
    }


def run_batch(assets,engine,cases):
    rows,traces,states,controls,starts,warms=amp.simulate_batch(assets,engine,cases,True)
    prior=read_rows(amp.OUTPUT/'trials.csv')
    result=[]
    for i,row in enumerate(rows):
        row.update(wobble_metrics(assets,states[i],starts[i]))
        references=[r for r in prior if float(r['N_wave'])==row['N_wave']
                    and float(r['amplitude_deg'])==row['amplitude_deg']
                    and float(r['phase_deg'])==0 and float(r['evaluation_cycles'])==12]
        row['step35_reference_available']=bool(references)
        replay_error=(abs(row['forward_displacement_m']-float(references[0]['forward_displacement_m']))
                      if references else None)
        row['step35_displacement_replay_error_m']=replay_error
        row['step35_displacement_replay_passed']=bool(references and replay_error<1e-6)
        assert row['feasible'] and (not references or row['step35_displacement_replay_passed'])
        result.append(row)
    return result


def append_cases(assets,engine,rows,wanted):
    path=OUTPUT/'wobble_trials.csv'
    seen={(r['N_wave'],r['amplitude_deg']) for r in rows}
    pending=[c for c in wanted if (c['N_wave'],c['amplitude_deg']) not in seen]
    for j in range(0,len(pending),engine.nthread):
        batch=pending[j:j+engine.nthread]
        rows.extend(run_batch(assets,engine,batch))
        iid._write_csv(path,rows); write_json('wobble_trials.json',rows)
        print(f'wobble scan: {min(j+len(batch),len(pending))}/{len(pending)} new; {len(rows)} total',flush=True)
    return rows


def scan(assets,engine,smoke=False):
    path=OUTPUT/'wobble_trials.csv'; rows=[numeric(r) for r in read_rows(path)]
    wanted=[amp.case(3,a,stage='wobble_scan') for a in [15.,30.,40.]] if smoke else [
        amp.case(w,a,stage='wobble_scan') for w in WAVES for a in AMPLITUDES]
    return append_cases(assets,engine,rows,wanted)


def select(rows):
    selected=[]; thresholds=[]
    for wave in WAVES:
        candidates=sorted((r for r in rows if r['N_wave']==wave),key=lambda r:r['amplitude_deg'])
        low=next(r for r in candidates if r['amplitude_deg']==15)
        high=next(r for r in candidates if r['amplitude_deg']==40)
        limits={m:low[m]+EXCESS_RETAINED*max(0.,high[m]-low[m]) for m in METRICS}
        thresholds.append({'N_wave':wave,**{f'{m}_limit':v for m,v in limits.items()}})
        for r in candidates:
            r['wobble_limits_passed']=all(r[m]<=limits[m]+1e-12 for m in METRICS)
            r['wobble_limit_violations']=';'.join(m for m in METRICS if r[m]>limits[m]+1e-12)
        feasible=[r for r in candidates if r['feasible'] and r['wobble_limits_passed']]
        assert feasible,f'No low-wobble candidate for wave {wave}'
        winner=max(feasible,key=lambda r:r['forward_displacement_m'])
        selected.append({
            'N_wave':wave,'selected_half_amplitude_deg':winner['amplitude_deg'],
            'forward_displacement_m':winner['forward_displacement_m'],
            'step35_40deg_displacement_m':high['forward_displacement_m'],
            'performance_retained_percent':100*winner['forward_displacement_m']/high['forward_displacement_m'],
            **{m:winner[m] for m in METRICS},
            **{f'{m}_15deg':low[m] for m in METRICS},
            **{f'{m}_40deg':high[m] for m in METRICS},
            **{f'{m}_limit':limits[m] for m in METRICS},
        })
    iid._write_csv(OUTPUT/'selected_amplitudes.csv',selected)
    iid._write_csv(OUTPUT/'wobble_limits.csv',thresholds)
    write_json('results.json',{'selection_rule':
        'Maximize forward COM displacement among candidates satisfying every separate wobble limit; '
        'each limit retains at most 60% of positive 15-to-40-degree excess.',
        'selected_per_wave':selected})
    iid._write_csv(OUTPUT/'wobble_trials.csv',rows); write_json('wobble_trials.json',rows)
    return selected


def figures(rows,selected):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    folder=OUTPUT/'figures'; folder.mkdir(exist_ok=True)
    chosen={r['N_wave']:r['selected_half_amplitude_deg'] for r in selected}
    labels=[('lateral_sway_pp_m','Lateral sway (m)'),('vertical_heave_pp_m','Vertical heave (m)'),
            ('chassis_tilt_rms_deg','Chassis tilt RMS (deg)'),('heading_rate_rms_deg_s','Heading-rate RMS (deg/s)')]
    fig,axes=plt.subplots(2,2,figsize=(10,7),sharex=True)
    for ax,(metric,label) in zip(axes.ravel(),labels):
        for wave in WAVES:
            group=sorted((r for r in rows if r['N_wave']==wave),key=lambda r:r['amplitude_deg'])
            line,=ax.plot([r['amplitude_deg'] for r in group],[r[metric] for r in group],'.-',label=f'{wave:g}')
            win=next(r for r in group if r['amplitude_deg']==chosen[wave])
            ax.plot(win['amplitude_deg'],win[metric],'*',ms=11,color=line.get_color())
        ax.set_ylabel(label); ax.grid(alpha=.18)
    axes[1,0].set_xlabel('Body-yaw half-amplitude (deg)'); axes[1,1].set_xlabel('Body-yaw half-amplitude (deg)')
    axes[0,0].legend(title=r'$N_{\mathrm{wave}}$',ncol=2,frameon=False)
    fig.tight_layout(); fig.savefig(folder/'wobble_diagnostics.png',dpi=220); fig.savefig(folder/'wobble_diagnostics.svg'); plt.close(fig)
    x=np.array(WAVES)
    fig,ax=plt.subplots(figsize=(6.5,4.2)); ax.plot(x,[chosen[w] for w in WAVES],'o-',color='#117c85')
    ax.axhline(40,color='#777',ls='--',label='Previous maximum-displacement setting')
    ax.set(xlabel=r'$N_{\mathrm{wave}}$',ylabel='Selected half-amplitude (deg)',xticks=x,ylim=(10,42))
    ax.grid(alpha=.18); ax.legend(frameon=False); fig.savefig(folder/'selected_low_wobble_amplitudes.png',dpi=220,bbox_inches='tight'); fig.savefig(folder/'selected_low_wobble_amplitudes.svg',bbox_inches='tight'); plt.close(fig)
    fig,ax=plt.subplots(figsize=(6.5,4.2))
    ax.plot(x,[r['forward_displacement_m'] for r in selected],'o-',label='Low-wobble selection',color='#117c85')
    ax.plot(x,[r['step35_40deg_displacement_m'] for r in selected],'o--',label='40 deg maximum-displacement setting',color='#777')
    ax.set(xlabel=r'$N_{\mathrm{wave}}$',ylabel='Forward displacement (m)',xticks=x); ax.grid(alpha=.18); ax.legend(frameon=False)
    fig.savefig(folder/'performance_tradeoff.png',dpi=220,bbox_inches='tight'); fig.savefig(folder/'performance_tradeoff.svg',bbox_inches='tight'); plt.close(fig)


def documentation(selected,model_hash):
    lines=['# Low-wobble original-contact amplitudes (Step 36)','',
      'This stage keeps the Step-35 physical N=10 model, original cave-grouped pitch, fixed leg-yaw targets, omega=4 rad/s, initialization, flat ground and 18.85-second COM-displacement objective. Only body-yaw half-amplitude is selected. Step 35 remains unchanged.','',
      'Wobble is not a scalar reward. At 50 Hz we separately retain: detrended 5–95% lateral COM sway, detrended 5–95% vertical COM heave, RMS material-frame chassis tilt relative to the upright reference, detrended whole-body heading-rate RMS, and actual body-yaw-joint RMS. Cubic detrending removes slow turning/drift while preserving the 12 gait-cycle oscillation.','',
      'Candidate amplitudes begin at 15, 20, 25, 30, 35 and 40 deg; the interval immediately above each coarse winner is then refined at 1-deg spacing. For each diagnostic and wave, the limit is W15 + 0.60*max(0,W40-W15), removing at least 40% of positive excess wobble. Among candidates passing every limit, the largest forward displacement is selected. This is an explicit engineering trade-off, not a universal perceptual threshold or continuous optimum.','',
      '| N_wave | Selected A (deg) | D (m) | Performance retained vs 40 deg (%) |','|---|---|---|---|']
    for r in selected: lines.append(f"| {r['N_wave']:g} | {r['selected_half_amplitude_deg']:g} | {r['forward_displacement_m']:.4f} | {r['performance_retained_percent']:.1f} |")
    lines.extend(['','All selected runs must also pass Step-35 numerical/upright gates. Model SHA-256: `'+model_hash+'`. Commands marked down/up are not direct contact-state measurements. Self-collision remains disabled in this unchanged model.','',
                  'Files: wobble_trials.csv/json; selected_amplitudes.csv; wobble_limits.csv; results.json; validation.json; figures/; demos/.'])
    (OUTPUT/'README.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def main():
    import mujoco
    import mujoco.rollout
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--mode',choices=['smoke','sweep','finalize','demos'],default='sweep'); p.add_argument('--workers',type=int,default=8); args=p.parse_args()
    OUTPUT.mkdir(parents=True,exist_ok=True)
    assets=iid._load_assets(ROOT); model_path=iid._model_path(ROOT); model_hash=hashlib.sha256(model_path.read_bytes()).hexdigest()
    old=json.loads((amp.OUTPUT/'run_configuration.json').read_text()); assert model_hash==old['model_sha256'] and mujoco.__version__==old['mujoco_version']
    write_json('run_configuration.json',{'model':str(model_path),'model_sha256':model_hash,'matches_step35_model':True,'n_segments':10,'N_wave':WAVES,'coarse_candidate_half_amplitudes_deg':AMPLITUDES,'refinement':'1-deg grid between each coarse winner and the next coarse candidate','body_omega_rad_s':iid.OMEGA_BODY,'leg_yaw_targets_deg':{'R':-5,'L':5},'original_contact':True,'iid':False,'wobble_sample_hz':50,'excess_wobble_retained':EXCESS_RETAINED,'evaluation_cycles':12,'duration_s':math.ceil(12*amp.PERIOD/assets.dt)*assets.dt,'self_collision_enabled':False,'mujoco_version':mujoco.__version__})
    with mujoco.rollout.Rollout(nthread=args.workers) as engine:
        engine.nthread=args.workers
        if args.mode=='smoke': scan(assets,engine,True); return
        rows=scan(assets,engine,False) if args.mode=='sweep' else [numeric(r) for r in read_rows(OUTPUT/'wobble_trials.csv')]
        coarse_complete=all(any(r['N_wave']==w and r['amplitude_deg']==a for r in rows) for w in WAVES for a in AMPLITUDES)
        assert coarse_complete and len({(r['N_wave'],r['amplitude_deg']) for r in rows})==len(rows)
        if args.mode=='sweep':
            preliminary=select(rows)
            fine=[]
            for r in preliminary:
                low=int(r['selected_half_amplitude_deg']); high=min(40,low+5)
                fine.extend(amp.case(r['N_wave'],float(a),stage='wobble_refine') for a in range(low+1,high))
            rows=append_cases(assets,engine,rows,fine)
        selected=select(rows); figures(rows,selected); documentation(selected,model_hash)
        checks={'model_unchanged':hashlib.sha256(model_path.read_bytes()).hexdigest()==model_hash,'all_48_coarse_candidates_present':coarse_complete,'unique_condition_keys':len({(r['N_wave'],r['amplitude_deg']) for r in rows})==len(rows),'all_trials_feasible':all(r['feasible'] for r in rows),'all_available_step35_replays_match':all(r.get('step35_displacement_replay_passed',False) for r in rows if r.get('step35_reference_available',True)),'all_selected_pass_separate_wobble_limits':all(next(q for q in rows if q['N_wave']==r['N_wave'] and q['amplitude_deg']==r['selected_half_amplitude_deg'])['wobble_limits_passed'] for r in selected),
                'selected_boundaries_refined':all(r['selected_half_amplitude_deg']==40 or any(q['N_wave']==r['N_wave'] and q['amplitude_deg']==r['selected_half_amplitude_deg']+1 for q in rows) for r in selected)}
        write_json('validation.json',{'passed':all(checks.values()),'checks':checks,'trial_count':len(rows)})
        if args.mode in {'sweep','demos'}:
            amp.OUTPUT=OUTPUT
            write_json('results.json',{
                'selection_rule':'Maximize forward COM displacement among candidates satisfying every separate wobble limit; each limit retains at most 60% of positive 15-to-40-degree excess.',
                'best_per_wave':[{'N_wave':r['N_wave'],'forward_displacement_m':r['forward_displacement_m']} for r in selected],
                'selected_per_wave':selected})
            demo_folder=OUTPUT/'demos'
            if demo_folder.exists():
                for generated in demo_folder.iterdir():
                    if generated.is_file(): generated.unlink()
            amp.render(assets,engine,[amp.case(r['N_wave'],r['selected_half_amplitude_deg'],stage='low_wobble_demo') for r in selected])
        print(json.dumps({'selected_per_wave':selected,'validation':checks},indent=2),flush=True)


if __name__=='__main__': main()
