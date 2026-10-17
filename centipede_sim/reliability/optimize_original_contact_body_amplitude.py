"""Step 35: bounded amplitude search, original cave contact, fixed leg yaw.

The objective is signed whole-robot COM displacement along world +y, measured
after settling, a cosine bridge, and four gait cycles. No physics is retuned.
"""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import time
from pathlib import Path

import numpy as np
import render_physical_robot_original_contact_no_leg_yaw as original

iid = original.iid
ROOT = iid._repo_root()
OUTPUT = ROOT / 'exp_data/reliability/step35_original_contact_amplitude_optimization'
WAVES = list(iid.DEFAULT_N_WAVE)
PERIOD = 2 * math.pi / iid.OMEGA_BODY


class AmplitudeController(original.OriginalContactNoYaw):
    def __init__(self, nu, indices, wave, amplitude, phase_deg=0):
        super().__init__(nu, indices, wave)
        self.amplitude = amplitude
        self.shift_s = math.radians(phase_deg) / iid.OMEGA_BODY

    def control(self, t, _decision_time_s=0):
        ctrl = super().control(t + self.shift_s)
        ctrl[self.indices[4]] *= self.amplitude / iid.YAW_HALF_AMPLITUDE_DEG
        return ctrl

    def stance_flags(self, t):
        # The inherited method calls self.control(), which applies the shift.
        return super().stance_flags(t)


def build_controls(assets, wave, amplitude, phase_deg=0, cycles=12, warmup=4):
    dt = assets.dt
    settle = math.ceil(iid.SETTLE_TIME_S / dt)
    bridge = math.ceil(iid.BRIDGE_TIME_S / dt)
    warm = math.ceil(warmup * PERIOD / dt)
    evaluate = math.ceil(cycles * PERIOD / dt)
    initial = iid._initial_target(assets.model, assets.indices)
    ctrl = np.empty((settle + bridge + warm + evaluate, assets.model.nu))
    ctrl[:settle] = np.arange(1, settle + 1)[:, None] / settle * initial
    sr, pr, sl, pl, by, bp = assets.indices
    phi = iid._spatial_increment(wave)
    t = np.arange(warm + evaluate) * dt
    phase = iid.OMEGA_BODY * t[:, None] + math.radians(phase_deg)
    locomotion = np.zeros((len(t), assets.model.nu))
    locomotion[:, sr] = math.radians(iid.LEG_OFFSET_DEG)
    locomotion[:, sl] = -math.radians(iid.LEG_OFFSET_DEG)
    locomotion[:, by] = math.radians(amplitude) * np.sin(phase + np.arange(9) * phi)
    caves = np.floor((phase + (np.arange(10) - .5) * phi + 1e-12) / math.pi).astype(int)
    right_down = caves % 2 == 0
    locomotion[:, pr] = np.where(right_down, math.radians(iid.LEG_DOWN_DEG), math.radians(iid.LEG_UP_DEG))
    locomotion[:, pl] = np.where(~right_down, math.radians(iid.LEG_DOWN_DEG), math.radians(iid.LEG_UP_DEG))
    alpha = .5 - .5 * np.cos(math.pi * np.arange(1, bridge + 1) / bridge)
    ctrl[settle:settle + bridge] = (1 - alpha[:, None]) * initial + alpha[:, None] * locomotion[0]
    ctrl[settle + bridge:] = locomotion
    return ctrl, settle + bridge + warm, warm * dt


def audit(assets):
    rows = []
    for wave in WAVES:
        for amplitude, phase in [(0, 0), (15, 0), (40, 0), (23.5, 90)]:
            ctrl, _, _ = build_controls(assets, wave, amplitude, phase)
            c = AmplitudeController(assets.model.nu, assets.indices, wave, amplitude, phase)
            offset = math.ceil(iid.SETTLE_TIME_S / assets.dt) + math.ceil(iid.BRIDGE_TIME_S / assets.dt)
            times = np.unique(np.r_[np.arange(0, len(ctrl) - offset, 137), 0, len(ctrl) - offset - 1])
            error = max(float(np.max(np.abs(ctrl[offset + k] - c.control(k * assets.dt)))) for k in times)
            flags_match = all(np.array_equal(c.stance_flags(k * assets.dt),
                np.column_stack([np.isclose(ctrl[offset+k, assets.indices[1]], math.radians(iid.LEG_DOWN_DEG)),
                                 np.isclose(ctrl[offset+k, assets.indices[3]], math.radians(iid.LEG_DOWN_DEG))])) for k in times)
            within = bool(np.all(ctrl >= assets.model.actuator_ctrlrange[:, 0] - 1e-12)
                          and np.all(ctrl <= assets.model.actuator_ctrlrange[:, 1] + 1e-12))
            rows.append(dict(N_wave=wave, amplitude_deg=amplitude, phase_deg=phase,
                             max_control_error_rad=error, within_control_limits=within,
                             stance_flags_match_pitch_commands=flags_match,
                             passed=error < 1e-12 and within and flags_match))
    result = dict(passed=all(r['passed'] for r in rows), rows=rows)
    write_json('controller_validation.json', result)
    assert result['passed'], result


def write_json(name, data):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / name).write_text(json.dumps(data, indent=2), encoding='utf-8')


def case(wave, amplitude, phase=0, cycles=12, warmup=4, stage='coarse'):
    return dict(N_wave=float(wave), amplitude_deg=float(amplitude), phase_deg=float(phase),
                evaluation_cycles=int(cycles), warmup_cycles=int(warmup), stage=stage)


def key(row):
    return tuple(row[k] for k in ['N_wave', 'amplitude_deg', 'phase_deg', 'evaluation_cycles', 'warmup_cycles'])


def extract(assets, controls, start, states, spec, c, batch_warnings):
    import mujoco
    model = assets.model
    d = mujoco.MjData(model)
    sample_ids = np.unique(np.r_[start - 1, np.arange(start, len(states), 50), len(states) - 1])
    chassis = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'Chasis{i}') for i in range(1, 11)]
    mujoco.mj_kinematics(model, d)
    # Re-rooted STL bodies do NOT share a local z axis. Derive each chassis'
    # material up direction from the common upright reference pose.
    reference_up_local = d.xmat[chassis].reshape(-1,3,3)[:,2,:].copy()
    poses = []
    min_up = 1.
    for idx in sample_ids:
        mujoco.mj_setState(model, d, states[idx], spec)
        # Kinematics on an observation-only copy: no contact-solver warm-start mutation.
        mujoco.mj_kinematics(model, d)
        mujoco.mj_comPos(model, d)
        up = float(np.min(np.einsum('bij,bj->bi', d.xmat[chassis].reshape(-1,3,3), reference_up_local)[:,2]))
        min_up = min(min_up, up)
        poses.append([max(0, (idx - start + 1) * assets.dt), *d.subtree_com[assets.root_body_id], up])
    poses = np.array(poses)
    displacement = poses[-1, 1:4] - poses[0, 1:4]
    by, pitch = assets.indices[4], np.r_[assets.indices[1], assets.indices[3]]
    qbody = model.jnt_qposadr[model.actuator_trnid[by, 0]]
    qpitch = model.jnt_qposadr[model.actuator_trnid[pitch, 0]]
    # FULLPHYSICS layout begins with time, followed by qpos.
    error = states[start:, 1 + qbody] - controls[start:, by]
    pitch_error = states[start:, 1 + qpitch] - controls[start:, pitch]
    finite = bool(np.all(np.isfinite(states)))
    time_ok = bool(np.allclose(np.diff(states[:, 0]), assets.dt, atol=1e-10, rtol=0))
    gates = dict(finite=finite, clock_continuous=time_ok, no_warnings=batch_warnings == 0,
                 all_chassis_upright_at_20Hz=min_up > 0)
    row = dict(c, duration_s=(len(states)-start)*assets.dt,
               forward_displacement_m=float(displacement[1]), lateral_displacement_m=float(displacement[0]),
               mean_forward_speed_m_s=float(displacement[1]/((len(states)-start)*assets.dt)),
               min_chassis_up_z=min_up, rms_body_tracking_error_deg=float(np.rad2deg(np.sqrt(np.mean(error**2)))),
               rms_pitch_tracking_error_deg=float(np.rad2deg(np.sqrt(np.mean(pitch_error**2)))),
               body_joint_actual_peak_deg=float(np.rad2deg(np.max(np.abs(states[start:, 1 + qbody])))),
               batch_warning_count=batch_warnings, feasible=all(gates.values()), gates=gates)
    return row, poses


def simulate_batch(assets, engine, cases, return_states=False):
    import mujoco
    controls, starts, warms = [], [], []
    for c in cases:
        u, start, warm = build_controls(assets, c['N_wave'], c['amplitude_deg'], c['phase_deg'],
                                       c['evaluation_cycles'], c['warmup_cycles'])
        controls.append(u); starts.append(start); warms.append(warm)
    controls = np.array(controls)
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    initial = np.empty(mujoco.mj_stateSize(assets.model, spec))
    mujoco.mj_getState(assets.model, mujoco.MjData(assets.model), initial, spec)
    data = [mujoco.MjData(assets.model) for _ in range(engine.nthread)]
    states, _ = engine.rollout(assets.model, data, np.repeat(initial[None], len(cases), axis=0),
                              controls, control_spec=mujoco.mjtState.mjSTATE_CTRL.value, chunk_size=1)
    warnings = sum(int(w.number) for d in data for w in d.warning)
    rows, traces = [], []
    for j, c in enumerate(cases):
        row, trace = extract(assets, controls[j], starts[j], states[j], spec, c, warnings)
        rows.append(row); traces.append(trace)
    return (rows, traces, states, controls, starts, warms) if return_states else rows


def run_cases(assets, engine, cases, rows):
    seen = {key(r) for r in rows}
    pending = [c for c in cases if key(c) not in seen]
    for start in range(0, len(pending), engine.nthread):
        batch = pending[start:start + engine.nthread]
        begin = time.perf_counter()
        rows.extend(simulate_batch(assets, engine, batch))
        iid._write_csv(OUTPUT / 'trials.csv', rows)
        write_json('trials.json', rows)
        print(f"{batch[0]['stage']}: {min(start+len(batch),len(pending))}/{len(pending)} new runs; "
              f"{len(rows)} total; batch {time.perf_counter()-begin:.1f}s", flush=True)
    return rows


def primary(rows, wave):
    return [r for r in rows if r['N_wave']==wave and r['phase_deg']==0 and
            r['evaluation_cycles']==12 and r['warmup_cycles']==4]


def best(rows, wave):
    return max((r for r in primary(rows, wave) if r['feasible']), key=lambda r:r['forward_displacement_m'])


def summarize(rows):
    optimal = []
    for w in WAVES:
        b = best(rows, w)
        base = next(r for r in primary(rows, w) if r['amplitude_deg']==15)
        matching = [r for r in rows if r['N_wave']==w and r['amplitude_deg']==b['amplitude_deg']
                    and r['evaluation_cycles']==12 and r['warmup_cycles']==4]
        values = [r['forward_displacement_m'] for r in matching]
        long_best = next((r for r in rows if r['N_wave']==w and r['amplitude_deg']==b['amplitude_deg']
                          and r['evaluation_cycles']==24 and r['phase_deg']==0), None)
        long_base = next((r for r in rows if r['N_wave']==w and r['amplitude_deg']==15
                          and r['evaluation_cycles']==24 and r['phase_deg']==0), None)
        optimal.append(dict(N_wave=w, optimum_half_amplitude_deg=b['amplitude_deg'],
                            forward_displacement_m=b['forward_displacement_m'],
                            baseline_15deg_displacement_m=base['forward_displacement_m'],
                            improvement_percent=100*(b['forward_displacement_m']/base['forward_displacement_m']-1),
                            boundary_optimum=b['amplitude_deg'] in (0.,40.),
                            phase_check_count=len(values), phase_check_min_m=min(values),
                            phase_check_max_m=max(values), phase_check_mean_m=float(np.mean(values)),
                            all_phase_checks_feasible=all(r['feasible'] for r in matching),
                            long_37p7s_optimum_displacement_m=long_best['forward_displacement_m'] if long_best else None,
                            long_37p7s_baseline_displacement_m=long_base['forward_displacement_m'] if long_base else None,
                            long_optimum_feasible=long_best['feasible'] if long_best else None))
    winner = max(optimal, key=lambda r:r['forward_displacement_m'])
    iid._write_csv(OUTPUT / 'optimal_amplitudes.csv', optimal)
    result = dict(best_per_wave=optimal, best_over_tested_waves=winner,
                  trial_count=len(rows), feasible_count=sum(r['feasible'] for r in rows),
                  scope='Bounded sampled maximum, not a continuous/global/real-hardware guarantee.')
    write_json('results.json', result)
    return result


def plots(rows, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':11, 'axes.spines.top':False, 'axes.spines.right':False})
    folder = OUTPUT / 'figures'; folder.mkdir(exist_ok=True)
    def save(fig, name):
        fig.savefig(folder / f'{name}.png', dpi=220, bbox_inches='tight')
        fig.savefig(folder / f'{name}.svg', bbox_inches='tight'); plt.close(fig)
    fig, ax = plt.subplots(figsize=(8.6,5.2))
    for i,w in enumerate(WAVES):
        selected=sorted(primary(rows,w), key=lambda r:r['amplitude_deg'])
        valid=[r for r in selected if r['feasible']]
        line,=ax.plot([r['amplitude_deg'] for r in valid], [r['forward_displacement_m'] for r in valid],
                      '.-', markersize=3, label=f'{w:g}')
        b=best(rows,w); ax.plot(b['amplitude_deg'], b['forward_displacement_m'], '*', color=line.get_color(), ms=12)
    ax.set(xlabel='Body-yaw half-amplitude (deg)', ylabel='Forward displacement (m)', xlim=(-.5,40.8))
    ax.grid(alpha=.18); ax.legend(title=r'$N_{\mathrm{wave}}$', ncol=2, frameon=False)
    save(fig,'amplitude_vs_displacement')
    opt=summary['best_per_wave']; x=np.array(WAVES)
    fig,ax=plt.subplots(figsize=(6.4,4.2))
    ax.plot(x,[r['optimum_half_amplitude_deg'] for r in opt],'o-',color='#117c85')
    ax.axhline(15,color='gray',ls='--',lw=1,label='Previous 15 deg')
    ax.set(xlabel=r'$N_{\mathrm{wave}}$',ylabel='Best half-amplitude (deg)',ylim=(0,43),xticks=x)
    ax.grid(alpha=.18); ax.legend(frameon=False); save(fig,'best_amplitude_vs_wave_number')
    fig,ax=plt.subplots(figsize=(6.4,4.2))
    ax.plot(x,[r['forward_displacement_m'] for r in opt],'o-',label='Optimized amplitude',color='#117c85')
    ax.plot(x,[r['baseline_15deg_displacement_m'] for r in opt],'o--',label='15 deg baseline',color='#777777')
    ax.set(xlabel=r'$N_{\mathrm{wave}}$',ylabel='Forward displacement (m)',xticks=x)
    ax.grid(alpha=.18); ax.legend(frameon=False); save(fig,'optimized_vs_15deg')


def render(assets, engine, selected):
    import mujoco
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw
    folder=OUTPUT/'demos'; folder.mkdir(exist_ok=True)
    manifest=[]
    rows,traces,states,controls,starts,warms=simulate_batch(assets,engine,selected,True)
    recorded=json.loads((OUTPUT/'results.json').read_text())['best_per_wave']
    for demo_index,c in enumerate(selected):
        reference=next(r['forward_displacement_m'] for r in recorded if r['N_wave']==c['N_wave'])
        replay_error=abs(rows[demo_index]['forward_displacement_m']-reference)
        assert replay_error < 1e-6 and rows[demo_index]['feasible']
        output=folder/f"Nwave{iid._wave_tag(c['N_wave'])}_A{c['amplitude_deg']:04.1f}deg_original.mp4"
        model=assets.model; d=mujoco.MjData(model)
        spec=mujoco.mjtState.mjSTATE_FULLPHYSICS.value
        model.vis.global_.offwidth=max(model.vis.global_.offwidth,960)
        model.vis.global_.offheight=max(model.vis.global_.offheight,600)
        renderer=mujoco.Renderer(model,height=600,width=960)
        camera,option=iid.base._camera_for(model,assets.root_body_id,iid.N_SEGMENTS,mujoco)
        controller=AmplitudeController(model.nu,assets.indices,c['N_wave'],c['amplitude_deg'])
        trace=traces[demo_index]; start_com=trace[0,1:4]
        preview=output.with_name(output.stem+'_preview.png')
        with imageio.get_writer(str(output),fps=30,codec='libx264',quality=8,pixelformat='yuv420p',macro_block_size=2) as writer:
            for frame in range(180):
                elapsed=(frame+1)/30; step=round(elapsed/assets.dt)-1; idx=starts[demo_index]+step
                mujoco.mj_setState(model,d,states[demo_index,idx],spec); d.ctrl[:]=controls[demo_index,idx]
                mujoco.mj_forward(model,d)
                flags=controller.stance_flags(warms[demo_index]+step*assets.dt)
                iid._set_leg_colors(model,assets.body_map,flags)
                renderer.update_scene(d,camera=camera,scene_option=option)
                canvas=Image.new('RGB',(960,682),'#eef1f2'); canvas.paste(Image.fromarray(renderer.render().copy()),(0,44))
                draw=ImageDraw.Draw(canvas)
                draw.text((15,9),f"N_wave={c['N_wave']:g} | Body amplitude +/-{c['amplitude_deg']:g} deg | Leg yaw fixed",font=iid._font(19,bold=True),fill='#17202a')
                disp=float(d.subtree_com[assets.root_body_id,1]-start_com[1])
                draw.text((15,652),f"Original contact | t={elapsed:.2f} s | forward displacement={disp:+.3f} m",font=iid._font(16),fill='#273746')
                draw.ellipse((783,657,795,669),fill='#2aa657'); draw.text((801,652),'down',font=iid._font(15),fill='#17202a')
                draw.ellipse((876,657,888,669),fill='#e6452b'); draw.text((896,652),'up',font=iid._font(15),fill='#17202a')
                writer.append_data(np.asarray(canvas))
                if frame==89: canvas.save(preview)
        renderer.close()
        with imageio.get_reader(str(output)) as reader:
            count=reader.count_frames(); assert count==180; reader.get_data(179)
        manifest.append(dict(c,video=str(output),preview=str(preview),frames=count,video_duration_s=6,
                             video_displacement_m=disp,evaluation_displacement_m=rows[demo_index]['forward_displacement_m'],
                             evaluation_replay_error_m=replay_error,video_decode_passed=True))
        iid._write_csv(folder/(output.stem+'_trajectory.csv'),
                       [dict(zip(['time_s','com_x_m','com_y_m','com_z_m','min_chassis_up_z'],r)) for r in trace])
        print(f"DEMO {output.name}: video D={disp:.3f}m",flush=True)
    iid._write_csv(OUTPUT/'demo_manifest.csv',manifest); write_json('demo_manifest.json',manifest)
    write_json('demo_validation.json',dict(passed=len(manifest)==len(selected),demo_count=len(manifest),
                all_180_frames=True,all_replay_errors_below_1e_6_m=True))


def report(summary, config):
    lines=['# Original-contact body-amplitude optimization (Step 35)','',
           'Only body-yaw amplitude varies. Physical N=10 center-rooted STL model, original cave-grouped binary pitch, fixed leg-yaw targets. No IID, no erasure, no model/armature/gain/solver changes.','',
           'Objective: D = COM_y(t_end) - COM_y(t_start), in metres along fixed world +y. Not path length, not absolute value of signed displacement, not normalized performance. Each primary trial has 1.5 s settling, 0.4 s cosine bridge, 4 warm-up cycles, then 12 evaluation cycles (18.850 s after timestep rounding). Warm-up displacement is excluded.','',
           'Search: N_wave = 0.5, 1, ..., 4; commanded body-joint half-amplitude A = 0..40 deg, 2.5-deg coarse grid, then 0.5-deg refinement within +/-2.5 deg of each coarse winner. Existing actuator bounds are +/-0.7 rad; these are not relaxed. A denotes joint target half-amplitude, not lateral body excursion or measured achieved joint angle.','',
           'Primary ranking uses the common phase-zero initialization. Selected winners and 15-deg baselines are separately checked at 90/180/270-degree starting phase, and for 24 evaluation cycles. These are deterministic sensitivity checks, not independent noise trials or confidence intervals. Numerical gates and tracking diagnostics are separate from displacement.','',
           '| N_wave | Best A (deg) | D (m) | D at 15 deg (m) | Gain (%) |','|---|---|---|---|---|']
    for r in summary['best_per_wave']:
        lines.append(f"| {r['N_wave']:g} | {r['optimum_half_amplitude_deg']:g} | {r['forward_displacement_m']:.4f} | {r['baseline_15deg_displacement_m']:.4f} | {r['improvement_percent']:.1f} |")
    lines.extend(['','## Limits of the claim','',
                  '- Best means the best feasible point sampled under this protocol; not a proof of a continuous global optimum. A winner at 40 deg is boundary-limited; a larger unconstrained optimum has not been ruled out.',
                  '- Existing model enables explicit robot-ground contact pairs but not robot self-collision. Large-amplitude winners require hardware-clearance validation before real-robot use; no claim of self-collision safety is made.',
                  '- Actuator torque clipping and tracking lag remain part of the unchanged model. Target amplitude is not achieved amplitude.',
                  '- The original 15-degree baseline and previous datasets remain unchanged. No optimizer winner is silently promoted into shared controller defaults.',
                  '- All upright checks inspect all ten chassis at 20 Hz; this is not a timestep/solver convergence proof.','',
                  'Files: trials.csv/json; optimal_amplitudes.csv; results.json; controller_validation.json; figures/; demos/ (6-second demonstrations, not the full 18.850-second objective).'])
    (OUTPUT/'README.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def main():
    import mujoco
    import mujoco.rollout
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['smoke','sweep','finalize','demos'],default='sweep')
    p.add_argument('--workers',type=int,default=8)
    args=p.parse_args()
    OUTPUT.mkdir(parents=True,exist_ok=True)
    assets=iid._load_assets(ROOT)
    model_path=iid._model_path(ROOT); model_hash=hashlib.sha256(model_path.read_bytes()).hexdigest()
    previous=json.loads((ROOT/'exp_data/reliability'/original.OUTPUT_NAME/'run_configuration.json').read_text())
    assert model_hash==previous['model_sha256']
    assert mujoco.__version__==previous['mujoco_version']
    config=dict(n_segments=10,N_wave=WAVES,amplitude_range_deg=[0,40],coarse_step_deg=2.5,
                refinement_step_deg=.5,body_omega_rad_s=iid.OMEGA_BODY,original_contact=True,iid=False,
                leg_yaw_targets_deg={'R':-5,'L':5},pitch_targets_deg={'down':5,'up':-40},
                primary_evaluation_cycles=12,primary_duration_s=math.ceil(12*PERIOD/assets.dt)*assets.dt,
                warmup_cycles=4,dt_s=assets.dt,model_sha256=model_hash,mujoco_version=mujoco.__version__,
                self_collision_enabled=False,objective='signed whole-robot COM displacement along world +y',
                workers=args.workers,controller_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    config['control_builder_sha256']=hashlib.sha256(inspect.getsource(build_controls).encode()).hexdigest()
    config['measurement_function_sha256']=hashlib.sha256(inspect.getsource(extract).encode()).hexdigest()
    write_json('run_configuration.json',config)
    audit(assets)
    path=OUTPUT/'trials.json'; rows=json.loads(path.read_text()) if path.exists() else []
    with mujoco.rollout.Rollout(nthread=args.workers) as engine:
        # Keep thread count explicit without depending on private pool internals.
        engine.nthread=args.workers
        if args.mode=='smoke':
            smoke=simulate_batch(assets,engine,[case(1,15),case(1,40)])
            write_json('smoke.json',smoke); print(json.dumps(smoke,indent=2))
            references=json.loads((ROOT/'exp_data/reliability'/original.OUTPUT_NAME/'demo_manifest.json').read_text())
            checks=[]
            for wave in [1.,1.5]:
                c=case(wave,15,stage='step34_replay'); c['evaluation_cycles']=6/PERIOD
                replay=simulate_batch(assets,engine,[c])[0]
                reference=next(r for r in references if r['N_wave']==wave)['signed_forward_displacement_m']
                error=abs(replay['forward_displacement_m']-reference)
                checks.append(dict(N_wave=wave,reference_m=reference,replay_m=replay['forward_displacement_m'],
                                   absolute_error_m=error,passed=error<1e-5))
            write_json('step34_replay_validation.json',checks); print(json.dumps(checks,indent=2))
            assert all(r['passed'] for r in checks)
            return
        if args.mode=='sweep':
            run_cases(assets,engine,[case(w,a) for w in WAVES for a in np.arange(0,40.01,2.5)],rows)
            fine=[]
            for w in WAVES:
                b=best(rows,w)['amplitude_deg']
                fine.extend(case(w,a,stage='refine') for a in np.arange(max(0,b-2.5),min(40,b+2.5)+.01,.5))
            run_cases(assets,engine,fine,rows)
            check=[]
            for w in WAVES:
                for a in sorted({15.,best(rows,w)['amplitude_deg']}):
                    check.extend(case(w,a,phase=phase,stage='phase_check') for phase in [90,180,270])
            run_cases(assets,engine,check,rows)
            longer=[case(w,a,cycles=24,stage='long_check') for w in WAVES
                    for a in sorted({15.,best(rows,w)['amplitude_deg']})]
            run_cases(assets,engine,longer,rows)
        result=summarize(rows); plots(rows,result); report(result,config)
        keys={key(r) for r in rows}
        checks=dict(model_unchanged=hashlib.sha256(model_path.read_bytes()).hexdigest()==model_hash,
                    coarse_grid_complete=all(key(case(w,a)) in keys for w in WAVES for a in np.arange(0,40.01,2.5)),
                    unique_trial_keys=len(keys)==len(rows),
                    phase_checks_complete=all(key(case(w,a,phase=p)) in keys for w in WAVES
                        for a in {15.,best(rows,w)['amplitude_deg']} for p in [0,90,180,270]),
                    long_checks_complete=all(key(case(w,a,cycles=24)) in keys for w in WAVES
                        for a in {15.,best(rows,w)['amplitude_deg']}),
                    all_selected_feasible=all(best(rows,w)['feasible'] for w in WAVES))
        refined_complete=True
        for w in WAVES:
            coarse_winner=max((r for r in primary(rows,w) if r['stage']=='coarse' and r['feasible']),
                              key=lambda r:r['forward_displacement_m'])['amplitude_deg']
            refined_complete &= all(key(case(w,a)) in keys for a in np.arange(
                max(0,coarse_winner-2.5),min(40,coarse_winner+2.5)+.01,.5))
        checks['refinement_grid_complete']=refined_complete
        write_json('validation.json',dict(passed=all(checks.values()),checks=checks,
                    all_trials_feasible=all(r['feasible'] for r in rows),trial_count=len(rows)))
        if args.mode=='demos':
            render(assets,engine,[case(w,best(rows,w)['amplitude_deg'],stage='demo') for w in WAVES])
        print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    main()
