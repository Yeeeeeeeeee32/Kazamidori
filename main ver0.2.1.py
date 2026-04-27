import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import math
import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkintermapview
import requests
import random
from collections import deque
import json
import webbrowser
import threading
import queue
from rocketpy import Environment, SolidMotor, Rocket, Flight

class KazamidoriUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Kazamidori_Project - 軌道・落下地点シミュレータ")
        self.geometry("1250x880")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.columnconfigure(0, weight=1, minsize=350)
        self.columnconfigure(1, weight=2)
        self.columnconfigure(2, weight=3, minsize=500)
        self.rowconfigure(0, weight=1)

        self.launch_lat = 35.6828
        self.launch_lon = 139.7590
        self.land_lat = self.launch_lat
        self.land_lon = self.launch_lon
        self.r90_radius = 10.0

        self.selected_motor_file = None
        self.selected_motor_name = "(未選択)"
        # Feat 6: keep burn/avg/max thrust as first-class state so the UI
        # label can be refreshed any time (not just right after a CSV load).
        self.motor_burn_time     = None
        self.motor_avg_thrust    = None
        self.motor_max_thrust    = None
        self.motor_burn_time = 0.0
        self.thrust_data = None

        self.surf_wind_history = deque(maxlen=300)
        # Feat 1: time-series storage with elapsed-second timestamps so the
        # wind speed graph can scroll horizontally as new samples arrive.
        # Each element is the tuple (t_sec_since_start, wind_speed_mps).
        self.surf_wind_time_history = deque(maxlen=300)
        self._wind_start_time = time.time()

        # ── 3-D view state (single source of truth) ───────────────────────────────
        self._fixed_elev     = 25       # locked elevation (deg)
        self._fixed_azim     = 45.0     # Feat 4: azimuth clamped to [0, 90]°
        self._azim_updating  = False    # re-entry guard for slider/drag sync
        self._rot_start_x    = None
        self._rot_start_azim = None

        # ── Uncertainty / dispersion settings (editable via Settings window) ──
        self.wind_uncertainty   = 0.20   # σ_wind / wind (dimensionless)
        self.thrust_uncertainty = 0.05   # σ_thrust / thrust
        self.landing_prob       = 90     # integer % for landing-circle confidence

        # ── Lock & Monitor state ───────────────────────────────────────────────
        self._baseline_wind     = None   # dict set at each successful sim
        self._monitor_after_id  = None   # tk after() id for cancellation
        self._settings_win      = None   # Toplevel ref (None or destroyed-ok)
        self._last_sim_data     = None   # cached last sim_data for auto-rerun

        # ── Result + compass bookkeeping (added with the v2 feature pass) ─────
        self._has_sim_result    = False
        self._compass_ax        = None

        # ── Operation mode state (v0.3 — four operation modes) ────────────────
        self.OPERATION_MODES = (
            "自由モード",
            "定点滞空最適化モード",
            "高度最適化モード",
            "有翼最適化モード",
        )
        self.operation_mode_var = tk.StringVar(value="自由モード")
        self.r_max_var          = tk.StringVar(value="50.0")
        self._optimizing        = False
        self._opt_queue         = queue.Queue()
        self._opt_stop_flag     = threading.Event()
        self._opt_thread        = None
        self._opt_progress_win  = None
        self._opt_progress_msg  = None
        self._opt_progress_bar  = None
        self._last_optimization_info = None

        self.create_data_section()
        self.create_profile_section()
        self.create_map_section()

        self.after(500, lambda: self.get_current_location(manual=False))
        self.after(1000, self.simulate_realtime_wind)

        self.update_plots()
        self.after(1000, self.fit_map_bounds)

        self.after(0, self._on_mode_change)

    def on_closing(self):
        if messagebox.askokcancel("終了", "プログラムを終了しますか？"):
            plt.close('all')
            self.quit()
            self.destroy()
            sys.exit()

    def get_current_location(self, manual=False):
        try:
            response = requests.get('https://ipinfo.io/json', timeout=3)
            loc = response.json().get('loc', '')
            if loc:
                lat, lon = loc.split(',')
                self.launch_lat, self.launch_lon = float(lat), float(lon)
                self.lat_entry.delete(0, tk.END)
                self.lat_entry.insert(0, lat)
                self.lon_entry.delete(0, tk.END)
                self.lon_entry.insert(0, lon)
                self.map_widget.set_position(self.launch_lat, self.launch_lon)
                self._clear_previous_landing()
                self.update_plots()
                self.fit_map_bounds()
                if manual:
                    messagebox.showinfo("現在地取得", f"現在地を取得しました:\nLat: {lat}\nLon: {lon}")
        except Exception as e:
            if manual:
                messagebox.showerror("取得エラー", f"現在地の取得に失敗しました。\n{e}")

    def simulate_realtime_wind(self):
        base_wind = float(self.surf_spd_slider.get())
        current_wind = max(0.0, random.gauss(base_wind, base_wind * 0.15))
        if random.random() < 0.05:
            current_wind *= 1.5

        # Feat 1: timestamped sample so the time-series graph can plot
        # (elapsed seconds, wind speed) and scroll as new data arrives.
        t_now = time.time() - self._wind_start_time
        self.surf_wind_history.append(current_wind)
        self.surf_wind_time_history.append((t_now, current_wind))

        avg = sum(self.surf_wind_history) / len(self.surf_wind_history)

        self.wind_avg_label.config(text=f"地表平均: {avg:.1f} m/s")
        self.wind_gust_label.config(text=f"最大瞬間: {max(self.surf_wind_history):.1f} m/s")

        # Fix 7 + Feat 1/2/3: keep the realtime label AND the bottom wind
        # sub-figure (time-series + 10-s average + compass) in sync at 1 Hz.
        try:
            self._update_realtime_wind_label()
        except Exception:
            pass
        try:
            self._update_wind_subplots()
        except Exception:
            pass
        self.after(1000, self.simulate_realtime_wind)

    # ── Wind helper (refactored out of run_simulation for readability) ────────
    @staticmethod
    def _wind_components(spd, dir_deg):
        rad = math.radians(dir_deg)
        return -spd * math.sin(rad), -spd * math.cos(rad)

    @staticmethod
    def _meters_per_degree(lat_deg):
        phi = math.radians(lat_deg)
        m_per_deg_lat = (111132.92
                         - 559.82 * math.cos(2 * phi)
                         + 1.175  * math.cos(4 * phi)
                         - 0.0023 * math.cos(6 * phi))
        m_per_deg_lon = (111412.84 * math.cos(phi)
                         - 93.5    * math.cos(3 * phi)
                         + 0.118   * math.cos(5 * phi))
        return m_per_deg_lat, m_per_deg_lon

    def _offset_to_latlon(self, lat0, lon0, dx_east, dy_north):
        m_lat, m_lon = self._meters_per_degree(lat0)
        return (lat0 + dy_north / m_lat,
                lon0 + dx_east  / m_lon)

    def _make_backfire_trigger(self, backfire_alt):
        triggered = [False]

        def trigger(p, h, y):
            if triggered[0]:
                return True
            if y[5] < 0 and h <= backfire_alt:
                triggered[0] = True
                return True
            return False

        return trigger

    def _gather_sim_params(self):
        try:
            launch_lat   = float(self.lat_entry.get())
            launch_lon   = float(self.lon_entry.get())
            elev         = float(self.elev_spin.get())
            azi          = float(self.azi_spin.get())
            rail         = float(self.rail_entry.get())
            airframe_mass = float(self.mass_entry.get())
            airframe_cg   = float(self.cg_entry.get())
            airframe_len  = float(self.len_entry.get())
            radius        = float(self.radius_entry.get())
            nose_len  = float(self.nose_len_entry.get())
            fin_root  = float(self.fin_root_entry.get())
            fin_tip   = float(self.fin_tip_entry.get())
            fin_span  = float(self.fin_span_entry.get())
            fin_pos   = float(self.fin_pos_entry.get())
            motor_pos      = float(self.motor_pos_entry.get())
            motor_dry_mass = float(self.motor_dry_mass_entry.get())
            backfire_delay = float(self.backfire_delay_entry.get())
            para_cd   = float(self.cd_entry.get())
            para_area = float(self.area_entry.get())
            para_lag  = float(self.lag_entry.get())
        except ValueError:
            messagebox.showerror("入力エラー",
                "未記入のパラメータがあります。\nすべての項目に数値を入力してください。")
            return None

        self.launch_lat = launch_lat
        self.launch_lon = launch_lon

        surf_spd = (sum(self.surf_wind_history) / len(self.surf_wind_history)
                    if self.surf_wind_history
                    else float(self.surf_spd_slider.get()))
        surf_dir = float(self.surf_dir_var.get())
        up_spd   = float(self.up_spd_var.get())
        up_dir   = float(self.up_dir_var.get())

        surf_u, surf_v = self._wind_components(surf_spd, surf_dir)
        up_u,   up_v   = self._wind_components(up_spd,   up_dir)
        wind_u_prof = [(0, 0), (3, surf_u), (100, up_u), (5000, up_u)]
        wind_v_prof = [(0, 0), (3, surf_v), (100, up_v), (5000, up_v)]

        if not self.thrust_data:
            messagebox.showerror("エンジン未選択",
                "エンジンが選択されていません。\n[Load Local CSV] から推力データを読み込んでください。")
            return None

        return {
            'launch_lat': launch_lat, 'launch_lon': launch_lon,
            'elev': elev, 'azi': azi, 'rail': rail,
            'airframe_mass': airframe_mass, 'airframe_cg': airframe_cg,
            'airframe_len': airframe_len, 'radius': radius,
            'nose_len': nose_len, 'fin_root': fin_root, 'fin_tip': fin_tip,
            'fin_span': fin_span, 'fin_pos': fin_pos,
            'motor_pos': motor_pos, 'motor_dry_mass': motor_dry_mass,
            'backfire_delay': backfire_delay,
            'para_cd': para_cd, 'para_area': para_area, 'para_lag': para_lag,
            'surf_spd': surf_spd, 'surf_dir': surf_dir,
            'up_spd': up_spd, 'up_dir': up_dir,
            'wind_u_prof': wind_u_prof, 'wind_v_prof': wind_v_prof,
            'thrust_data': [list(p) for p in self.thrust_data],
            'motor_burn_time': self.motor_burn_time,
        }

    def _simulate_once(self, elev, azi, params):
        try:
            airframe_mass = max(0.01, params['airframe_mass'])
            airframe_len  = max(0.01, params['airframe_len'])
            radius        = max(0.001, params['radius'])
            airframe_cg   = params['airframe_cg']
            nose_len      = params['nose_len']
            fin_root      = params['fin_root']
            fin_tip       = params['fin_tip']
            fin_span      = params['fin_span']
            fin_pos       = params['fin_pos']
            motor_pos     = params['motor_pos']
            motor_dry_mass = params['motor_dry_mass']
            backfire_delay = params['backfire_delay']
            para_cd    = params['para_cd']
            para_area  = params['para_area']
            para_lag   = params['para_lag']
            rail       = params['rail']
            launch_lat = params['launch_lat']
            launch_lon = params['launch_lon']
            wind_u_prof = params['wind_u_prof']
            wind_v_prof = params['wind_v_prof']
            thrust_data = params['thrust_data']
            motor_burn_time = params['motor_burn_time']

            if not thrust_data:
                return {'ok': False, 'error': 'No thrust data'}

            safe_burn_time = max(0.1, motor_burn_time)
            backfire_time  = safe_burn_time + backfire_delay

            I_z  = 0.5  * airframe_mass * (radius ** 2)
            I_xy = (1/12) * airframe_mass * (3 * (radius ** 2) + airframe_len ** 2)

            env = Environment(latitude=launch_lat, longitude=launch_lon, elevation=0)
            env.set_atmospheric_model(
                type="custom_atmosphere", pressure=None, temperature=300,
                wind_u=wind_u_prof, wind_v=wind_v_prof,
            )

            def _build_rocket():
                motor = SolidMotor(
                    thrust_source=thrust_data,
                    burn_time=safe_burn_time,
                    grain_number=1, grain_density=1815,
                    grain_outer_radius=radius * 0.8,
                    grain_initial_inner_radius=0.005,
                    grain_initial_height=0.1,
                    nozzle_radius=radius * 0.8, throat_radius=0.005,
                    interpolation_method="linear",
                    nozzle_position=0,
                    coordinate_system_orientation="nozzle_to_combustion_chamber",
                    dry_mass=motor_dry_mass,
                    dry_inertia=(1e-5, 1e-5, 1e-6),
                    grain_separation=0.0,
                    grains_center_of_mass_position=0.0,
                    center_of_dry_mass_position=0.0,
                )
                rk = Rocket(
                    radius=radius, mass=airframe_mass, inertia=(I_xy, I_xy, I_z),
                    power_off_drag=para_cd, power_on_drag=para_cd,
                    center_of_mass_without_motor=-airframe_cg,
                )
                rk.add_motor(motor, position=-motor_pos)
                rk.add_nose(length=nose_len, kind="vonKarman", position=0.0)
                rk.add_trapezoidal_fins(
                    n=4, root_chord=fin_root, tip_chord=fin_tip,
                    span=fin_span, position=-fin_pos,
                )
                return rk

            # Pass 1: no-chute flight to find apogee + backfire altitude
            rk1 = _build_rocket()
            fl1 = Flight(
                rocket=rk1, environment=env,
                rail_length=rail, inclination=elev, heading=azi,
                terminate_on_apogee=True,
            )
            t1_arr = fl1.z[:, 0]
            z1_arr = fl1.z[:, 1]
            if backfire_time >= t1_arr[-1]:
                backfire_alt = float(z1_arr[-1])
            else:
                idx_bf_p1 = int((np.abs(t1_arr - backfire_time)).argmin())
                backfire_alt = float(z1_arr[idx_bf_p1])
            backfire_alt = max(backfire_alt, 1.0)

            # Pass 2: full flight with altitude-based parachute trigger
            rk2 = _build_rocket()
            trig = self._make_backfire_trigger(backfire_alt)
            rk2.add_parachute(
                "Main",
                cd_s=para_cd * para_area,
                trigger=trig,
                sampling_rate=105,
                lag=para_lag,
            )
            fl2 = Flight(
                rocket=rk2, environment=env,
                rail_length=rail, inclination=elev, heading=azi,
                terminate_on_apogee=False,
            )

            t_vals  = fl2.z[:, 0]
            x_vals  = fl2.x[:, 1]
            y_vals  = fl2.y[:, 1]
            z_vals  = fl2.z[:, 1]
            vz_vals = fl2.vz[:, 1]

            descending = vz_vals < 0
            below_alt  = z_vals <= backfire_alt
            bf_cands   = np.where(descending & below_alt)[0]
            if len(bf_cands) > 0:
                idx_bf = int(bf_cands[0])
            else:
                idx_bf = int(np.argmax(z_vals))

            bf_abs_time    = float(t_vals[idx_bf])
            para_open_time = bf_abs_time + para_lag
            if para_open_time <= t_vals[-1]:
                idx_para = int((np.abs(t_vals - para_open_time)).argmin())
            else:
                idx_para = -1

            apogee_idx = int(np.argmax(z_vals))
            apogee_m   = float(z_vals[apogee_idx])
            impact_x   = float(x_vals[-1])
            impact_y   = float(y_vals[-1])
            r_horiz    = math.hypot(impact_x, impact_y)
            hang_time  = float(t_vals[-1])

            return {
                'ok': True,
                'apogee_m': apogee_m, 'hang_time': hang_time,
                'impact_x': impact_x, 'impact_y': impact_y,
                'r_horiz': r_horiz,
                't_vals': t_vals, 'x_vals': x_vals, 'y_vals': y_vals,
                'z_vals': z_vals, 'vz_vals': vz_vals,
                'idx_bf': idx_bf, 'idx_para': idx_para,
                'bf_abs_time': bf_abs_time, 'para_open_time': para_open_time,
                'backfire_alt': backfire_alt, 'apogee_idx': apogee_idx,
                'elev': elev, 'azi': azi,
            }
        except ZeroDivisionError:
            return {'ok': False,
                    'error': 'ZeroDivisionError (離陸失敗 or unstable attitude)'}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

    def _apply_sim_result_to_ui(self, res, params, override_r90=None):
        x_vals  = res['x_vals'];  y_vals  = res['y_vals']
        z_vals  = res['z_vals'];  vz_vals = res['vz_vals']
        t_vals  = res['t_vals']
        elev    = res['elev'];    azi     = res['azi']

        azi_rad   = math.radians(azi)
        downrange = x_vals * math.sin(azi_rad) + y_vals * math.cos(azi_rad)

        idx_bf   = res['idx_bf']
        idx_para = res['idx_para']
        bf_z_val = float(z_vals[idx_bf]) if 0 <= idx_bf < len(z_vals) else 0.0
        bf_dr    = float(downrange[idx_bf]) if 0 <= idx_bf < len(downrange) else 0.0

        self.land_lat, self.land_lon = self._offset_to_latlon(
            params['launch_lat'], params['launch_lon'],
            res['impact_x'], res['impact_y'])

        apogee_idx = res['apogee_idx']
        fall_time  = t_vals[-1] - t_vals[apogee_idx]
        horiz_dist = res['r_horiz']
        surf_spd   = params['surf_spd']
        wind_sigma_m   = surf_spd * self.wind_uncertainty * max(fall_time, 0.0)
        thrust_sigma_m = self.thrust_uncertainty * horiz_dist
        combined_sigma = math.hypot(wind_sigma_m, thrust_sigma_m)
        z_score = self._prob_to_z(self.landing_prob)
        self.r90_radius = z_score * combined_sigma
        if (override_r90 is not None
                and isinstance(override_r90, (int, float))
                and math.isfinite(override_r90)
                and override_r90 > 0):
            self.r90_radius = float(override_r90)

        self.apogee_label.config(text=f"Apogee: {res['apogee_m']:.1f} m")
        self.velocity_label.config(text=f"Impact Vel: {abs(float(vz_vals[-1])):.1f} m/s")

        self._baseline_wind = {
            "surf_spd": params['surf_spd'], "surf_dir": params['surf_dir'],
            "up_spd":   params['up_spd'],   "up_dir":   params['up_dir'],
        }

        sim_data = {
            'x': x_vals, 'y': y_vals, 'z': z_vals,
            'downrange': downrange,
            'impact_dr': downrange[-1], 'r90': self.r90_radius,
            'wind_u_prof': params['wind_u_prof'],
            'wind_v_prof': params['wind_v_prof'],
            'azi': azi,
            'bf_z': bf_z_val, 'bf_dr': bf_dr,
            'bf_time': res['bf_abs_time'],
            'bf_x': float(x_vals[idx_bf]) if 0 <= idx_bf < len(x_vals) else 0.0,
            'bf_y': float(y_vals[idx_bf]) if 0 <= idx_bf < len(y_vals) else 0.0,
            'para_time': res['para_open_time'],
            'idx_para': idx_para, 'idx_bf': idx_bf,
            'impact_x': res['impact_x'], 'impact_y': res['impact_y'],
            'apogee_m': res['apogee_m'],
        }
        self._has_sim_result = True
        self._last_sim_data  = sim_data
        self.update_plots(sim_data)
        self.fit_map_bounds()
        return sim_data

    def run_simulation(self):
        mode_val = self.operation_mode_var.get() if hasattr(
            self, 'operation_mode_var') else "自由モード"

        if mode_val != "自由モード":
            if self._optimizing:
                messagebox.showinfo("最適化実行中",
                    "既に最適化計算が実行中です。\n完了までお待ちください。")
                return
            params = self._gather_sim_params()
            if params is None:
                return
            try:
                r_max = float(self.r_max_var.get())
                if r_max <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showerror("入力エラー",
                    "指定半径 r_max に正の数値を入力してください。")
                return
            self._run_optimization_threaded(mode_val, params, r_max)
            return

        self._last_optimization_info = None
        self._render_current_params()
        return

    def _render_current_params(self, override_r90=None):
        params = self._gather_sim_params()
        if params is None:
            return False
        res = self._simulate_once(params['elev'], params['azi'], params)
        if not res['ok']:
            if "ZeroDivisionError" in res.get('error', ''):
                messagebox.showerror(
                    "離陸失敗 または 姿勢計算破綻",
                    "シミュレーションの計算が破綻しました。\n\n"
                    "【主な原因】\n"
                    "1. モーターの推力が弱すぎてレールから動き出せなかった\n"
                    "2. 空力パラメータ(CG、フィン等)により機体が極めて不安定\n\n"
                    f"・選択エンジン: {self.selected_motor_name}\n"
                )
            else:
                messagebox.showerror("Sim Error",
                    f"RocketPy実行エラー:\n{res.get('error', 'unknown')}")
            return False
        self._apply_sim_result_to_ui(res, params, override_r90=override_r90)
        return True

    def _monte_carlo_r90(self, elev, azi, base_params,
                         n_trials=8, stop_flag=None):
        distances = []
        succeeded = 0
        rng = random.Random()
        wu = max(self.wind_uncertainty, 0.0)
        tu = max(self.thrust_uncertainty, 0.0)
        dir_sigma_deg = wu * 60.0

        base_surf = max(base_params['surf_spd'], 0.1)
        base_up   = max(base_params['up_spd'],   0.1)
        raw_thrust = base_params['thrust_data']

        for _ in range(n_trials):
            if stop_flag is not None and stop_flag.is_set():
                break
            surf_spd = max(0.0, rng.gauss(base_params['surf_spd'], wu * base_surf))
            up_spd   = max(0.0, rng.gauss(base_params['up_spd'],   wu * base_up))
            surf_dir = base_params['surf_dir'] + rng.gauss(0.0, dir_sigma_deg)
            up_dir   = base_params['up_dir']   + rng.gauss(0.0, dir_sigma_deg)

            s_u, s_v = self._wind_components(surf_spd, surf_dir)
            u_u, u_v = self._wind_components(up_spd,   up_dir)

            thrust_scale = max(0.1, 1.0 + rng.gauss(0.0, tu))
            perturbed_thrust = [[t, T * thrust_scale] for (t, T) in raw_thrust]

            p = dict(base_params)
            p['wind_u_prof'] = [(0, 0), (3, s_u), (100, u_u), (5000, u_u)]
            p['wind_v_prof'] = [(0, 0), (3, s_v), (100, u_v), (5000, u_v)]
            p['thrust_data'] = perturbed_thrust

            r = self._simulate_once(elev, azi, p)
            if r['ok']:
                distances.append(math.hypot(r['impact_x'], r['impact_y']))
                succeeded += 1

        if not distances:
            return float('inf'), 0.0
        distances.sort()
        p_idx = max(0, min(len(distances) - 1,
                           int(round((self.landing_prob / 100.0)
                                     * len(distances))) - 1))
        return distances[p_idx], succeeded / n_trials

    def _optimize_worker(self, mode, base_params, r_max):
        try:
            if mode == "定点滞空最適化モード":
                elev_grid = [60, 66, 72, 78, 84, 90]
                azi_grid  = [0, 30, 60, 90, 120, 150,
                             180, 210, 240, 270, 300, 330]
                use_mc = True

                def objective(res, mc_r=None):
                    if not res['ok']:
                        return float('-inf')
                    r = res['r_horiz']
                    if mc_r is None:
                        if r > r_max:
                            return float('-inf')
                        return (r_max - r) + res['hang_time']
                    if r + mc_r > r_max:
                        return float('-inf')
                    return (r_max - r) + res['hang_time']

            elif mode == "高度最適化モード":
                elev_grid = [60, 66, 72, 78, 84, 90]
                azi_grid  = [0, 45, 90, 135, 180, 225, 270, 315]
                use_mc = True

                def objective(res, mc_r=None):
                    if not res['ok']:
                        return float('-inf')
                    r = res['r_horiz']
                    if mc_r is None:
                        if r > r_max:
                            return float('-inf')
                        return res['apogee_m']
                    if r + mc_r > r_max:
                        return float('-inf')
                    return res['apogee_m']

            elif mode == "有翼最適化モード":
                elev_grid = [60, 66, 72, 78, 84, 90]
                azi_grid  = [0, 45, 90, 135, 180, 225, 270, 315]
                use_mc = True

                def objective(res, mc_r=None):
                    if not res['ok']:
                        return float('-inf')
                    r = res['r_horiz']
                    if mc_r is None:
                        if r > r_max:
                            return float('-inf')
                        return res['hang_time']
                    if r + mc_r > r_max:
                        return float('-inf')
                    return res['hang_time']
            else:
                self._opt_queue.put(('error', f'Unknown mode: {mode}'))
                return

            candidates = []
            total = len(elev_grid) * len(azi_grid)
            done = 0
            phase1_weight = 0.6 if use_mc else 1.0

            self._opt_queue.put(
                ('progress', f"フェーズ1: 粗探索 (0/{total})", 0.0))

            for e_ in elev_grid:
                for a_ in azi_grid:
                    if self._opt_stop_flag.is_set():
                        self._opt_queue.put(('cancelled', None))
                        return
                    res = self._simulate_once(e_, a_, base_params)
                    done += 1
                    if res['ok']:
                        score = objective(res, mc_r=None)
                        candidates.append((score, e_, a_, res))
                    frac = (done / total) * phase1_weight
                    self._opt_queue.put((
                        'progress',
                        f"フェーズ1: 粗探索 ({done}/{total}) "
                        f"elev={e_:.0f}° azi={a_:.0f}°",
                        frac))

            if not candidates:
                self._opt_queue.put((
                    'error',
                    '全ての候補でシミュレーションが失敗しました。\n'
                    'パラメータを確認してください。'))
                return

            candidates.sort(key=lambda x: -x[0] if math.isfinite(x[0])
                                              else float('inf'))

            if not use_mc:
                finite = [c for c in candidates if math.isfinite(c[0])]
                if not finite:
                    self._opt_queue.put((
                        'error',
                        f'制約 (r ≤ {r_max:.1f} m) を満たす候補がありません。\n'
                        'r_max を大きくするか、機体 / 風設定を見直してください。'))
                    return
                score, best_e, best_a, best_res = finite[0]
                best_mc_r = None
            else:
                top_n = min(5, len(candidates))
                mc_trials = 8
                best = None

                for i in range(top_n):
                    if self._opt_stop_flag.is_set():
                        self._opt_queue.put(('cancelled', None))
                        return
                    _, e_, a_, res = candidates[i]
                    mc_r, succ = self._monte_carlo_r90(
                        e_, a_, base_params,
                        n_trials=mc_trials,
                        stop_flag=self._opt_stop_flag)
                    score = objective(res, mc_r=mc_r)
                    phase2_span = (1 - phase1_weight) * 0.75
                    prog_frac = phase1_weight + (i + 1) / top_n * phase2_span
                    self._opt_queue.put((
                        'progress',
                        f"フェーズ2: MC検証 ({i+1}/{top_n}) "
                        f"elev={e_:.0f}° azi={a_:.0f}°  "
                        f"MC r={mc_r:.1f}m (≤{r_max:.1f}m?)",
                        prog_frac))
                    if math.isfinite(score):
                        if best is None or score > best[0]:
                            best = (score, e_, a_, res, mc_r)

                if best is None:
                    self._opt_queue.put((
                        'error',
                        f'制約 (r + MC {self.landing_prob}%円半径 ≤ {r_max:.1f} m) を\n'
                        '満たす候補がありませんでした。\n'
                        'r_max を大きくするか、風 / 機体設定を見直してください。'))
                    return

                score, best_e, best_a, best_res, best_mc_r = best

            if self._opt_stop_flag.is_set():
                self._opt_queue.put(('cancelled', None))
                return
            self._opt_queue.put((
                'progress',
                f"フェーズ3: 最終MC解析 (elev={best_e:.1f}° azi={best_a:.1f}°)",
                0.9))
            final_mc_trials = 16
            final_mc_r, final_mc_succ = self._monte_carlo_r90(
                best_e, best_a, base_params,
                n_trials=final_mc_trials,
                stop_flag=self._opt_stop_flag)
            if self._opt_stop_flag.is_set():
                self._opt_queue.put(('cancelled', None))
                return
            if math.isfinite(final_mc_r):
                reported_mc_r = final_mc_r
            else:
                reported_mc_r = best_mc_r

            self._opt_queue.put(('progress', 'フェーズ3: 完了', 1.0))
            self._opt_queue.put(('done', {
                'mode': mode, 'r_max': r_max,
                'elev': best_e, 'azi': best_a, 'score': score,
                'result': best_res, 'mc_r': reported_mc_r,
                'mc_success': final_mc_succ,
                'mc_trials': final_mc_trials,
            }))
        except Exception as e:
            self._opt_queue.put(('error', f'最適化中にエラー: {e}'))

    def _run_optimization_threaded(self, mode, params, r_max):
        self._optimizing = True
        self._opt_stop_flag.clear()
        try:
            while True:
                self._opt_queue.get_nowait()
        except queue.Empty:
            pass

        win = tk.Toplevel(self)
        self._opt_progress_win = win
        win.title(f"最適化中 — {mode}")
        win.geometry("460x170")
        win.resizable(False, False)
        win.transient(self)

        frm = ttk.Frame(win, padding=12)
        frm.pack(fill='both', expand=True)
        ttk.Label(frm, text=f"モード: {mode}",
                  font=("Arial", 10, "bold")).pack(anchor='w')
        ttk.Label(frm,
                  text=f"指定半径 r_max = {r_max:.1f} m").pack(anchor='w')
        self._opt_progress_msg = tk.StringVar(value="準備中...")
        ttk.Label(frm, textvariable=self._opt_progress_msg,
                  font=("Arial", 9)).pack(anchor='w', pady=(6, 3))
        self._opt_progress_bar = ttk.Progressbar(
            frm, mode='determinate', maximum=100)
        self._opt_progress_bar.pack(fill='x', pady=(0, 8))
        ttk.Button(frm, text="キャンセル",
                   command=self._cancel_optimization).pack(anchor='e')
        win.protocol("WM_DELETE_WINDOW", self._cancel_optimization)

        try:
            self.run_button.state(["disabled"])
        except Exception:
            pass

        self._opt_thread = threading.Thread(
            target=self._optimize_worker,
            args=(mode, params, r_max),
            daemon=True)
        self._opt_thread.start()
        self.after(150, self._poll_optimization)

    def _cancel_optimization(self):
        self._opt_stop_flag.set()
        if self._opt_progress_msg is not None:
            try:
                self._opt_progress_msg.set("キャンセル中...")
            except Exception:
                pass

    def _poll_optimization(self):
        if not self._optimizing:
            return
        drained = False
        try:
            while True:
                msg = self._opt_queue.get_nowait()
                drained = True
                kind = msg[0]
                if kind == 'progress':
                    _, text, frac = msg
                    if self._opt_progress_msg is not None:
                        try:
                            self._opt_progress_msg.set(text)
                        except Exception:
                            pass
                    if self._opt_progress_bar is not None and frac is not None:
                        try:
                            self._opt_progress_bar['value'] = max(
                                0.0, min(100.0, float(frac) * 100.0))
                        except Exception:
                            pass
                elif kind == 'done':
                    self._finish_optimization(msg[1])
                    return
                elif kind == 'cancelled':
                    self._finish_optimization(None, cancelled=True)
                    return
                elif kind == 'error':
                    self._finish_optimization(None, error=msg[1])
                    return
        except queue.Empty:
            pass
        self.after(150, self._poll_optimization)

    def _finish_optimization(self, payload, cancelled=False, error=None):
        self._optimizing = False
        try:
            if self._opt_progress_win is not None:
                self._opt_progress_win.destroy()
        except Exception:
            pass
        self._opt_progress_win  = None
        self._opt_progress_msg  = None
        self._opt_progress_bar  = None
        try:
            self.run_button.state(["!disabled"])
        except Exception:
            pass

        if cancelled:
            messagebox.showinfo("最適化キャンセル",
                                "最適化を中断しました。\n"
                                "最適値は設定されていません。")
            return
        if error:
            messagebox.showerror("最適化エラー", error)
            return
        if payload is None:
            return

        best_elev = float(payload['elev'])
        best_azi  = float(payload['azi'])
        res       = payload['result']
        mc_r      = payload.get('mc_r')
        mode      = payload['mode']
        r_max     = payload['r_max']

        try:
            self.elev_spin.delete(0, tk.END)
            self.elev_spin.insert(0, f"{best_elev:.1f}")
        except Exception:
            pass
        try:
            self.azi_spin.delete(0, tk.END)
            self.azi_spin.insert(0, f"{best_azi:.1f}")
        except Exception:
            pass

        try:
            self.update_idletasks()
        except Exception:
            pass

        mode_short_map = {
            "定点滞空最適化モード": "Hover",
            "高度最適化モード":     "Altitude",
            "有翼最適化モード":     "Winged",
        }
        hover_score = None
        if mode == "定点滞空最適化モード":
            try:
                hover_score = (r_max - float(res['r_horiz'])) + float(res['hang_time'])
            except Exception:
                hover_score = None

        self._last_optimization_info = {
            'elev':        best_elev,
            'azi':         best_azi,
            'mode':        mode,
            'mode_short':  mode_short_map.get(mode, mode),
            'mc_r':        mc_r,
            'r_max':       r_max,
            'hover_score': hover_score,
        }

        final_ok = self._render_current_params(override_r90=mc_r)
        if not final_ok:
            messagebox.showwarning(
                "最終再現シミュレーション失敗",
                "最適値の最終再現計算に失敗しました。\n"
                "最適角度はUIに反映済みですので、もう一度RUNを押すと再計算されます。")

        lines = [
            f"【{mode} — 最適化完了】",
            "",
            f"★ 最適発射仰角 (Best Elevation): {best_elev:.1f}°",
            f"★ 最適発射方位 (Best Azimuth):   {best_azi:.1f}°",
            "",
            f"Apogee:     {res['apogee_m']:.1f} m",
            f"滞空時間:   {res['hang_time']:.2f} s",
            f"水平距離 r: {res['r_horiz']:.1f} m  /  r_max: {r_max:.1f} m",
        ]
        if mc_r is not None and math.isfinite(mc_r):
            trials = payload.get('mc_trials', 0)
            lines.append(
                f"MC {self.landing_prob}%円半径: {mc_r:.1f} m  "
                f"(≤ r_max={r_max:.1f} m,  試行={trials}回)")
        if hover_score is not None:
            lines.append("")
            lines.append(
                f"★ 定点滞空スコア (r_max - r + t): {hover_score:.2f}")
        lines.append("")
        if final_ok:
            lines.append("最適値はElevation/Azimuth入力欄に自動反映されました。")
            lines.append("3Dプロファイル・マップに最適軌道を描画しました。")
            lines.append("")
            lines.append("⚠ ウィンドモニター (Lock & Monitor) を自動ONにしました。")
            lines.append("   風の変化が許容差を超えるとアラートと再計算が走ります。")
        messagebox.showinfo("最適化完了 / Optimization Complete",
                            "\n".join(lines))

        if final_ok:
            self._auto_enable_monitor_mode()

    _MODE_DEFAULT_RMAX = {
        "自由モード":            None,
        "定点滞空最適化モード":   50.0,
        "高度最適化モード":       250.0,
        "有翼最適化モード":       250.0,
    }

    def _apply_mode_default_rmax(self, mode):
        default = self._MODE_DEFAULT_RMAX.get(mode)
        if default is None:
            return
        try:
            self.r_max_var.set(f"{default:.1f}")
        except Exception:
            pass

    def _on_mode_change(self, event=None):
        mode = self.operation_mode_var.get()
        self._apply_mode_default_rmax(mode)

        if mode == "自由モード":
            try:
                self.rmax_label.grid_remove()
                self.rmax_entry.grid_remove()
            except Exception:
                pass
        else:
            try:
                self.rmax_label.grid()
                self.rmax_entry.grid()
            except Exception:
                pass
            self._release_lock_if_active(reason_label="⭘ Unlocked")
        try:
            self.lock_monitor_check.state(["!disabled"])
        except Exception:
            pass

    # ── Azimuth helpers ───────────────────────────────────────────────────────
    def _set_azim(self, azim, source="code"):
        if self._azim_updating:
            return
        self._azim_updating = True
        try:
            a = float(azim)
            a = ((a + 180.0) % 360.0) - 180.0
            if a < 0:
                a = 0.0
            elif a > 90:
                a = 90.0
            self._fixed_azim = a

            if hasattr(self, 'azim_label'):
                self.azim_label.config(text=f"{a:+.0f}°")

            if source != "slider" and hasattr(self, 'azim_var'):
                try:
                    if abs(self.azim_var.get() - a) > 0.5:
                        self.azim_var.set(a)
                except tk.TclError:
                    pass

            if hasattr(self, 'ax'):
                try:
                    self.ax.view_init(elev=self._fixed_elev, azim=a)
                    if getattr(self, '_compass_ax', None) is not None:
                        self._draw_compass()
                    self.canvas.draw_idle()
                except Exception:
                    pass
        finally:
            self._azim_updating = False

    def _on_azim_slider(self, value):
        self._set_azim(value, source="slider")

    def _reset_azim(self):
        self._set_azim(45.0, source="code")

    def _on_wheel_rotate_azim(self, event, delta_override=None):
        d = delta_override if delta_override is not None else getattr(event, 'delta', 0)
        if d == 0:
            return
        step = 5.0 if d > 0 else -5.0
        new_azim = self._fixed_azim + step
        self._set_azim(new_azim, source="code")
        return "break"

    def _on_canvas_press(self, event):
        if event.inaxes is self.ax and event.button == 1:
            self._rot_start_x    = event.x
            self._rot_start_azim = self._fixed_azim

    def _on_canvas_motion(self, event):
        if self._rot_start_x is None:
            return
        if event.button != 1:
            return
        dx = event.x - self._rot_start_x
        self._set_azim(self._rot_start_azim - dx * 0.4, source="drag")

    def _on_canvas_release(self, event):
        self._rot_start_x = None

    def _on_view_changed(self, event=None):
        if not hasattr(self, 'ax'):
            return
        try:
            ax_azim = float(self.ax.azim)
            ax_elev = float(self.ax.elev)
        except Exception:
            return

        azim_drift = abs(((ax_azim - self._fixed_azim) + 180.0) % 360.0 - 180.0)
        elev_drift = abs(ax_elev - self._fixed_elev)
        if azim_drift < 0.5 and elev_drift < 0.5:
            return

        self._fixed_azim = ((ax_azim + 180.0) % 360.0) - 180.0
        self._fixed_elev = ax_elev

        if hasattr(self, 'azim_var'):
            self._azim_updating = True
            try:
                try:
                    self.azim_var.set(self._fixed_azim)
                except tk.TclError:
                    pass
                if hasattr(self, 'azim_label'):
                    self.azim_label.config(text=f"{self._fixed_azim:+.0f}°")
            finally:
                self._azim_updating = False

        self._draw_compass()
        self.canvas.draw_idle()

    # ── Uncertainty helpers ──────────────────────────────────────────────────
    def _prob_to_z(self, pct):
        table = {50: 0.674, 68: 1.000, 80: 1.282, 85: 1.440,
                 90: 1.645, 95: 1.960, 99: 2.576}
        return table.get(int(pct), 1.645)

    def _open_settings_window(self):
        if self._settings_win is not None:
            try:
                if self._settings_win.winfo_exists():
                    self._settings_win.lift()
                    return
            except tk.TclError:
                pass

        win = tk.Toplevel(self)
        self._settings_win = win
        win.title("Uncertainty Settings")
        win.geometry("380x260")
        win.resizable(False, False)
        win.transient(self)

        frm = ttk.Frame(win, padding=14)
        frm.pack(fill="both", expand=True)
        frm.columnconfigure(0, weight=1)
        frm.columnconfigure(1, weight=0)

        ttk.Label(frm, text="Monte-Carlo / Uncertainty Parameters",
                  font=("Arial", 10, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))

        ttk.Label(frm, text="Wind speed uncertainty (%):").grid(
            row=1, column=0, sticky="w", pady=4)
        wind_var = tk.StringVar(value=f"{self.wind_uncertainty * 100:.1f}")
        ttk.Entry(frm, textvariable=wind_var, width=10).grid(
            row=1, column=1, sticky="e", pady=4)

        ttk.Label(frm, text="Motor thrust uncertainty (±%):").grid(
            row=2, column=0, sticky="w", pady=4)
        thrust_var = tk.StringVar(value=f"{self.thrust_uncertainty * 100:.1f}")
        ttk.Entry(frm, textvariable=thrust_var, width=10).grid(
            row=2, column=1, sticky="e", pady=4)

        ttk.Label(frm, text="Landing circle probability (%):").grid(
            row=3, column=0, sticky="w", pady=4)
        prob_var = tk.StringVar(value=str(self.landing_prob))
        prob_combo = ttk.Combobox(frm, textvariable=prob_var, width=8,
                                  values=[50, 68, 80, 85, 90, 95, 99],
                                  state="normal")
        prob_combo.grid(row=3, column=1, sticky="e", pady=4)

        ttk.Label(frm,
                  text="(Re-run the simulation to apply the new settings.)",
                  font=("Arial", 8), foreground="gray").grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(6, 10))

        btn_f = ttk.Frame(frm)
        btn_f.grid(row=5, column=0, columnspan=2, sticky="ew")
        btn_f.columnconfigure(0, weight=1)
        btn_f.columnconfigure(1, weight=1)

        def apply_and_close():
            try:
                w  = float(wind_var.get())   / 100.0
                th = float(thrust_var.get()) / 100.0
                p  = int(float(prob_var.get()))
                if w < 0 or th < 0:
                    raise ValueError("Uncertainty must be ≥ 0.")
                if not 1 <= p <= 99:
                    raise ValueError("Probability must be between 1 and 99.")
                prob_changed = (p != self.landing_prob)
                self.wind_uncertainty   = w
                self.thrust_uncertainty = th
                self.landing_prob       = p
                if prob_changed:
                    self._release_lock_if_active(reason_label="⭘ Unlocked")
                messagebox.showinfo(
                    "Settings Applied",
                    f"Wind uncertainty   : ±{w*100:.1f}%\n"
                    f"Thrust uncertainty: ±{th*100:.1f}%\n"
                    f"Landing confidence: {p}%",
                    parent=win)
                win.destroy()
                self._settings_win = None
            except ValueError as e:
                messagebox.showerror("Invalid input",
                                     f"Could not parse settings:\n{e}",
                                     parent=win)

        def cancel():
            win.destroy()
            self._settings_win = None

        ttk.Button(btn_f, text="Apply & Close",
                   command=apply_and_close).grid(row=0, column=0,
                                                 sticky="ew", padx=(0, 3))
        ttk.Button(btn_f, text="Cancel",
                   command=cancel).grid(row=0, column=1,
                                        sticky="ew", padx=(3, 0))
        win.protocol("WM_DELETE_WINDOW", cancel)

    # ── Lock & Monitor helpers ───────────────────────────────────────────────
    def _release_lock_if_active(self, reason_label=None):
        if not hasattr(self, 'lock_monitor_var'):
            return
        was_locked = False
        try:
            was_locked = bool(self.lock_monitor_var.get())
        except Exception:
            pass
        if was_locked:
            try:
                self.lock_monitor_var.set(False)
                self._toggle_lock_monitor()
            except Exception:
                pass
        if reason_label is not None:
            try:
                self.monitor_status_label.config(
                    text=reason_label, foreground="gray", background="")
            except Exception:
                pass

    def _toggle_lock_monitor(self):
        if self.lock_monitor_var.get():
            self._capture_wind_baseline()
            try:
                self.run_button.state(["disabled"])
            except Exception:
                pass
            self.monitor_status_label.config(
                text="🔒 LOCKED — monitoring wind",
                foreground="green")
            self._schedule_monitor_tick()
        else:
            try:
                self.run_button.state(["!disabled"])
            except Exception:
                pass
            self.monitor_status_label.config(
                text="⭘ Unlocked",
                foreground="gray")
            if self._monitor_after_id is not None:
                try:
                    self.after_cancel(self._monitor_after_id)
                except Exception:
                    pass
                self._monitor_after_id = None

    def _auto_enable_monitor_mode(self):
        try:
            if not self.lock_monitor_var.get():
                self.lock_monitor_var.set(True)
                self._toggle_lock_monitor()
            else:
                self._capture_wind_baseline()
        except Exception:
            pass

    def _wind_avg_recent(self, window_sec=10.0):
        """Feat 3: rolling average of surface wind over the past ``window_sec``.

        Used both as the live reference line on the wind-speed time series
        and as the comparison value inside ``_monitor_wind_tick`` so the
        monitor reacts to *sustained* drift rather than single-tick noise.
        """
        history = list(getattr(self, 'surf_wind_time_history', []) or [])
        if not history:
            try:
                return float(self.surf_spd_slider.get())
            except Exception:
                return 0.0
        t_latest = history[-1][0]
        recent = [w for (t, w) in history if t >= t_latest - window_sec]
        if not recent:
            recent = [history[-1][1]]
        return sum(recent) / len(recent)

    def _capture_wind_baseline(self):
        """Snapshot current wind values to compare against during monitoring.

        Feat 3 / Fix 6: the surface baseline is the past-10-second moving
        average rather than the long-running average over the full history,
        so a fresh lock reflects the *current* atmospheric state. Using the
        same window on both sides of the comparison (baseline AND
        ``_monitor_wind_tick``'s ``cur_surf``) ensures the monitor reacts to
        real drift instead of being smothered by long-window smoothing.
        """
        try:
            surf_spd = self._wind_avg_recent(window_sec=10.0)
            self._baseline_wind = {
                "surf_spd": surf_spd,
                "surf_dir": float(self.surf_dir_var.get()),
                "up_spd":   float(self.up_spd_var.get()),
                "up_dir":   float(self.up_dir_var.get()),
            }
        except (ValueError, AttributeError):
            self._baseline_wind = None

    def _schedule_monitor_tick(self):
        self._monitor_after_id = self.after(2000, self._monitor_wind_tick)

    @staticmethod
    def _angle_diff(a, b):
        return abs(((a - b) + 180.0) % 360.0 - 180.0)

    def _monitor_wind_tick(self):
        """Periodic wind check.

        Fix 6 — Tolerance logic:
          The previous version compared the *long-running* average of
          ``surf_wind_history`` (300-sample deque ≈ 5 minutes) against
          itself — both baseline and current ended up nearly equal,
          so the threshold was effectively never crossed. We now use
          the past-10-second rolling average (matches the red horizontal
          line on the time-series graph) on BOTH sides of the comparison,
          so a real change in conditions actually crosses the threshold.

        Fix 7 — ``_flash_alert`` removed:
          The old call to ``self._flash_alert()`` raised
          ``AttributeError: '_tkinter.tkapp' object has no attribute '_flash_alert'``
          because that helper never existed.  We now show a blocking
          ``messagebox.showwarning`` pop-up and then automatically execute
          ``run_simulation()`` to recompute the trajectory under the new wind.
        """
        self._monitor_after_id = None
        if not self.lock_monitor_var.get():
            return
        if self._baseline_wind is None:
            self._schedule_monitor_tick()
            return

        try:
            # Fix 6: 10-second rolling avg, matching how the baseline was captured.
            cur_surf     = self._wind_avg_recent(window_sec=10.0)
            cur_surf_dir = float(self.surf_dir_var.get())
            cur_up       = float(self.up_spd_var.get())
            cur_up_dir   = float(self.up_dir_var.get())
        except (ValueError, AttributeError):
            self._schedule_monitor_tick()
            return

        b = self._baseline_wind
        SPD_TOL, DIR_TOL = 2.0, 15.0

        # Fix 6: explicit absolute differences — easier to reason about and
        # easier to surface in the warning popup.
        surf_spd_diff = abs(cur_surf     - b["surf_spd"])
        up_spd_diff   = abs(cur_up       - b["up_spd"])
        surf_dir_diff = self._angle_diff(cur_surf_dir, b["surf_dir"])
        up_dir_diff   = self._angle_diff(cur_up_dir,   b["up_dir"])

        exceeded = (
            surf_spd_diff > SPD_TOL or
            up_spd_diff   > SPD_TOL or
            surf_dir_diff > DIR_TOL or
            up_dir_diff   > DIR_TOL
        )

        if exceeded:
            # Fix 7: messagebox.showwarning replaces the missing _flash_alert.
            # showwarning is modal — it returns only after the user dismisses
            # it, so the ensuing run_simulation() call is sequential and safe.
            messagebox.showwarning(
                "風速変化警告 / Wind Tolerance Exceeded",
                "風の状態が許容差を超えました。\n"
                "自動的に再計算を実行します。\n\n"
                f"地表風速: 基準 {b['surf_spd']:.1f} m/s → 現在 {cur_surf:.1f} m/s "
                f"(Δ={surf_spd_diff:.2f}, 許容±{SPD_TOL:.1f})\n"
                f"地表風向: 基準 {b['surf_dir']:.0f}° → 現在 {cur_surf_dir:.0f}° "
                f"(Δ={surf_dir_diff:.0f}°, 許容±{DIR_TOL:.0f}°)\n"
                f"上層風速: 基準 {b['up_spd']:.1f} m/s → 現在 {cur_up:.1f} m/s "
                f"(Δ={up_spd_diff:.2f}, 許容±{SPD_TOL:.1f})\n"
                f"上層風向: 基準 {b['up_dir']:.0f}° → 現在 {cur_up_dir:.0f}° "
                f"(Δ={up_dir_diff:.0f}°, 許容±{DIR_TOL:.0f}°)"
            )
            # Temporarily allow the internal RUN call, then re-lock.
            try:
                self.run_button.state(["!disabled"])
            except Exception:
                pass
            try:
                self.run_simulation()
            except Exception as e:
                messagebox.showerror("再計算エラー",
                                     f"自動再計算に失敗しました:\n{e}")
            try:
                self.run_button.state(["disabled"])
            except Exception:
                pass
            # Refresh baseline so the next tick compares against the NEW state,
            # not the just-superseded one (prevents repeated immediate retriggers).
            self._capture_wind_baseline()

        self._schedule_monitor_tick()

    # ── Realtime wind display ────────────────────────────────────────────────
    def _update_realtime_wind_label(self):
        lbl = getattr(self, 'realtime_wind_label', None)
        if lbl is None:
            return
        try:
            surf_spd, surf_dir, up_spd, up_dir = self._read_current_wind()
            hist = getattr(self, 'surf_wind_history', None) or []
            gust = max(hist) if hist else surf_spd
            # Feat 4: show only numerical values — no "Now:" prefix needed.
            lbl.config(
                text=(f"Surface: {surf_spd:.1f} m/s  @ {surf_dir:.0f}°"
                      f"   (Gust {gust:.1f})"
                      f"   |   Upper: {up_spd:.1f} m/s @ {up_dir:.0f}°"),
            )
        except Exception:
            pass

    # ── Params scroll helpers ────────────────────────────────────────────────
    def _on_params_wheel(self, event, delta_override=None):
        canvas = getattr(self, '_params_canvas', None)
        if canvas is None:
            return "break"
        d = delta_override if delta_override is not None else getattr(event, 'delta', 0)
        try:
            step = -1 if d > 0 else 1
            canvas.yview_scroll(step, "units")
        except Exception:
            pass
        return "break"

    def _bind_params_wheel_recursive(self, widget):
        try:
            widget.bind("<MouseWheel>", self._on_params_wheel, add="+")
            widget.bind(
                "<Button-4>",
                lambda e: self._on_params_wheel(e, delta_override=+120),
                add="+",
            )
            widget.bind(
                "<Button-5>",
                lambda e: self._on_params_wheel(e, delta_override=-120),
                add="+",
            )
        except Exception:
            pass
        try:
            for child in widget.winfo_children():
                self._bind_params_wheel_recursive(child)
        except Exception:
            pass

    # ── Layout helper ────────────────────────────────────────────────────────
    def _apply_safe_layout(self):
        import warnings
        try:
            if getattr(self, 'ax', None) is not None:
                self.ax.set_position(list(self._PLOT_RECT))
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', UserWarning)
                self.fig.subplots_adjust(
                    left=self._PLOT_RECT[0],
                    right=self._PLOT_RECT[0] + self._PLOT_RECT[2],
                    bottom=self._PLOT_RECT[1],
                    top=self._PLOT_RECT[1] + self._PLOT_RECT[3],
                )
        except Exception:
            pass

    # ── Compass + fixed-label drawing helpers ────────────────────────────────
    def _draw_compass(self):
        if getattr(self, '_compass_ax', None) is not None:
            try:
                self._compass_ax.remove()
            except Exception:
                pass
            self._compass_ax = None

        cax = self.fig.add_axes([0.83, 0.04, 0.14, 0.14], facecolor='none',
                                zorder=20)
        self._compass_ax = cax
        cax.set_xlim(-1.4, 1.4); cax.set_ylim(-1.4, 1.4)
        cax.set_aspect('equal')
        cax.set_xticks([]); cax.set_yticks([])
        for sp in cax.spines.values():
            sp.set_visible(False)

        cax.add_patch(plt.Circle((0, 0), 1.15, fill=True,
                                 facecolor='white', edgecolor='gray',
                                 lw=0.8, alpha=0.85))

        a = math.radians(self._fixed_azim)
        e = math.radians(self._fixed_elev)
        def proj(vx, vy):
            sx = vx * (-math.sin(a)) + vy * math.cos(a)
            sy = (vx * (-math.sin(e) * math.cos(a))
                  + vy * (-math.sin(e) * math.sin(a)))
            return sx, sy

        def norm(x, y):
            r = math.hypot(x, y) or 1.0
            return x / r, y / r

        nx, ny = norm(*proj(0, 1))
        ex, ey = norm(*proj(1, 0))
        sx, sy = -nx, -ny
        wx, wy = -ex, -ey

        R = 0.78
        cax.annotate('', xy=(nx * R, ny * R), xytext=(0, 0),
                     arrowprops=dict(arrowstyle='-|>', color='red', lw=1.8))
        for dx, dy in ((ex, ey), (sx, sy), (wx, wy)):
            cax.annotate('', xy=(dx * R, dy * R), xytext=(0, 0),
                         arrowprops=dict(arrowstyle='-|>',
                                         color='dimgray', lw=1.0))

        cax.text(nx * 1.10, ny * 1.10, 'N',
                 color='red', fontsize=9, fontweight='bold',
                 ha='center', va='center')
        cax.text(ex * 1.10, ey * 1.10, 'E',
                 color='dimgray', fontsize=8, ha='center', va='center')
        cax.text(sx * 1.10, sy * 1.10, 'S',
                 color='dimgray', fontsize=8, ha='center', va='center')
        cax.text(wx * 1.10, wy * 1.10, 'W',
                 color='dimgray', fontsize=8, ha='center', va='center')

    def _apply_fixed_axis_labels(self):
        self.ax.set_xlabel('')
        self.ax.set_ylabel('')
        self.ax.set_zlabel('')
        self.ax.text2D(0.02, 0.02, 'Altitude (Up)',
                       transform=self.ax.transAxes,
                       ha='left', va='bottom',
                       fontsize=8, fontweight='bold', color='#333333')

    def _clear_previous_landing(self):
        self.land_lat = self.launch_lat
        self.land_lon = self.launch_lon
        self.r90_radius = 0.0
        self._has_sim_result = False
        self._last_sim_data = None

    _PLOT_RECT      = (0.02, 0.00, 0.96, 0.92)
    _TOP_STRIP_FRAC = 0.92

    def update_plots(self, data=None):
        self.fig.clear()
        self._compass_ax = None
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_position(list(self._PLOT_RECT))

        if not data:
            self.ax.scatter([0], [0], [0], marker='^', color='blue',
                            s=60, zorder=6, label='Launch')
            self.ax.set_xlim(-60, 60); self.ax.set_ylim(-60, 60)
            self.ax.set_zlim(0, 60)
            self._apply_fixed_axis_labels()
            self.ax.view_init(elev=self._fixed_elev, azim=self._fixed_azim)
            # Feat 5: two-column legend.
            self.ax.legend(loc='upper right',
                           bbox_to_anchor=(0.98, 0.985),
                           bbox_transform=self.fig.transFigure,
                           ncol=2, fontsize=10, framealpha=0.85)
            self._apply_safe_layout()
            self._draw_compass()
            self.canvas.draw()
            self.draw_map_elements()
            return

        x_vals   = data['x']
        y_vals   = data['y']
        z_vals   = data['z']
        r90      = data['r90']
        impact_x = data['impact_x']
        impact_y = data['impact_y']
        bf_time  = data.get('bf_time')
        bf_x     = data.get('bf_x')
        bf_y     = data.get('bf_y')
        bf_z     = data.get('bf_z')
        para_time = data.get('para_time')
        idx_para  = data.get('idx_para', -1)
        idx_bf    = data.get('idx_bf',   -1)
        wind_u_prof = data['wind_u_prof']
        wind_v_prof = data['wind_v_prof']

        alt_max  = float(np.max(z_vals)) if len(z_vals) > 0 else 100.0
        has_bf   = idx_bf   != -1 and idx_bf   < len(x_vals)
        has_para = idx_para != -1 and idx_para < len(x_vals)

        lw = 2.0
        if has_bf and has_para:
            self.ax.plot(x_vals[:idx_bf+1], y_vals[:idx_bf+1], z_vals[:idx_bf+1],
                         color='royalblue', lw=lw, label='Powered / Coast')
            self.ax.plot(x_vals[idx_bf:idx_para+1], y_vals[idx_bf:idx_para+1], z_vals[idx_bf:idx_para+1],
                         color='darkorange', lw=lw, label='Freefall (post-backfire)')
            self.ax.plot(x_vals[idx_para:], y_vals[idx_para:], z_vals[idx_para:],
                         color='deepskyblue', lw=lw, linestyle='--', label='Under Canopy')
            self.ax.scatter([x_vals[idx_para]], [y_vals[idx_para]], [z_vals[idx_para]],
                            marker='v', color='limegreen', s=60, zorder=5,
                            label='Fully Open')
        elif has_bf:
            self.ax.plot(x_vals[:idx_bf+1], y_vals[:idx_bf+1], z_vals[:idx_bf+1],
                         color='royalblue', lw=lw, label='Powered / Coast')
            self.ax.plot(x_vals[idx_bf:], y_vals[idx_bf:], z_vals[idx_bf:],
                         color='darkorange', lw=lw, label='Freefall (no chute)')
        elif has_para:
            self.ax.plot(x_vals[:idx_para+1], y_vals[:idx_para+1], z_vals[:idx_para+1],
                         color='royalblue', lw=lw, label='Freefall')
            self.ax.plot(x_vals[idx_para:], y_vals[idx_para:], z_vals[idx_para:],
                         color='deepskyblue', lw=lw, linestyle='--', label='Under Canopy')
        else:
            self.ax.plot(x_vals, y_vals, z_vals, color='royalblue', lw=lw, label='Trajectory')

        if len(z_vals) > 0:
            ap_idx = int(np.argmax(z_vals))
            ax_, ay_, az_ = x_vals[ap_idx], y_vals[ap_idx], z_vals[ap_idx]
            self.ax.plot([ax_, ax_], [ay_, ay_], [0, az_],
                         color='gray', linestyle=':', lw=1.2)
            self.ax.scatter([ax_], [ay_], [az_],
                            marker='*', color='gold', s=120, zorder=6,
                            label='Apogee')

        if bf_x is not None and bf_z is not None:
            self.ax.scatter([bf_x], [bf_y], [bf_z],
                            marker='X', color='magenta', s=80, zorder=6,
                            label='Backfire')
            self.ax.plot([bf_x, bf_x], [bf_y, bf_y], [0, bf_z],
                         color='magenta', linestyle=':', lw=1.0, alpha=0.6)

        self.ax.scatter([0], [0], [0], marker='^', color='blue', s=60, zorder=6,
                        label='Launch')

        self.ax.scatter([impact_x], [impact_y], [0],
                        marker='o', color='red', s=60, zorder=6, label='Impact')

        theta = np.linspace(0, 2 * math.pi, 72)
        cx = impact_x + r90 * np.cos(theta)
        cy = impact_y + r90 * np.sin(theta)
        cz = np.zeros_like(theta)
        self.ax.plot(cx, cy, cz, color='red', lw=1.5, alpha=0.6,
                     label=f'Landing Area ({self.landing_prob}%)')

        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        n_pts = 60
        disc_theta = np.linspace(0, 2 * math.pi, n_pts)
        disc_x = impact_x + r90 * np.cos(disc_theta)
        disc_y = impact_y + r90 * np.sin(disc_theta)
        disc_z = np.zeros(n_pts)
        verts  = [list(zip(disc_x, disc_y, disc_z))]
        poly   = Poly3DCollection(verts, alpha=0.12, facecolor='red', edgecolor='none')
        self.ax.add_collection3d(poly)

        self.ax.plot(x_vals, y_vals, np.zeros_like(z_vals),
                     color='gray', lw=0.8, alpha=0.35, linestyle='--')

        z_keys   = [p[0] for p in wind_u_prof]
        u_vals_w = [p[1] for p in wind_u_prof]
        v_vals_w = [p[1] for p in wind_v_prof]
        arrow_len = max(r90 * 0.4, alt_max * 0.12, 3.0)
        for alt in np.linspace(0, alt_max, 6):
            u_a = np.interp(alt, z_keys, u_vals_w)
            v_a = np.interp(alt, z_keys, v_vals_w)
            spd = math.sqrt(u_a**2 + v_a**2)
            if spd < 1e-6:
                continue
            scale = arrow_len * (spd / max(
                math.sqrt(u_vals_w[-1]**2 + v_vals_w[-1]**2), 0.1))
            self.ax.quiver(0, 0, alt,
                           u_a * scale / (spd + 1e-9),
                           v_a * scale / (spd + 1e-9),
                           0,
                           color='limegreen', lw=1.2, arrow_length_ratio=0.3)
            self.ax.text(u_a * scale / (spd + 1e-9),
                         v_a * scale / (spd + 1e-9),
                         alt + alt_max * 0.02,
                         f'{spd:.1f}m/s', color='green', fontsize=7)

        self.ax.view_init(elev=self._fixed_elev, azim=self._fixed_azim)

        self._apply_fixed_axis_labels()
        self.ax.tick_params(labelsize=7)

        downrange_m = math.hypot(impact_x, impact_y)
        apogee_m    = data.get('apogee_m',
                               float(np.max(z_vals)) if len(z_vals) > 0 else 0.0)
        para_str    = f'{para_time:.2f} s' if para_time is not None else '— s'
        bf_str      = f'{bf_time:.2f} s' if bf_time is not None else '— s'
        stats_text = (
            f'Apogee:           {apogee_m:.1f} m\n'
            f'Backfire:         {bf_str}\n'
            f'Parachute Open:   {para_str}\n'
            f'Downrange:        {downrange_m:.1f} m\n'
            f'Landing Radius:   {self.r90_radius:.1f} m  ({self.landing_prob}%)'
        )
        opt_info = getattr(self, '_last_optimization_info', None)
        if opt_info:
            stats_text += (
                f'\nBest Elevation:   {opt_info["elev"]:.1f}°\n'
                f'Best Azimuth:     {opt_info["azi"]:.1f}°'
            )
        self.fig.text(
            0.02, 0.985, stats_text,
            ha='left', va='top',
            fontsize=10, fontweight='bold', color='#222222',
            family='monospace',
            bbox=dict(boxstyle='round,pad=0.35',
                      facecolor='white', edgecolor='gray', alpha=0.9))

        all_horiz = np.concatenate([x_vals, y_vals, [impact_x + r90, impact_x - r90,
                                                      impact_y + r90, impact_y - r90]])
        h_range = max(float(np.max(all_horiz) - np.min(all_horiz)), 1.0)
        mid_x   = float((np.max(x_vals) + np.min(x_vals)) / 2)
        mid_y   = float((np.max(y_vals) + np.min(y_vals)) / 2)
        self.ax.set_xlim(mid_x - h_range * 0.6, mid_x + h_range * 0.6)
        self.ax.set_ylim(mid_y - h_range * 0.6, mid_y + h_range * 0.6)
        self.ax.set_zlim(0, alt_max * 1.15)

        # Feat 5: 3-D profile legend in two columns per spec.
        self.ax.legend(loc='upper right',
                       bbox_to_anchor=(0.98, 0.985),
                       bbox_transform=self.fig.transFigure,
                       ncol=2, fontsize=10, framealpha=0.85)
        self._apply_safe_layout()
        self._draw_compass()
        self.canvas.draw()
        try:
            self._update_wind_subplots()
        except Exception:
            pass
        self.draw_map_elements()
        try:
            self.fit_map_bounds()
        except Exception:
            pass

    def draw_map_elements(self):
        self.map_widget.delete_all_polygon()
        self.map_widget.set_polygon(self.get_circle_coords(self.launch_lat, self.launch_lon, 2.5), fill_color="blue")
        self.map_widget.set_polygon(self.get_circle_coords(self.launch_lat, self.launch_lon, 50), outline_color="blue")
        if getattr(self, '_has_sim_result', False) and self.r90_radius > 0:
            self.map_widget.set_polygon(self.get_circle_coords(self.land_lat, self.land_lon, 2.5), fill_color="red")
            self.map_widget.set_polygon(self.get_circle_coords(self.land_lat, self.land_lon, self.r90_radius), outline_color="red", border_width=2)

    def get_circle_coords(self, lat, lon, radius_m):
        coords = []
        for i in range(36):
            angle = math.pi * 2 * i / 36
            dx, dy = radius_m * math.cos(angle), radius_m * math.sin(angle)
            d_lat = (dy / 6378137.0) * (180 / math.pi)
            d_lon = (dx / (6378137.0 * math.cos(math.pi * lat / 180))) * (180 / math.pi)
            coords.append((lat + d_lat, lon + d_lon))
        return coords

    def fit_map_bounds(self):
        try:
            launch_ring = 50.0
            land_ring   = max(getattr(self, 'r90_radius', 0.0) or 0.0, 2.5)

            m_lat, m_lon = self._meters_per_degree(self.launch_lat)

            def _ring_extents(lat, lon, r_m):
                dlat = r_m / m_lat
                dlon = r_m / m_lon
                return (lat - dlat, lat + dlat, lon - dlon, lon + dlon)

            have_landing = (getattr(self, '_has_sim_result', False)
                            and land_ring > 0
                            and hasattr(self, 'land_lat'))

            lat_mins, lat_maxs, lon_mins, lon_maxs = [], [], [], []
            for la, lo, r in ([(self.launch_lat, self.launch_lon, launch_ring)]
                               + ([(self.land_lat, self.land_lon, land_ring)]
                                  if have_landing else [])):
                lamin, lamax, lomin, lomax = _ring_extents(la, lo, r)
                lat_mins.append(lamin); lat_maxs.append(lamax)
                lon_mins.append(lomin); lon_maxs.append(lomax)

            min_lat, max_lat = min(lat_mins), max(lat_maxs)
            min_lon, max_lon = min(lon_mins), max(lon_maxs)

            pad_lat = (max_lat - min_lat) * 0.10
            pad_lon = (max_lon - min_lon) * 0.10
            pad_lat = max(pad_lat, 5.0 / m_lat)
            pad_lon = max(pad_lon, 5.0 / m_lon)

            self.map_widget.fit_bounding_box(
                (max_lat + pad_lat, min_lon - pad_lon),
                (min_lat - pad_lat, max_lon + pad_lon),
            )
        except Exception:
            pass

    def on_parameter_edit_af(self, event=None):
        if self.af_name_label.cget("text") != "Airframe: (未選択)":
            self.af_name_label.config(text="Airframe: (未選択)")
        self._release_lock_if_active(reason_label="⭘ Unlocked (param changed)")

    def on_parameter_edit_para(self, event=None):
        if self.para_name_label.cget("text") != "Parachute: (未選択)":
            self.para_name_label.config(text="Parachute: (未選択)")
        self._release_lock_if_active(reason_label="⭘ Unlocked (param changed)")

    def _collect_airframe_dict(self):
        return {
            "mass":            float(self.mass_entry.get()),
            "cg":              float(self.cg_entry.get()),
            "length":          float(self.len_entry.get()),
            "radius":          float(self.radius_entry.get()),
            "nose_length":     float(self.nose_len_entry.get()),
            "fin_root":        float(self.fin_root_entry.get()),
            "fin_tip":         float(self.fin_tip_entry.get()),
            "fin_span":        float(self.fin_span_entry.get()),
            "fin_pos":         float(self.fin_pos_entry.get()),
            "motor_pos":       float(self.motor_pos_entry.get()),
            "motor_dry_mass":  float(self.motor_dry_mass_entry.get()),
            "backfire_delay":  float(self.backfire_delay_entry.get()),
        }

    def _collect_parachute_dict(self):
        return {
            "cd":   float(self.cd_entry.get()),
            "area": float(self.area_entry.get()),
            "lag":  float(self.lag_entry.get()),
        }

    def _apply_airframe_dict(self, af):
        def _set(entry, val):
            entry.delete(0, tk.END)
            if val is not None:
                entry.insert(0, str(val))
        _set(self.mass_entry,           af.get("mass",           "0.0872"))
        _set(self.cg_entry,              af.get("cg",             "0.21"))
        _set(self.len_entry,             af.get("length",         "0.383"))
        _set(self.radius_entry,          af.get("radius",         "0.015"))
        _set(self.nose_len_entry,        af.get("nose_length",    "0.08"))
        _set(self.fin_root_entry,        af.get("fin_root",       "0.04"))
        _set(self.fin_tip_entry,         af.get("fin_tip",        "0.02"))
        _set(self.fin_span_entry,        af.get("fin_span",       "0.03"))
        _set(self.fin_pos_entry,         af.get("fin_pos",        "0.35"))
        _set(self.motor_pos_entry,       af.get("motor_pos",      "0.38"))
        _set(self.motor_dry_mass_entry,  af.get("motor_dry_mass", "0.015"))
        if "backfire_delay" in af:
            _set(self.backfire_delay_entry, af["backfire_delay"])

    def _apply_parachute_dict(self, pa):
        self.cd_entry.delete(0,  tk.END); self.cd_entry.insert(0,  str(pa.get("cd",   "")))
        self.area_entry.delete(0, tk.END); self.area_entry.insert(0, str(pa.get("area", "")))
        self.lag_entry.delete(0,  tk.END); self.lag_entry.insert(0,  str(pa.get("lag",  "")))

    def save_config(self):
        filepath = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json")],
            title="Save Rocket Config")
        if not filepath:
            return
        try:
            data = {
                "version":   2,
                "airframe":  self._collect_airframe_dict(),
                "parachute": self._collect_parachute_dict(),
            }
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
            base = os.path.basename(filepath)
            self.af_name_label.config(text=f"Airframe: {base}")
            self.para_name_label.config(text=f"Parachute: {base}")
            messagebox.showinfo("完了", f"機体+パラシュート設定を保存しました。\n{base}")
        except Exception as e:
            messagebox.showerror("エラー", f"保存失敗:\n{e}")

    def load_config(self):
        filepath = filedialog.askopenfilename(
            filetypes=[("JSON Files", "*.json")],
            title="Load Rocket Config")
        if not filepath:
            return
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            base = os.path.basename(filepath)

            af = data.get("airframe")
            pa = data.get("parachute")

            applied = []
            if af is None and pa is None:
                if "mass" in data or "fin_root" in data:
                    self._apply_airframe_dict(data)
                    applied.append("Airframe")
                    self.af_name_label.config(text=f"Airframe: {base}")
                if "cd" in data or "area" in data or "lag" in data:
                    self._apply_parachute_dict(data)
                    applied.append("Parachute")
                    self.para_name_label.config(text=f"Parachute: {base}")
            else:
                if af is not None:
                    self._apply_airframe_dict(af)
                    applied.append("Airframe")
                    self.af_name_label.config(text=f"Airframe: {base}")
                if pa is not None:
                    self._apply_parachute_dict(pa)
                    applied.append("Parachute")
                    self.para_name_label.config(text=f"Parachute: {base}")

            if not applied:
                raise ValueError(
                    "JSON に airframe / parachute のどちらも含まれていません。")
            messagebox.showinfo(
                "完了",
                f"{' + '.join(applied)} 設定を読み込みました。\n{base}")
        except Exception as e:
            messagebox.showerror("エラー", f"読込失敗:\n{e}")

    def save_af_settings(self):   self.save_config()
    def load_af_settings(self):   self.load_config()
    def save_para_settings(self): self.save_config()
    def load_para_settings(self): self.load_config()

    def open_thrustcurve_web(self):
        try:
            webbrowser.open("https://www.thrustcurve.org/motors/search.html")
            messagebox.showinfo("ブラウザ起動", "ThrustCurve.org の検索ページを開きました。\n必要なモーターをCSV形式（RockSim等）でダウンロードし、[Load Local CSV] から読み込んでください。")
        except Exception as e:
            messagebox.showerror("エラー", f"ブラウザを開けませんでした。\n{e}")

    def load_local_motor(self):
        filepath = filedialog.askopenfilename(
            title="Select Motor CSV File",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
        if not filepath:
            return

        try:
            motor_name = os.path.basename(filepath).replace('.csv', '')
            time_thrust_points = []

            with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                for line in f:
                    parts = line.strip().replace('"', '').split(',')
                    if len(parts) >= 2:
                        try:
                            t = float(parts[0])
                            T = float(parts[1])
                            time_thrust_points.append([t, T])
                        except ValueError:
                            if parts[0].strip().lower() in ["motor:", "motor"]:
                                motor_name = parts[1].strip()
                            pass

            if not time_thrust_points:
                raise ValueError("ファイル内に有効な数値データが見つかりませんでした。RockSim形式のCSVか確認してください。")

            if time_thrust_points[0][0] != 0.0:
                time_thrust_points.insert(0, [0.0, time_thrust_points[0][1]])

            burn_time = time_thrust_points[-1][0]

            thrusts = [p[1] for p in time_thrust_points]
            max_thrust = max(thrusts) if thrusts else 0.0
            total_impulse = 0.0
            for i in range(1, len(time_thrust_points)):
                t0, T0 = time_thrust_points[i - 1]
                t1, T1 = time_thrust_points[i]
                total_impulse += (T0 + T1) * 0.5 * (t1 - t0)
            avg_thrust = (total_impulse / burn_time) if burn_time > 0 else 0.0

            self.selected_motor_file = filepath
            self.thrust_data = time_thrust_points
            self.selected_motor_name = motor_name
            self.motor_burn_time = burn_time
            self.motor_avg_thrust = avg_thrust
            self.motor_max_thrust = max_thrust
            self.motor_ui_label.config(text=f"Engine: {motor_name}")
            if hasattr(self, 'motor_specs_label'):
                self.motor_specs_label.config(
                    text=(f"Burn: {burn_time:.2f} s   "
                          f"Avg: {avg_thrust:.1f} N   "
                          f"Max: {max_thrust:.1f} N"))

            messagebox.showinfo(
                "読込完了",
                "ローカルのモーターデータ(CSV)を読み込みました。\n"
                f"・エンジン名  : {motor_name}\n"
                f"・燃焼時間    : {burn_time:.3f} s\n"
                f"・平均推力    : {avg_thrust:.1f} N\n"
                f"・最大推力    : {max_thrust:.1f} N")

        except Exception as e:
            messagebox.showerror("読込エラー", f"モーターファイルの読み込みに失敗しました:\n{e}")

    def create_data_section(self):
        outer = ttk.Frame(self)
        outer.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        outer.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)

        self._params_canvas = tk.Canvas(
            outer, borderwidth=0, highlightthickness=0)
        self._params_canvas.grid(row=0, column=0, sticky="nsew")
        vbar = ttk.Scrollbar(outer, orient="vertical",
                             command=self._params_canvas.yview)
        vbar.grid(row=0, column=1, sticky="ns")
        self._params_canvas.configure(yscrollcommand=vbar.set)

        frame = ttk.Frame(self._params_canvas, padding=(4, 4))
        self._params_inner = frame
        self._params_window = self._params_canvas.create_window(
            (0, 0), window=frame, anchor="nw")
        frame.columnconfigure(0, weight=1)

        def _on_inner_configure(event):
            bbox = self._params_canvas.bbox("all")
            if bbox:
                self._params_canvas.configure(
                    scrollregion=(0, 0, bbox[2], bbox[3]))
            else:
                self._params_canvas.configure(scrollregion=(0, 0, 0, 0))
        frame.bind("<Configure>", _on_inner_configure)

        def _on_canvas_configure(event):
            self._params_canvas.itemconfigure(
                self._params_window, width=event.width)
        self._params_canvas.bind("<Configure>", _on_canvas_configure)

        self._params_canvas.bind("<MouseWheel>", self._on_params_wheel)
        self._params_canvas.bind(
            "<Button-4>", lambda e: self._on_params_wheel(e, delta_override=+120))
        self._params_canvas.bind(
            "<Button-5>", lambda e: self._on_params_wheel(e, delta_override=-120))
        frame.bind("<MouseWheel>", self._on_params_wheel)
        frame.bind(
            "<Button-4>", lambda e: self._on_params_wheel(e, delta_override=+120))
        frame.bind(
            "<Button-5>", lambda e: self._on_params_wheel(e, delta_override=-120))
        self._params_canvas.bind(
            "<Enter>", lambda e: self._params_canvas.focus_set())

        def param_row(parent, label_text, row, default=""):
            ttk.Label(parent, text=label_text, font=("Arial", 8)).grid(
                row=row, column=0, sticky="w", padx=(4, 2), pady=1)
            e = ttk.Entry(parent, width=7, font=("Arial", 8))
            e.grid(row=row, column=1, sticky="e", padx=(2, 4), pady=1)
            if default:
                e.insert(0, default)
            return e

        def _unlock_on_edit(event=None):
            self._release_lock_if_active(reason_label="⭘ Unlocked (param changed)")

        # ── Engine ────────────────────────────────────────────────────────────
        engine_info_f = ttk.Frame(frame)
        engine_info_f.grid(row=0, column=0, sticky="ew", pady=(0, 1))
        engine_info_f.columnconfigure(0, weight=1)
        self.motor_ui_label = ttk.Label(
            engine_info_f, text=f"Engine: {self.selected_motor_name}",
            font=("Arial", 9, "bold"), foreground="#B22222")
        self.motor_ui_label.grid(row=0, column=0, sticky="w")
        self.motor_specs_label = ttk.Label(
            engine_info_f, text="Burn: — s   Avg: — N   Max: — N",
            font=("Arial", 8), foreground="#555555")
        self.motor_specs_label.grid(row=1, column=0, sticky="w")

        mbf = ttk.Frame(frame); mbf.grid(row=1, column=0, sticky="ew", pady=(0, 3))
        mbf.columnconfigure(0, weight=1); mbf.columnconfigure(1, weight=1)
        ttk.Button(mbf, text="[ThrustCurve Web]", command=self.open_thrustcurve_web).grid(
            row=0, column=0, sticky="ew", padx=(0, 1))
        ttk.Button(mbf, text="[Load CSV]", command=self.load_local_motor).grid(
            row=0, column=1, sticky="ew", padx=(1, 0))

        ttk.Separator(frame, orient="horizontal").grid(row=2, column=0, sticky="ew", pady=3)

        # ── Airframe ──────────────────────────────────────────────────────────
        self.af_name_label = ttk.Label(frame, text="Airframe: (未選択)", font=("Arial", 8, "bold"))
        self.af_name_label.grid(row=3, column=0, sticky="w")

        af_lf = ttk.LabelFrame(frame, text="Airframe", padding=(2, 2))
        af_lf.grid(row=4, column=0, sticky="ew", pady=(1, 2))
        af_lf.columnconfigure(0, weight=1); af_lf.columnconfigure(1, weight=0)

        self.mass_entry          = param_row(af_lf, "Dry Mass (kg)",       0)
        self.cg_entry            = param_row(af_lf, "CG from Nose (m)",    1)
        self.len_entry           = param_row(af_lf, "Length (m)",          2)
        self.radius_entry        = param_row(af_lf, "Radius (m)",          3)

        aero_lf = ttk.LabelFrame(frame, text="Aero & Motor", padding=(2, 2))
        aero_lf.grid(row=5, column=0, sticky="ew", pady=(1, 2))
        aero_lf.columnconfigure(0, weight=1); aero_lf.columnconfigure(1, weight=0)

        self.nose_len_entry      = param_row(aero_lf, "Nose Length (m)",      0)
        self.fin_root_entry      = param_row(aero_lf, "Fin Root (m)",         1)
        self.fin_tip_entry       = param_row(aero_lf, "Fin Tip (m)",          2)
        self.fin_span_entry      = param_row(aero_lf, "Fin Span (m)",         3)
        self.fin_pos_entry       = param_row(aero_lf, "Fin Pos fr Nose (m)",  4)
        self.motor_pos_entry     = param_row(aero_lf, "Motor Pos fr Nose (m)",5)
        self.motor_dry_mass_entry= param_row(aero_lf, "Motor Dry Mass (kg)",  6)
        self.backfire_delay_entry= param_row(aero_lf, "Backfire Delay (s)",   7)

        af_entries = [self.mass_entry, self.cg_entry, self.len_entry, self.radius_entry,
                      self.nose_len_entry, self.fin_root_entry, self.fin_tip_entry,
                      self.fin_span_entry, self.fin_pos_entry, self.motor_pos_entry,
                      self.motor_dry_mass_entry, self.backfire_delay_entry]
        for e in af_entries:
            e.bind("<KeyRelease>", self.on_parameter_edit_af)

        ttk.Separator(frame, orient="horizontal").grid(row=6, column=0, sticky="ew", pady=3)

        # ── Parachute ─────────────────────────────────────────────────────────
        self.para_name_label = ttk.Label(frame, text="Parachute: (未選択)", font=("Arial", 8, "bold"))
        self.para_name_label.grid(row=7, column=0, sticky="w")

        para_lf = ttk.LabelFrame(frame, text="Parachute", padding=(2, 2))
        para_lf.grid(row=8, column=0, sticky="ew", pady=(1, 2))
        para_lf.columnconfigure(0, weight=1); para_lf.columnconfigure(1, weight=0)

        self.cd_entry   = param_row(para_lf, "Cd",       0)
        self.area_entry = param_row(para_lf, "Area (m²)",1)
        self.lag_entry  = param_row(para_lf, "Lag (s)",  2)

        self.cd_entry.bind("<KeyRelease>",   self.on_parameter_edit_para)
        self.area_entry.bind("<KeyRelease>", self.on_parameter_edit_para)
        self.lag_entry.bind("<KeyRelease>",  self.on_parameter_edit_para)

        para_btn_f = ttk.Frame(frame); para_btn_f.grid(row=9, column=0, sticky="ew", pady=(0, 2))
        para_btn_f.columnconfigure(0, weight=1); para_btn_f.columnconfigure(1, weight=1)
        ttk.Button(para_btn_f, text="Load Rocket Config",
                   command=self.load_config).grid(
            row=0, column=0, sticky="ew", padx=(0, 1))
        ttk.Button(para_btn_f, text="Save Rocket Config",
                   command=self.save_config).grid(
            row=0, column=1, sticky="ew", padx=(1, 0))

        ttk.Separator(frame, orient="horizontal").grid(row=10, column=0, sticky="ew", pady=3)

        # ── Launcher ──────────────────────────────────────────────────────────
        ttk.Label(frame, text="Launcher", font=("Arial", 9, "bold")).grid(
            row=11, column=0, sticky="w", pady=(0, 1))

        launch_lf = ttk.LabelFrame(frame, text="Position & Rail", padding=(2, 2))
        launch_lf.grid(row=12, column=0, sticky="ew", pady=(1, 2))
        launch_lf.columnconfigure(0, weight=1); launch_lf.columnconfigure(1, weight=0)

        ttk.Label(launch_lf, text="Lat:", font=("Arial", 8)).grid(
            row=0, column=0, sticky="w", padx=(4, 2), pady=1)
        self.lat_entry = ttk.Entry(launch_lf, width=11, font=("Arial", 8))
        self.lat_entry.insert(0, str(self.launch_lat))
        self.lat_entry.grid(row=0, column=1, sticky="e", padx=(2, 4), pady=1)

        ttk.Label(launch_lf, text="Lon:", font=("Arial", 8)).grid(
            row=1, column=0, sticky="w", padx=(4, 2), pady=1)
        self.lon_entry = ttk.Entry(launch_lf, width=11, font=("Arial", 8))
        self.lon_entry.insert(0, str(self.launch_lon))
        self.lon_entry.grid(row=1, column=1, sticky="e", padx=(2, 4), pady=1)

        ttk.Label(launch_lf, text="Rail (m):", font=("Arial", 8)).grid(
            row=2, column=0, sticky="w", padx=(4, 2), pady=1)
        self.rail_entry = ttk.Entry(launch_lf, width=7, font=("Arial", 8))
        self.rail_entry.insert(0, "1.0")
        self.rail_entry.grid(row=2, column=1, sticky="e", padx=(2, 4), pady=1)

        ttk.Label(launch_lf, text="Elevation:", font=("Arial", 8)).grid(
            row=3, column=0, sticky="w", padx=(4, 2), pady=1)
        self.elev_spin = ttk.Spinbox(launch_lf, from_=0, to=90, width=6, font=("Arial", 8))
        self.elev_spin.set("85")
        self.elev_spin.grid(row=3, column=1, sticky="e", padx=(2, 4), pady=1)

        ttk.Label(launch_lf, text="Azimuth:", font=("Arial", 8)).grid(
            row=4, column=0, sticky="w", padx=(4, 2), pady=1)
        self.azi_spin = ttk.Spinbox(launch_lf, from_=0, to=360, width=6, font=("Arial", 8))
        self.azi_spin.set("0")
        self.azi_spin.grid(row=4, column=1, sticky="e", padx=(2, 4), pady=1)

        for _w in (self.lat_entry, self.lon_entry, self.rail_entry,
                   self.elev_spin, self.azi_spin):
            _w.bind("<KeyRelease>", _unlock_on_edit)

        loc_btn_f = ttk.Frame(frame); loc_btn_f.grid(row=13, column=0, sticky="ew", pady=(0, 2))
        loc_btn_f.columnconfigure(0, weight=1); loc_btn_f.columnconfigure(1, weight=1)
        ttk.Button(loc_btn_f, text="Get Location (IP)",
                   command=lambda: self.get_current_location(manual=True)).grid(
            row=0, column=0, sticky="ew", padx=(0, 1))
        ttk.Button(loc_btn_f, text="Update Map",
                   command=self.update_map_center).grid(
            row=0, column=1, sticky="ew", padx=(1, 0))

        ttk.Separator(frame, orient="horizontal").grid(row=14, column=0, sticky="ew", pady=3)

        # ── Operation Mode ────────────────────────────────────────────────────
        mode_lf = ttk.LabelFrame(frame, text="Operation Mode", padding=(4, 4))
        mode_lf.grid(row=15, column=0, sticky="ew", pady=(1, 2))
        mode_lf.columnconfigure(0, weight=1)
        mode_lf.columnconfigure(1, weight=0)

        self.mode_combo = ttk.Combobox(
            mode_lf, textvariable=self.operation_mode_var,
            values=list(self.OPERATION_MODES),
            state="readonly", font=("Arial", 9))
        self.mode_combo.grid(row=0, column=0, columnspan=2,
                             sticky="ew", padx=2, pady=(0, 4))
        self.mode_combo.bind("<<ComboboxSelected>>", self._on_mode_change)

        self.rmax_label = ttk.Label(mode_lf, text="指定半径 r_max (m):",
                                    font=("Arial", 8))
        self.rmax_label.grid(row=1, column=0, sticky="w",
                             padx=(4, 2), pady=1)
        self.rmax_entry = ttk.Entry(mode_lf, textvariable=self.r_max_var,
                                    width=8, font=("Arial", 8))
        self.rmax_entry.grid(row=1, column=1, sticky="e",
                             padx=(2, 4), pady=1)
        self.rmax_label.grid_remove()
        self.rmax_entry.grid_remove()

        # ── Run + results ─────────────────────────────────────────────────────
        self.run_button = ttk.Button(frame, text="🚀  RUN SIMULATION",
                                     command=self.run_simulation)
        self.run_button.grid(row=16, column=0, sticky="ew",
                             ipady=4, pady=(2, 4))
        res_f = ttk.Frame(frame); res_f.grid(row=17, column=0, sticky="ew")
        res_f.columnconfigure(0, weight=1); res_f.columnconfigure(1, weight=1)
        self.apogee_label   = ttk.Label(res_f, text="Apogee: -- m",
                                        font=("Arial", 9, "bold"))
        self.apogee_label.grid(row=0, column=0, sticky="w", padx=4)
        self.velocity_label = ttk.Label(res_f, text="Impact: -- m/s",
                                        font=("Arial", 9, "bold"))
        self.velocity_label.grid(row=0, column=1, sticky="e", padx=4)

        ttk.Separator(frame, orient="horizontal").grid(row=18, column=0, sticky="ew", pady=3)

        # ── Lock & Monitor + Settings row ─────────────────────────────────────
        lock_f = ttk.Frame(frame)
        lock_f.grid(row=19, column=0, sticky="ew", pady=(0, 2))
        lock_f.columnconfigure(0, weight=1)
        lock_f.columnconfigure(1, weight=0)

        self.lock_monitor_var = tk.BooleanVar(value=False)
        self.lock_monitor_check = ttk.Checkbutton(
            lock_f, text="🔒 Lock & Monitor",
            variable=self.lock_monitor_var,
            command=self._toggle_lock_monitor)
        self.lock_monitor_check.grid(row=0, column=0, sticky="w")
        ttk.Button(lock_f, text="⚙ Settings",
                   command=self._open_settings_window).grid(
            row=0, column=1, sticky="e")

        self.monitor_status_label = tk.Label(
            lock_f, text="⭘ Unlocked",
            foreground="gray", font=("Arial", 8), anchor="w")
        self.monitor_status_label.grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(2, 0))

        ttk.Separator(frame, orient="horizontal").grid(row=20, column=0, sticky="ew", pady=3)

        # ── Wind (manual input — to be replaced by API) ───────────────────────
        wind_lf = ttk.LabelFrame(frame, text="Wind (Dir: 0=N, 90=E)", padding=(2, 2))
        wind_lf.grid(row=21, column=0, sticky="ew", pady=(1, 2))
        wind_lf.columnconfigure(1, weight=1)

        ttk.Label(wind_lf, text="100m spd:", font=("Arial", 8)).grid(
            row=0, column=0, sticky="w", padx=(4, 2), pady=1)
        self.up_spd_var = tk.StringVar(value="8.0")
        up_spd_entry = ttk.Entry(wind_lf, textvariable=self.up_spd_var,
                                 width=5, font=("Arial", 8))
        up_spd_entry.grid(row=0, column=1, sticky="ew", padx=2, pady=1)
        ttk.Label(wind_lf, text="dir:", font=("Arial", 8)).grid(
            row=0, column=2, sticky="w", padx=(2, 2))
        self.up_dir_var = tk.StringVar(value="90")
        up_dir_entry = ttk.Entry(wind_lf, textvariable=self.up_dir_var,
                                 width=4, font=("Arial", 8))
        up_dir_entry.grid(row=0, column=3, sticky="e", padx=(0, 4), pady=1)

        ttk.Label(wind_lf, text="3m spd:", font=("Arial", 8)).grid(
            row=1, column=0, sticky="w", padx=(4, 2), pady=1)
        self.surf_spd_slider = ttk.Scale(wind_lf, from_=0, to=15, orient="horizontal")
        self.surf_spd_slider.set(4.0)
        self.surf_spd_slider.grid(row=1, column=1, sticky="ew", padx=2, pady=1)
        ttk.Label(wind_lf, text="dir:", font=("Arial", 8)).grid(
            row=1, column=2, sticky="w", padx=(2, 2))
        self.surf_dir_var = tk.StringVar(value="100")
        surf_dir_entry = ttk.Entry(wind_lf, textvariable=self.surf_dir_var,
                                   width=4, font=("Arial", 8))
        surf_dir_entry.grid(row=1, column=3, sticky="e", padx=(0, 4), pady=1)

        def _on_wind_input_change(*_a):
            try:
                self._update_realtime_wind_label()
                self._update_wind_subplots()
            except Exception:
                pass
        for _v in (self.up_spd_var, self.up_dir_var, self.surf_dir_var):
            try:
                _v.trace_add('write', _on_wind_input_change)
            except Exception:
                try:
                    _v.trace('w', lambda *a: _on_wind_input_change())
                except Exception:
                    pass
        self.surf_spd_slider.configure(command=lambda _v=None: _on_wind_input_change())

        stats_f = ttk.Frame(wind_lf)
        stats_f.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(2, 2))
        self.wind_avg_label  = ttk.Label(stats_f, text="Avg: -- m/s",  foreground="green",
                                         font=("Arial", 8))
        self.wind_avg_label.pack(side="left", padx=4)
        self.wind_gust_label = ttk.Label(stats_f, text="Gust: -- m/s", foreground="red",
                                         font=("Arial", 8))
        self.wind_gust_label.pack(side="right", padx=4)

        self._bind_params_wheel_recursive(self._params_inner)

    def create_profile_section(self):
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        frame = ttk.Frame(self, padding=10, relief="solid", borderwidth=1)
        frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        frame.rowconfigure(0, weight=3)
        frame.rowconfigure(1, weight=0)
        frame.rowconfigure(2, weight=0)
        frame.rowconfigure(3, weight=1)
        frame.columnconfigure(0, weight=1)

        self.fig = plt.figure(figsize=(6.4, 5.2), dpi=100)
        self.ax  = self.fig.add_subplot(111, projection='3d')

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        rot_bar = ttk.Frame(frame)
        rot_bar.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        rot_bar.columnconfigure(1, weight=1)

        ttk.Label(rot_bar, text="↻ Rotate:", font=("Arial", 8)).grid(
            row=0, column=0, sticky="w", padx=(2, 4))

        init_azim_ui = max(0.0, min(90.0, float(self._fixed_azim)))
        self._fixed_azim = init_azim_ui
        self.azim_var = tk.DoubleVar(value=init_azim_ui)
        self.azim_slider = ttk.Scale(
            rot_bar, from_=0, to=90, orient="horizontal",
            variable=self.azim_var, command=self._on_azim_slider
        )
        self.azim_slider.grid(row=0, column=1, sticky="ew", padx=2)

        self.azim_label = ttk.Label(
            rot_bar, text=f"{self._fixed_azim:+.0f}°",
            font=("Arial", 8), width=6, anchor="e"
        )
        self.azim_label.grid(row=0, column=2, sticky="e", padx=(4, 2))

        ttk.Button(rot_bar, text="Reset", width=6,
                   command=self._reset_azim).grid(row=0, column=3, padx=(4, 2))

        try:
            self.canvas.get_tk_widget().bind(
                "<MouseWheel>", self._on_wheel_rotate_azim)
            self.canvas.get_tk_widget().bind(
                "<Button-4>",
                lambda e: self._on_wheel_rotate_azim(e, delta_override=+120))
            self.canvas.get_tk_widget().bind(
                "<Button-5>",
                lambda e: self._on_wheel_rotate_azim(e, delta_override=-120))
        except Exception:
            pass

        self.canvas.mpl_connect('button_press_event',   self._on_canvas_press)
        self.canvas.mpl_connect('motion_notify_event',  self._on_canvas_motion)
        self.canvas.mpl_connect('button_release_event', self._on_canvas_release)
        self.canvas.mpl_connect('draw_event',           self._on_view_changed)

        # ── Bottom wind figure: time-series + compass (Feat 1+2+3) ────────────
        self.wind_fig = plt.figure(figsize=(6.4, 2.0), dpi=100)
        gs = self.wind_fig.add_gridspec(
            1, 2, width_ratios=[3.2, 1.0], wspace=0.35)
        self.wind_ax_spd     = self.wind_fig.add_subplot(gs[0, 0])
        self.wind_ax_compass = self.wind_fig.add_subplot(gs[0, 1], projection='polar')
        self.wind_fig.subplots_adjust(left=0.10, right=0.95, top=0.88, bottom=0.22)

        # Realtime wind label (Feat 4: numerical-only display).
        rt_bar = ttk.Frame(frame)
        rt_bar.grid(row=2, column=0, sticky="ew", pady=(4, 0))
        rt_bar.columnconfigure(0, weight=1)
        self.realtime_wind_label = ttk.Label(
            rt_bar,
            text="Surface: -- m/s  @ --°   |   Upper: -- m/s @ --°",
            font=("Arial", 10, "bold"),
            foreground="#1a237e",
            background="#fffde7",
            anchor="center",
            padding=(6, 2),
            relief="groove",
        )
        self.realtime_wind_label.grid(row=0, column=0, sticky="ew")

        self.wind_canvas = FigureCanvasTkAgg(self.wind_fig, master=frame)
        self.wind_canvas.get_tk_widget().grid(row=3, column=0, sticky="nsew",
                                              pady=(4, 0))

        self._update_wind_subplots()
        self._update_realtime_wind_label()

    def _read_current_wind(self):
        def _get(var_name, default):
            v = getattr(self, var_name, None)
            if v is None:
                return default
            try:
                return float(v.get())
            except Exception:
                return default
        hist = getattr(self, 'surf_wind_history', None) or []
        if hist:
            surf_spd = float(hist[-1])
        else:
            surf_spd = _get('surf_spd_slider', 0.0)
        surf_dir = _get('surf_dir_var', 0.0)
        up_spd   = _get('up_spd_var',   0.0)
        up_dir   = _get('up_dir_var',   0.0)
        return surf_spd, surf_dir, up_spd, up_dir

    def _update_wind_subplots(self):
        """Feat 1+2+3+4: time-series wind speed graph (with 10-s moving
        average horizontal line) on the left, real-time wind-direction
        compass on the right.

        • Horizontal axis = elapsed seconds since program start
        • Vertical axis   = wind speed (m/s)
        • Red dashed ``axhline`` overlays the past-10-second moving average
        • Compass arrows reflect the *current* surface + upper directions
        • The redundant "Now:" prefix has been removed from the overlay tag
        """
        try:
            import numpy as np
        except Exception:
            return

        spd_ax = getattr(self, 'wind_ax_spd', None)
        cmp_ax = getattr(self, 'wind_ax_compass', None)
        if spd_ax is None or cmp_ax is None:
            return

        surf_spd, surf_dir, up_spd, up_dir = self._read_current_wind()

        # ── Left: Time-series wind speed (Feat 1 + 2 + 3) ────────────────────
        spd_ax.clear()
        spd_ax.set_title('Wind Speed (Time Series)', fontsize=10)
        spd_ax.set_xlabel('Time (s ago)', fontsize=9)
        spd_ax.set_ylabel('Wind Speed (m/s)', fontsize=9)
        spd_ax.tick_params(labelsize=8)
        spd_ax.grid(True, alpha=0.3)

        history = list(getattr(self, 'surf_wind_time_history', []) or [])
        if history:
            t_latest = history[-1][0]
            # Convert to "seconds ago": 0 = now, negative = past
            ts_rel = [h[0] - t_latest for h in history]
            ws = [h[1] for h in history]
            spd_ax.plot(ts_rel, ws,
                        linestyle='-', linewidth=1.6,
                        marker='.', markersize=3,
                        color='#1f77b4', label='Surface')

            # 10-second moving average as a horizontal red line.
            avg_10s = self._wind_avg_recent(window_sec=10.0)
            spd_ax.axhline(y=avg_10s, color='red', linestyle='--',
                           linewidth=1.4,
                           label=f'10s Avg: {avg_10s:.1f} m/s')

            # Upper wind speed as a horizontal green line.
            spd_ax.axhline(y=up_spd, color='green', linestyle=':',
                           linewidth=1.4,
                           label=f'Upper: {up_spd:.1f} m/s')

            # x-axis: -60 (past) on the left, 0 (now) on the right.
            spd_ax.set_xlim(-60.0, 1.0)

            y_max = max(max(ws), avg_10s, up_spd, 1.0) * 1.25
            spd_ax.set_ylim(0, y_max)
            spd_ax.legend(fontsize=8, loc='upper left')
        else:
            spd_ax.text(0.5, 0.5, 'Waiting for wind data...',
                        transform=spd_ax.transAxes,
                        ha='center', va='center', color='gray', fontsize=9)
            spd_ax.set_xlim(-60.0, 1.0)
            spd_ax.set_ylim(0, 10)

        # ── Right: real-time wind-direction compass (Feat 2) ─────────────────
        cmp_ax.clear()
        cmp_ax.set_title('Wind From', fontsize=10, pad=8)
        cmp_ax.set_theta_zero_location('N')
        cmp_ax.set_theta_direction(-1)
        cmp_ax.set_xticks(np.deg2rad([0, 90, 180, 270]))
        cmp_ax.set_xticklabels(['N', 'E', 'S', 'W'], fontsize=9)
        cmp_ax.set_yticklabels([])
        cmp_ax.set_ylim(0, 1.0)
        cmp_ax.grid(True, alpha=0.3)

        def _draw_arrow(deg_from, color, label):
            try:
                theta = np.deg2rad(float(deg_from))
            except Exception:
                return
            cmp_ax.annotate(
                '', xy=(theta, 0.95), xytext=(theta, 0.0),
                arrowprops=dict(arrowstyle='->', color=color, lw=2.0),
            )
            cmp_ax.text(theta, 1.12, label, color=color,
                        fontsize=8, ha='center', va='center')

        _draw_arrow(up_dir,   '#1f77b4', 'Upper')
        _draw_arrow(surf_dir, '#ff7f0e', 'Surface')

        try:
            self.wind_canvas.draw_idle()
        except Exception:
            pass
        try:
            self._update_realtime_wind_label()
        except Exception:
            pass

    def create_map_section(self):
        frame = ttk.Frame(self, padding=10, relief="solid", borderwidth=1)
        frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)

        ctrl_frame = ttk.Frame(frame)
        ctrl_frame.pack(fill="x", pady=(0, 5))
        ttk.Label(ctrl_frame, text="Map View", font=("Arial", 10, "bold")).pack(side="left")
        ttk.Button(ctrl_frame, text="[軌道全体を表示 / Center Map]", command=self.fit_map_bounds).pack(side="right")

        self.map_widget = tkintermapview.TkinterMapView(frame)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=ja&x={x}&y={y}&z={z}&s=Ga")
        self.map_widget.set_position(self.launch_lat, self.launch_lon)

    def update_map_center(self):
        try:
            self.launch_lat = float(self.lat_entry.get())
            self.launch_lon = float(self.lon_entry.get())
            try:
                self.map_widget.set_position(self.launch_lat, self.launch_lon)
            except Exception:
                pass
            self._clear_previous_landing()
            self.update_plots()
            self.fit_map_bounds()
        except ValueError:
            pass


if __name__ == "__main__":
    app = KazamidoriUI()
    app.mainloop()