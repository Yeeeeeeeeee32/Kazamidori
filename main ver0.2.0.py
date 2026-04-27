# 現在のモードからロック機能をすべて削除したものを自由モードとする
# 発射点を中心として半径r_max (m)に着地する中で，発射点から着地点の距離をr, 滞空時間をtとして，r_max-r+tが最大となる定点滞空最適化モード
# 発射点を中心として指定した半径内に90%着地する中で、最高高度に到達するための高度最適化モード
# 発射点を中心として指定した半径内に90%着地する中で、滞空時間が最大になるための有翼最適化モード
# ver 0.2.0




# 回転を90に制限
# 画面が小さいときにパラメータセクションを縮小
# 風速のグラフ（平均風速に線），風向のコンパス？，を表示させる
# モータのデータを選択後に表示
# Airframe, Parachute設定の読み込みを同じファイルから
# ver 0.2.1

# 風向，風速の標準偏差を求める(着地点の不確かさ評価のため)
# 風の不確かさ評価
# ver 0.2.2

# オフラインマップモードを追加し，指定場所の半径500mを取得
# ver0.2.3

# Windy.com API, 風速計との連携
# パラシュート設定をホーム画面で選択，設定，保存（パラシュートの形状でいくつかのデフォルトを設定）
# ver 1.0.0

# 回転を90に制限
# 画面が小さいときにパラメータセクションを縮小
# 定点滞空得点表示
# 着地点計算の時に，着地点の指定確率の円がr_max半径内になるように条件を変更する

# 実際に発射する手順として
# 最適化計算(高度or定点or有翼)により発射角，方位決定(風速風向許容差決定)
# 許容差を超えないかモニター
# 超えない場合はグリーン，超える場合は再計算し警告を流す

# 将来的には地形とかも考慮できるといいかもね
# 自作風速計の外れ値除外計算を追加




import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import math
import os
import sys
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

# RocketPyのインポート
try:
    from rocketpy import Environment, SolidMotor, Rocket, Flight
    ROCKETPY_AVAILABLE = True
except ImportError:
    ROCKETPY_AVAILABLE = False

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
        self.motor_burn_time = 0.0
        self.thrust_data = None

        self.surf_wind_history = deque(maxlen=300)

        # ── 3-D view state (single source of truth) ───────────────────────────────
        self._fixed_elev     = 25       # locked elevation (deg)
        self._fixed_azim     = -60.0    # current azimuth  (deg, -180..180)
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
        # `_has_sim_result` gates the landing markers on both 3-D plot and map
        # so old results vanish cleanly when the launch point is relocated.
        # `_compass_ax` is a handle to the inset axes that draws the rotating
        # compass rose; we recreate it on every view change.
        self._has_sim_result    = False
        self._compass_ax        = None

        # ── Operation mode state (v0.3 — four operation modes) ────────────────
        # Four modes:
        #   "自由モード"             — manual mode, Wind Monitoring disabled
        #   "定点滞空最適化モード"   — hovering optimization: max (r_max - r) + t,  r ≤ r_max
        #   "高度最適化モード"       — altitude optimization: max apogee, MC 90% circle ≤ r_max
        #   "有翼最適化モード"       — winged optimization:   max hang time, MC 90% circle ≤ r_max
        # The optimization runs on a worker thread to keep the UI responsive;
        # progress is reported back via `_opt_queue` and polled via `after()`.
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
        # Remembers the most recent optimization's optimal (elev, azi) +
        # mode name, so the 3-D profile stats block can surface the
        # "Best" line alongside Apogee / Downrange / Landing Radius.
        # Cleared whenever a Free-Mode run happens so the readout stays
        # honest about which state the stats correspond to.
        self._last_optimization_info = None

        self.create_data_section()
        self.create_profile_section()
        self.create_map_section()

        self.after(500, lambda: self.get_current_location(manual=False))
        self.after(1000, self.simulate_realtime_wind)

        self.update_plots()
        self.after(1000, self.fit_map_bounds)

        # Apply initial operation-mode state: Free Mode hides r_max and
        # disables the Lock & Monitor checkbutton. Deferred to `after(0, …)`
        # so the widget tree is fully realised before we touch its state.
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
                # Launch point moved → clear stale impact markers from any
                # previous simulation so they don't appear at the wrong place.
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

        self.surf_wind_history.append(current_wind)
        avg = sum(self.surf_wind_history) / len(self.surf_wind_history)

        self.wind_avg_label.config(text=f"地表平均: {avg:.1f} m/s")
        self.wind_gust_label.config(text=f"最大瞬間: {max(self.surf_wind_history):.1f} m/s")
        self.after(1000, self.simulate_realtime_wind)

    # ── Wind helper (refactored out of run_simulation for readability) ────────
    @staticmethod
    def _wind_components(spd, dir_deg):
        """Convert a meteorological (speed, direction) wind to (u, v) components.

        `dir_deg` is the *origin* direction in compass degrees (0=N, 90=E),
        which is the meteorology convention. RocketPy's wind profile expects
        +u toward East and +v toward North, so we flip the sign of both
        components (wind blowing FROM the north pushes the rocket SOUTH).

        Returns:
            tuple[float, float]: (u_east, v_north) in m/s.
        """
        rad = math.radians(dir_deg)
        return -spd * math.sin(rad), -spd * math.cos(rad)

    @staticmethod
    def _meters_per_degree(lat_deg):
        """Return ``(m_per_deg_lat, m_per_deg_lon)`` for the given latitude.

        Uses WGS84 series approximations rather than treating Earth as a
        sphere of radius 6 378 137 m. Accurate to a few centimetres at
        kilometre scale; the constant-radius approach drifted by ~0.3 %
        near 35 °N (Japanese launch sites) — enough to visibly mis-place
        the dispersion ring on the map (feature #1).
        """
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
        """Add an East / North offset (metres) to a (lat, lon) pair.

        Shared by `_apply_sim_result_to_ui` and `get_circle_coords` so the
        landing point and the dispersion ring around it use the *same*
        metres-per-degree conversion — fixing the small but visible
        mis-alignment between the two (feature #1).
        """
        m_lat, m_lon = self._meters_per_degree(lat0)
        return (lat0 + dy_north / m_lat,
                lon0 + dx_east  / m_lon)

    def _make_backfire_trigger(self, backfire_alt):
        """Build the parachute trigger callable for RocketPy's add_parachute.

        Extracted out of `run_simulation` to keep the simulation method
        focused on flight-setup logic. The trigger fires once the rocket is
        descending (vz < 0) AND has fallen to or below `backfire_alt`, which
        is derived from the desired absolute time-after-burnout via a fast
        first-pass altitude-vs-time lookup.

        A single-element list is used as a latch so the trigger stays True
        for the rest of the flight after firing — RocketPy may call the
        trigger many times per integration step.

        Args:
            backfire_alt: altitude (m AGL) at which to deploy the parachute.

        Returns:
            Callable[[Any, float, np.ndarray], bool] suitable for RocketPy.
        """
        triggered = [False]

        def trigger(p, h, y):
            # y[5] is the vertical velocity component in RocketPy's state vector.
            if triggered[0]:
                return True
            if y[5] < 0 and h <= backfire_alt:
                triggered[0] = True
                return True
            return False

        return trigger

    # ── Parameter gathering + headless single-flight helper (v0.3) ───────────
    # These helpers exist so the optimization routines (modes 2/3/4) can
    # run RocketPy flights repeatedly with different (elev, azi) pairs
    # without touching the UI and without having to re-parse the input
    # fields every call. `run_simulation` in Free Mode also uses them so
    # the physics code has a single home.
    def _gather_sim_params(self):
        """Parse every UI input into a dict. Returns None and shows an error
        messagebox if anything can't be parsed. Must be called from the main
        thread (it reads tkinter widgets).
        """
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

        # Commit the parsed launch location immediately so the rest of the
        # code (and the map) agrees with the typed value.
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
        """Run one RocketPy flight with the given launch angles + param dict.

        Headless: does NOT touch tkinter widgets, so it is safe to call from
        a worker thread. Returns a dict with keys:
          ok        — True on success, False on error
          error     — error message (only if ok is False)
          apogee_m  — peak altitude (m AGL)
          hang_time — total flight time from ignition to landing (s)
          impact_x, impact_y — landing offset from launch (m East, m North)
          r_horiz   — horizontal distance from launch to landing (m)
          t_vals, x_vals, y_vals, z_vals, vz_vals — raw sampled arrays
          idx_bf, idx_para, bf_abs_time, para_open_time, backfire_alt
          apogee_idx, elev, azi
        """
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
            hang_time  = float(t_vals[-1])  # total flight time from ignition

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
        """Take the dict returned by _simulate_once and update all UI state.

        Called from the main thread only. Handles:
          • populating land_lat / land_lon / r90_radius
          • apogee & velocity labels
          • Lock & Monitor baseline capture
          • building sim_data for update_plots + map redraw

        If `override_r90` is supplied (finite positive float), it REPLACES
        the sigma-based dispersion estimate. This lets the optimization
        pipeline surface the Monte-Carlo-derived landing circle on both
        the 3-D profile and the 2-D map, rather than the rougher analytic
        fallback.

        Returns the sim_data dict in case the caller wants it.
        """
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
        # Optimization pipeline overrides the sigma-based estimate with a
        # Monte-Carlo-derived landing circle so the post-optimization
        # display matches the constraint that was actually enforced.
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
        if not ROCKETPY_AVAILABLE:
            return
        # ── Mode dispatch (v0.3 — four operation modes) ─────────────────────
        # Free Mode             → run once with the user-typed elev/azi.
        # Hover / Alt / Winged  → gather params + r_max, then kick off a
        #                          background optimization thread; the
        #                          best candidate is replayed through the
        #                          normal render path on completion.
        mode_val = self.operation_mode_var.get() if hasattr(
            self, 'operation_mode_var') else "自由モード"

        if mode_val != "自由モード":
            # Reject a nested run while an optimization is already executing.
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

        # ── Free Mode path ───────────────────────────────────────────────────
        # Clear any stale optimization tag so the "Best Elev/Azi" line
        # vanishes from the 3-D profile stats block — the readout now
        # corresponds to user-typed angles, not an optimization result.
        self._last_optimization_info = None
        # NOTE (v0.4 spec change): Free Mode no longer auto-enables
        # Monitor Mode. The user can edit parameters freely and press
        # RUN as many times as they want without the UI locking on
        # them. The Lock & Monitor widget itself is disabled while Free
        # Mode is active — see `_on_mode_change`.
        self._render_current_params()
        return

    # ── Shared simulate+render helper (used by Free Mode AND by the
    #    post-optimization final render) ──────────────────────────────────
    # Both entry points need to: read the spinboxes, run exactly one
    # RocketPy flight, push the result through the render path. Pulling
    # it into a single helper also guarantees the two callers stay in
    # lockstep on error handling and on the sim_data it produces.
    def _render_current_params(self, override_r90=None):
        """Run one RocketPy flight using whatever is currently in the UI
        input fields and draw it onto the 3-D profile and map.

        Returns True on a successful render, False on input/sim errors.
        Does NOT touch Monitor Mode — callers decide whether to arm it.
        """
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

    # ── NEW: optimization implementation (modes 2/3/4) ───────────────────────
    # Monte-Carlo landing-circle estimator. Perturbs wind (speed + direction)
    # and motor thrust by the same σ the uncertainty-settings window uses.
    # Reports the landing_prob-th percentile of horizontal impact distance,
    # which is compared against r_max as the go/no-go constraint for modes 3 & 4.
    def _monte_carlo_r90(self, elev, azi, base_params,
                         n_trials=8, stop_flag=None):
        """Return (percentile_distance_m, success_fraction).

        Tries `n_trials` perturbed simulations. Returns (inf, 0.0) if every
        trial crashes — that flags the candidate as unusable.
        """
        distances = []
        succeeded = 0
        rng = random.Random()
        wu = max(self.wind_uncertainty, 0.0)
        tu = max(self.thrust_uncertainty, 0.0)
        # Wind direction σ scales with the wind-speed uncertainty so a
        # calmer site is assumed to be more directionally consistent.
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
        """Worker-thread entry point for the grid-search optimizer.

        **Search variables are strictly limited to (elev, azi) only.**
        Every other parameter — parachute deployment delay, motor, mass,
        wind profile, etc. — is taken from `base_params`, which was parsed
        from the UI on the main thread and is NEVER mutated in here.

        Two phases for MC modes (altitude / winged):
          Phase 1 — coarse grid, nominal sim only. Cheap filter.
          Phase 2 — MC evaluation of the top-N candidates, picks the best
                    candidate that actually respects the r_max constraint.
          Phase 3 — a final MC pass at the chosen (elev, azi) with a
                    larger sample size, so the user-facing landing circle
                    is a high-quality estimate rather than the noisy 8-trial
                    value from phase 2.

        Hover mode ("定点滞空最適化モード") has no MC constraint in phase 1,
        but still gets a phase-3 MC pass so the map's landing circle is
        populated from a dispersion sample rather than the sigma fallback.

        All communication with the main thread goes through `_opt_queue`.
        Honours `_opt_stop_flag` for cooperative cancellation.
        """
        try:
            if mode == "定点滞空最適化モード":
                # Hover: maximize (r_max - r) + t subject to r <= r_max
                elev_grid = [60, 68, 75, 80, 85, 88]
                azi_grid  = [0, 30, 60, 90, 120, 150,
                             180, 210, 240, 270, 300, 330]
                use_mc = False

                def objective(res, mc_r=None):
                    if not res['ok']:
                        return float('-inf')
                    if res['r_horiz'] > r_max:
                        return float('-inf')
                    return (r_max - res['r_horiz']) + res['hang_time']

            elif mode == "高度最適化モード":
                # Altitude: maximize apogee; constraint = MC circle ≤ r_max
                elev_grid = [70, 78, 83, 87, 90]
                azi_grid  = [0, 45, 90, 135, 180, 225, 270, 315]
                use_mc = True

                def objective(res, mc_r=None):
                    if not res['ok']:
                        return float('-inf')
                    if mc_r is not None and mc_r > r_max:
                        return float('-inf')
                    return res['apogee_m']

            elif mode == "有翼最適化モード":
                # Winged: maximize hang time; constraint = MC circle ≤ r_max
                elev_grid = [60, 68, 75, 82, 88]
                azi_grid  = [0, 45, 90, 135, 180, 225, 270, 315]
                use_mc = True

                def objective(res, mc_r=None):
                    if not res['ok']:
                        return float('-inf')
                    if mc_r is not None and mc_r > r_max:
                        return float('-inf')
                    return res['hang_time']
            else:
                self._opt_queue.put(('error', f'Unknown mode: {mode}'))
                return

            # ── Phase 1 ─ coarse deterministic grid ────────────────────────
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

            # Rank by nominal objective (higher = better). For MC modes the
            # nominal score ignores the constraint, so this just gives us
            # a sensible ordering to Monte-Carlo from the top down.
            candidates.sort(key=lambda x: -x[0] if math.isfinite(x[0])
                                              else float('inf'))

            if not use_mc:
                # Hover: phase 1 already has the constraint baked into
                # `objective` (returns -inf when r > r_max), so the best
                # finite candidate is the answer. If all are -inf the
                # constraint is infeasible with this grid.
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
                # ── Phase 2 ─ Monte-Carlo refinement for top-N candidates ─
                top_n = min(5, len(candidates))
                mc_trials = 8  # small sample so UX stays under a couple minutes
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
                    # Reserve the last ~15 % of the bar for the phase-3
                    # final MC run at the winning candidate.
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
                        f'制約 (MC {self.landing_prob}%円 ≤ {r_max:.1f} m) を\n'
                        '満たす候補がありませんでした。\n'
                        'r_max を大きくするか、風 / 機体設定を見直してください。'))
                    return

                score, best_e, best_a, best_res, best_mc_r = best

            # ── Phase 3 ─ Final high-sample MC at the winning (elev, azi) ──
            # This is the "final simulation including Monte Carlo" required
            # by the spec. Runs on the worker thread so the progress bar is
            # accurate right up to completion.
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
            # Prefer the refined phase-3 estimate if it's finite;
            # otherwise fall back to the phase-2 value (or None).
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
        """Spin up the optimizer in a daemon thread + show progress Toplevel.

        The main thread keeps draining `_opt_queue` via `_poll_optimization`
        so UI stays responsive. `_finish_optimization` consumes the final
        payload (or cancellation / error) and wires the result back into
        the regular render path.
        """
        self._optimizing = True
        self._opt_stop_flag.clear()
        # Drain any stale messages from a previous run.
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

        # Disable the RUN button while the optimizer is live so the user
        # can't kick off a concurrent run.
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
        """Signal the worker thread to stop at the next checkpoint."""
        self._opt_stop_flag.set()
        if self._opt_progress_msg is not None:
            try:
                self._opt_progress_msg.set("キャンセル中...")
            except Exception:
                pass

    def _poll_optimization(self):
        """Main-thread pump that drains `_opt_queue` on a tk after() cadence."""
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
        """Tear down the progress dialog and wire the result back into the UI.

        This is the single place where the optimization outcome becomes
        visible. Responsibilities in order:

          1. Tear down the progress Toplevel and re-enable RUN.
          2. Push the optimal (elev, azi) into the UI spinboxes so the
             main screen now reflects the winning configuration.
          3. Re-parse UI params (spinboxes now hold the optimal values)
             and run ONE final, clean `_simulate_once` at (elev, azi).
          4. Hand the result to `_apply_sim_result_to_ui`, passing the
             phase-3 Monte-Carlo radius as `override_r90` so the circle
             drawn on the 3-D profile and the 2-D map is MC-derived.
          5. Store `_last_optimization_info` so `update_plots` can add a
             "Best Elev/Azi" line to the stats block in the profile view.
          6. Show a pop-up summary dialog with the exact optimal numbers
             — this is the explicit notification the user asked for.
          7. Re-arm Monitor Mode so wind drift from the optimal state is
             tracked.
        """
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

        # ── Step 2 — push optimal angles into the launcher spinboxes ─────
        # This is an explicit spec requirement: the main-screen input
        # fields must be overwritten so the user sees the optimal values
        # directly in the entry widgets.
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

        # Flush any pending Tk idle callbacks (including variable traces
        # that fired when we overwrote the spinboxes — notably the
        # Lock-unlock trace) BEFORE we re-read them in the render step.
        # Without this, a trace handler could re-arm the UI in a state
        # that collides with the render we're about to do.
        try:
            self.update_idletasks()
        except Exception:
            pass

        # ── Step 5 — remember the optimum for the profile stats block ────
        # Set BEFORE the render below so `update_plots` sees it and adds
        # the "Best Elevation / Best Azimuth" line to the stats readout.
        mode_short_map = {
            "定点滞空最適化モード": "Hover",
            "高度最適化モード":     "Altitude",
            "有翼最適化モード":     "Winged",
        }
        self._last_optimization_info = {
            'elev':       best_elev,
            'azi':        best_azi,
            'mode':       mode,
            'mode_short': mode_short_map.get(mode, mode),
            'mc_r':       mc_r,
            'r_max':      r_max,
        }

        # ── Steps 3 & 4 — final clean simulation + render with MC circle ─
        # Spec requirement: "Using those set values, automatically execute
        # the simulation and the graph/map rendering process exactly once".
        # We call the same helper Free Mode uses, so rendering is IDENTICAL
        # to what pressing RUN manually would produce — just with the MC
        # dispersion radius substituted in for the landing circle.
        final_ok = self._render_current_params(override_r90=mc_r)
        if not final_ok:
            messagebox.showwarning(
                "最終再現シミュレーション失敗",
                "最適値の最終再現計算に失敗しました。\n"
                "最適角度はUIに反映済みですので、もう一度RUNを押すと再計算されます。")

        # ── Step 6 — explicit pop-up notification of the optimal solution ─
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
        lines.append("")
        if final_ok:
            lines.append("最適値はElevation/Azimuth入力欄に自動反映されました。")
            lines.append("3Dプロファイル・マップに最適軌道を描画しました。")
            lines.append("")
            lines.append("⚠ ウィンドモニター (Lock & Monitor) を自動ONにしました。")
            lines.append("   風の変化が許容差を超えるとアラートと再計算が走ります。")
        messagebox.showinfo("最適化完了 / Optimization Complete",
                            "\n".join(lines))

        # ── Step 7 — arm Monitor Mode for drift watch at the optimum ─────
        # Spec branch: optimization modes MUST auto-enter Monitor Mode
        # after the final render, so the locked configuration is watched
        # for wind drift and triggers re-calculation on a tolerance
        # violation. Free Mode explicitly does NOT do this (handled in
        # `run_simulation`).
        if final_ok:
            self._auto_enable_monitor_mode()

    # Per-mode default for r_max (指定半径). Apogee- and wing-based modes
    # need a much larger permissible dispersion radius than the hovering
    # mode, which is deliberately restrictive (the goal is a pinpoint
    # hover near r=0).
    _MODE_DEFAULT_RMAX = {
        "自由モード":            None,    # r_max hidden
        "定点滞空最適化モード":   50.0,
        "高度最適化モード":       250.0,
        "有翼最適化モード":       250.0,
    }

    def _apply_mode_default_rmax(self, mode):
        """Overwrite the r_max StringVar with the default for `mode`.

        Hovering Mode keeps a 50 m default; Altitude and Winged modes use
        250 m because their objectives (peak altitude / hang time) favour
        higher-energy trajectories that naturally disperse further.
        """
        default = self._MODE_DEFAULT_RMAX.get(mode)
        if default is None:
            return   # Free Mode — nothing to set
        try:
            self.r_max_var.set(f"{default:.1f}")
        except Exception:
            pass

    def _on_mode_change(self, event=None):
        """Combobox handler — adjusts r_max visibility + Lock widget state.

        Every mode change (Free ↔ Hover ↔ Altitude ↔ Winged, in any
        direction) force-releases the Lock & Monitor toggle because a
        captured wind baseline is only valid for the configuration that
        was just optimized — switching modes invalidates that baseline.
        """
        mode = self.operation_mode_var.get()

        # Also pick the appropriate default r_max for the new mode so the
        # user doesn't have to re-enter it every time they switch. Only
        # overwrite the field on mode changes — not on init, where the
        # StringVar already has its own initial value.
        self._apply_mode_default_rmax(mode)

        if mode == "自由モード":
            # Hide r_max field
            try:
                self.rmax_label.grid_remove()
                self.rmax_entry.grid_remove()
            except Exception:
                pass
            # Free Mode disables the Lock feature entirely. Release any
            # active lock first so the UI is consistent with the new state.
            self._release_lock_if_active(
                reason_label="自由モード (ロック機能無効)")
            try:
                self.lock_monitor_check.state(["disabled"])
            except Exception:
                pass
        else:
            try:
                self.rmax_label.grid()
                self.rmax_entry.grid()
            except Exception:
                pass
            # Switching between optimization modes — re-enable the widget
            # but drop any existing lock (baseline is mode-specific).
            try:
                self.lock_monitor_check.state(["!disabled"])
            except Exception:
                pass
            self._release_lock_if_active(reason_label="⭘ Unlocked")

    # ── Azimuth (horizontal rotation) helpers ────────────────────────────────────
    def _set_azim(self, azim, source="code"):
        """Single source of truth for 3-D azimuth.

        Called from three places:
          • slider command (source="slider")
          • mouse drag     (source="drag")
          • programmatic   (source="code")

        Keeps `self._fixed_azim`, the slider variable, and the label in sync, then
        schedules a lightweight redraw via draw_idle so updates feel smooth.
        Also refreshes the compass rose so its arrows track the new view.
        """
        if self._azim_updating:
            return
        self._azim_updating = True
        try:
            a = float(azim)
            # Wrap to -180..180 so the slider stays inside its range
            a = ((a + 180.0) % 360.0) - 180.0
            self._fixed_azim = a

            if hasattr(self, 'azim_label'):
                self.azim_label.config(text=f"{a:+.0f}°")

            # Sync slider only when change is real (avoids feedback loop / jitter)
            if source != "slider" and hasattr(self, 'azim_var'):
                try:
                    if abs(self.azim_var.get() - a) > 0.5:
                        self.azim_var.set(a)
                except tk.TclError:
                    pass

            if hasattr(self, 'ax'):
                try:
                    self.ax.view_init(elev=self._fixed_elev, azim=a)
                    # Compass must reflect the new azimuth in real time.
                    if getattr(self, '_compass_ax', None) is not None:
                        self._draw_compass()
                    self.canvas.draw_idle()
                except Exception:
                    pass
        finally:
            self._azim_updating = False

    def _on_azim_slider(self, value):
        # Tk Scale passes the value as a string
        self._set_azim(value, source="slider")

    def _reset_azim(self):
        self._set_azim(-60.0, source="code")

    # ── Mouse-drag rotation handlers ─────────────────────────────────────────────
    # Bound as methods (rather than nested closures inside create_profile_section)
    # so they're discoverable when reading the class. Horizontal drag rotates
    # azimuth via our slider-aware path; matplotlib's built-in 3-D rotation still
    # handles vertical drag for elevation, and `_on_view_changed` syncs the
    # compass back from whichever path produced the latest view.
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
        """Sync compass + slider whenever the 3-D view angle changes.

        Triggered by matplotlib's `draw_event`, which captures BOTH our custom
        horizontal-drag handler and matplotlib's built-in vertical/free 3-D
        rotation. Reads the current `ax.azim` / `ax.elev`, updates our
        single-source-of-truth state, and redraws the compass to match.

        Recursion is naturally bounded: after one redraw cycle the drift
        between `ax.azim/elev` and `_fixed_azim/_fixed_elev` falls below the
        threshold, so the handler bails out.
        """
        if not hasattr(self, 'ax'):
            return
        try:
            ax_azim = float(self.ax.azim)
            ax_elev = float(self.ax.elev)
        except Exception:
            return

        # Drift in azimuth handles 360° wrap; elevation is plain linear.
        azim_drift = abs(((ax_azim - self._fixed_azim) + 180.0) % 360.0 - 180.0)
        elev_drift = abs(ax_elev - self._fixed_elev)
        if azim_drift < 0.5 and elev_drift < 0.5:
            return

        self._fixed_azim = ((ax_azim + 180.0) % 360.0) - 180.0
        self._fixed_elev = ax_elev

        # Sync slider/label without re-triggering view_init.
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

        # Refresh the compass to match the new view angle.
        self._draw_compass()
        self.canvas.draw_idle()

    # ── Uncertainty helpers ──────────────────────────────────────────────────────
    def _prob_to_z(self, pct):
        """One-tail standard-normal z-score for a given confidence level.

        We keep a small lookup table so the code has no hard SciPy dependency.
        Unknown percentages fall back to the 90 % value (1.645).
        """
        table = {50: 0.674, 68: 1.000, 80: 1.282, 85: 1.440,
                 90: 1.645, 95: 1.960, 99: 2.576}
        return table.get(int(pct), 1.645)

    def _open_settings_window(self):
        """Open a Toplevel for Monte-Carlo / uncertainty parameters."""
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
                # A changed landing-probability invalidates the locked
                # baseline because the dispersion-circle radius now uses
                # a different z-score. Release the lock so the user
                # consciously re-RUNs with the new setting.
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

    # ── Lock & Monitor helpers ───────────────────────────────────────────────────
    def _release_lock_if_active(self, reason_label=None):
        """Force Lock & Monitor OFF.

        Called whenever a setting that invalidates the captured-baseline
        state changes (landing probability, operation mode, airframe /
        parachute parameters). Idempotent — a no-op if already unlocked.

        When `reason_label` is provided, that text replaces the default
        "⭘ Unlocked" status so the user sees WHY the lock disengaged.
        """
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
                self._toggle_lock_monitor()   # does the real work
            except Exception:
                pass
        if reason_label is not None:
            try:
                self.monitor_status_label.config(
                    text=reason_label, foreground="gray", background="")
            except Exception:
                pass

    def _toggle_lock_monitor(self):
        """Checkbox handler. Lock RUN + start periodic wind check when ON."""
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
        """Flip the Lock & Monitor toggle ON after a successful simulation.

        Spec-correct call sites (v0.4):
          • `_finish_optimization` — optimization modes (Hover / Altitude /
            Winged) MUST auto-lock after the final render so the chosen
            configuration is watched for wind drift.
          • Free Mode explicitly does NOT call this — the user is free to
            tweak parameters and re-RUN without the UI locking on them.
            The Lock widget is disabled entirely in Free Mode via
            `_on_mode_change`.

        Idempotent — if the toggle is already on we just refresh the wind
        baseline so the comparison floor reflects the freshest sample.
        """
        try:
            if not self.lock_monitor_var.get():
                self.lock_monitor_var.set(True)
                self._toggle_lock_monitor()
            else:
                # Already locked; just refresh the baseline to "now".
                self._capture_wind_baseline()
        except Exception:
            # UI may not be fully constructed in unusual call paths; ignore.
            pass

    def _capture_wind_baseline(self):
        """Snapshot current wind values to compare against during monitoring."""
        try:
            surf_spd = (sum(self.surf_wind_history) / len(self.surf_wind_history)
                        if self.surf_wind_history
                        else float(self.surf_spd_slider.get()))
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
        """Minimum absolute difference between two compass bearings (deg)."""
        return abs(((a - b) + 180.0) % 360.0 - 180.0)

    def _monitor_wind_tick(self):
        """Periodic wind check. If out of tolerance, flash + auto rerun."""
        self._monitor_after_id = None
        if not self.lock_monitor_var.get():
            return
        if self._baseline_wind is None:
            self._schedule_monitor_tick()
            return

        try:
            cur_surf = (sum(self.surf_wind_history) / len(self.surf_wind_history)
                        if self.surf_wind_history else 0.0)
            cur_surf_dir = float(self.surf_dir_var.get())
            cur_up       = float(self.up_spd_var.get())
            cur_up_dir   = float(self.up_dir_var.get())
        except ValueError:
            self._schedule_monitor_tick()
            return

        b = self._baseline_wind
        SPD_TOL, DIR_TOL = 2.0, 15.0
        exceeded = (
            abs(cur_surf    - b["surf_spd"]) >= SPD_TOL or
            abs(cur_up      - b["up_spd"])   >= SPD_TOL or
            self._angle_diff(cur_surf_dir, b["surf_dir"]) >= DIR_TOL or
            self._angle_diff(cur_up_dir,   b["up_dir"])   >= DIR_TOL
        )

        if exceeded:
            self._flash_alert()
            # Temporarily allow the internal RUN call, then re-lock.
            try:
                self.run_button.state(["!disabled"])
            except Exception:
                pass
            self.run_simulation()
            try:
                self.run_button.state(["disabled"])
            except Exception:
                pass
            # Baseline is refreshed inside run_simulation on success.

        self._schedule_monitor_tick()

    # `_flash_alert` was removed with feature #7 — the flashing red banner
    # has been replaced by a blocking `messagebox.showwarning` pop-up in
    # `_monitor_wind_tick`, which is both more noticeable and doesn't
    # animate behind a dialog the user is already reading.

    # ── Compass + fixed-label drawing helpers ────────────────────────────────────
    def _draw_compass(self):
        """Draw a small 2-D compass rose in the bottom-right of the figure.

        The N/E/S/W arrows track the current 3-D view so the user can see
        which screen direction corresponds to each cardinal direction.

        Re-callable: removes any prior compass axes before adding a fresh
        one. Since free mouse rotation is disabled (feature #3) the only
        caller that changes the view is `_set_azim`, which invokes this
        helper to keep the rose in sync with the [0, 180]° slider.
        """
        # Tear down a previous compass axes (if any) so re-draws don't pile up.
        if getattr(self, '_compass_ax', None) is not None:
            try:
                self._compass_ax.remove()
            except Exception:
                pass
            self._compass_ax = None

        # Bottom-right corner — leaves the top of the figure free for the
        # legend (which now sits above the axes after feature #1).
        cax = self.fig.add_axes([0.83, 0.04, 0.14, 0.14], facecolor='none',
                                zorder=20)
        self._compass_ax = cax
        cax.set_xlim(-1.4, 1.4); cax.set_ylim(-1.4, 1.4)
        cax.set_aspect('equal')
        cax.set_xticks([]); cax.set_yticks([])
        for sp in cax.spines.values():
            sp.set_visible(False)

        # Faint background disc so labels stay readable over the 3-D axes.
        cax.add_patch(plt.Circle((0, 0), 1.15, fill=True,
                                 facecolor='white', edgecolor='gray',
                                 lw=0.8, alpha=0.85))

        a = math.radians(self._fixed_azim)
        e = math.radians(self._fixed_elev)
        # Projection of a world vector (vx, vy, 0) onto the 2-D screen.
        # Derived from matplotlib's right/up basis for the 3-D view.
        def proj(vx, vy):
            sx = vx * (-math.sin(a)) + vy * math.cos(a)
            sy = (vx * (-math.sin(e) * math.cos(a))
                  + vy * (-math.sin(e) * math.sin(a)))
            return sx, sy

        def norm(x, y):
            r = math.hypot(x, y) or 1.0
            return x / r, y / r

        nx, ny = norm(*proj(0, 1))   # North = +y world
        ex, ey = norm(*proj(1, 0))   # East  = +x world
        sx, sy = -nx, -ny
        wx, wy = -ex, -ey

        R = 0.78
        # North arrow in red (convention)
        cax.annotate('', xy=(nx * R, ny * R), xytext=(0, 0),
                     arrowprops=dict(arrowstyle='-|>', color='red', lw=1.8))
        # Other cardinals in gray
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
        """Replace matplotlib's rotating 3-D axis labels with fixed 2-D text.

        Placing text via `text2D` with `transform=ax.transAxes` keeps them
        anchored to figure-relative positions regardless of azimuth.
        """
        self.ax.set_xlabel('')
        self.ax.set_ylabel('')
        self.ax.set_zlabel('')
        # Compass-direction text annotations ("East →" / "North ↑") were
        # removed per user request — the compass inset (top-right) already
        # shows N/E/S/W, so duplicating that information with text on the
        # plot body was redundant clutter.
        # Altitude label is retained because the vertical dimension has
        # no corresponding compass-inset indicator.
        self.ax.text2D(0.02, 0.02, 'Altitude (Up)',
                       transform=self.ax.transAxes,
                       ha='left', va='bottom',
                       fontsize=8, fontweight='bold', color='#333333')

    def _clear_previous_landing(self):
        """Reset stored landing state so old markers don't survive a relocation.

        Implements feature #4. Called whenever the launch point moves (via
        IP geolocate or the [Update Map] button). The 3-D plot is gated by
        the presence of `data` in `update_plots`; the map is gated by
        `_has_sim_result` in `draw_map_elements`. Resetting `land_lat/lon`
        and `r90_radius` ensures a stale impact circle can't be re-drawn
        even if `draw_map_elements` is called directly.
        """
        self.land_lat = self.launch_lat
        self.land_lon = self.launch_lon
        self.r90_radius = 0.0
        self._has_sim_result = False
        self._last_sim_data = None

    # ── Explicit axes rectangle for the 3-D view ────────────────────────────────
    # `tight_layout(rect=…)` was unreliable with a figure-anchored legend +
    # a 3-D axes — matplotlib could re-measure the "tight" bounds slightly
    # differently each redraw, which meant the plot visibly shrank every
    # time `run_simulation` triggered a rebuild. The axes is now pinned to
    # a fixed figure rectangle so every redraw produces the identical
    # geometry and the plot sits directly under the legend/stats strip
    # (no wasted bottom whitespace).
    #
    # Units are figure fractions (0..1). The top 22 % is reserved for the
    # parallel (side-by-side) strip: stats readout on the LEFT, plot legend
    # on the RIGHT, both anchored to the figure (not the axes) so they stay
    # horizontally aligned regardless of axes content.
    _PLOT_RECT      = (0.02, 0.00, 0.96, 0.78)   # (left, bottom, width, height)
    _TOP_STRIP_FRAC = 0.78                       # axes top → strip bottom

    def update_plots(self, data=None):
        # Rebuild the axes as 3D (clear() on Axes3D can leave stale state)
        self.fig.clear()
        # fig.clear() removes our compass inset axes too — drop the stale handle
        # so `_draw_compass` doesn't try to .remove() an already-detached axes.
        self._compass_ax = None
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        self.ax = self.fig.add_subplot(111, projection='3d')
        # Anchor the axes to the explicit figure rectangle defined above so
        # the plot never shrinks between redraws and always sits directly
        # below the top legend/stats strip.
        self.ax.set_position(list(self._PLOT_RECT))

        if not data:
            # Pre-flight (or post-relocate) view: show only the launch marker
            # and an empty arena. The 50-m target ring has been removed
            # (feature #6).
            self.ax.set_title("3D Trajectory Profile",
                              fontsize=9, fontweight='bold', pad=18)
            self.ax.scatter([0], [0], [0], marker='^', color='blue',
                            s=60, zorder=6, label='Launch')
            self.ax.set_xlim(-60, 60); self.ax.set_ylim(-60, 60)
            self.ax.set_zlim(0, 60)
            self._apply_fixed_axis_labels()
            self.ax.view_init(elev=self._fixed_elev, azim=self._fixed_azim)
            # Pre-flight: only a Launch marker exists, so the "stats" side
            # of the strip is empty — still anchor the legend to the RIGHT
            # so once a simulation runs the visual position doesn't jump.
            self.ax.legend(loc='upper right',
                           bbox_to_anchor=(0.98, 0.985),
                           bbox_transform=self.fig.transFigure,
                           ncol=2, fontsize=7, framealpha=0.85)
            self.fig.tight_layout(rect=[0, 0, 1.0, self._TOP_STRIP_FRAC])
            self._draw_compass()
            self.canvas.draw()
            self.draw_map_elements()
            return

        # ── Unpack data ───────────────────────────────────────────────────────────
        x_vals   = data['x']          # East  (RocketPy x = East for heading=0)
        y_vals   = data['y']          # North
        z_vals   = data['z']          # Altitude
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

        # ── Phase colours ─────────────────────────────────────────────────────────
        # Phase 1 – Powered/coast:        solid blue
        # Phase 2 – Freefall post-backfire: solid orange
        # Phase 3 – Under canopy:          dashed deepskyblue
        lw = 2.0
        if has_bf and has_para:
            self.ax.plot(x_vals[:idx_bf+1], y_vals[:idx_bf+1], z_vals[:idx_bf+1],
                         color='royalblue', lw=lw, label='Powered / Coast')
            self.ax.plot(x_vals[idx_bf:idx_para+1], y_vals[idx_bf:idx_para+1], z_vals[idx_bf:idx_para+1],
                         color='darkorange', lw=lw, label='Freefall (post-backfire)')
            self.ax.plot(x_vals[idx_para:], y_vals[idx_para:], z_vals[idx_para:],
                         color='deepskyblue', lw=lw, linestyle='--', label='Under Canopy')
            # Chute-open marker
            # Legend label is STATIC per feature #5 — the exact
            # parachute-open time is already surfaced in the stats readout
            # block, so the legend just names the marker.
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

        # ── Apogee marker ─────────────────────────────────────────────────────────
        if len(z_vals) > 0:
            ap_idx = int(np.argmax(z_vals))
            ax_, ay_, az_ = x_vals[ap_idx], y_vals[ap_idx], z_vals[ap_idx]
            # Vertical dashed drop-line from apogee to ground
            self.ax.plot([ax_, ax_], [ay_, ay_], [0, az_],
                         color='gray', linestyle=':', lw=1.2)
            # Legend label is STATIC per feature #5 — the measured apogee
            # altitude lives in the stats readout, so the legend just
            # identifies the marker type.
            self.ax.scatter([ax_], [ay_], [az_],
                            marker='*', color='gold', s=120, zorder=6,
                            label='Apogee')

        # ── Backfire marker ───────────────────────────────────────────────────────
        if bf_x is not None and bf_z is not None:
            # Legend label is STATIC per feature #5 — the backfire time
            # is reported in the stats block.
            self.ax.scatter([bf_x], [bf_y], [bf_z],
                            marker='X', color='magenta', s=80, zorder=6,
                            label='Backfire')
            # Drop-line to ground
            self.ax.plot([bf_x, bf_x], [bf_y, bf_y], [0, bf_z],
                         color='magenta', linestyle=':', lw=1.0, alpha=0.6)

        # ── Launch point ──────────────────────────────────────────────────────────
        self.ax.scatter([0], [0], [0], marker='^', color='blue', s=60, zorder=6,
                        label='Launch')

        # ── Impact point ──────────────────────────────────────────────────────────
        self.ax.scatter([impact_x], [impact_y], [0],
                        marker='o', color='red', s=60, zorder=6, label='Impact')

        # 50-m target zone removed in feature #6 — it cluttered the plot.

        # ── Landing-area circle on the ground plane (radius from Monte-Carlo) ────
        # Legend label is dynamic so it always reflects the chosen probability
        # (feature #5).
        theta = np.linspace(0, 2 * math.pi, 72)
        cx = impact_x + r90 * np.cos(theta)
        cy = impact_y + r90 * np.sin(theta)
        cz = np.zeros_like(theta)
        self.ax.plot(cx, cy, cz, color='red', lw=1.5, alpha=0.6,
                     label=f'Landing Area ({self.landing_prob}%)')

        # Filled semi-transparent disc for the landing area
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        n_pts = 60
        disc_theta = np.linspace(0, 2 * math.pi, n_pts)
        disc_x = impact_x + r90 * np.cos(disc_theta)
        disc_y = impact_y + r90 * np.sin(disc_theta)
        disc_z = np.zeros(n_pts)
        verts  = [list(zip(disc_x, disc_y, disc_z))]
        poly   = Poly3DCollection(verts, alpha=0.12, facecolor='red', edgecolor='none')
        self.ax.add_collection3d(poly)

        # ── Ground grid projection (shadow of trajectory) ─────────────────────────
        self.ax.plot(x_vals, y_vals, np.zeros_like(z_vals),
                     color='gray', lw=0.8, alpha=0.35, linestyle='--')

        # ── Wind quivers ─────────────────────────────────────────────────────────
        # Drawn at the launch column (x=0, y=0) at several altitudes.
        # Arrow direction shows where the wind pushes the rocket (opposite of wind origin).
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
            # RocketPy: wind_u = East component (positive → pushes East)
            #           wind_v = North component
            self.ax.quiver(0, 0, alt,
                           u_a * scale / (spd + 1e-9),
                           v_a * scale / (spd + 1e-9),
                           0,
                           color='limegreen', lw=1.2, arrow_length_ratio=0.3)
            self.ax.text(u_a * scale / (spd + 1e-9),
                         v_a * scale / (spd + 1e-9),
                         alt + alt_max * 0.02,
                         f'{spd:.1f}m/s', color='green', fontsize=7)

        # ── Lock elevation/azimuth from our single source of truth ───────────────
        self.ax.view_init(elev=self._fixed_elev, azim=self._fixed_azim)

        # ── Axes labels & style (fixed-screen labels + title) ─────────────────────
        self._apply_fixed_axis_labels()
        self.ax.set_title('3D Trajectory Profile',
                          fontsize=9, fontweight='bold', pad=18)
        self.ax.tick_params(labelsize=7)

        # ── Consolidated stats readout (feature #2 + #3) ─────────────────────────
        # Apogee, backfire time, parachute deployment time, downrange and the
        # predicted landing-circle radius are now anchored to a single corner
        # of the plot so all the headline numbers are visible at a glance —
        # no more chasing different labels around the UI.
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
        # Append the optimization outcome, if the last rendered flight
        # came from an optimization pass (modes 2/3/4). Cleared when a
        # Free-Mode run happens so the label never lies.
        opt_info = getattr(self, '_last_optimization_info', None)
        if opt_info:
            stats_text += (
                f'\nBest Elevation:   {opt_info["elev"]:.1f}°\n'
                f'Best Azimuth:     {opt_info["azi"]:.1f}°'
                f'   [{opt_info["mode_short"]}]'
            )
        # Anchor the stats to the FIGURE (not the axes) at the top-left
        # of the reserved strip so it sits side-by-side with the legend
        # (which is anchored top-right of the same strip) — the two
        # elements form a single horizontal band above the plot area.
        self.fig.text(
            0.02, 0.985, stats_text,
            ha='left', va='top',
            fontsize=7, fontweight='bold', color='#222222',
            family='monospace',
            bbox=dict(boxstyle='round,pad=0.35',
                      facecolor='white', edgecolor='gray', alpha=0.9))

        # Keep aspect roughly equal on x/y; z can compress
        all_horiz = np.concatenate([x_vals, y_vals, [impact_x + r90, impact_x - r90,
                                                      impact_y + r90, impact_y - r90]])
        h_range = max(float(np.max(all_horiz) - np.min(all_horiz)), 1.0)
        mid_x   = float((np.max(x_vals) + np.min(x_vals)) / 2)
        mid_y   = float((np.max(y_vals) + np.min(y_vals)) / 2)
        self.ax.set_xlim(mid_x - h_range * 0.6, mid_x + h_range * 0.6)
        self.ax.set_ylim(mid_y - h_range * 0.6, mid_y + h_range * 0.6)
        self.ax.set_zlim(0, alt_max * 1.15)

        # Legend is anchored to the FIGURE top-right, side-by-side with the
        # stats box on the top-left (both occupy the top strip above the
        # axes). `ncol=3` keeps the horizontal legend compact enough not
        # to collide with the stats box on the left.
        self.ax.legend(loc='upper right',
                       bbox_to_anchor=(0.98, 0.985),
                       bbox_transform=self.fig.transFigure,
                       ncol=3, fontsize=7, framealpha=0.85)
        # Reserve the top strip (1 − _TOP_STRIP_FRAC) for stats + legend.
        self.fig.tight_layout(rect=[0, 0, 1.0, self._TOP_STRIP_FRAC])
        # Compass is added AFTER tight_layout so it keeps its fixed corner.
        self._draw_compass()
        self.canvas.draw()
        self.draw_map_elements()

    def draw_map_elements(self):
        self.map_widget.delete_all_polygon()
        # Always show launch point + 50 m reference ring around it.
        self.map_widget.set_polygon(self.get_circle_coords(self.launch_lat, self.launch_lon, 2.5), fill_color="blue")
        self.map_widget.set_polygon(self.get_circle_coords(self.launch_lat, self.launch_lon, 50), outline_color="blue")
        # Landing markers only after a valid simulation has produced them
        # (feature #4 — clears stale data when the launch point moves).
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
            margin_m = max(self.r90_radius * 1.5, 150.0)
            # Feature #1 fix — reuse the same WGS84 metres-per-degree helper
            # used by `_offset_to_latlon` and `get_circle_coords`, instead of
            # the 111 320 m constant-radius approximation that was here
            # before. Keeps every map coordinate calculation on ONE shared
            # conversion so the padding, landing dot, and dispersion ring
            # all agree.
            m_lat, m_lon = self._meters_per_degree(self.launch_lat)
            pad_lat = margin_m / m_lat
            pad_lon = margin_m / m_lon

            max_lat = max(self.launch_lat, self.land_lat) + pad_lat
            min_lat = min(self.launch_lat, self.land_lat) - pad_lat
            max_lon = max(self.launch_lon, self.land_lon) + pad_lon
            min_lon = min(self.launch_lon, self.land_lon) - pad_lon

            self.map_widget.fit_bounding_box((max_lat, min_lon), (min_lat, max_lon))
        except Exception:
            pass

    def on_parameter_edit_af(self, event=None):
        if self.af_name_label.cget("text") != "Airframe: (未選択)":
            self.af_name_label.config(text="Airframe: (未選択)")

    def on_parameter_edit_para(self, event=None):
        if self.para_name_label.cget("text") != "Parachute: (未選択)":
            self.para_name_label.config(text="Parachute: (未選択)")

    def save_af_settings(self):
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON Files", "*.json")])
        if filepath:
            try:
                data = {
                    "mass": float(self.mass_entry.get()),
                    "cg": float(self.cg_entry.get()),
                    "length": float(self.len_entry.get()),
                    "radius": float(self.radius_entry.get()),
                    "nose_length": float(self.nose_len_entry.get()),
                    "fin_root": float(self.fin_root_entry.get()),
                    "fin_tip": float(self.fin_tip_entry.get()),
                    "fin_span": float(self.fin_span_entry.get()),
                    "fin_pos": float(self.fin_pos_entry.get()),
                    "motor_pos": float(self.motor_pos_entry.get()),
                    "motor_dry_mass": float(self.motor_dry_mass_entry.get()),
                    "backfire_delay": float(self.backfire_delay_entry.get())
                }
                with open(filepath, 'w') as f:
                    json.dump(data, f, indent=4)
                messagebox.showinfo("完了", "機体設定を保存しました。")
                self.af_name_label.config(text=f"Airframe: {os.path.basename(filepath)}")
            except Exception as e:
                messagebox.showerror("エラー", f"保存失敗:\n{e}")

    def load_af_settings(self):
        filepath = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
        if filepath:
            try:
                with open(filepath, 'r') as f:
                    data = json.load(f)

                self.mass_entry.delete(0, tk.END); self.mass_entry.insert(0, str(data.get("mass", "0.0872")))
                self.cg_entry.delete(0, tk.END); self.cg_entry.insert(0, str(data.get("cg", "0.21")))
                self.len_entry.delete(0, tk.END); self.len_entry.insert(0, str(data.get("length", "0.383")))
                self.radius_entry.delete(0, tk.END); self.radius_entry.insert(0, str(data.get("radius", "0.015")))

                self.nose_len_entry.delete(0, tk.END); self.nose_len_entry.insert(0, str(data.get("nose_length", "0.08")))
                self.fin_root_entry.delete(0, tk.END); self.fin_root_entry.insert(0, str(data.get("fin_root", "0.04")))
                self.fin_tip_entry.delete(0, tk.END); self.fin_tip_entry.insert(0, str(data.get("fin_tip", "0.02")))
                self.fin_span_entry.delete(0, tk.END); self.fin_span_entry.insert(0, str(data.get("fin_span", "0.03")))
                self.fin_pos_entry.delete(0, tk.END); self.fin_pos_entry.insert(0, str(data.get("fin_pos", "0.35")))

                self.motor_pos_entry.delete(0, tk.END); self.motor_pos_entry.insert(0, str(data.get("motor_pos", "0.38")))
                self.motor_dry_mass_entry.delete(0, tk.END); self.motor_dry_mass_entry.insert(0, str(data.get("motor_dry_mass", "0.015")))

                self.backfire_delay_entry.delete(0, tk.END)
                if "backfire_delay" in data:
                    self.backfire_delay_entry.insert(0, str(data["backfire_delay"]))

                self.af_name_label.config(text=f"Airframe: {os.path.basename(filepath)}")
                messagebox.showinfo("完了", "機体設定を読み込みました。")
            except Exception as e:
                messagebox.showerror("エラー", f"読込失敗:\n{e}")

    def save_para_settings(self):
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON Files", "*.json")])
        if filepath:
            try:
                data = {
                    "cd": float(self.cd_entry.get()),
                    "area": float(self.area_entry.get()),
                    "lag": float(self.lag_entry.get())
                }
                with open(filepath, 'w') as f:
                    json.dump(data, f, indent=4)
                messagebox.showinfo("完了", "パラシュート設定を保存しました。")
                self.para_name_label.config(text=f"Parachute: {os.path.basename(filepath)}")
            except Exception as e:
                messagebox.showerror("エラー", f"保存失敗:\n{e}")

    def load_para_settings(self):
        filepath = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
        if filepath:
            try:
                with open(filepath, 'r') as f:
                    data = json.load(f)
                self.cd_entry.delete(0, tk.END); self.cd_entry.insert(0, str(data.get("cd", "")))
                self.area_entry.delete(0, tk.END); self.area_entry.insert(0, str(data.get("area", "")))
                self.lag_entry.delete(0, tk.END); self.lag_entry.insert(0, str(data.get("lag", "")))
                self.para_name_label.config(text=f"Parachute: {os.path.basename(filepath)}")
                messagebox.showinfo("完了", "パラシュート設定を読み込みました。")
            except Exception as e:
                messagebox.showerror("エラー", f"読込失敗:\n{e}")

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

            self.selected_motor_file = filepath
            self.thrust_data = time_thrust_points
            self.selected_motor_name = motor_name
            self.motor_burn_time = burn_time
            self.motor_ui_label.config(text=f"Engine: {motor_name}")

            messagebox.showinfo("読込完了", f"ローカルのモーターデータ(CSV)を読み込みました。\n・エンジン名: {motor_name}\n・燃焼時間: {burn_time:.3f} s")

        except Exception as e:
            messagebox.showerror("読込エラー", f"モーターファイルの読み込みに失敗しました:\n{e}")

    def create_data_section(self):
        # ── No scroll: frame fills the column directly ────────────────────────────
        frame = ttk.Frame(self, padding=(4, 4))
        frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        frame.columnconfigure(0, weight=1)

        # Helper: compact label+entry row using grid inside a parent frame
        # Returns the Entry widget.
        def param_row(parent, label_text, row, default=""):
            ttk.Label(parent, text=label_text, font=("Arial", 8)).grid(
                row=row, column=0, sticky="w", padx=(4, 2), pady=1)
            e = ttk.Entry(parent, width=7, font=("Arial", 8))
            e.grid(row=row, column=1, sticky="e", padx=(2, 4), pady=1)
            if default:
                e.insert(0, default)
            return e

        # ── Engine ────────────────────────────────────────────────────────────────
        self.motor_ui_label = ttk.Label(frame, text=f"Engine: {self.selected_motor_name}",
                                        font=("Arial", 9, "bold"), foreground="#B22222")
        self.motor_ui_label.grid(row=0, column=0, sticky="w", pady=(0, 1))
        mbf = ttk.Frame(frame); mbf.grid(row=1, column=0, sticky="ew", pady=(0, 3))
        mbf.columnconfigure(0, weight=1); mbf.columnconfigure(1, weight=1)
        ttk.Button(mbf, text="[ThrustCurve Web]", command=self.open_thrustcurve_web).grid(
            row=0, column=0, sticky="ew", padx=(0, 1))
        ttk.Button(mbf, text="[Load CSV]", command=self.load_local_motor).grid(
            row=0, column=1, sticky="ew", padx=(1, 0))

        ttk.Separator(frame, orient="horizontal").grid(row=2, column=0, sticky="ew", pady=3)

        # ── Airframe ──────────────────────────────────────────────────────────────
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

        af_btn_f = ttk.Frame(frame); af_btn_f.grid(row=6, column=0, sticky="ew", pady=(0, 2))
        af_btn_f.columnconfigure(0, weight=1); af_btn_f.columnconfigure(1, weight=1)
        ttk.Button(af_btn_f, text="Load AF JSON", command=self.load_af_settings).grid(
            row=0, column=0, sticky="ew", padx=(0, 1))
        ttk.Button(af_btn_f, text="Save AF JSON", command=self.save_af_settings).grid(
            row=0, column=1, sticky="ew", padx=(1, 0))

        af_entries = [self.mass_entry, self.cg_entry, self.len_entry, self.radius_entry,
                      self.nose_len_entry, self.fin_root_entry, self.fin_tip_entry,
                      self.fin_span_entry, self.fin_pos_entry, self.motor_pos_entry,
                      self.motor_dry_mass_entry, self.backfire_delay_entry]
        for e in af_entries:
            e.bind("<KeyRelease>", self.on_parameter_edit_af)

        ttk.Separator(frame, orient="horizontal").grid(row=7, column=0, sticky="ew", pady=3)

        # ── Parachute ─────────────────────────────────────────────────────────────
        self.para_name_label = ttk.Label(frame, text="Parachute: (未選択)", font=("Arial", 8, "bold"))
        self.para_name_label.grid(row=8, column=0, sticky="w")

        para_lf = ttk.LabelFrame(frame, text="Parachute", padding=(2, 2))
        para_lf.grid(row=9, column=0, sticky="ew", pady=(1, 2))
        para_lf.columnconfigure(0, weight=1); para_lf.columnconfigure(1, weight=0)

        self.cd_entry   = param_row(para_lf, "Cd",       0)
        self.area_entry = param_row(para_lf, "Area (m²)",1)
        self.lag_entry  = param_row(para_lf, "Lag (s)",  2)

        para_btn_f = ttk.Frame(frame); para_btn_f.grid(row=10, column=0, sticky="ew", pady=(0, 2))
        para_btn_f.columnconfigure(0, weight=1); para_btn_f.columnconfigure(1, weight=1)
        ttk.Button(para_btn_f, text="Load Para JSON", command=self.load_para_settings).grid(
            row=0, column=0, sticky="ew", padx=(0, 1))
        ttk.Button(para_btn_f, text="Save Para JSON", command=self.save_para_settings).grid(
            row=0, column=1, sticky="ew", padx=(1, 0))

        self.cd_entry.bind("<KeyRelease>",   self.on_parameter_edit_para)
        self.area_entry.bind("<KeyRelease>", self.on_parameter_edit_para)
        self.lag_entry.bind("<KeyRelease>",  self.on_parameter_edit_para)

        ttk.Separator(frame, orient="horizontal").grid(row=11, column=0, sticky="ew", pady=3)

        # ── Launcher ──────────────────────────────────────────────────────────────
        ttk.Label(frame, text="Launcher", font=("Arial", 9, "bold")).grid(
            row=12, column=0, sticky="w", pady=(0, 1))

        launch_lf = ttk.LabelFrame(frame, text="Position & Rail", padding=(2, 2))
        launch_lf.grid(row=13, column=0, sticky="ew", pady=(1, 2))
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

        loc_btn_f = ttk.Frame(frame); loc_btn_f.grid(row=14, column=0, sticky="ew", pady=(0, 2))
        loc_btn_f.columnconfigure(0, weight=1); loc_btn_f.columnconfigure(1, weight=1)
        ttk.Button(loc_btn_f, text="Get Location (IP)",
                   command=lambda: self.get_current_location(manual=True)).grid(
            row=0, column=0, sticky="ew", padx=(0, 1))
        ttk.Button(loc_btn_f, text="Update Map",
                   command=self.update_map_center).grid(
            row=0, column=1, sticky="ew", padx=(1, 0))

        ttk.Separator(frame, orient="horizontal").grid(row=15, column=0, sticky="ew", pady=3)

        wind_lf = ttk.LabelFrame(frame, text="Wind (Dir: 0=N, 90=E)", padding=(2, 2))
        wind_lf.grid(row=16, column=0, sticky="ew", pady=(1, 2))
        wind_lf.columnconfigure(1, weight=1)

        ttk.Label(wind_lf, text="100m spd:", font=("Arial", 8)).grid(
            row=0, column=0, sticky="w", padx=(4, 2), pady=1)
        self.up_spd_var = tk.StringVar(value="8.0")
        ttk.Entry(wind_lf, textvariable=self.up_spd_var, width=5,
                  font=("Arial", 8)).grid(row=0, column=1, sticky="ew", padx=2, pady=1)
        ttk.Label(wind_lf, text="dir:", font=("Arial", 8)).grid(
            row=0, column=2, sticky="w", padx=(2, 2))
        self.up_dir_var = tk.StringVar(value="90")
        ttk.Entry(wind_lf, textvariable=self.up_dir_var, width=4,
                  font=("Arial", 8)).grid(row=0, column=3, sticky="e", padx=(0, 4), pady=1)

        ttk.Label(wind_lf, text="3m spd:", font=("Arial", 8)).grid(
            row=1, column=0, sticky="w", padx=(4, 2), pady=1)
        self.surf_spd_slider = ttk.Scale(wind_lf, from_=0, to=15, orient="horizontal")
        self.surf_spd_slider.set(4.0)
        self.surf_spd_slider.grid(row=1, column=1, sticky="ew", padx=2, pady=1)
        ttk.Label(wind_lf, text="dir:", font=("Arial", 8)).grid(
            row=1, column=2, sticky="w", padx=(2, 2))
        self.surf_dir_var = tk.StringVar(value="100")
        ttk.Entry(wind_lf, textvariable=self.surf_dir_var, width=4,
                  font=("Arial", 8)).grid(row=1, column=3, sticky="e", padx=(0, 4), pady=1)

        stats_f = ttk.Frame(wind_lf)
        stats_f.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(2, 2))
        self.wind_avg_label  = ttk.Label(stats_f, text="Avg: -- m/s",  foreground="green",
                                         font=("Arial", 8))
        self.wind_avg_label.pack(side="left", padx=4)
        self.wind_gust_label = ttk.Label(stats_f, text="Gust: -- m/s", foreground="red",
                                         font=("Arial", 8))
        self.wind_gust_label.pack(side="right", padx=4)

        ttk.Separator(frame, orient="horizontal").grid(row=17, column=0, sticky="ew", pady=3)

        # ── Lock & Monitor + Settings row ─────────────────────────────────────────
        lock_f = ttk.Frame(frame)
        lock_f.grid(row=18, column=0, sticky="ew", pady=(0, 2))
        lock_f.columnconfigure(0, weight=1)
        lock_f.columnconfigure(1, weight=0)

        self.lock_monitor_var = tk.BooleanVar(value=False)
        # Stashed on self so `_on_mode_change` can disable it in Free Mode
        # (per user spec: "Wind Monitoring / Auto-Lock Feature is completely
        # disabled" in Free Mode).
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

        # ── Operation Mode (v0.3 — four modes) ─────────────────────────────
        # Placed immediately above the RUN button so the user can see /
        # change the active mode right before launching a simulation. The
        # r_max field is grid_remove()'d in Free Mode and shown in all
        # three optimization modes via `_on_mode_change`.
        ttk.Separator(frame, orient="horizontal").grid(
            row=19, column=0, sticky="ew", pady=3)
        mode_lf = ttk.LabelFrame(frame, text="Operation Mode", padding=(4, 4))
        mode_lf.grid(row=20, column=0, sticky="ew", pady=(1, 2))
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
        # Hidden by default — Free Mode is the default mode.
        self.rmax_label.grid_remove()
        self.rmax_entry.grid_remove()

        # ── Run + results ─────────────────────────────────────────────────────────
        self.run_button = ttk.Button(frame, text="🚀  RUN SIMULATION",
                                     command=self.run_simulation)
        self.run_button.grid(row=21, column=0, sticky="ew",
                             ipady=4, pady=(2, 4))
        res_f = ttk.Frame(frame); res_f.grid(row=22, column=0, sticky="ew")
        res_f.columnconfigure(0, weight=1); res_f.columnconfigure(1, weight=1)
        self.apogee_label   = ttk.Label(res_f, text="Apogee: -- m",
                                        font=("Arial", 9, "bold"))
        self.apogee_label.grid(row=0, column=0, sticky="w", padx=4)
        self.velocity_label = ttk.Label(res_f, text="Impact: -- m/s",
                                        font=("Arial", 9, "bold"))
        self.velocity_label.grid(row=0, column=1, sticky="e", padx=4)

    def create_profile_section(self):
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 – registers 3d projection
        frame = ttk.Frame(self, padding=10, relief="solid", borderwidth=1)
        frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        # Canvas takes most of the vertical space; rotation bar fixed below it
        frame.rowconfigure(0, weight=1)
        frame.rowconfigure(1, weight=0)
        frame.columnconfigure(0, weight=1)

        # Slightly larger figure (feature #1) so the trajectory has more
        # breathing room. The Tk grid column already weights this section
        # (`columnconfigure(1, weight=2)` in __init__), so the canvas will
        # stretch to fill the available space — figsize just sets the
        # rendered DPI proportions.
        self.fig = plt.figure(figsize=(6.4, 5.2), dpi=100)
        self.ax  = self.fig.add_subplot(111, projection='3d')

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # ── Horizontal rotation bar (azimuth slider) ─────────────────────────────
        # Controls the 3-D view's horizontal pivot. Synchronises with mouse drag
        # so either input yields the same view angle.
        rot_bar = ttk.Frame(frame)
        rot_bar.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        rot_bar.columnconfigure(1, weight=1)

        ttk.Label(rot_bar, text="↻ Rotate:", font=("Arial", 8)).grid(
            row=0, column=0, sticky="w", padx=(2, 4))

        self.azim_var = tk.DoubleVar(value=self._fixed_azim)
        self.azim_slider = ttk.Scale(
            rot_bar, from_=-180, to=180, orient="horizontal",
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

        # ── Mouse interaction wiring ─────────────────────────────────────────────
        # Drag handlers used to be nested closures inside this method
        # (refactor #9). They're now bound methods so the wiring stays
        # discoverable and can be unit-tested in isolation.
        # `draw_event` (feature #7) keeps the compass synchronised with both
        # our custom drag and matplotlib's native 3-D rotate (vertical drag).
        self.canvas.mpl_connect('button_press_event',   self._on_canvas_press)
        self.canvas.mpl_connect('motion_notify_event',  self._on_canvas_motion)
        self.canvas.mpl_connect('button_release_event', self._on_canvas_release)
        self.canvas.mpl_connect('draw_event',           self._on_view_changed)

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
            # Feature #1 fix — actually re-center the map widget itself.
            # Previously only the internal lat/lon state was updated while the
            # tkintermapview kept the OLD tile centre, which made the newly-
            # drawn launch / landing circles appear offset from the visible
            # map area (the classic "coordinates don't match the drawn
            # position" symptom).
            try:
                self.map_widget.set_position(self.launch_lat, self.launch_lon)
            except Exception:
                pass
            # Launch point moved → previous landing data is stale (feature #4).
            self._clear_previous_landing()
            self.update_plots()
            self.fit_map_bounds()
        except ValueError:
            pass


if __name__ == "__main__":
    app = KazamidoriUI()
    app.mainloop()