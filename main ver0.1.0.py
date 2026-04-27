# 座標とマップ上の位置がずれてるかも
# East→などの表示を消去する
# マウスによるRotationを無効として，Rotationの範囲を0~180度に限定する
# DownRangeが50mを超えた場合文字の色を赤にする
# 凡例で動的データの表示をなくす
# プロファイルセクションで，Apogeeなどの算出データと凡例を並列でグラフの上に表示させる
# 再計算のときに，赤で許容差を出たと示すのではなく，ウィンドウで警告し，再計算する
# パラメータに変更が加えられたときに自動でロックを解除する
# ver 0.0.1

# 現在のモードからロック機能をすべて削除したものを自由モードとする
# 発射点を中心として半径r_max (m)に着地する中で，発射点から着地点の距離をr, 滞空時間をtとして，r_max-r+tが最大となる定点滞空最適化モード
# 発射点を中心として指定した半径内に90%着地する中で、最高高度に到達するための高度最適化モード
# 発射点を中心として指定した半径内に90%着地する中で、滞空時間が最大になるための有翼最適化モード
# ver 0.2.0

# Windy.com API, 風速計との連携
# オフラインマップモードを追加し，指定場所の半径500mを取得
# 風速のグラフ（平均風速に線），風向のコンパス？，を表示させる
# 風向，風速の標準偏差を求める(着地点の不確かさ評価のため)
# 風の不確かさ評価
# パラシュート設定をホーム画面で選択，設定，保存（パラシュートの形状でいくつかのデフォルトを設定）
# 自作風速計の外れ値除外計算を追加
# ver 1.0.0

# 実際に発射する手順として
# 最適化計算(高度or定点or有翼)により発射角，方位決定(風速風向許容差決定)
# 許容差を超えないかモニター
# 超えない場合はグリーン，超える場合は再計算し警告を流す

# 将来的には地形とかも考慮できるといいかもね

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
        self._fixed_azim     = 90.0     # current azimuth  (deg, clamped to [0, 180])
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

        self.create_data_section()
        self.create_profile_section()
        self.create_map_section()

        self.after(500, lambda: self.get_current_location(manual=False))
        self.after(1000, self.simulate_realtime_wind)

        self.update_plots()
        self.after(1000, self.fit_map_bounds)

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

        Shared by `run_simulation` and `get_circle_coords` so the landing
        point and the dispersion ring around it use the *same* metres-per-
        degree conversion — fixing the small but visible mis-alignment
        between the two (feature #1).
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

    def run_simulation(self):
        if not ROCKETPY_AVAILABLE:
            return
        try:
            try:
                self.launch_lat = float(self.lat_entry.get())
                self.launch_lon = float(self.lon_entry.get())
                elev  = float(self.elev_spin.get())
                azi   = float(self.azi_spin.get())
                rail  = float(self.rail_entry.get())

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
                messagebox.showerror("入力エラー", "未記入のパラメータがあります。\nすべての項目に数値を入力してください。")
                return

            surf_spd = sum(self.surf_wind_history) / len(self.surf_wind_history) \
                       if self.surf_wind_history else float(self.surf_spd_slider.get())
            surf_dir = float(self.surf_dir_var.get())
            up_spd   = float(self.up_spd_var.get())
            up_dir   = float(self.up_dir_var.get())

            # Wind components — converted via the dedicated helper method
            # rather than a nested function (refactor #9).
            surf_u, surf_v = self._wind_components(surf_spd, surf_dir)
            up_u,   up_v   = self._wind_components(up_spd,   up_dir)

            wind_u_prof = [(0, 0), (3, surf_u), (100, up_u), (5000, up_u)]
            wind_v_prof = [(0, 0), (3, surf_v), (100, up_v), (5000, up_v)]

            env = Environment(latitude=self.launch_lat, longitude=self.launch_lon, elevation=0)
            env.set_atmospheric_model(
                type="custom_atmosphere", pressure=None, temperature=300,
                wind_u=wind_u_prof, wind_v=wind_v_prof
            )

            if not getattr(self, 'thrust_data', None):
                messagebox.showerror("エンジン未選択", "エンジンが選択されていません。\n[Load Local CSV] から推力データを読み込んでください。")
                return

            airframe_len  = max(0.01,  airframe_len)
            airframe_mass = max(0.01,  airframe_mass)
            radius        = max(0.001, radius)

            I_z  = 0.5  * airframe_mass * (radius ** 2)
            I_xy = (1/12) * airframe_mass * (3 * (radius ** 2) + airframe_len ** 2)

            safe_burn_time = max(0.1, self.motor_burn_time)
            backfire_time  = safe_burn_time + backfire_delay

            motor = SolidMotor(
                thrust_source=self.thrust_data,
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
                center_of_dry_mass_position=0.0
            )

            rocket = Rocket(
                radius=radius, mass=airframe_mass, inertia=(I_xy, I_xy, I_z),
                power_off_drag=para_cd, power_on_drag=para_cd,
                center_of_mass_without_motor=-airframe_cg
            )
            rocket.add_motor(motor, position=-motor_pos)
            rocket.add_nose(length=nose_len, kind="vonKarman", position=0.0)
            rocket.add_trapezoidal_fins(
                n=4, root_chord=fin_root, tip_chord=fin_tip,
                span=fin_span, position=-fin_pos
            )

            # ── Time-based trigger via altitude peak-tracking ─────────────────────
            #
            # Problem with tick counters: RocketPy does NOT call the trigger at a
            # guaranteed t=0; the first call time is internal and version-dependent,
            # making tick-based timing unreliable.
            #
            # Problem with initial_solution / two-segment approach: the rail and
            # quaternion injection cause "underground" errors across RocketPy versions.
            #
            # Solution: run a FAST first pass (no parachute, terminate at apogee) to
            # learn the apogee time, then run the real simulation with a trigger that
            # fires when h crosses the altitude corresponding to backfire_time on the
            # ascent curve. This is purely altitude-based (which RocketPy handles
            # correctly in every version) but derived from the time we actually want.
            # ─────────────────────────────────────────────────────────────────────

            # ── Pass 1: no-chute flight to find apogee and altitude at backfire_time
            rocket_pass1 = Rocket(
                radius=radius, mass=airframe_mass, inertia=(I_xy, I_xy, I_z),
                power_off_drag=para_cd, power_on_drag=para_cd,
                center_of_mass_without_motor=-airframe_cg
            )
            motor_pass1 = SolidMotor(
                thrust_source=self.thrust_data, burn_time=safe_burn_time,
                grain_number=1, grain_density=1815,
                grain_outer_radius=radius * 0.8, grain_initial_inner_radius=0.005,
                grain_initial_height=0.1, nozzle_radius=radius * 0.8,
                throat_radius=0.005, interpolation_method="linear",
                nozzle_position=0,
                coordinate_system_orientation="nozzle_to_combustion_chamber",
                dry_mass=motor_dry_mass, dry_inertia=(1e-5, 1e-5, 1e-6),
                grain_separation=0.0, grains_center_of_mass_position=0.0,
                center_of_dry_mass_position=0.0
            )
            rocket_pass1.add_motor(motor_pass1, position=-motor_pos)
            rocket_pass1.add_nose(length=nose_len, kind="vonKarman", position=0.0)
            rocket_pass1.add_trapezoidal_fins(
                n=4, root_chord=fin_root, tip_chord=fin_tip,
                span=fin_span, position=-fin_pos
            )
            flight_pass1 = Flight(
                rocket=rocket_pass1, environment=env,
                rail_length=rail, inclination=elev, heading=azi,
                terminate_on_apogee=True
            )

            t1_arr = flight_pass1.z[:, 0]
            z1_arr = flight_pass1.z[:, 1]

            # Find altitude at backfire_time from pass-1 trajectory
            if backfire_time >= t1_arr[-1]:
                # backfire is at or after apogee → trigger altitude = apogee altitude
                # (chute will open at apogee, i.e. immediately on descent)
                backfire_alt = float(z1_arr[-1])
            else:
                idx_bf_p1    = int((np.abs(t1_arr - backfire_time)).argmin())
                backfire_alt = float(z1_arr[idx_bf_p1])

            # Clamp to a sensible minimum so we never trigger on the pad
            backfire_alt = max(backfire_alt, 1.0)

            # ── Pass 2: real flight with altitude-based trigger ────────────────────
            # Trigger callable is built by `_make_backfire_trigger` (a class method,
            # extracted from a previously-nested closure for readability — refactor
            # #9). It fires once the rocket is descending AND has fallen below
            # `backfire_alt`, which corresponds to the requested time-after-burnout
            # on the ascent curve.
            backfire_trigger = self._make_backfire_trigger(backfire_alt)

            rocket.add_parachute(
                "Main",
                cd_s=para_cd * para_area,
                trigger=backfire_trigger,
                sampling_rate=105,
                lag=para_lag
            )

            flight = Flight(
                rocket=rocket, environment=env,
                rail_length=rail, inclination=elev, heading=azi,
                terminate_on_apogee=False
            )

            t_vals  = flight.z[:, 0]
            x_vals  = flight.x[:, 1]
            y_vals  = flight.y[:, 1]
            z_vals  = flight.z[:, 1]
            vz_vals = flight.vz[:, 1]

            azi_rad   = math.radians(azi)
            downrange = x_vals * math.sin(azi_rad) + y_vals * math.cos(azi_rad)

            # ── Backfire index: first point on DESCENT at or below backfire_alt ────
            descending_mask = vz_vals < 0
            below_alt_mask  = z_vals <= backfire_alt
            bf_candidates   = np.where(descending_mask & below_alt_mask)[0]
            if len(bf_candidates) > 0:
                idx_bf    = int(bf_candidates[0])
                bf_z_val  = float(z_vals[idx_bf])
                bf_dr     = float(downrange[idx_bf])
            else:
                idx_bf   = int(np.argmax(z_vals))  # fallback: apogee
                bf_z_val = float(z_vals[idx_bf])
                bf_dr    = float(downrange[idx_bf])

            # ── Parachute fully-open index (backfire + lag) ───────────────────────
            bf_abs_time    = float(t_vals[idx_bf])
            para_open_time = bf_abs_time + para_lag
            if para_open_time <= t_vals[-1]:
                idx_para = int((np.abs(t_vals - para_open_time)).argmin())
            else:
                idx_para = -1

            # ── Landing coordinates ───────────────────────────────────────────────
            # Funnelled through the shared `_offset_to_latlon` helper so the
            # landing point and the dispersion ring drawn around it use the
            # *same* metres-per-degree conversion (feature #1 fix).
            self.land_lat, self.land_lon = self._offset_to_latlon(
                self.launch_lat, self.launch_lon,
                float(x_vals[-1]), float(y_vals[-1]),
            )

            # ── Landing-circle dispersion (uses Settings-window parameters) ──────
            # Two independent sources, combined in quadrature:
            #   • Wind sigma  = (wind_uncertainty fraction) × surface wind × fall_time
            #   • Thrust sigma ≈ (thrust_uncertainty fraction) × horizontal range
            # Then we scale by the one-tail z-score for the chosen confidence level.
            apogee_idx = int(np.argmax(z_vals))
            fall_time  = t_vals[-1] - t_vals[apogee_idx]
            horiz_dist = math.hypot(float(x_vals[-1]), float(y_vals[-1]))
            wind_sigma_m   = surf_spd * self.wind_uncertainty * max(fall_time, 0.0)
            thrust_sigma_m = self.thrust_uncertainty * horiz_dist
            combined_sigma = math.hypot(wind_sigma_m, thrust_sigma_m)
            z_score = self._prob_to_z(self.landing_prob)
            self.r90_radius = z_score * combined_sigma  # kept var name for compat

            self.apogee_label.config(text=f"Apogee: {z_vals[apogee_idx]:.1f} m")
            self.velocity_label.config(text=f"Impact Vel: {abs(vz_vals[-1]):.1f} m/s")

            # ── Baseline capture for Lock & Monitor ──────────────────────────────
            self._baseline_wind = {
                "surf_spd": surf_spd, "surf_dir": surf_dir,
                "up_spd":   up_spd,   "up_dir":   up_dir,
            }

            sim_data = {
                'x': x_vals, 'y': y_vals, 'z': z_vals,
                'downrange': downrange,
                'impact_dr': downrange[-1], 'r90': self.r90_radius,
                'wind_u_prof': wind_u_prof, 'wind_v_prof': wind_v_prof, 'azi': azi,
                'bf_z': bf_z_val, 'bf_dr': bf_dr, 'bf_time': bf_abs_time,
                'bf_x': float(x_vals[idx_bf]), 'bf_y': float(y_vals[idx_bf]),
                'para_time': para_open_time, 'idx_para': idx_para,
                'idx_bf': idx_bf,
                'impact_x': float(x_vals[-1]), 'impact_y': float(y_vals[-1]),
                'apogee_m': float(z_vals[apogee_idx]),
            }
            # Mark that we have a fresh result BEFORE drawing so map/3-D plot
            # know to render landing markers.
            self._has_sim_result = True
            self._last_sim_data  = sim_data
            self.update_plots(sim_data)
            self.fit_map_bounds()
            # Auto-arm Monitor Mode after every successful simulation
            # (feature #8). Idempotent: doesn't touch state if already locked.
            self._auto_enable_monitor_mode()

        except ZeroDivisionError:
            messagebox.showerror(
                "離陸失敗 または 姿勢計算破綻",
                "シミュレーションの計算が破綻しました。\n\n"
                "【主な原因】\n"
                "1. モーターの推力が弱すぎてレールから動き出せなかった\n"
                "2. 空力パラメータ（CG、フィン等）により機体が極めて不安定\n\n"
                f"・選択エンジン: {self.selected_motor_name}\n"
            )
        except Exception as e:
            messagebox.showerror("Sim Error", f"RocketPy実行エラー:\n{e}")

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
            # Clamp to [0, 180] so the slider stays inside its restricted range
            # (feature #3 — full 360° rotation removed alongside mouse drag).
            a = max(0.0, min(180.0, a))
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
        # Mid-span default within the [0, 180]° slider range (feature #3).
        self._set_azim(90.0, source="code")

    # ── Mouse rotation: fully disabled (feature #3) ─────────────────────────────
    # Previous versions of this class wired up `_on_canvas_press/_motion/_release`
    # and `_on_view_changed` via `canvas.mpl_connect(...)` so the user could
    # drag to rotate the 3-D view. Feature #3 disables free-drag entirely —
    # the ONLY way to change the azimuth is now the [0, 180]° slider, and
    # matplotlib's built-in 3-D rotate is turned off via
    # `ax.disable_mouse_rotation()` in both `create_profile_section` and
    # `update_plots`. The handler methods were removed with the wiring so
    # stale dead code can't be accidentally reconnected later.

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
                self.wind_uncertainty   = w
                self.thrust_uncertainty = th
                self.landing_prob       = p
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

        Implements feature #8: as soon as `run_simulation()` finishes drawing
        results, we engage Monitor Mode automatically so the user doesn't
        have to remember to lock the configuration. Idempotent — if the
        toggle is already on we just refresh the wind baseline so the
        comparison floor reflects the freshest sample.
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

    def _auto_unlock_if_locked(self, *_args):
        """Disengage Lock & Monitor on any manual parameter edit (feature #8).

        When Monitor Mode is engaged the displayed simulation reflects the
        parameters that were active at the time of the last
        `run_simulation()`. As soon as the user starts editing parameters
        again the lock is silently released so they can iterate freely —
        re-engaging the lock requires an explicit re-run.

        The `*_args` swallow lets this be bound to any event source:
          • `Entry.bind('<KeyRelease>', ...)`            → passes an event
          • `Spinbox(command=...)` / `Scale(command=...)` → passes a value
          • `StringVar.trace_add('write', ...)`           → passes (name, idx, mode)
        """
        try:
            if self.lock_monitor_var.get():
                self.lock_monitor_var.set(False)
                self._toggle_lock_monitor()
                self.monitor_status_label.config(
                    text="⭘ Unlocked (parameter edited)",
                    foreground="orange", background="")
        except (tk.TclError, AttributeError):
            pass

    def _on_surf_spd_change(self, *_args):
        """Tk `Scale` command shim that funnels into the auto-unlock path."""
        self._auto_unlock_if_locked()

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
            # Feature #7 — pop a modal warning before the auto-recalc kicks
            # off, instead of flashing the UI red. The banner still flips
            # to a high-contrast warning state so the lock status remains
            # obvious in the periphery while the dialog is open.
            try:
                self.monitor_status_label.config(
                    text="⚠ WIND OUT OF TOLERANCE — RE-SIMULATING",
                    foreground="white", background="red")
            except tk.TclError:
                pass
            messagebox.showwarning(
                "Wind Out of Tolerance",
                "Wind conditions have shifted outside the configured\n"
                "tolerance (±2.0 m/s or ±15°).\n\n"
                "The trajectory will now be re-simulated with the\n"
                "current wind values."
            )
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
            # Restore the normal monitor banner.
            try:
                self.monitor_status_label.config(
                    text="🔒 LOCKED — monitoring wind",
                    foreground="green", background="")
            except tk.TclError:
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
        """Blank matplotlib's rotating 3-D axis labels.

        Direction annotations (East →, North ↑, Altitude (Up)) were
        removed in feature #2 — the compass rose at the bottom-right of
        the figure already conveys orientation, so the inline labels were
        redundant and crowded the plot area.
        """
        self.ax.set_xlabel('')
        self.ax.set_ylabel('')
        self.ax.set_zlabel('')

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
    # Units are figure fractions (0..1). The top 16 % is reserved for the
    # stats readout (top-left) + legend (top-right) strip.
    _PLOT_RECT      = (0.02, 0.00, 0.96, 0.84)   # (left, bottom, width, height)
    _TOP_STRIP_FRAC = 0.84                       # axes top → strip bottom

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
        # Disable matplotlib's built-in 3-D rotate on every rebuild so the
        # only way to change the view is the [0, 180]° slider (feature #3).
        try:
            self.ax.disable_mouse_rotation()
        except AttributeError:
            pass

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
            # Legend lives in the FIGURE top-RIGHT corner so the stats
            # readout (added once a sim runs) sits parallel on the figure
            # top-LEFT — the two share the strip above the plot side-by-
            # side (feature #4).
            self.ax.legend(loc='upper right',
                           bbox_to_anchor=(0.99, 0.99),
                           bbox_transform=self.fig.transFigure,
                           ncol=2, fontsize=7, framealpha=0.85)
            # Axes position was pinned via `_PLOT_RECT` above, so no
            # `tight_layout` call here — see note on `_PLOT_RECT`.
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
        # Legend label is now STATIC per feature #5 — the per-run probability
        # is still surfaced inside the stats readout block so no information
        # is lost.
        theta = np.linspace(0, 2 * math.pi, 72)
        cx = impact_x + r90 * np.cos(theta)
        cy = impact_y + r90 * np.sin(theta)
        cz = np.zeros_like(theta)
        self.ax.plot(cx, cy, cz, color='red', lw=1.5, alpha=0.6,
                     label='Landing Area')

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

        # ── Stats readout (features #2 / #3 / #4 / #6) ──────────────────────────
        # Stats sit at the TOP-LEFT of the figure, parallel to the legend at
        # the TOP-RIGHT (feature #4). Each line is rendered as its OWN
        # fig.text artist so individual lines can carry different colours —
        # specifically the Downrange line, which flips RED when the landing
        # point falls more than 50 m from the launch (feature #6).
        downrange_m = math.hypot(impact_x, impact_y)
        apogee_m    = data.get('apogee_m',
                               float(np.max(z_vals)) if len(z_vals) > 0 else 0.0)
        para_str    = f'{para_time:.2f} s' if para_time is not None else '— s'
        bf_str      = f'{bf_time:.2f} s' if bf_time is not None else '— s'

        DOWNRANGE_WARN_M = 50.0
        downrange_color = 'red' if downrange_m > DOWNRANGE_WARN_M else '#222222'
        stats_lines = [
            (f'Apogee:         {apogee_m:.1f} m',                       '#222222'),
            (f'Backfire:       {bf_str}',                               '#222222'),
            (f'Parachute Open: {para_str}',                             '#222222'),
            (f'Downrange:      {downrange_m:.1f} m',                    downrange_color),
            (f'Landing Radius: {self.r90_radius:.1f} m ({self.landing_prob}%)',
                                                                        '#222222'),
        ]
        # Render each line at successive y in figure coords so the block
        # lives ABOVE the axes (alongside the legend, not inside the plot).
        line_h, y_top = 0.022, 0.985
        for i, (line, color) in enumerate(stats_lines):
            self.fig.text(0.015, y_top - i * line_h, line,
                          ha='left', va='top',
                          fontsize=8, fontweight='bold', color=color,
                          family='monospace')

        # Keep aspect roughly equal on x/y; z can compress
        all_horiz = np.concatenate([x_vals, y_vals, [impact_x + r90, impact_x - r90,
                                                      impact_y + r90, impact_y - r90]])
        h_range = max(float(np.max(all_horiz) - np.min(all_horiz)), 1.0)
        mid_x   = float((np.max(x_vals) + np.min(x_vals)) / 2)
        mid_y   = float((np.max(y_vals) + np.min(y_vals)) / 2)
        self.ax.set_xlim(mid_x - h_range * 0.6, mid_x + h_range * 0.6)
        self.ax.set_ylim(mid_y - h_range * 0.6, mid_y + h_range * 0.6)
        self.ax.set_zlim(0, alt_max * 1.15)

        # Legend lives in the FIGURE top-RIGHT (feature #4) so it sits
        # parallel to the stats readout in the FIGURE top-LEFT.
        self.ax.legend(loc='upper right',
                       bbox_to_anchor=(0.99, 0.99),
                       bbox_transform=self.fig.transFigure,
                       ncol=2, fontsize=7, framealpha=0.85)
        # Axes position was pinned via `_PLOT_RECT` near the top of
        # `update_plots`, so no `tight_layout` is called here. That keeps
        # the plot from shrinking on successive redraws and anchors it
        # directly below the legend/stats strip instead of drifting toward
        # the bottom of the figure.
        self._draw_compass()
        self.canvas.draw()
        self.draw_map_elements()

    def draw_map_elements(self):
        self.map_widget.delete_all_polygon()
        # Always show launch point + 50 m reference ring around it.
        self.map_widget.set_polygon(self.get_circle_coords(self.launch_lat, self.launch_lon, 2.5), fill_color="blue")
        self.map_widget.set_polygon(self.get_circle_coords(self.launch_lat, self.launch_lon, 50), outline_color="blue")
        # Landing markers only after a valid simulation has produced them
        # (feature #4 — clears stale data when the launch point is relocated).
        if getattr(self, '_has_sim_result', False) and self.r90_radius > 0:
            self.map_widget.set_polygon(self.get_circle_coords(self.land_lat, self.land_lon, 2.5), fill_color="red")
            self.map_widget.set_polygon(self.get_circle_coords(self.land_lat, self.land_lon, self.r90_radius), outline_color="red", border_width=2)

    def get_circle_coords(self, lat, lon, radius_m):
        # Share the same lat/lon conversion as `_offset_to_latlon` so the
        # circle drawn around a centre point sits perfectly concentric
        # with it (fixes feature #1 misalignment).
        coords = []
        m_lat, m_lon = self._meters_per_degree(lat)
        for i in range(36):
            angle = math.pi * 2 * i / 36
            dx_east  = radius_m * math.cos(angle)
            dy_north = radius_m * math.sin(angle)
            coords.append((lat + dy_north / m_lat,
                           lon + dx_east  / m_lon))
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
            # Feature #8 — manual edit during Monitor Mode releases the lock.
            e.bind("<KeyRelease>", self._auto_unlock_if_locked, add="+")

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
        # Feature #8 — manual parachute edits release Monitor Mode too.
        for _e in (self.cd_entry, self.area_entry, self.lag_entry):
            _e.bind("<KeyRelease>", self._auto_unlock_if_locked, add="+")

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

        # Feature #8 — any launcher edit also releases Monitor Mode.
        # Spinboxes need both the typed-in path (`<KeyRelease>`) AND the
        # arrow-click path (the `command=` callback) covered.
        for _w in (self.lat_entry, self.lon_entry, self.rail_entry,
                   self.elev_spin, self.azi_spin):
            _w.bind("<KeyRelease>", self._auto_unlock_if_locked, add="+")
        self.elev_spin.config(command=self._auto_unlock_if_locked)
        self.azi_spin.config(command=self._auto_unlock_if_locked)

        loc_btn_f = ttk.Frame(frame); loc_btn_f.grid(row=14, column=0, sticky="ew", pady=(0, 2))
        loc_btn_f.columnconfigure(0, weight=1); loc_btn_f.columnconfigure(1, weight=1)
        ttk.Button(loc_btn_f, text="Get Location (IP)",
                   command=lambda: self.get_current_location(manual=True)).grid(
            row=0, column=0, sticky="ew", padx=(0, 1))
        ttk.Button(loc_btn_f, text="Update Map",
                   command=self.update_map_center).grid(
            row=0, column=1, sticky="ew", padx=(1, 0))

        ttk.Separator(frame, orient="horizontal").grid(row=15, column=0, sticky="ew", pady=3)

        # ── Wind ──────────────────────────────────────────────────────────────────
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

        # Feature #8 — wind edits also break the Monitor Mode lock.
        for _v in (self.up_spd_var, self.up_dir_var, self.surf_dir_var):
            _v.trace_add("write",
                         lambda *a: self._auto_unlock_if_locked())
        self.surf_spd_slider.config(command=self._on_surf_spd_change)

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
        ttk.Checkbutton(lock_f, text="🔒 Lock & Monitor",
                        variable=self.lock_monitor_var,
                        command=self._toggle_lock_monitor).grid(
            row=0, column=0, sticky="w")
        ttk.Button(lock_f, text="⚙ Settings",
                   command=self._open_settings_window).grid(
            row=0, column=1, sticky="e")

        self.monitor_status_label = tk.Label(
            lock_f, text="⭘ Unlocked",
            foreground="gray", font=("Arial", 8), anchor="w")
        self.monitor_status_label.grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(2, 0))

        # ── Run + results ─────────────────────────────────────────────────────────
        self.run_button = ttk.Button(frame, text="🚀  RUN SIMULATION",
                                     command=self.run_simulation)
        self.run_button.grid(row=19, column=0, sticky="ew",
                             ipady=4, pady=(2, 4))
        res_f = ttk.Frame(frame); res_f.grid(row=20, column=0, sticky="ew")
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

        # Slider range restricted to [0, 180]° per feature #3 — full
        # 360° free rotation was removed alongside mouse drag.
        self.azim_var = tk.DoubleVar(value=self._fixed_azim)
        self.azim_slider = ttk.Scale(
            rot_bar, from_=0, to=180, orient="horizontal",
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
        # Free-rotation via mouse drag is DISABLED (feature #3). Both our
        # custom horizontal drag handler AND matplotlib's built-in 3-D
        # rotate are turned off so the only path to changing the view is
        # the slider, which is itself constrained to [0, 180]°. The
        # compass is kept in sync by `_set_azim`, so the `draw_event`
        # listener is no longer required.
        try:
            self.ax.disable_mouse_rotation()
        except AttributeError:
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