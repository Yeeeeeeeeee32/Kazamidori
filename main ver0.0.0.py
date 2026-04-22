# パラシュート設定をホーム画面で選択，設定，保存（パラシュートの形状でいくつかのデフォルトを設定）
# 開傘後の挙動
# 現在のモードを自由モードとする
# 発射点を中心として半径r_max (m)に着地する中で，発射点から着地点の距離をr, 滞空時間をtとして，r_max-r+tが最大となる定点滞空最適化モード
# 発射点を中心として指定した半径内に90%着地する中で、最高高度に到達するための高度最適化モード
# 発射点を中心として指定した半径内に90%着地する中で、滞空時間が最大になるための有翼最適化モード
# 3D軌道プロファイル
# Windy.com API, 風速計との連携
# オフラインマップモードを追加し，指定場所の半径500mを取得
# 自作風速計の外れ値計算を追加

# 入力完了したら緑，できてなかったら赤

# 実際に発射する手順として
# 最適化計算(高度or定点)
# 発射角，方位決定(風速風向許容差決定)
# 許容差を超えないかモニター
# 超えない場合はグリーン，超える場合はapogee，軌道再計算し警告を流す

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

            def get_uv(spd, dir_deg):
                rad = math.radians(dir_deg)
                return -spd * math.sin(rad), -spd * math.cos(rad)

            surf_u, surf_v = get_uv(surf_spd, surf_dir)
            up_u,   up_v   = get_uv(up_spd,   up_dir)

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
            # The trigger fires when:
            #   • the rocket is DESCENDING (vz < 0)  — ensures we're past apogee
            #   • altitude h <= backfire_alt          — matches the time we want
            # This correctly handles backfire_delay = 0 (fires at apogee) and any
            # positive delay (fires on the way down at the matching altitude).
            _triggered = [False]

            def backfire_trigger(p, h, y):
                # y[5] is vz in RocketPy's state vector
                if _triggered[0]:
                    return True
                descending = y[5] < 0
                below_alt  = h <= backfire_alt
                if descending and below_alt:
                    _triggered[0] = True
                    return True
                return False

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
            earth_r = 6378137.0
            self.land_lat = self.launch_lat + (y_vals[-1] / earth_r) * (180 / math.pi)
            self.land_lon = self.launch_lon + (
                x_vals[-1] / (earth_r * math.cos(math.pi * self.launch_lat / 180))
            ) * (180 / math.pi)

            # ── r90 dispersion ────────────────────────────────────────────────────
            apogee_idx = int(np.argmax(z_vals))
            fall_time  = t_vals[-1] - t_vals[apogee_idx]
            wind_sigma = surf_spd * 0.2
            self.r90_radius = 1.645 * wind_sigma * fall_time

            self.apogee_label.config(text=f"Apogee: {z_vals[apogee_idx]:.1f} m")
            self.velocity_label.config(text=f"Impact Vel: {abs(vz_vals[-1]):.1f} m/s")

            sim_data = {
                'downrange': downrange, 'z': z_vals,
                'impact_dr': downrange[-1], 'r90': self.r90_radius,
                'wind_u_prof': wind_u_prof, 'wind_v_prof': wind_v_prof, 'azi': azi,
                'bf_z': bf_z_val, 'bf_dr': bf_dr, 'bf_time': bf_abs_time,
                'para_time': para_open_time, 'idx_para': idx_para,
                'idx_bf': idx_bf
            }
            self.update_plots(sim_data)
            self.fit_map_bounds()

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

    def update_plots(self, data=None):
        self.ax.clear()
        if not data:
            self.ax.set_title("Vertical Profile (Downrange)")
            self.ax.set_xlabel("Downrange (m)")
            self.ax.set_ylabel("Altitude (m)")
            self.canvas.draw()
            self.draw_map_elements()
            return

        dr, z, impact_dr, r90 = data['downrange'], data['z'], data['impact_dr'], data['r90']
        wind_u_prof, wind_v_prof, azi = data['wind_u_prof'], data['wind_v_prof'], data['azi']

        bf_z = data.get('bf_z')
        bf_dr = data.get('bf_dr')
        bf_time = data.get('bf_time')
        para_time = data.get('para_time')
        idx_para = data.get('idx_para', -1)
        idx_bf   = data.get('idx_bf', -1)

        alt_max = max(z) if len(z) > 0 else 100

        # ── Apogee marker ──
        if len(z) > 0:
            apogee_idx = int(np.argmax(z))
            apogee_z = z[apogee_idx]
            apogee_dr = dr[apogee_idx]
            self.ax.plot([0, apogee_dr], [apogee_z, apogee_z], color="gray", linestyle="--", linewidth=1.5)
            self.ax.plot(apogee_dr, apogee_z, marker="*", color="orange", markersize=8)
            self.ax.text(0, apogee_z, f' Apogee: {apogee_z:.1f} m', color="dimgray",
                         va="bottom", ha="left", fontsize=9, fontweight="bold")

        # ── Trajectory: 3 phases ──────────────────────────────────────────────────
        # Phase 1 (blue solid):      launch → backfire
        # Phase 2 (orange solid):    backfire → chute fully open  (freefall/tumbling)
        # Phase 3 (deepskyblue dash): chute fully open → landing
        has_bf    = idx_bf   != -1 and idx_bf   < len(dr)
        has_para  = idx_para != -1 and idx_para < len(dr)

        if has_bf and has_para:
            # Phase 1
            self.ax.plot(dr[:idx_bf + 1], z[:idx_bf + 1],
                         color="blue", linewidth=1.8, label="Powered / Coast")
            # Phase 2
            self.ax.plot(dr[idx_bf:idx_para + 1], z[idx_bf:idx_para + 1],
                         color="darkorange", linewidth=1.8, label="Freefall (post-backfire)")
            # Phase 3
            self.ax.plot(dr[idx_para:], z[idx_para:],
                         color="deepskyblue", linestyle="--", linewidth=1.8, label="Under Canopy")
            # Chute-open marker
            self.ax.plot(dr[idx_para], z[idx_para], marker="v", color="green", markersize=8)
            self.ax.text(dr[idx_para], z[idx_para] + (alt_max * 0.02),
                         f' Fully Open ({para_time:.1f}s)', color="green",
                         va="bottom", ha="right", fontsize=9, fontweight="bold")
        elif has_bf and not has_para:
            # Chute never opened
            self.ax.plot(dr[:idx_bf + 1], z[:idx_bf + 1],
                         color="blue", linewidth=1.8, label="Powered / Coast")
            self.ax.plot(dr[idx_bf:], z[idx_bf:],
                         color="darkorange", linewidth=1.8, label="Freefall (no chute)")
            self.ax.text(impact_dr, 0, ' Crash (Chute not open)', color="red",
                         va="bottom", ha="right", fontsize=9, fontweight="bold")
        else:
            # Fallback: no backfire index available
            if has_para:
                self.ax.plot(dr[:idx_para + 1], z[:idx_para + 1],
                             color="blue", linewidth=1.8, label="Freefall")
                self.ax.plot(dr[idx_para:], z[idx_para:],
                             color="deepskyblue", linestyle="--", linewidth=1.8, label="Under Canopy")
            else:
                self.ax.plot(dr, z, color="blue", linewidth=1.8, label="Trajectory")
                self.ax.text(impact_dr, 0, ' Crash (Chute not open)', color="red",
                             va="bottom", ha="right", fontsize=9, fontweight="bold")

        # ── Ground line, impact, dispersion ──
        self.ax.axhline(0, color="black")
        self.ax.plot(impact_dr, 0, 'ro', markersize=7, label="Impact", zorder=5)
        self.ax.plot([impact_dr - r90, impact_dr + r90], [0, 0],
                     color="red", linewidth=5, alpha=0.5, label="90% Dispersion")

        # ── Backfire marker ──
        if bf_z is not None and bf_dr is not None:
            self.ax.plot(bf_dr, bf_z, marker="X", color="magenta", markersize=9, zorder=6)
            self.ax.text(bf_dr, bf_z - (alt_max * 0.03),
                         f' Backfire ({bf_time:.1f}s)', color="magenta",
                         va="top", ha="left", fontsize=9, fontweight="bold")

        # ── Wind arrows (anchored at x=0 so they don't overlap trajectory) ──────
        # Arrows show wind at each altitude level, starting from x=0 (launch point).
        z_keys = [p[0] for p in wind_u_prof]
        u_vals_w = [p[1] for p in wind_u_prof]
        v_vals_w = [p[1] for p in wind_v_prof]
        azi_rad  = math.radians(azi)
        arrow_anchor = min(0.0, float(np.min(dr))) - abs(impact_dr) * 0.05
        for alt in np.linspace(0, alt_max, 6):
            u_alt = np.interp(alt, z_keys, u_vals_w)
            v_alt = np.interp(alt, z_keys, v_vals_w)
            eff_wind = u_alt * math.sin(azi_rad) + v_alt * math.cos(azi_rad)
            arrow_scale = max(3.0, abs(impact_dr) * 0.15)
            x_start = arrow_anchor
            x_end   = arrow_anchor + eff_wind * arrow_scale
            self.ax.annotate('', xy=(x_end, alt), xytext=(x_start, alt),
                             arrowprops=dict(arrowstyle="->", color="green", lw=1.5))
            spd_abs = math.sqrt(u_alt ** 2 + v_alt ** 2)
            self.ax.text(x_end, alt + alt_max * 0.015,
                         f"{spd_abs:.1f} m/s", color="green",
                         va='bottom', ha='center', fontsize=8)

        self.ax.set_title("Vertical Profile (Downrange)")
        self.ax.set_xlabel("Downrange (m)")
        self.ax.set_ylabel("Altitude (m)")
        self.ax.legend(loc="upper right")
        self.canvas.draw()
        self.draw_map_elements()

    def draw_map_elements(self):
        self.map_widget.delete_all_polygon()
        self.map_widget.set_polygon(self.get_circle_coords(self.launch_lat, self.launch_lon, 2.5), fill_color="blue")
        self.map_widget.set_polygon(self.get_circle_coords(self.launch_lat, self.launch_lon, 50), outline_color="blue")
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
            lat_degree_m = 111320.0
            lon_degree_m = 111320.0 * math.cos(math.radians(self.launch_lat))
            pad_lat = margin_m / lat_degree_m
            pad_lon = margin_m / lon_degree_m

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
        container = ttk.Frame(self)
        container.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        canvas = tk.Canvas(container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        frame = ttk.Frame(canvas)

        frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        frame_id = canvas.create_window((0, 0), window=frame, anchor="nw")
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(frame_id, width=e.width))

        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # モーター設定
        self.motor_ui_label = ttk.Label(frame, text=f"Engine: {self.selected_motor_name}", font=("Arial", 10, "bold"), foreground="#B22222")
        self.motor_ui_label.pack(anchor="w", pady=(0, 2))
        motor_btn_frame = ttk.Frame(frame)
        motor_btn_frame.pack(fill="x", pady=2)
        ttk.Button(motor_btn_frame, text="[Open Web (ThrustCurve)]", command=self.open_thrustcurve_web).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(motor_btn_frame, text="[Load Local CSV]", command=self.load_local_motor).pack(side="right", expand=True, fill="x", padx=(2, 0))

        ttk.Separator(frame, orient="horizontal").pack(fill="x", pady=10)

        # 機体設定
        self.af_name_label = ttk.Label(frame, text="Airframe: (未選択)", font=("Arial", 9, "bold"))
        self.af_name_label.pack(anchor="w")

        af_lf = ttk.LabelFrame(frame, text="Airframe General Settings")
        af_lf.pack(fill="x", pady=(2, 5))

        f_mass = ttk.Frame(af_lf); f_mass.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_mass, text="Dry Mass (kg):").pack(side="left")
        self.mass_entry = ttk.Entry(f_mass, width=8); self.mass_entry.pack(side="right")

        f_cg = ttk.Frame(af_lf); f_cg.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_cg, text="CG Pos from Nose (m):").pack(side="left")
        self.cg_entry = ttk.Entry(f_cg, width=8); self.cg_entry.pack(side="right")

        f_len = ttk.Frame(af_lf); f_len.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_len, text="Total Length (m):").pack(side="left")
        self.len_entry = ttk.Entry(f_len, width=8); self.len_entry.pack(side="right")

        f_rad = ttk.Frame(af_lf); f_rad.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_rad, text="Radius (m):").pack(side="left")
        self.radius_entry = ttk.Entry(f_rad, width=8); self.radius_entry.pack(side="right")

        aero_lf = ttk.LabelFrame(frame, text="Aero & Motor Placement")
        aero_lf.pack(fill="x", pady=(2, 5))

        f_nlen = ttk.Frame(aero_lf); f_nlen.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_nlen, text="Nose Length (m):").pack(side="left")
        self.nose_len_entry = ttk.Entry(f_nlen, width=8); self.nose_len_entry.pack(side="right")

        f_froot = ttk.Frame(aero_lf); f_froot.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_froot, text="Fin Root Chord (m):").pack(side="left")
        self.fin_root_entry = ttk.Entry(f_froot, width=8); self.fin_root_entry.pack(side="right")

        f_ftip = ttk.Frame(aero_lf); f_ftip.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_ftip, text="Fin Tip Chord (m):").pack(side="left")
        self.fin_tip_entry = ttk.Entry(f_ftip, width=8); self.fin_tip_entry.pack(side="right")

        f_fspan = ttk.Frame(aero_lf); f_fspan.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_fspan, text="Fin Span (m):").pack(side="left")
        self.fin_span_entry = ttk.Entry(f_fspan, width=8); self.fin_span_entry.pack(side="right")

        f_fpos = ttk.Frame(aero_lf); f_fpos.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_fpos, text="Fin Pos from Nose (m):").pack(side="left")
        self.fin_pos_entry = ttk.Entry(f_fpos, width=8); self.fin_pos_entry.pack(side="right")

        f_mpos = ttk.Frame(aero_lf); f_mpos.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_mpos, text="Motor Pos from Nose (m):").pack(side="left")
        self.motor_pos_entry = ttk.Entry(f_mpos, width=8); self.motor_pos_entry.pack(side="right")

        f_mdm = ttk.Frame(aero_lf); f_mdm.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_mdm, text="Motor Dry Mass (kg):").pack(side="left")
        self.motor_dry_mass_entry = ttk.Entry(f_mdm, width=8); self.motor_dry_mass_entry.pack(side="right")

        f_bf = ttk.Frame(aero_lf); f_bf.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_bf, text="Backfire Delay (s):").pack(side="left")
        self.backfire_delay_entry = ttk.Entry(f_bf, width=8); self.backfire_delay_entry.pack(side="right")

        f_abtn = ttk.Frame(aero_lf); f_abtn.pack(fill="x", pady=5, padx=5)
        ttk.Button(f_abtn, text="Load JSON", command=self.load_af_settings).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(f_abtn, text="Save JSON", command=self.save_af_settings).pack(side="right", expand=True, fill="x", padx=(2, 0))

        entries = [self.mass_entry, self.cg_entry, self.len_entry, self.radius_entry,
                   self.nose_len_entry, self.fin_root_entry, self.fin_tip_entry,
                   self.fin_span_entry, self.fin_pos_entry, self.motor_pos_entry,
                   self.motor_dry_mass_entry, self.backfire_delay_entry]
        for entry in entries:
            entry.bind("<KeyRelease>", self.on_parameter_edit_af)

        # パラシュート設定
        self.para_name_label = ttk.Label(frame, text="Parachute: (未選択)", font=("Arial", 9, "bold"))
        self.para_name_label.pack(anchor="w", pady=(5, 0))

        para_lf = ttk.LabelFrame(frame, text="Parachute Settings")
        para_lf.pack(fill="x", pady=(2, 10))

        f_cd = ttk.Frame(para_lf); f_cd.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_cd, text="Cd:").pack(side="left")
        self.cd_entry = ttk.Entry(f_cd, width=8); self.cd_entry.pack(side="right")

        f_area = ttk.Frame(para_lf); f_area.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_area, text="Area (m²):").pack(side="left")
        self.area_entry = ttk.Entry(f_area, width=8); self.area_entry.pack(side="right")

        f_lag = ttk.Frame(para_lf); f_lag.pack(fill="x", pady=2, padx=5)
        ttk.Label(f_lag, text="Lag (s):").pack(side="left")
        self.lag_entry = ttk.Entry(f_lag, width=8); self.lag_entry.pack(side="right")

        f_pbtn = ttk.Frame(para_lf); f_pbtn.pack(fill="x", pady=5, padx=5)
        ttk.Button(f_pbtn, text="Load JSON", command=self.load_para_settings).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(f_pbtn, text="Save JSON", command=self.save_para_settings).pack(side="right", expand=True, fill="x", padx=(2, 0))

        self.cd_entry.bind("<KeyRelease>", self.on_parameter_edit_para)
        self.area_entry.bind("<KeyRelease>", self.on_parameter_edit_para)
        self.lag_entry.bind("<KeyRelease>", self.on_parameter_edit_para)

        ttk.Label(frame, text="Launcher Setting", font=("Arial", 10, "bold")).pack(anchor="w", pady=(0, 5))
        f_lat = ttk.Frame(frame); f_lat.pack(fill="x")
        ttk.Label(f_lat, text="Lat:").pack(side="left")
        self.lat_entry = ttk.Entry(f_lat, width=12); self.lat_entry.insert(0, str(self.launch_lat)); self.lat_entry.pack(side="right")
        f_lon = ttk.Frame(frame); f_lon.pack(fill="x")
        ttk.Label(f_lon, text="Lon:").pack(side="left")
        self.lon_entry = ttk.Entry(f_lon, width=12); self.lon_entry.insert(0, str(self.launch_lon)); self.lon_entry.pack(side="right")

        ttk.Button(frame, text="[Get Current Location (IP)]", command=lambda: self.get_current_location(manual=True)).pack(fill="x", pady=(5, 2))
        ttk.Button(frame, text="[Manual Update Map]", command=self.update_map_center).pack(fill="x", pady=(0, 5))

        f_rail = ttk.Frame(frame); f_rail.pack(fill="x", pady=2)
        ttk.Label(f_rail, text="Rail Length (m):").pack(side="left")
        self.rail_entry = ttk.Entry(f_rail, width=6); self.rail_entry.insert(0, "1.0"); self.rail_entry.pack(side="right")
        f_e = ttk.Frame(frame); f_e.pack(fill="x")
        ttk.Label(f_e, text="Elevation:").pack(side="left")
        self.elev_spin = ttk.Spinbox(f_e, from_=0, to=90, width=6); self.elev_spin.set("85"); self.elev_spin.pack(side="right")
        f_a = ttk.Frame(frame); f_a.pack(fill="x")
        ttk.Label(f_a, text="Azimuth:").pack(side="left")
        self.azi_spin = ttk.Spinbox(f_a, from_=0, to=360, width=6); self.azi_spin.set("0"); self.azi_spin.pack(side="right")

        wind_lf = ttk.LabelFrame(frame, text="Wind Profile (Dir: 0=N, 90=E, 180=S)")
        wind_lf.pack(fill="x", pady=10)
        up_f = ttk.Frame(wind_lf); up_f.pack(fill="x", pady=2, padx=5)
        ttk.Label(up_f, text="上空100m (Windy API):", font=("Arial", 9, "bold")).pack(anchor="w")
        up_in = ttk.Frame(up_f); up_in.pack(fill="x")
        self.up_spd_var = tk.StringVar(value="8.0")
        self.up_dir_var = tk.StringVar(value="90")
        ttk.Entry(up_in, textvariable=self.up_spd_var, width=5).pack(side="left")
        ttk.Label(up_in, text="m/s,  ").pack(side="left")
        ttk.Entry(up_in, textvariable=self.up_dir_var, width=5).pack(side="left")
        ttk.Label(up_in, text="deg").pack(side="left")

        surf_f = ttk.Frame(wind_lf); surf_f.pack(fill="x", pady=2, padx=5)
        ttk.Label(surf_f, text="地表3m (自作風速計):", font=("Arial", 9, "bold")).pack(anchor="w")
        surf_in = ttk.Frame(surf_f); surf_in.pack(fill="x")
        self.surf_spd_slider = ttk.Scale(surf_in, from_=0, to=15, orient="horizontal")
        self.surf_spd_slider.set(4.0)
        self.surf_spd_slider.pack(side="left", fill="x", expand=True, padx=(0, 5))
        self.surf_dir_var = tk.StringVar(value="100")
        ttk.Entry(surf_in, textvariable=self.surf_dir_var, width=5).pack(side="right")
        ttk.Label(surf_in, text="deg").pack(side="right")

        stats_f = ttk.Frame(wind_lf); stats_f.pack(fill="x", pady=5, padx=5)
        self.wind_avg_label = ttk.Label(stats_f, text="地表平均: -- m/s", foreground="green"); self.wind_avg_label.pack(side="left")
        self.wind_gust_label = ttk.Label(stats_f, text="最大瞬間: -- m/s", foreground="red"); self.wind_gust_label.pack(side="right")

        ttk.Button(frame, text="🚀 RUN SIMULATION", command=self.run_simulation).pack(fill="x", pady=10, ipady=5)
        self.apogee_label = ttk.Label(frame, text="Apogee: -- m", font=("Arial", 11, "bold")); self.apogee_label.pack()
        self.velocity_label = ttk.Label(frame, text="Impact Vel: -- m/s", font=("Arial", 11, "bold")); self.velocity_label.pack()

    def create_profile_section(self):
        frame = ttk.Frame(self, padding=10, relief="solid", borderwidth=1)
        frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        self.fig, self.ax = plt.subplots(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

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
            self.update_plots()
            self.fit_map_bounds()
        except ValueError:
            pass


if __name__ == "__main__":
    app = KazamidoriUI()
    app.mainloop()