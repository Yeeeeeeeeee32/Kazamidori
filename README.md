# Kazamidori
The Kazamidori Project is a high-precision trajectory prediction and optimization simulator developed to achieve successful "pinpoint landings within a specified area" (e.g., within a 50-meter radius) for model rocket launches under strong wind conditions, such as those at the Tanegashima Space Center Rocket Contest.

Powered by the open-source 6-DOF (Six Degrees of Freedom) flight simulator RocketPy as its backend computational engine, it integrates a user-friendly GUI, map visualization features, and various external API connections. This powerful combination provides robust support for practical "wind reading" and "launcher (launch pad) angle optimization" directly at the launch site.

Features
🌍 Real-Time Mapping

Overlays the launch point, predicted landing point, and target area (e.g., a 50m radius circle) onto an interactive map.

Automatic acquisition of the current location (latitude and longitude) via IP address.

🌬️ Hybrid Wind Modeling

Integrates "upper-level weather model forecasted winds" obtained from the Windy.com API (or similar) with "ground-level measured winds" from local anemometers to reflect changes in wind speed and direction by altitude (wind shear) into the simulation in real-time.

🎯 Landing Point Uncertainty Evaluation (Dispersion Prediction)

Taking into account uncertainties such as wind gusts and parachute deployment delays, it statistically calculates the error radius within which the rocket will land with a specified probability (e.g., 90%) and draws it directly on the map.

📊 2D / 3D Trajectory Profiles

2D vertical profile of downrange (forward distance) and altitude, complete with wind direction and speed vector displays for each altitude layer.

3D flight trajectory visualization equipped with a dynamic compass that syncs with mouse viewpoint rotation.

⚙️ External Tool & Database Integration

Automatic importing of rocket specifications and parachute settings (Cd value, reference area, etc.) from OpenRocket (.ork) files.

Search for real-world motors (engines) using the ThrustCurve.org API, with automatic downloading of thrust data (.eng / .csv) and seamless integration into the calculations.

Tech Stack
Language: Python 3.x

Simulation & Physics: RocketPy, NumPy, SciPy

GUI & Visualization: Tkinter, Matplotlib

Mapping: tkintermapview

API Communication & Data Processing: requests, json, xml.etree.ElementTree, zipfile

Background
Under strong wind conditions along coastlines like Tanegashima, complex weather conditions arise where wind direction and speed differ significantly between the ground and upper altitudes due to surface friction and land-sea breezes. This project was developed as a "practical decision-making tool for the launch site" to achieve safe and accurate pinpoint landings under such harsh environments, designed specifically to be highly effective even with the limited resources of university clubs and similar groups.

Basic Usage
Run main.py (or launch the pre-built .exe file).

In the left data section, input the rocket's Dry Mass, CG Pos, Parachute Cd, Area, etc., or automatically load them from an .ork file.

Click the Search Engine button to call the ThrustCurve API, then download and apply the thrust curve of the motor you plan to use.

Set the on-site wind speed and direction, as well as the launch pad's Elevation and Azimuth angles.

Click 🚀 RUN SIMULATION and find the optimal launch angle that ensures the rocket lands within the target circle while monitoring the landing point, predicted altitude (Apogee), and descent velocity on the map.
