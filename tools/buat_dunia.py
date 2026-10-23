#!/usr/bin/env python3
"""Pembuat dunia simulasi ASV: kolam, kapal dua thruster, gerbang pelampung."""
import math
from pathlib import Path

# ---- Parameter lintasan (silakan diubah) ----
JUMLAH_GERBANG = 10
JARAK_GERBANG = 3.0    # jarak antar gerbang sepanjang lintasan (m)
LEBAR_GERBANG = 2.0    # jarak bola merah-hijau (m), sesuai pedoman KKI 2026
JARAK_AWAL = 4.0       # gerbang pertama dari titik start (m)
BELOKAN = 1.0          # amplitudo kelokan lintasan (m); 0 = lurus
RADIUS_BOLA = 0.12


def thruster(nama, y):
    return f"""
      <link name="prop_{nama}">
        <pose>-0.5 {y} -0.06 0 0 0</pose>
        <inertial>
          <mass>0.2</mass>
          <inertia><ixx>0.0005</ixx><iyy>0.0005</iyy><izz>0.0005</izz></inertia>
        </inertial>
        <visual name="visual">
          <pose>0 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.05</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
      <joint name="prop_{nama}_joint" type="revolute">
        <parent>lambung</parent>
        <child>prop_{nama}</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit><lower>-1e12</lower><upper>1e12</upper></limit>
        </axis>
      </joint>
      <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
        <joint_name>prop_{nama}_joint</joint_name>
        <thrust_coefficient>0.004422</thrust_coefficient>
        <fluid_density>1000</fluid_density>
        <propeller_diameter>0.1</propeller_diameter>
        <max_thrust_cmd>20</max_thrust_cmd>
        <min_thrust_cmd>-20</min_thrust_cmd>
      </plugin>"""


def apung_lambung():
    """Gaya apung dihitung dari 6 bola (3 kolom x 2 baris) di bawah lambung."""
    hasil = []
    for ix, x in enumerate((-0.3, 0.0, 0.3)):
        for iy, y in enumerate((-0.17, 0.17)):
            hasil.append(f"""
        <collision name="apung_{ix}_{iy}">
          <pose>{x} {y} -0.032 0 0 0</pose>
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
        </collision>""")
    return "".join(hasil)


def kapal():
    return f"""
    <model name="kapal_anif">
      <pose>0 0 0.057 0 0 0</pose>
      <link name="lambung">
        <inertial>
          <pose>0 0 -0.03 0 0 0</pose>
          <mass>7.6</mass>
          <inertia><ixx>0.18</ixx><iyy>0.55</iyy><izz>0.71</izz></inertia>
        </inertial>
        <visual name="badan">
          <geometry><box><size>0.9 0.5 0.15</size></box></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="penanda_haluan">
          <pose>0.35 0 0.08 0 0 0</pose>
          <geometry><box><size>0.2 0.5 0.01</size></box></geometry>
          <material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material>
        </visual>
        <sensor name="kamera_depan" type="camera">
          <pose>0.4 0 0.15 0 0.1 0</pose>
          <always_on>true</always_on>
          <update_rate>15</update_rate>
          <topic>kamera_depan</topic>
          <camera>
            <horizontal_fov>1.396</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>50</far></clip>
          </camera>
        </sensor>
        {apung_lambung()}
      </link>
      {thruster("kiri", 0.2)}
      {thruster("kanan", -0.2)}
      <plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
        <link_name>lambung</link_name>
        <xU>-4</xU><xAbsU>-6</xAbsU>
        <yV>-25</yV><yAbsV>-10</yAbsV>
        <zW>-300</zW>
        <kP>-6</kP><mQ>-15</mQ>
        <nR>-5</nR><nAbsR>0</nAbsR>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_frame>odom</odom_frame>
        <robot_base_frame>kapal_anif</robot_base_frame>
        <odom_publish_frequency>20</odom_publish_frequency>
        <dimensions>3</dimensions>
      </plugin>
    </model>"""


def bola(nama, x, y, rgb):
    r, g, b = rgb
    return f"""
    <model name="{nama}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><sphere><radius>{RADIUS_BOLA}</radius></sphere></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def pelampung():
    hasil = []
    for i in range(JUMLAH_GERBANG):
        x = JARAK_AWAL + i * JARAK_GERBANG
        tengah = BELOKAN * math.sin(i * 0.7)
        # Kapal menghadap +x, jadi +y adalah sisi kiri. Sesuaikan dengan Petunjuk Teknis.
        hasil.append(bola(f"merah_{i+1}", x, tengah + LEBAR_GERBANG / 2, (0.9, 0.1, 0.1)))
        hasil.append(bola(f"hijau_{i+1}", x, tengah - LEBAR_GERBANG / 2, (0.1, 0.7, 0.1)))
    return "".join(hasil)


def dunia():
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="kolam_asv">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>1000</default_density>
        <density_change>
          <above_depth>0</above_depth>
          <density>1</density>
        </density_change>
      </graded_buoyancy>
    </plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.6 0.8 0.95 1</background>
    </scene>
    <light type="directional" name="matahari">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>

    <model name="air">
      <static>true</static>
      <pose>15 0 -0.01 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>80 40 0.02</size></box></geometry>
          <material><ambient>0.1 0.35 0.6 1</ambient><diffuse>0.1 0.35 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
    {kapal()}
    {pelampung()}
  </world>
</sdf>
"""


if __name__ == "__main__":
    keluaran = Path(__file__).resolve().parent.parent / "worlds" / "kolam_asv.sdf"
    keluaran.parent.mkdir(parents=True, exist_ok=True)
    keluaran.write_text(dunia())
    print("Dunia ditulis ke", keluaran)
