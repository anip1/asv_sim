import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'asv_sim'


def tree(src):
    """Salin seluruh isi folder (mis. models/) ke share/."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anif',
    maintainer_email='nazharanif2@gmail.com',
    description='Simulasi ASV untuk KKI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keselamatan = asv_sim.keselamatan:main',
            'dock_biru = asv_sim.dock_biru:main',
            'dashboard = asv_sim.dashboard_web:main',
            'mixer = asv_sim.mixer_thruster:main',
            'autopilot = asv_sim.autopilot_gerbang:main',
        ],
    },
)
