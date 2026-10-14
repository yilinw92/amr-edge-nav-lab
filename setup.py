from setuptools import setup, find_packages
from glob import glob

setup(
    name="amr_edge_lab", version="0.2.0", packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/amr_edge_lab"]),
        ("share/amr_edge_lab", ["package.xml"]),
        ("share/amr_edge_lab/launch", glob("launch/*.py")),
        ("share/amr_edge_lab/config", glob("config/*.yaml") + glob("config/*.pgm")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Yilin Wang", maintainer_email="32149560+yilinw92@users.noreply.github.com",
    description="AMR navigation portfolio with synthetic sensors", license="MIT",
    entry_points={"console_scripts": [
        "sim_world = amr_edge_lab.sim_world:main",
        "depth_projector = amr_edge_lab.depth_node:main",
        "mission_manager = amr_edge_lab.route_node:main",
        "velocity_gate = amr_edge_lab.safety_node:main",
        "offline_replay = amr_edge_lab.offline:main",
    ]},
)
