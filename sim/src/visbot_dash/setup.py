from setuptools import setup

package_name = "visbot_dash"
setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/web", ["web/index.html"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Amil Agrawal",
    maintainer_email="amil@amilcodes.dev",
    description="Mission-control dashboard for the SysVRC sim testbed",
    license="MIT",
    entry_points={"console_scripts": ["dash_node = visbot_dash.dash_node:main"]},
)
