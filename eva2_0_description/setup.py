
from setuptools import setup

setup(
    name='eva2_0_description',
    version='0.0.1',
    packages=['scripts'],
    data_files=[
        ('share/eva2_0_description/urdf', ['urdf/eva2_0.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jai-bhugra',
    maintainer_email='jai@example.com',
    description='EVA 2.0 URDF and FK nodes',
    license='TODO',
    entry_points={
        'console_scripts': [
            'fk_l_node=scripts.fk_l_node:main',
            'fk_r_node=scripts.fk_r_node:main',
            'fk_h_node=scripts.fk_h_node:main',
            'servo_test=scripts.servo_test:main',
        ],
    },
)
