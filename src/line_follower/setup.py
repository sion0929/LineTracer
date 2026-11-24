import os
from glob import glob
from setuptools import find_packages, setup

pkg = 'line_follower'


def tree(src, dst):
    out = []
    for root, _, files in os.walk(src):
        if files:
            rel = os.path.relpath(root, src)
            d = os.path.join('share', pkg, dst, rel) if rel != '.' else os.path.join('share', pkg, dst)
            out.append((d, [os.path.join(root, f) for f in files]))
    return out


setup(
    name=pkg,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        ('share/' + pkg + '/launch', glob('launch/*.py')),
        ('share/' + pkg + '/config', glob('config/*.yaml')),
        ('share/' + pkg + '/worlds', glob('worlds/*.sdf')),
    ] + tree('models', 'models'),
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': ['follower_node = line_follower.follower_node:main']},
)
