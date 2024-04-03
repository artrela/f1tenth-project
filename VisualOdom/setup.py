from setuptools import setup

setup(
   name='VisualOdom',
   version='1.0',
   description='A prototype visual odometry package',
   author='Alec Trela & Sridvei Kaza',
   author_email='atrela@andrew.cmu.edu, sridevik@andrew.cmu.edu',
   packages=['VisualOdom'],  #same as name
   install_requires=['opencv-python', 'numpy', 'matplotlib'], #external packages as dependencies
)