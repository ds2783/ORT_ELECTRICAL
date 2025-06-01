Please add a virtual-env with imgui[full] installed.
Then append the "site-packages" directory of the venv to your $PYTHONPATH in .bashrc after setup.bash.
Using a similar command to: export PYTHONPATH="${PYTHONPATH}:/your-absolute-path/.your--venv/lib/python3.12/site-packages"

Compile COLCON with your virtual-env sourced, you will also need to install all 
required packages for the rest of the project, currently the list includes:
setuptools
pyymal
typeguard
empy

Failure to do such will render the GUI inert.
