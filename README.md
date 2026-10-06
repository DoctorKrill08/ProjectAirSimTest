#Download an Environment for the server
-> I am going to use blocks:
https://github.com/iamaisim/ProjectAirSim/releases
-> On version 1.01 there is Blocks-Windows-1.0.1.zip 
-> Download that
-> Extract it somewhere
-> Open it and you should see an Unreal Engine app called Blocks
-> Open it. If windows pops up, clicck more info then run anyway
-> Setup Unreal Engine, and then keep it open.

#SETUP VENV (Windows)
##UPDATE PIP
python -m pip install --upgrade pip
python -m pip install setuptools wheel

##Install venv
python -m pip install virtualenv
python -m venv airsim-venv
airsim-venv\Scripts\activate



#REQUIREMENTS:
-> You can run the following command:
pip install -r requirements.txt

##What the actual requirements are:
pip install projectairsim==1.0.2
pip install pandas
