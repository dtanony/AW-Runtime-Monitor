## AW-Runtime-Monitor

This is a runtime monitor for [Autoware](https://github.com/dtanony/autoware0412), supporting:
- Recording the state (position, velocity, etc.) of the ego vehicle and other objects, 
together with camera videos and ADS internal states (e.g., perceived objects and control commands) during execution.

- Shielding for the control module. Specifically, the shield checks the safety of issued control commands. 
If a command is unsafe, the shield activates AEB. This safety shield acts as a final protective layer for the ADS.

### Usage
1. Install [AWSIM-Labs](https://github.com/dtanony/AWSIM-Labs) and [Autoware](https://github.com/dtanony/autoware0412) 
by following their installation instructions.
2. Clone this repo and install dependencies
```bash
git clone https://github.com/dtanony/AW-Runtime-Monitor.git
cd AW-Runtime-Monitor

# It is recommended to use a virtual environment. For example, use venv
# python -m venv .venv
# source .venv/bin/activate

pip install -r requirements.txt
source ~/autoware/install/setup.bash
python main.py -o <path-to-save-traces>
```

Note that you need to source Autoware's setup file before launching the monitor.
Autoware is assumed to be installed in the home directory (`~`). 
If it is installed elsewhere, update the path accordingly.

By default, the shield is enable. To disable it, use option `-v false`.
For more details about the tool usage, use `python main.py -h`.

```bash
$ python main.py -h
usage: main.py [-h] [-o OUTPUT] [-f FORMAT] [-n NO_SIM] [-v VERIFY_CONTROL_CMD]

Runtime Monitor for Autoware and AWSIM simulator. Adjust the component to record data by modifying
file config.yaml

options:
  -h, --help            show this help message and exit
  -o OUTPUT, --output OUTPUT
                        Output YAML filename (default: auto-generated with timestamp)
  -f FORMAT, --format FORMAT
                        either json or yaml (default: json)
  -n NO_SIM, --no_sim NO_SIM
                        Simulation number, use as suffix to the file name (default: 1)
  -v VERIFY_CONTROL_CMD, --verify_control_cmd VERIFY_CONTROL_CMD
                        To verify the safety of control commands, i.e., enable shielding (true or
                        false, default: yes)
```

To configure which types of data will be recorded, modify [config.yaml](config.yaml) file.

By default, trace will be started recording when the autonomous driving mode is ready, and 
the recording will stop once the ego reaches its destination. 
Recorded text data will be saved to `<path-to-save-traces>.json` file, while
video will be saved to `<path-to-save-traces>_footage.mp4` file. 

### Using with AWSIM-Script
The monitor can be used with [AWSIM-Script](https://github.com/dtanony/AWSIM-Script-Client)
to execute a single scenario or a sequence of scenarios. 
When multiple scenarios are provided, the monitor saves a separate trace for each one. 
More details are available at https://github.com/dtanony/AWSIM-Script-Client.

