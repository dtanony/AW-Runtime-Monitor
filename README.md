## AW-Runtime-Monitor

This is a runtime monitor for [Autoware](https://github.com/dtanony/autoware0412), supporting:
- Recording the state (position, velocity, etc.) of the ego vehicle and other objects, 
together with camera videos and ADS internal states (e.g., perceived objects and control commands) during execution.

- Shielding for the control module. Specifically, the shield checks the safety of issued control commands. 
If a command is unsafe, the shield activates AEB. This safety shield acts as a final protective layer for the ADS.

### Usage
1. Install [AWSIM-Labs](https://github.com/dtanony/AWSIM-Labs) and [Autoware](https://github.com/dtanony/autoware0412) 
by following their provided instructions.
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