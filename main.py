from recorder.Recorder import *

script_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(script_dir, 'config.yaml')

def parse_args():
    parser = argparse.ArgumentParser(description='Runtime Monitor for Autoware and AWSIM simulator. '
                                                 'Adjust the component to record data by modifying file config.yaml')
    parser.add_argument('-o', '--output',
                      help='Output trace file name (default: auto-generated with timestamp)')
    parser.add_argument('-f', '--format',
                      help='either json or yaml (default: json)')
    parser.add_argument('-n', '--no_sim',
                      help='Simulation number, use as suffix to the file name (default: 1)')
    parser.add_argument('-v', '--verify_control_cmd',
                        help='To verify the safety of control commands, i.e., enable shielding (true or false, default: yes)')
    return parser.parse_args()

def load_config(yaml_file):
    with open(yaml_file, 'r') as f:
        config = yaml.safe_load(f)
    topics_recording = []
    for entry in config['components_recording']:
        component_name = entry['name']
        cls = globals()[component_name + 'Topic']
        topic_obj = cls()  # Create an instance
        topics_recording.append(topic_obj)
    return topics_recording

def main():
    # Parse command line arguments
    cli_args = parse_args()
    to_verify_control_cmd = True
    if str(cli_args.verify_planning_trajectory).lower() == "false":
        to_verify_control_cmd = False
    no_sim = int(cli_args.no_sim) if cli_args.no_sim else 1
    format = 'json'
    if cli_args.format == 'yaml':
        format = 'yaml'
    elif cli_args.format is not None and cli_args.format != 'json':
        print('Format must be either json or yaml. Json is used by default.')

    awsim_client_op_state_topic = AWSIMClientOpStateTrackerTopic()
    topics_to_record_data = load_config(config_path)

    topics = topics_to_record_data +\
    [
        awsim_client_op_state_topic
    ]
    rclpy.init()
    recorder = MultiTopicRecorder(topics,
                                  output_filename=cli_args.output,
                                  format=format,
                                  no_sim=no_sim,
                                  to_verify_control_cmd=to_verify_control_cmd)

    recorder.subscribe()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print('\nForce stopping.')
        recorder.dump_data()
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
