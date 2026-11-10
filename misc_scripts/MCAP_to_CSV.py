import csv
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

def convert_mcap_to_csv(mcap_filepath, csv_filepath):
    # Dictionary to group messages by their exact timestamp (t_ns)
    data_by_time = {}

    print(f"Reading {mcap_filepath}...")
    with open(mcap_filepath, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, proto_msg in reader.iter_decoded_messages():
            t_ns = message.log_time
            if t_ns not in data_by_time:
                data_by_time[t_ns] = {}
            
            # Extract Cartesian Pose
            if channel.topic == "/fk_pose":
                data_by_time[t_ns]['pose_x'] = proto_msg.pose.position.x
                data_by_time[t_ns]['pose_y'] = proto_msg.pose.position.y
                data_by_time[t_ns]['pose_z'] = proto_msg.pose.position.z
            
            # Extract Joint Positions and Velocities
            elif channel.topic == "/joint_states":
                for i, joint in enumerate(proto_msg.joints):
                    data_by_time[t_ns][f'j{i+1}_pos'] = joint.position
                    data_by_time[t_ns][f'j{i+1}_vel'] = joint.velocity

    print(f"Writing to {csv_filepath}...")
    with open(csv_filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Create CSV Header
        header = ['time_s', 'pose_x', 'pose_y', 'pose_z']
        for i in range(1, 7):
            header.extend([f'j{i}_pos', f'j{i}_vel'])
        writer.writerow(header)

        # Sort chronologically and write rows
        start_time = None
        for t_ns in sorted(data_by_time.keys()):
            if start_time is None:
                start_time = t_ns
            
            t_s = (t_ns - start_time) / 1e9
            row_data = data_by_time[t_ns]
            
            row = [
                t_s,
                row_data.get('pose_x', 0.0),
                row_data.get('pose_y', 0.0),
                row_data.get('pose_z', 0.0),
            ]
            for i in range(1, 7):
                row.append(row_data.get(f'j{i}_pos', 0.0))
                row.append(row_data.get(f'j{i}_vel', 0.0))
            
            writer.writerow(row)
            
    print("Conversion complete!")

if __name__ == "__main__":
    convert_mcap_to_csv("test9.mcap", "test9_data.csv")