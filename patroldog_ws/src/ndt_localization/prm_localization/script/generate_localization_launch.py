import re 
import argparse

parser = argparse.ArgumentParser(description="Parsing parameters for generate pcd map for ndt_localization ...")
parser.add_argument('--map_name', default='untitled', help='Name of your map')
args = parser.parse_args()


if __name__ == '__main__':
    new_map_name = args.map_name + '.pcd'
   
    demo_content = ""
    with open('../launch/demo.launch', 'r') as f:
        demo_content = f.read()

    regex = r'script/(.*?).pcd'
    new_launch_content = re.sub(regex, 'script/' + new_map_name, demo_content)
    
    with open('../launch/tencent_offline.launch', 'w') as f:
        f.write(new_launch_content)
 