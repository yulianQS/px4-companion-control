"""Synthetic ROS camera->tag pose->shared-memory test; no camera or flight UART."""
import json
from pathlib import Path
import subprocess
import tempfile
import time
import rclpy
from std_msgs.msg import String

def run():
    root=Path(__file__).resolve().parents[1]
    binaries=root/'ros2_ws/install/drone_vision/lib/drone_vision'
    build=root/'ros2_ws/build/drone_vision'
    rclpy.init(); node=rclpy.create_node('metric_pose_graph_test')
    publisher=node.create_publisher(String,'/_vision_test/phase',1)
    processes=[]
    with tempfile.TemporaryDirectory(prefix='metric-graph-') as temp:
        folder=Path(temp); log=(folder/'nodes.log').open('wb')
        try:
            subprocess.run([str(build/'vision_algorithms_test'),str(folder/'tag.bgr')],check=True)
            for binary,args in [('tag_detector_node',['-p','calibration_file:='+str(root/'config/camera_intrinsics.yaml'),'-p','tag_size_m:=0.12']),
                ('follow_preview_node',['-p','snapshot_directory:='+str(folder/'snapshots')])]:
                processes.append(subprocess.Popen([str(binaries/binary),'--ros-args']+args,stdout=log,stderr=log))
            processes.append(subprocess.Popen([str(build/'vision_test_image_source'),'--ros-args','-p','image_path:='+str(folder/'tag.bgr')],stdout=log,stderr=log))
            snapshot=folder/'snapshots/status.json';deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                publisher.publish(String(data='tag'));rclpy.spin_once(node,timeout_sec=.1)
                if snapshot.exists():
                    value=json.loads(snapshot.read_text())
                    if value.get('pose_valid'):
                        assert value['calibrated'] and value['target_found']
                        assert .45<value['distance_m']<.65,value
                        assert value['translation_m'][2]==value['distance_m']
                        assert value['tag_size_m']==.12
                        assert not value['actuation_enabled']
                        print('PASS: ROS calibrated square pose -> local snapshot, distance=%.4fm'%value['distance_m'])
                        return
                assert all(p.poll() is None for p in processes),(folder/'nodes.log').read_text(errors='replace')
            raise AssertionError((folder/'nodes.log').read_text(errors='replace'))
        finally:
            for p in processes:p.terminate()
            for p in processes:p.wait(timeout=5)
            log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':run()
