import hashlib,json,tempfile,unittest
from pathlib import Path
import cv2
import numpy as np
import rosbag,rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from dataset_bag_tools import poses,images


class ExactCompressedPairs(unittest.TestCase):
    def test_original_jpeg_bytes_and_nanosecond_pose_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'dataset/images').mkdir(parents=True)
            (p/'dataset/frames.json').write_text('[]');(p/'dataset/frames.jsonl').write_text('')
            (p/'rgb_pt.pcd').write_bytes(b'synthetic test only')
            stamp=rospy.Time(1,123456789)
            _,encoded=cv2.imencode('.jpg',np.full((24,32,3),100,np.uint8),[cv2.IMWRITE_JPEG_QUALITY,92])
            payload=encoded.tobytes();image=CompressedImage(format='jpeg',data=payload);image.header.stamp=stamp
            pose=PoseStamped();pose.header.stamp=stamp;pose.pose.orientation.w=1;pose.pose.position.x=2.
            with rosbag.Bag(str(p/'sensors.bag'),'w') as bag:
                bag.write('/r3live/input/image/compressed',image,stamp)
                bag.write('/roomgs/optical_pose',pose,stamp)
            poses(p);selection=p/'selection.json';selection.write_text(json.dumps(dict(additional_stamp_ns=[stamp.to_nsec()])))
            images(p,selection)
            rows=json.loads((p/'dataset/frames.json').read_text());self.assertEqual(len(rows),1)
            row=rows[0];self.assertEqual(row['stamp_ns'],1123456789);self.assertEqual(row['c2w'][0][3],2.)
            path=p/'dataset'/row['file_path'];self.assertEqual(path.suffix,'.jpg')
            self.assertEqual(path.read_bytes(),payload)
            self.assertEqual(row['sha256'],hashlib.sha256(payload).hexdigest())
            self.assertFalse(json.loads((p/'dataset_extension.json').read_text())['accepted_complete'])


if __name__=='__main__':unittest.main()
