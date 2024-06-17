import unittest

# note: this is actually an integration test with knowrob.
# note: as such, knowrob2 must be running for this to work.

from pycram.knowledge.knowrob_knowledge import KnowrobKnowledge
from pycram.designators.action_designator import PickUpAction
from pycram.designator import DesignatorDescription, ObjectDesignatorDescription

from pycram.datastructures.pose import Pose

from rosservice import ROSServiceIOException

class TestKnowrobKnowledge(unittest.TestCase):
    def test_ask_for_object(self):
        # init connection
        kb = KnowrobKnowledge()
        try:
            if not kb.is_available:
                self.skipTest('Knowrob2 is not available, skipping test')
        except ROSServiceIOException:
            self.skipTest('No ros master available, skipping test')
        kb.connect()

        # init test pose
        # make sure this matches the pose in the query below
        pose = Pose(
            position = [0,1,2],
            orientation = [0,0,0.70711,0.70711],
            frame = 'map'
        )
        # create test object
        result_of_create = kb.prolog_client.once("create_object(Object, soma:'CerealBox', ['map', [0,1,2], [0,0,0.70711,0.70711]], [shape(box(0.1,0.2,0.3))])")
        obj_name = result_of_create["Object"]
        
        # ask for it back
        result = kb.query_pose_for_object(
            PickUpAction(arms=[],grasps=[],
                         object_designator_description=ObjectDesignatorDescription(
                             names=[obj_name])))

        # this always fails because it also compares the timestamps
        # self.assertEqual(result.object_designator_description.pose, pose)

        # copy over the variable header information
        pose.header.seq = result.object_designator_description.pose.header.seq
        pose.header.stamp = result.object_designator_description.pose.header.stamp

        try:
            # now it should work
            self.assertEqual(result.object_designator_description.pose, pose)
            
        finally:
            # remove the object from knowrob
            kb.prolog_client.once(f"kb_unproject(triple(_,_,'{obj_name}'))")


if __name__ == '__main__':
    unittest.main()
