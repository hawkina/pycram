from pycram.designators.action_designator import LookAtAction, DetectAction
from pycram.plan_failures import PerceptionObjectNotFound
from pycram.process_modules import hsrb_process_modules
from pycram.process_module import real_robot
from pycram.external_interfaces import robokudo



def test_perception():
    # LookAtAction().resolve().perform()
    with real_robot:
        try:  # todo changed couch_table to pickup_location_name
            object_desig = DetectAction(technique='region').resolve().perform()
        except PerceptionObjectNotFound:
            object_desig = {}
            print(object_desig)
        return object_desig


