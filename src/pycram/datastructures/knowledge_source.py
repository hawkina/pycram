from abc import ABC, abstractmethod
from ..designator import DesignatorDescription
from ..plan_failures import PlanFailure

class KnowledgeNotAvailable(PlanFailure):
    """Thrown when a knowledge source can not provide the information for a query."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class KnowledgeSource(ABC):
    """
    Base class for all knowledge sources, knowledge sources provide information for specific use cases which the
    robot might encounter during execution.
    """

    def __init__(self, name: str, priority: int):
        """

        :param name: Name of this knowledge source
        :param priority: Priority by which this knowledge source is queried
        """
        self.name = name
        self.priority = priority

    @property
    @abstractmethod
    def is_available(self) -> bool:
        """
        Check if the knowledge source is available

        :return: True if the knowledge source is available and can be queried
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """
        Check if the knowledge source is connected

        :return: True if the knowledge source is connected
        """
        raise NotImplementedError

    @abstractmethod
    def connect(self):
        """
        Connect to the knowledge source
        """
        raise NotImplementedError

    def __str__(self):
        return f"{self.name} - Priority:{self.priority}"

    # @abstractmethod
    # def query(self, designator):
    #     raise NotImplementedError


class QueryKnowledge:
    def query_pose_for_object(self, designator: DesignatorDescription) -> DesignatorDescription:
        """
        Query the pose for an object from the knowledge source

        :param designator: Designator for the object
        :return: Designator with the pose information
        :raises KnowledgeNotAvailable: If the pose for the object is not available in this knowledge source
        """
        raise KnowledgeNotAvailable(f"Pose for object {designator} not available in {self.name}")

    def query_grasp_for_object(self, designator: DesignatorDescription) -> DesignatorDescription:
        """
        Query the grasp for an object from the knowledge source

        :param designator: Designator for the object
        :return: Designator with the grasp information
        :raises KnowledgeNotAvailable: If the grasp for the object is not available in this knowledge source
        """
        raise KnowledgeNotAvailable(f"Grasp for object {designator} not available in {self.name}")


class UpdateKnowledge:
    pass

