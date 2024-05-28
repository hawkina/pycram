<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> db8874e... [knowledge] Update architecture for knowledge engine
# from ..datastructures.knowledge_engine import KnowledgeEngine
#
# import os
# import sys
<<<<<<< HEAD

# path = os.path.dirname(os.path.abspath(__file__))
#
# for py in [f[:-3] for f in os.listdir(path) if f.endswith('.py') and f != '__init__.py']:
#     mod = __import__('.'.join([__name__, py]), fromlist=[py])
#     classes = [getattr(mod, x) for x in dir(mod) if isinstance(getattr(mod, x), type)]
#     print(classes)
#     for cls in classes:
#         setattr(sys.modules[__name__], cls.__name__, cls)

# knowledge_engine = KnowledgeEngine()
#
=======
from ..datastructures.knowledge_engine import KnowledgeEngine
=======
>>>>>>> db8874e... [knowledge] Update architecture for knowledge engine

# path = os.path.dirname(os.path.abspath(__file__))
#
# for py in [f[:-3] for f in os.listdir(path) if f.endswith('.py') and f != '__init__.py']:
#     mod = __import__('.'.join([__name__, py]), fromlist=[py])
#     classes = [getattr(mod, x) for x in dir(mod) if isinstance(getattr(mod, x), type)]
#     print(classes)
#     for cls in classes:
#         setattr(sys.modules[__name__], cls.__name__, cls)

<<<<<<< HEAD
>>>>>>> 7d16285... [knowledge] Added Knowledge engine
=======
# knowledge_engine = KnowledgeEngine()
#
>>>>>>> db8874e... [knowledge] Update architecture for knowledge engine
