"""Load the one canonical implementation for legacy source-tree imports."""
from pathlib import Path
import sys

def load(namespace,name):
    root=Path(__file__).resolve().parents[2]/'src/calibration/mocap_validation/aims_mocap_validation'
    sys.path.insert(0,str(root))
    source=root/'aims_mocap_validation'/(name+'.py')
    namespace['__package__']='aims_mocap_validation'
    namespace['__file__']=str(source)
    exec(compile(source.read_text(),str(source),'exec'),namespace)
