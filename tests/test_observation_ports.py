"""Check deferred hook lookups too; an import-only check cannot reach them."""
from pathlib import Path
import ast,sys,unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from common.observation import api
class ObservationPorts(unittest.TestCase):
    def test_all_declared_observation_lookups_exist(self):
        missing=[]
        for p in (ROOT/'src').rglob('*.py'):
            tree=ast.parse(p.read_text());aliases=set()
            for node in ast.walk(tree):
                if (isinstance(node,ast.Assign) and isinstance(node.value,ast.Call)
                    and isinstance(node.value.func,ast.Name) and node.value.func.id=='_observer_api'):
                    aliases.update(t.id for t in node.targets if isinstance(t,ast.Name))
            for node in ast.walk(tree):
                if not isinstance(node,ast.Attribute):continue
                v=node.value
                selected=(isinstance(v,ast.Name) and v.id in aliases or isinstance(v,ast.Call)
                    and isinstance(v.func,ast.Name) and v.func.id=='_observer_api')
                if selected and not hasattr(api(''),node.attr):missing.append((str(p.relative_to(ROOT)),node.lineno,node.attr))
        self.assertEqual(missing,[])
if __name__=='__main__':unittest.main()
