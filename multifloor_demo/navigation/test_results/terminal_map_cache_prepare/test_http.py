"""Actual HTTP whitelist behavior, no ROS import, node or physics."""
import ast,copy,json,tempfile,threading,types,urllib.request,urllib.error
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
HERE=Path(__file__).resolve().parent
tree=ast.parse((HERE/'mission_server_candidate.py').read_text())
function=copy.deepcopy(next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='handler'))
with tempfile.TemporaryDirectory() as directory:
    root=Path(directory)/'demo';root.mkdir();outside=Path(directory)/'outside.txt';outside.write_text('private outside')
    files={'README.md':'public README','web/index.html':'public web','docs/architecture.md':'public docs',
           'private.txt':'private root','scripts/mission_server.py':'private source',
           'simulation/scenario.json':'private configuration','runs/secret/map_metadata.json':'private run',
           'navigation/test_results/private.txt':'private tests'}
    for name,content in files.items():
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(content)
    (root/'web/leak.txt').symlink_to(outside)
    ns={'ROOT':root,'Path':Path,'SimpleHTTPRequestHandler':SimpleHTTPRequestHandler,'urlsplit':urlsplit,'json':json}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),'actual_proposed_handler','exec'),ns)
    coordinator=types.SimpleNamespace(lock=threading.RLock(),run_dir=None)
    server=ThreadingHTTPServer(('127.0.0.1',0),ns['handler'](coordinator));server.daemon_threads=True
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    cases=[('/README.md',200,'public README'),('/README.md?v=1',200,'public README'),
           ('/web/index.html',200,'public web'),('/docs/architecture.md',200,'public docs'),
           ('/private.txt',404,None),('/scripts/mission_server.py',404,None),
           ('/simulation/scenario.json',404,None),('/runs/secret/map_metadata.json',404,None),
           ('/navigation/test_results/private.txt',404,None),('/web/../private.txt',404,None),
           ('/web/%2e%2e/private.txt',404,None),('/docs/../README.md',404,None),
           ('/web/%2e%2e/README.md',404,None),('/web/leak.txt',404,None)]
    records=[]
    try:
        for path,expected,body in cases:
            try:
                response=urllib.request.urlopen(f'http://127.0.0.1:{server.server_port}'+path,timeout=2)
            except urllib.error.HTTPError as error:response=error
            with response:
                received=response.read().decode();status=response.status
                ok=status==expected and (body is None or received==body)
                records.append(dict(path=path,expected=expected,actual=status,passed=ok,
                    no_store=response.headers.get('Cache-Control')=='no-store',
                    no_private_body='private outside' not in received and 'private source' not in received))
        # The exact README name must not turn an external symlink into a public document.
        (root/'README.md').unlink();(root/'README.md').symlink_to(outside)
        try:
            response=urllib.request.urlopen(f'http://127.0.0.1:{server.server_port}/README.md',timeout=2)
        except urllib.error.HTTPError as error:response=error
        with response:
            received=response.read().decode();records.append(dict(path='README external symlink',expected=404,actual=response.status,
                passed=response.status==404,no_store=response.headers.get('Cache-Control')=='no-store',no_private_body='private outside' not in received))
    finally:
        server.shutdown();server.server_close();thread.join(timeout=2)
    result=dict(passed=all(x['passed'] and x['no_private_body'] and x['no_store'] for x in records),
        checks=len(records),records=records,scope=__doc__,owned_HTTP_thread_clean=not thread.is_alive(),
        ROS_initialized=False,production_modified=False)
    (HERE/'http_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],checks=len(records),owned_HTTP_thread_clean=result['owned_HTTP_thread_clean'])))
    raise SystemExit(not result['passed'])
